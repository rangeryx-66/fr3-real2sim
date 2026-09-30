"""Reuse generic Dex1 manifold around unmodified GraspGenX handle candidates.

Runs in the geometry Python environment. Source RGB-D and target mask are the
only scene inputs; this script has no access to Isaac articulation ground truth.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from grasp_compare.adapters import read_candidates
from grasp_compare.collision import Dex1SceneCollision
from grasp_compare.scene import load_scene
from r1a7_grasp_adaptation import Variant, manifold_variants


def local_handle_frame(tree, points, origin, radius=.035):
    """Anchor and tangent from the handle patch nearest this raw grasp."""
    _, nearest = tree.query(origin)
    neighborhood = tree.query_ball_point(points[nearest], radius)
    patch = points[neighborhood]
    if len(patch) < 8:
        _, indices = tree.query(points[nearest], k=min(24,len(points)))
        patch = points[np.atleast_1d(indices)]
    anchor = np.median(patch, axis=0)
    centered = patch - anchor
    _, singular, vectors = np.linalg.svd(centered, full_matrices=False)
    tangent = vectors[0]
    if singular[0] < 1e-5:
        raise ValueError('handle patch is too small to estimate a tangent')
    return anchor, tangent


def handle_variants(raw, anchor, tangent, limit, pad_center=None):
    """Bounded door-handle manifold: slide, axial roll, and insertion depth."""
    raw = np.asarray(raw, dtype=float)
    count = 0
    for variant in manifold_variants(raw, anchor, limit=min(8,limit)):
        yield variant
        count += 1
        if count >= limit: return
    if pad_center is not None:
        # Centre the local handle patch on the actual official pad aperture.
        # Use the generic orientation search; no asset-specific TCP offset.
        for pitch in (-20., -10., 0., 10., 20.):
            T=raw.copy()
            T[:3,:3]=raw[:3,:3]@Rotation.from_euler('y',pitch,degrees=True).as_matrix()
            T[:3,3]=anchor-T[:3,:3]@pad_center
            distance=float(np.linalg.norm(T[:3,3]-raw[:3,3]))
            if distance>.065:continue
            yield Variant(T,f'official_pad_center_pitch{pitch:+.0f}',distance,float(np.deg2rad(abs(pitch))))
            count+=1
            if count>=limit:return
    settings = [(s,0.,0.) for s in (-.012,.012,-.022,.022)]
    settings += [(0.,a,0.) for a in (-15.,15.,-30.,30.)]
    settings += [(0.,0.,d) for d in (-.008,.008)]
    settings += [(s,a,d) for s in (-.012,.012,-.022,.022)
                 for a in (-15.,15.,-30.,30.) for d in (0.,-.008,.008)]
    for slide, angle, depth in settings:
        if slide == angle == depth == 0: continue
        T = raw.copy()
        T[:3,3] += slide*tangent + depth*raw[:3,2]
        T[:3,:3] = Rotation.from_rotvec(np.deg2rad(angle)*tangent).as_matrix() @ raw[:3,:3]
        yield Variant(T,f'handle_slide{slide:+.3f}_axial{angle:+.0f}_depth{depth:+.3f}',
            float(np.linalg.norm(T[:3,3]-raw[:3,3])),float(np.deg2rad(abs(angle))))
        count += 1
        if count >= limit: return


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True,type=Path)
    parser.add_argument('--native',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--per-raw-limit',type=int,default=16)
    parser.add_argument('--top-k',type=int,default=100)
    parser.add_argument('--max-raw',type=int,default=100)
    parser.add_argument('--raw-ranks',type=int,nargs='*',
                        help='optional source ranks for a small ablation')
    parser.add_argument('--clearance-m',type=float,default=.003)
    args=parser.parse_args()
    scene=load_scene(args.scene)
    checker=Dex1SceneCollision(scene,clearance_m=args.clearance_m)
    target=scene.points_B[scene.target_mask]
    pad_low,pad_high=checker._contact_bounds(round(checker.geometry.open_width,5))
    pad_center=(pad_low+pad_high)/2
    tree=cKDTree(target)
    raw=read_candidates(args.native,'graspgenx',scene)
    if args.raw_ranks is not None:
        selected_ranks=set(args.raw_ranks)
        raw=[candidate for candidate in raw if candidate.rank in selected_ranks]
    else:
        raw=raw[:args.max_raw]
    accepted=[];raw_decisions=[];variant_decisions=[]
    counts={'raw':len(raw),'variants':0,'target_near':0,
                        'collision_free':0,'collision':0,'low_clearance':0}
    for candidate in raw:
        anchor,tangent=local_handle_frame(tree,target,candidate.T_B_TCP[:3,3])
        raw_check=checker.check(candidate.T_B_TCP,candidate.width_m)
        raw_decisions.append({'rank':candidate.rank,'score':candidate.score,
            'T_B_TCP':candidate.T_B_TCP.tolist(),'collision':raw_check,
            'closure':checker.check_closure(candidate.T_B_TCP),
            'local_anchor_B':anchor.tolist(),'local_handle_axis_B':tangent.tolist()})
        for variant in handle_variants(candidate.T_B_TCP,anchor,tangent,args.per_raw_limit,pad_center):
            if variant.label=='raw': continue
            counts['variants']+=1
            distance=float(tree.query(variant.transform[:3,3])[0])
            record={'raw_rank':candidate.rank,'variant':variant.label,
                'translation_m':variant.translation_m,
                'rotation_rad':variant.rotation_rad,'target_distance_m':distance}
            if distance>.05:
                record['status']='OUTSIDE_TARGET_REGION'
                variant_decisions.append(record)
                continue
            counts['target_near']+=1
            check=checker.check(variant.transform,candidate.width_m)
            if check['status']=='FREE':
                closure=checker.check_closure(variant.transform)
                check={**closure, 'approach':check}
            record.update(status=check['status'],collision=check)
            variant_decisions.append(record)
            counts['collision' if check['status']=='COLLISION' else
                   'collision_free' if check['status']=='FREE' else 'low_clearance']+=1
            if check['status']!='FREE':continue
            score=float(candidate.score - 3*variant.translation_m - .1*variant.rotation_rad)
            accepted.append({'raw_rank':candidate.rank,'raw_score':candidate.score,
                'score':score,'variant':variant.label,'T_B_TCP':variant.transform.tolist(),
                'translation_m':variant.translation_m,'rotation_rad':variant.rotation_rad,
                'target_distance_m':distance,'width_m':candidate.width_m,'collision':check})
    accepted.sort(key=lambda row:row['score'],reverse=True)
    first_by_raw={}
    for row in accepted:
        first_by_raw.setdefault(row['raw_rank'],row)
    # Preserve coverage across source grasp regions before filling remaining
    # slots by score. This changes ranking, not the model's raw predictions.
    selected=list(first_by_raw.values())
    selected_ids={id(row) for row in selected}
    selected.extend(row for row in accepted if id(row) not in selected_ids)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'kind':'Dex1 local manifold; raw GraspGenX unchanged',
        'counts':counts,'clearance_m':args.clearance_m,
        'raw_decisions':raw_decisions,
        'variant_decisions':variant_decisions,
        'candidates':selected[:args.top_k]},indent=2))
    print(json.dumps({'counts':counts,'saved':min(len(selected),args.top_k),
                      'distinct_raw_ranks':len(first_by_raw)}),flush=True)


if __name__=='__main__':main()
