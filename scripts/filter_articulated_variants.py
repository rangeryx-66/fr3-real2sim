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
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from grasp_compare.adapters import read_candidates
from grasp_compare.collision import Dex1SceneCollision
from grasp_compare.scene import load_scene
from r1a7_grasp_adaptation import manifold_variants


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True,type=Path)
    parser.add_argument('--native',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--per-raw-limit',type=int,default=16)
    parser.add_argument('--top-k',type=int,default=100)
    args=parser.parse_args()
    scene=load_scene(args.scene)
    checker=Dex1SceneCollision(scene)
    target=scene.points_B[scene.target_mask]
    anchor=np.median(target,axis=0)
    tree=cKDTree(target)
    raw=read_candidates(args.native,'graspgenx',scene)
    accepted=[];raw_decisions=[];variant_decisions=[]
    counts={'raw':len(raw),'variants':0,'target_near':0,
                        'collision_free':0,'collision':0,'low_clearance':0}
    for candidate in raw:
        raw_check=checker.check(candidate.T_B_TCP,candidate.width_m)
        raw_decisions.append({'rank':candidate.rank,'score':candidate.score,
            'T_B_TCP':candidate.T_B_TCP.tolist(),'collision':raw_check})
        for variant in manifold_variants(candidate.T_B_TCP,anchor,limit=args.per_raw_limit):
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
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'kind':'Dex1 local manifold; raw GraspGenX unchanged',
        'counts':counts,'raw_decisions':raw_decisions,
        'variant_decisions':variant_decisions,
        'candidates':accepted[:args.top_k]},indent=2))
    print(json.dumps({'counts':counts,'saved':min(len(accepted),args.top_k)}),flush=True)


if __name__=='__main__':main()
