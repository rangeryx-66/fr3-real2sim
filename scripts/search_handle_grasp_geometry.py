"""Bounded, asset-independent handle-frame search using official Dex1 meshes."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.stats import qmc

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from grasp_compare.scene import load_scene
from grasp_compare.collision import Dex1SceneCollision
from filter_articulated_variants import local_handle_frame


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    scene=load_scene(args.scene);checker=Dex1SceneCollision(scene)
    source=json.loads(args.reference.read_text())['candidates'][0]
    raw=np.asarray(source['T_B_TCP']);tree=cKDTree(checker.target_B)
    anchor,tangent=local_handle_frame(tree,checker.target_B,raw[:3,3])
    if tangent@raw[:3,0]<0:tangent=-tangent
    normal=raw[:3,2]-tangent*(tangent@raw[:3,2]);normal/=np.linalg.norm(normal)
    lateral=np.cross(normal,tangent)
    frame=np.column_stack((tangent,lateral,normal))
    low,high=checker._contact_bounds(round(checker.geometry.open_width,5))
    pad_center=(low+high)/2
    settings=[('raw',np.zeros(3),np.zeros(3))]
    # Same small grid for every handle; translations are along the measured
    # handle, lateral direction, and approach normal, never world/object axes.
    offsets=[(0,0,0),(.006,0,0),(-.006,0,0),(0,.004,0),
             (0,-.004,0),(0,0,.004),(0,0,-.004)]
    for centre in ('raw','pad'):
        for pitch in (-20,-15,-10,-5,0,5,10,12,14,16,18,20):
            for offset in offsets:settings.append((centre,np.array(offset),np.array([0,pitch,0])))
    for centre in ('raw','pad'):
        for u in qmc.Halton(6,scramble=False).random(40)[1:]:
            settings.append((centre,(u[:3]*2-1)*.008,(u[3:]*2-1)*15))
    report={'kind':'bounded handle-frame grasp search; raw perception unchanged',
            'frame_B':frame.tolist(),'anchor_B':anchor.tolist(),
            'pad_center_TCP':pad_center.tolist(),'reference':source,
            'bounds':{'local_translation_m':.008,'rotation_deg':20,
                      'max_total_translation_m':.040,'clearance_m':.003},
            'decisions':[],'candidates':[]}
    existing=json.loads(args.output.read_text()) if args.output.exists() else None
    if existing:
        if existing['reference']!=source:raise ValueError('resume reference changed')
        report=existing
    completed={row['variant'] for row in report['decisions']}
    args.output.parent.mkdir(parents=True,exist_ok=True);seen=set()
    for centre,offset,angles in settings:
        rotation=frame@Rotation.from_euler('xyz',angles,degrees=True).as_matrix()@frame.T
        T=raw.copy();T[:3,:3]=rotation@raw[:3,:3]
        T[:3,3]=(raw[:3,3] if centre=='raw' else anchor-T[:3,:3]@pad_center)+frame@offset
        signature=tuple(np.round(T[:3].ravel(),7))
        if signature in seen:continue
        seen.add(signature)
        if f'local_{len(seen):03d}' in completed:continue
        translation=float(np.linalg.norm(T[:3,3]-raw[:3,3]));angle=float(Rotation.from_matrix(rotation).magnitude())
        row={'raw_rank':source['raw_rank'],'variant':f'local_{len(seen):03d}',
             'centre':centre,'offset_handle_m':offset.tolist(),'rpy_handle_deg':angles.tolist(),
             'translation_m':translation,'rotation_rad':angle,'T_B_TCP':T.tolist()}
        if translation>.040 or angle>np.deg2rad(20.01):row['status']='OUTSIDE_SEARCH_BOUNDS'
        elif tree.query(T[:3,3])[0]>.05:row['status']='OUTSIDE_TARGET_REGION'
        else:
            row['approach']=checker.check(T)
            row['status']=row['approach']['status']
            if row['status']=='FREE':
                row['closure']=checker.check_closure(T);row['status']=row['closure']['status']
                if row['status']=='CLOSURE_UNVERIFIED':report['candidates'].append(row)
        report['decisions'].append(row)
        args.output.write_text(json.dumps(report,indent=2))
        print(row['variant'],row['status'],len(report['candidates']),flush=True)
    report['candidates'].sort(key=lambda r:r['translation_m']+.03*r['rotation_rad'])
    args.output.write_text(json.dumps(report,indent=2))


if __name__=='__main__':main()
