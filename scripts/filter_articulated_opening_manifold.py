"""Small handle manifold around selected raw seeds, using only saved RGB-D."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from filter_articulated_variants import local_handle_frame
from grasp_compare.scene import load_scene
from grasp_compare.adapters import read_candidates
from grasp_compare.collision import Dex1SceneCollision


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--raw-ranks',type=int,nargs='+',default=[82])
    p.add_argument('--axial-degrees',type=float,nargs='+',default=[-10.,0.,10.])
    p.add_argument('--slide-mm',type=float,nargs='+',default=[-8.,0.,8.])
    args=p.parse_args()
    scene=load_scene(args.source/'grasp_scene.npz')
    raw=read_candidates(args.source/'graspgenx_native.json','graspgenx',scene)
    target=scene.points_B[scene.target_mask];tree=cKDTree(target)
    checker=Dex1SceneCollision(scene,clearance_m=.003)
    candidates=[];decisions=[]
    settings=[(0.,0.,0.)]+[(s/1000,a,0.) for s in args.slide_mm
        for a in args.axial_degrees if s!=0 or a!=0]+[(0.,0.,d) for d in (-.004,.004)]
    for candidate in raw:
        if candidate.rank not in args.raw_ranks:continue
        anchor,axis=local_handle_frame(tree,target,candidate.T_B_TCP[:3,3])
        for slide,angle,depth in settings:
            T=candidate.T_B_TCP.copy()
            T[:3,3]+=slide*axis+depth*T[:3,2]
            T[:3,:3]=Rotation.from_rotvec(np.deg2rad(angle)*axis).as_matrix()@T[:3,:3]
            label=f'opening_slide{slide:+.3f}_axial{angle:+.0f}_depth{depth:+.3f}'
            check=checker.check(T,candidate.width_m)
            row={'raw_rank':candidate.rank,'variant':label,'T_B_TCP':T.tolist(),
                'score':float(candidate.score),'width_m':candidate.width_m,
                'translation_m':float(np.linalg.norm(T[:3,3]-candidate.T_B_TCP[:3,3])),
                'rotation_rad':float(abs(np.deg2rad(angle))),
                'local_anchor_B':anchor.tolist(),'local_handle_axis_B':axis.tolist(),
                'collision':check,'target_distance_m':float(tree.query(T[:3,3])[0])}
            decisions.append(row)
            if check['status']=='FREE' and row['target_distance_m']<=.05:candidates.append(row)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'kind':'bounded handle manifold; unmodified raw model output',
        'source':str(args.source),'tested':len(decisions),'candidates':candidates,'decisions':decisions},indent=2))
    print(json.dumps({'tested':len(decisions),'collision_free':len(candidates)}))


if __name__=='__main__':main()
