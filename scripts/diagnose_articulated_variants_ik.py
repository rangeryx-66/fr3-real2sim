"""Offline URDF IK prescreen of Dex1 variants; diagnostic, not MoveIt success.

Input variants may be evaluator-only. This script never moves the robot and
must not be counted as a SAM3 or physical grasp run.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from urdf_chain import KinematicChain


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--variants',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--deadline-s',type=float,default=300)
    p.add_argument('--seeds',type=int,default=4)
    args=p.parse_args()
    variants=json.loads(args.variants.read_text())['candidates']
    model=ROOT/'config/r1a7_dex1.urdf'
    names=tuple(f'J{i}' for i in range(1,8))
    joints={j.get('name'):j for j in ET.parse(model).findall('joint')}
    lo=np.array([float(joints[n].find('limit').get('lower')) for n in names])+.01
    hi=np.array([float(joints[n].find('limit').get('upper')) for n in names])-.01
    chain=KinematicChain(model,'r1a7_world','r1a7_tcp',names)
    rng=np.random.default_rng(47686)
    rows=[];end=time.monotonic()+args.deadline_s
    for index,item in enumerate(variants):
        if time.monotonic()>=end:break
        target=np.asarray(item['T_B_TCP'])
        def residual(q):
            T=chain.forward(dict(zip(names,q)))
            rotation=(Rotation.from_matrix(target[:3,:3]).inv()*
                Rotation.from_matrix(T[:3,:3])).as_rotvec()
            return np.r_[T[:3,3]-target[:3,3],.08*rotation]
        best=None
        for seed in [np.array([0.,1.3,1.,-1.3,0.,0.,0.]),
                     *rng.uniform(lo,hi,size=(max(0,args.seeds-1),7))]:
            fit=least_squares(residual,np.clip(seed,lo,hi),bounds=(lo,hi),
                max_nfev=100,ftol=1e-4,xtol=1e-4,gtol=1e-4)
            error=np.linalg.norm(fit.fun)
            if best is None or error<best[0]:best=(error,fit.x,fit.fun)
            if np.linalg.norm(fit.fun[:3])<.004 and np.linalg.norm(fit.fun[3:])/.08<np.deg2rad(3):
                break
        pos=float(np.linalg.norm(best[2][:3]));rot=float(np.rad2deg(np.linalg.norm(best[2][3:])/.08))
        row={'index':index,'raw_rank':item['raw_rank'],'variant':item['variant'],
            'score':item['score'],'translation_m':item['translation_m'],
            'rotation_rad':item['rotation_rad'],'position_error_m':pos,
            'orientation_error_deg':rot,'kinematic_feasible':pos<.004 and rot<3.,
            'joint_margin_rad':float(np.min(np.minimum(best[1]-lo,hi-best[1]))),
            'q':best[1].tolist()}
        rows.append(row)
        if index%10==9 or row['kinematic_feasible']:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps({'kind':'offline URDF IK, not MoveIt/physical success',
                'tested':len(rows),'feasible':sum(r['kinematic_feasible'] for r in rows),
                'rows':rows},indent=2))
            print(f"{len(rows)}/{len(variants)} feasible={sum(r['kinematic_feasible'] for r in rows)}",flush=True)
    args.output.write_text(json.dumps({'kind':'offline URDF IK, not MoveIt/physical success',
        'tested':len(rows),'feasible':sum(r['kinematic_feasible'] for r in rows),
        'rows':rows},indent=2))


if __name__=='__main__':main()
