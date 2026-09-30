"""Read-only MoveIt search for a collision-free R1 start posture in a live scene."""
import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np
import rclpy

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from articulated_demo.backend import ArticulatedBackend,Failure
from r1a7_backend import JOINTS
import r1a7_plant as plant


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=18816)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--samples-per-scale',type=int,default=200)
    args=p.parse_args()
    plant.URL=f'http://127.0.0.1:{args.port}'
    rclpy.init();node=None
    try:
        node=ArticulatedBackend(Path(
            '/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1/urdf/47686.urdf'))
        measured=node.measured()
        initial=dict(zip(measured.joint_state.name,measured.joint_state.position))
        home=np.asarray([initial[n] for n in JOINTS])
        rng=np.random.default_rng(47686)
        bounds=np.asarray([node.limits[n] for n in JOINTS])
        rows=[];best=None
        for scale in (.15,.35,.7,1.2):
            found=0
            for _ in range(args.samples_per_scale):
                q=np.clip(home+rng.normal(0,scale,size=7),bounds[:,0]+.02,bounds[:,1]-.02)
                state=type(measured)()
                state.joint_state.name=list(measured.joint_state.name)
                values={**initial,**dict(zip(JOINTS,q))}
                state.joint_state.position=[float(values[n]) for n in state.joint_state.name]
                try:node.validate(state)
                except Failure:continue
                found+=1
                distance=float(np.linalg.norm(q-home))
                row={'q':q.tolist(),'distance_from_current_rad':distance,
                     'focus_margin_rad':float(node.margin(state))}
                if best is None or distance<best['distance_from_current_rad']:best=row
                if len(rows)<40:rows.append(row)
            print(scale,found,flush=True)
        report={'kind':'read-only state-validity search; no robot motion',
                'home_q':home.tolist(),'best':best,'first_safe_states':rows}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2))
        print(json.dumps({'best':best}),flush=True)
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
