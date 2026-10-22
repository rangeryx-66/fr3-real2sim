"""Fast MoveIt home-collision scan of a bounded cabinet installation grid."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from articulated_demo.backend import ArticulatedBackend,Failure,matrix
from run_articulated_47686 import ready
from run_r1a7_generalization import SIM,start,stop
import r1a7_plant as plant


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gpu',type=int,default=6)
    p.add_argument('--port',type=int,default=18826)
    p.add_argument('--ros-domain',type=int,default=123)
    args=p.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    reference=(.45,.05,-90.)
    env={**os.environ,'R1A7_BASE_POSE':'0.329,-0.175,0.237,56.295',
         'R1A7_PEDESTAL_SIZE':'0.10,0.10,0.20',
         'R1A7_PLANT_PORT':str(args.port),'R1A7_RUN_DIR':str(output),
         'ROS_DOMAIN_ID':str(args.ros_domain),'ROS_LOCALHOST_ONLY':'1',
         'NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost',
         'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
    os.environ.update({k:env[k] for k in ('R1A7_PLANT_PORT','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY')})
    plant.URL=f'http://127.0.0.1:{args.port}'
    processes=[];node=None;rows=[]
    try:
        processes.append(start([SIM,'-u',str(ROOT/'src/r1a7_articulated_sim_server.py'),
            '--gpu',str(args.gpu),'--port',str(args.port),
            '--asset-x',str(reference[0]),'--asset-y',str(reference[1]),
            '--asset-yaw-deg',str(reference[2])],output/'isaac.log',env))
        processes.append(start(['ros2','launch',str(ROOT/'src/r1a7_moveit.launch.py')],
            output/'moveit.log',env))
        processes.append(start([sys.executable,'-u',str(ROOT/'src/r1a7_ros_bridge.py')],
            output/'bridge.log',env))
        ready(args.port,processes,time.monotonic()+150)
        rclpy.init();node=ArticulatedBackend(Path(
            '/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1/urdf/47686.urdf'))
        state=plant.state();boxes=state['collision_boxes']
        home=node.measured()
        for x in (.20,.275,.35,.425,.50):
            for y in (.05,.125,.20,.275):
                for yaw in (-150.,-120.,-90.):
                    R=Rotation.from_euler('z',yaw-reference[2],degrees=True)
                    changed=[]
                    for box in boxes:
                        item=dict(box)
                        if item['id']=='cabinet_fixture':
                            item['center']=[x,y,item['center'][2]]
                        else:
                            center=np.asarray(item['center'])-np.array([reference[0],reference[1],0.])
                            item['center']=(np.array([x,y,0.])+R.apply(center)).tolist()
                            initial=Rotation.from_quat(np.roll(item['quaternion_wxyz'],-1))
                            item['quaternion_wxyz']=np.roll((R*initial).as_quat(),1).tolist()
                        changed.append(item)
                    node.scene(collision_boxes=changed,door_collision_mode='obb')
                    try:
                        node.validate(home)
                        valid=True;detail=None
                    except Failure as error:
                        valid=False;detail=str(error)
                    rows.append({'asset_x_m':x,'asset_y_m':y,'asset_yaw_deg':yaw,
                                 'home_collision_free':valid,'collision_detail':detail,
                                 'predicted_door_box':next(b for b in changed if b['id']=='cabinet_door')})
                    (output/'report.json').write_text(json.dumps({'kind':'MoveIt OBB home-state collision scan; no grasp execution',
                        'reference_pose':reference,'reference_joint_q_rad':state['joint_q'],
                        'reference_boxes':boxes,'tested':len(rows),'rows':rows},indent=2))
                    print(len(rows),x,y,yaw,valid,flush=True)
    finally:
        try:
            if node is not None:
                node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
        finally:stop(processes)


if __name__=='__main__':main()
