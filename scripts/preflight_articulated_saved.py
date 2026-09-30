"""Replay saved cabinet-relative grasp poses through MoveIt without execution.

The source grasp poses and RGB-D are immutable. A pose is moved rigidly with
the cabinet and its measured hinge angle. This is an IK/planning diagnostic,
not a new perception trial or an articulated-object success.
"""
import argparse
from collections import Counter
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
from articulated_demo.preflight import candidate_preflight,probe_ik
from run_articulated_47686 import ready
from run_r1a7_generalization import SIM,start,stop
import r1a7_plant as plant


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--max-candidates',type=int,default=40)
    p.add_argument('--random-seeds',type=int,default=8)
    p.add_argument('--timeout-s',type=float,default=.25)
    p.add_argument('--ablation-candidates',type=int,default=5)
    p.add_argument('--require-home-valid',action='store_true')
    p.add_argument('--asset-x',type=float)
    p.add_argument('--asset-y',type=float)
    p.add_argument('--asset-yaw-deg',type=float)
    p.add_argument('--home-q',type=float,nargs=7)
    p.add_argument('--gpu',type=int,default=6)
    p.add_argument('--port',type=int,default=18816)
    p.add_argument('--ros-domain',type=int,default=122)
    args=p.parse_args()
    source=json.loads((args.source/'report.json').read_text())
    original=source['asset_installation']
    placement=dict(original)
    home_q=args.home_q if args.home_q is not None else source.get('home_q',[0.,1.3,1.,-1.3,0.,0.,0.])
    if args.asset_x is not None:placement['x_m']=args.asset_x
    if args.asset_y is not None:placement['y_m']=args.asset_y
    if args.asset_yaw_deg is not None:placement['yaw_deg']=args.asset_yaw_deg
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    variants=json.loads((args.source/'dex1_variants.json').read_text())['candidates']
    candidates=variants[:args.max_candidates]
    delta=Rotation.from_euler('z',placement['yaw_deg']-original['yaw_deg'],
                              degrees=True).as_matrix()
    original_xy=np.array([original['x_m'],original['y_m'],0.])
    placed_xy=np.array([placement['x_m'],placement['y_m'],0.])
    hinge_delta=np.eye(4)
    def placed_pose(item):
        T=np.asarray(item['T_B_TCP']).copy()
        T[:3,:3]=delta@T[:3,:3]
        T[:3,3]=placed_xy+delta@(T[:3,3]-original_xy)
        return hinge_delta@T
    report={'kind':'saved-candidate MoveIt preflight; no perception rerun or execution',
            'source':str(args.source.resolve()),'source_capture_sha256':source['capture']['sha256'],
            'asset_installation':placement,'candidate_results':[],
            'home_q':home_q,
            'transformed_from_source_installation':placement!=original,
            'status':'STARTED'}
    def save(): (output/'report.json').write_text(json.dumps(report,indent=2,default=str))
    save()
    os.environ['R1A7_PLANT_PORT']=str(args.port)
    os.environ['ROS_DOMAIN_ID']=str(args.ros_domain)
    os.environ['ROS_LOCALHOST_ONLY']='1'
    plant.URL=f'http://127.0.0.1:{args.port}'
    env={**os.environ,'R1A7_BASE_POSE':'0.329,-0.175,0.237,56.295',
         'R1A7_PEDESTAL_SIZE':'0.10,0.10,0.20',
         'R1A7_RUN_DIR':str(output),'NO_PROXY':'127.0.0.1,localhost',
         'no_proxy':'127.0.0.1,localhost','OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
    processes=[];node=None
    try:
        processes.append(start([SIM,'-u',str(ROOT/'src/r1a7_articulated_sim_server.py'),
            '--gpu',str(args.gpu),'--port',str(args.port),
            '--asset-x',str(placement['x_m']),'--asset-y',str(placement['y_m']),
            '--asset-yaw-deg',str(placement['yaw_deg']),
            '--fixture-height-m',str(placement['fixture_height_m']),
            '--home-q',*[str(x) for x in home_q],
            '--camera-offset',*[str(x) for x in source['camera_offset_m']]],output/'isaac.log',env))
        processes.append(start(['ros2','launch',str(ROOT/'src/r1a7_moveit.launch.py')],
                               output/'moveit.log',env))
        processes.append(start([sys.executable,'-u',str(ROOT/'src/r1a7_ros_bridge.py')],
                               output/'bridge.log',env))
        state=ready(args.port,processes,time.monotonic()+150)
        report['settled_joint_q_rad']=state['joint_q']
        report['collision_boxes']=state['collision_boxes']
        if not plant.command({'op':'pause'},timeout=10).get('ok'):
            raise RuntimeError('Isaac physics pause failed')
        report['planning_joint_q_rad']=float(plant.state()['joint_q'])
        capture=output/'capture.npz'
        plant.command({'op':'capture','path':str(capture),
            'evaluation_mask_path':str(output/'eval_gt_handle_mask.npy')},timeout=90)
        rclpy.init();node=ArticulatedBackend(
            Path('/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1/urdf/47686.urdf'))
        node.scene();report['fk_tcp_check']=node.check_fk()
        live=plant.state()
        source_q=float(source.get('planning_joint_q_rad',
            source['asset_joint']['initial_q_rad']))
        live_q=float(live['joint_q'])
        joint=live['joint_name'];moving=live['moving_link']
        root_now=node.chain.root_to_link(moving,{joint:live_q})
        root_source=node.chain.root_to_link(moving,{joint:source_q})
        T_moving_now=matrix(live['moving_pose']['position'],
                            live['moving_pose']['quaternion_wxyz'])
        T_world_root=T_moving_now@np.linalg.inv(root_now)
        T_moving_source=T_world_root@root_source
        hinge_delta=T_moving_now@np.linalg.inv(T_moving_source)
        report['hinge_frame_alignment']={'source_joint_q_rad':source_q,
            'live_joint_q_rad':live_q,
            'method':'rigid source moving-link grasp transform to live moving link'}
        report['handle_visible_pixels_evaluation_only']=int(
            np.load(output/'eval_gt_handle_mask.npy').sum())
        try:
            node.validate(node.measured())
            report['home_state_validity']={'valid':True}
        except Failure as error:
            report['home_state_validity']={'valid':False,'category':error.category,
                                           'detail':str(error)}
        if args.require_home_valid and not report['home_state_validity']['valid']:
            report['status']='HOME_COLLISION';save();return
        if abs(live_q) > np.deg2rad(2):
            report['status']='INITIAL_DOOR_DRIFT';save();return
        if report['handle_visible_pixels_evaluation_only'] < 30:
            report['status']='HANDLE_NOT_VISIBLE';save();return
        for item in candidates:
            result=candidate_preflight(node,placed_pose(item),
                random_seeds=args.random_seeds,timeout_s=args.timeout_s)
            report['candidate_results'].append({'raw_rank':item['raw_rank'],
                'variant':item['variant'],**result})
            save()
        home=node.measured()
        ablation=[]
        diverse=[];seen=set()
        for item,result in zip(candidates,report['candidate_results']):
            if result['status'] not in seen:
                diverse.append(item);seen.add(result['status'])
        diverse.extend(item for item in candidates if item not in diverse)
        for item in diverse[:min(args.ablation_candidates,len(candidates))]:
            trials=[]
            for seeds,timeout,optimized in ((0,.1,0),(8,.25,2),(16,.5,4)):
                outcome,_=probe_ik(node,placed_pose(item),home,
                    random_seeds=seeds,timeout_s=timeout,
                    optimized_seeds=optimized)
                trials.append({'random_seeds':seeds,'optimized_seeds':optimized,
                    'timeout_s':timeout,'kinematic_ik':outcome['kinematic_ik'],
                    'collision_free':outcome['collision_free'],
                    'margin_over_005':outcome['margin_over_005']})
            ablation.append({'raw_rank':item['raw_rank'],'variant':item['variant'],
                             'trials':trials})
        report['ik_ablation']=ablation
        report['funnel']={'dex1_collision_free':len(candidates),
            'kinematic_ik':sum(x['grasp']['kinematic_ik']>0 for x in report['candidate_results']),
            'collision_free_ik':sum(x['grasp']['collision_free']>0 for x in report['candidate_results']),
            'margin_over_005':sum(x['grasp']['margin_over_005']>0 for x in report['candidate_results']),
            'margin_over_008':sum(x['grasp']['margin_over_008']>0 for x in report['candidate_results']),
            'pregrasp_ik':sum(x.get('pregrasp',{}).get('kinematic_ik',0)>0 for x in report['candidate_results']),
            'approach':sum('approach_points' in x for x in report['candidate_results']),
            'pregrasp_plan':sum('pregrasp_plan_points' in x for x in report['candidate_results']),
            'full_arc':sum(x['status']=='FULL_PATH_PLANNED' for x in report['candidate_results']),
            'status':dict(Counter(x['status'] for x in report['candidate_results']))}
        report['status']='PREFLIGHT_COMPLETE';save()
        print(json.dumps(report['funnel']),flush=True)
    finally:
        try:
            if node is not None:
                node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
        finally:
            save();stop(processes)


if __name__=='__main__':main()
