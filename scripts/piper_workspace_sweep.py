#!/usr/bin/env python3
"""Offline Piper mount sweep over the frozen Arena grasps.

This process never sends a trajectory or gripper command.  It re-expresses the
unchanged saved AnyGrasp JSON and complete frozen scene in each candidate base
frame, then calls the same MoveIt IK, collision, Cartesian, attachment, lift,
and (in full mode) global planning checks as the execution backend.
"""
from __future__ import annotations
import argparse,copy,json,os,sys,time,xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from moveit_msgs.msg import RobotState,CollisionObject,AttachedCollisionObject,AllowedCollisionEntry,PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene,GetPositionIK
from sensor_msgs.msg import JointState

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from backend import Backend,Failure,PROFILE,TOUCH,GROUP,TCP,stamped
from clutter_backend import ClutterBackend,collision_box,transform
from robot_profile import gripper_positions
from workspace_mount import from_dict,transform_pose,transform_grasp_data

OBJECTS=('mustard','raisin','hidden_tuna','bowl','banana','sugar','soup','mug')

def halton(index,base):
    result=0.;f=1.
    while index:
        f/=base;result+=f*(index%base);index//=base
    return result

def generate_mounts(n):
    mounts=[dict(id=0,base_xyz_m=[0.,0.,0.],base_yaw_deg=0.,workspace_shift_xyz_m=[0.,0.,0.])]
    ranges=[(-.12,.22),(-.16,.16),(-.06,.10),(-45.,45.)]
    for i in range(1,n):
        u=[halton(i,b) for b in (2,3,5,7)]
        v=[lo+(hi-lo)*x for (lo,hi),x in zip(ranges,u)]
        mounts.append(dict(id=i,base_xyz_m=v[:3],base_yaw_deg=v[3],workspace_shift_xyz_m=[0.,0.,0.]))
    return mounts

def load_cases(root):
    cases=[]
    for obj in OBJECTS:
        for result_path in sorted((root/obj).glob('B_seed_*.json')):
            result=json.loads(result_path.read_text())
            grasp_path=root/obj/'inputs'/f"seed_{int(result['seed']):04d}_grasps.json"
            coverage={int(c['rank']):float(c.get('hand_geometry',{}).get('min_pad_coverage',1.0))
                      for c in result.get('candidates',[])}
            cases.append(dict(object=obj,seed=int(result['seed']),result=result,
                              grasps=json.loads(grasp_path.read_text()),pad_coverage=coverage))
    if len(cases)!=40:raise RuntimeError(f'expected 40 frozen episodes, got {len(cases)}')
    return cases

class OfflineSweep(ClutterBackend):
    def __init__(self):
        Backend.__init__(self)
        self.mode='B';self.margin=.001;self.attached=None;self.last_contacts=[];self.planning_seconds=0.
        self.home=RobotState();self.home.joint_state=JointState()
        # Match the complete measured state used by the physical executor.  In
        # particular Piper's two finger joints are independent MoveIt state
        # variables even though the bridge derives them from one width command.
        # Omitting them silently evaluates collisions with a closed/default
        # finger state and substantially under-counts finger-table collisions.
        finger_state=gripper_positions(PROFILE,PROFILE.open_width_m)
        self.home.joint_state.name=list(PROFILE.arm_joints)+list(finger_state)
        self.home.joint_state.position=list(PROFILE.home)+list(finger_state.values())
        self.home.is_diff=False
        self.case=None
    def measured(self):
        state=copy.deepcopy(self.home);state.is_diff=True
        if self.attached is not None:state.attached_collision_objects=[copy.deepcopy(self.attached)]
        return state
    def phase(self,name):self.stage=name
    def reset_scene(self):
        had_attachment=self.attached is not None;self.attached=None
        req=ApplyPlanningScene.Request();scene=req.scene;scene.is_diff=True;scene.robot_state.is_diff=True
        if had_attachment:
            detach=AttachedCollisionObject();detach.object.id='box';detach.object.operation=CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects=[detach]
        if had_attachment:self.apply(req)
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        req.scene.world.collision_objects.append(collision_box('table',[.7,.7,.05],self.case['table_T']))
        req.scene.world.collision_objects.append(collision_box('box',[.045,.045,.05],self.case['target_T']))
        for ob in self.case['obstacles']:
            T=np.eye(4);T[:3,3]=ob['position']
            req.scene.world.collision_objects.append(collision_box(ob['id'],np.asarray(ob['aabb_size'])+2*self.margin,T))
        self.apply(req)
        for link in TOUCH:self.allow_collision_pair('box',link,True)
        self.support_contact(False)
    def set_case(self,case,config):
        result=case['result'];target_p,target_q=transform_pose(result['initial_target']['position'],result['initial_target']['quaternion_wxyz'],config)
        target_T=transform(target_p,target_q)
        obstacles=[];R=config.T_base_nominal[:3,:3]
        for original in result['initial_layout']:
            p,q=transform_pose(original['position'],original['quaternion_wxyz'],config)
            ob=dict(original);ob['position']=p.tolist();ob['quaternion_wxyz']=q.tolist();ob['aabb_size']=(np.abs(R)@np.asarray(original['aabb_size'])).tolist();obstacles.append(ob)
        table_p,table_q=transform_pose([.5,0,-.025],[1,0,0,0],config)
        self.case=dict(target_T=target_T,obstacles=obstacles,table_T=transform(table_p,table_q))
        self.initial=dict(box=target_p.tolist(),box_quat=target_q.tolist())
        self.reset_scene()
        return transform_grasp_data(case['grasps'],config)
    def fast_ik(self,T,seed):
        req=GetPositionIK.Request();ik=req.ik_request;ik.group_name=GROUP;ik.ik_link_name=TCP
        ik.pose_stamped=stamped(T);ik.robot_state=seed;ik.avoid_collisions=False;ik.timeout.nanosec=50_000_000
        result=self.call('compute_ik',req)
        if result.error_code.val!=1:raise Failure('NO_IK',str(result.error_code.val))
        self.validate(result.solution);return result.solution
    def coarse_candidate(self,g,data):
        detail=dict(rank=g['rank'],status='UNKNOWN',grasp_ik=False,pregrasp_ik=False)
        try:
            if not 0<g['width']<=PROFILE.open_width_m:raise Failure('GRIPPER_WIDTH','out of range')
            from frames import grasp_to_robot_tcp
            pre,grasp,_=grasp_to_robot_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'],PROFILE.grasp_tip_offset_m)
            gs=self.fast_ik(grasp,self.measured());detail['grasp_ik']=True
            self.fast_ik(pre,gs);detail['pregrasp_ik']=True;detail['status']='COARSE_VALID'
        except Failure as e:detail['status']=e.category;detail['detail']=str(e)
        return detail

def evaluate(node,cases,mount,mode):
    config=from_dict(mount);statuses=Counter();episodes=[];valid_total=0;grasp_ik=0;pregrasp_ik=0;started=time.monotonic()
    for case in cases:
        data=node.set_case(case,config);details=[]
        for g in data['grasps']:
            if mode=='coarse':detail=node.coarse_candidate(g,data)
            else:
                node.planning_seconds=0.;detail,plan=node.candidate(g,data)
                detail['grasp_ik']=detail.get('stage')!='GRASP_IK' or detail.get('status')=='VALID'
                detail['pregrasp_ik']=detail.get('stage') not in ('GRASP_IK','PREGRASP') or detail.get('status')=='VALID'
            valid_status='COARSE_VALID' if mode=='coarse' else 'VALID'
            coverage=case['pad_coverage'].get(int(g['rank']),1.0)
            if detail['status']==valid_status and coverage<.045:
                detail.update(status='INSUFFICIENT_PAD_OVERLAP',
                              detail=f'pad coverage {coverage:.6f} < 0.045000')
            details.append(detail);statuses[detail['status']]+=1
            grasp_ik+=bool(detail.get('grasp_ik'));pregrasp_ik+=bool(detail.get('pregrasp_ik'))
        valid_status='COARSE_VALID' if mode=='coarse' else 'VALID';valid=sum(d['status']==valid_status for d in details);valid_total+=valid
        episodes.append(dict(object=case['object'],seed=case['seed'],has_executable=valid>0,executable_candidates=valid,status_counts=dict(Counter(d['status'] for d in details))))
    total=sum(len(c['grasps']['grasps']) for c in cases)
    return dict(mount=mount,mode=mode,episodes_with_executable=sum(e['has_executable'] for e in episodes),
                executable_candidates=valid_total,total_candidates=total,grasp_ik_success=grasp_ik,
                pregrasp_ik_success=pregrasp_ik,status_counts=dict(statuses),wall_seconds=time.monotonic()-started,
                episodes=episodes)

def main():
    p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mounts',type=Path);p.add_argument('--generate',type=int,default=65);p.add_argument('--mode',choices=['coarse','full'],default='coarse')
    p.add_argument('--start-index',type=int,default=0);p.add_argument('--end-index',type=int);a=p.parse_args()
    mounts=json.loads(a.mounts.read_text()) if a.mounts else generate_mounts(a.generate)
    a.output.mkdir(parents=True,exist_ok=True);cases=load_cases(a.results_root)
    rclpy.init();node=OfflineSweep()
    try:
        for i,mount in enumerate(mounts):
            if i<a.start_index or (a.end_index is not None and i>=a.end_index):continue
            path=a.output/f'mount_{int(mount.get("id",i)):03d}.json'
            if path.exists():continue
            result=evaluate(node,cases,mount,a.mode);path.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='episodes'}),flush=True)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
