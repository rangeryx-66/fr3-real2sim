"""Existing unloaded-route alternatives, checked with actual execution timing.

Only the plan's joint route can change. Visual grasp, geometry, IK, controller,
collision predicates and every safety limit remain untouched.
"""
import json,sys,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.motion import timed_path
from native_g1_safe_park import route_to_joint,parking_targets
from agent_command_protocol_v12 import atomic_json

def repair(g,policy,config,solver,write_plan=True,report_folder=None):
    p=g['a'].output;plan=json.loads((p/'visual_plan.json').read_text());original=plan['preplan'];destination=report_folder or p;m=g['model'];moving=g['moving']().copy();fingers=np.asarray(g['robot'].get_joint_positions())[g['fingers']].copy()
    def checked(q):
        if m.margin(q)<=policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
        return g['collision'].check(m.poses(q,g['base'],finger_q=fingers),moving,False)
    def evaluate(path):
        try:
            count=sum(1 for _ in timed_path(solver,g['base'],path,1/240,policy.motion('PREGRASP'),np.array(config['arm_speed_limits'])/3,policy.data['arm_command_acceleration_rad_s2']['FREE'],checked))
            return dict(valid=True,timed_samples=count)
        except RuntimeError as e:return dict(valid=False,reason=repr(e))
    report=dict(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),same_visual_grasp=True,T_grasp=plan['T'],original_preplan=original,alternatives=[],direct=evaluate(original),valid=False)
    if report['direct']['valid']:report['valid']=True;report['selected']='original';atomic_json(destination/'AUTOMATIC_ROUTE_REPAIR.json',report);return report
    for label,station in list(parking_targets(g))[1:]:
        a,why1=route_to_joint(g,np.asarray(original[0]),station);b,why2=route_to_joint(g,station,np.asarray(original[-1]));item=dict(station=label,first_reason=why1,second_reason=why2);report['alternatives'].append(item)
        if a is None or b is None:continue
        path=a+b[1:];item.update(evaluate(path))
        if not item['valid']:continue
        report.update(valid=True,selected=label,preplan=path)
        plan['preplan']=path;plan['route_provenance']='existing parking_targets + route_to_joint, full original timed_path check; no grasp change';
        if write_plan:atomic_json(p/'visual_plan.json',plan)
        break
    atomic_json(destination/'AUTOMATIC_ROUTE_REPAIR.json',report);return report

def offline(p,cooked,output):
    from interaction_runtime.kinematics import FastNativeModel
    from cross_robot_adapter import RobotPinkIK
    from xarm7_native_execution import mapped_export
    from interactive_twin_recovery.mobile import physical_scene_class
    from revolute10_fixed import authored_handle_colliders
    from interaction_runtime.policy import Policy
    j=json.loads((p/'job.json').read_text());c=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text());plan=json.loads((p/'visual_plan.json').read_text());m=FastNativeModel(None,Path(j['asset_root']),json.loads((Path(j['source'])/'report.json').read_text()));export=json.loads(cooked.read_text());allowed=authored_handle_colliders(export,m.manifest['interaction_geometry']['selection']);scene=physical_scene_class(ROOT)(mapped_export(export),m,allowed)
    view=sorted((p/'rgbd').glob('generation*'))[-1];Q=np.asarray(json.loads((view/'camera.json').read_text())['robot_q']);g=dict(a=SimpleNamespace(output=p),model=m,robot=SimpleNamespace(get_joint_positions=lambda:Q),fingers=c['finger_indices'],moving=lambda:scene.moving_reference,collision=scene,base=plan['base'],wrist_config=json.loads((p/'WRIST_CALIBRATION.json').read_text())['configuration'])
    return repair(g,Policy.load(ROOT/'configs/interaction_runtime_v1.json'),c['gripper_config'],RobotPinkIK(c['gripper_config']),write_plan=False,report_folder=output)
if __name__=='__main__':print(json.dumps(offline(Path(sys.argv[1]),Path(sys.argv[2]),Path(sys.argv[3])),indent=2))
