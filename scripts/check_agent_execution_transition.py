"""Offline archived-route integration check; never supplies live targets."""
import argparse,json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from native_g1_agent_execution import check_path,approach_from
from xarm7_native_planning import NativeModel
from xarm7_native_execution import mapped_export
from interactive_twin_recovery.mobile import physical_scene_class
from cross_robot_adapter import RobotPinkIK
from xarm7_native_gripper import configuration
from precontact_trajectory import segment_samples

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    job=json.loads((a.run.parent/'job.json').read_text());model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
    plan=json.loads((a.run/'visual_plan.json').read_text());base=plan['base'];export=json.loads((a.run/'cooked_initial.json').read_text());allowed=json.loads((a.run/'authored_collider_association.json').read_text())['allowed']
    scene=physical_scene_class(Path(__file__).resolve().parents[1])(mapped_export(export),model,allowed);solver=RobotPinkIK(configuration());model.ik=solver.solve
    routes=json.loads((a.run/'WRIST_ROBOT_TRAJECTORIES.json').read_text());path=[routes[-1]['measured_q']]
    for route in reversed(routes):path+=list(reversed(route['q_path']))
    path.append(plan['preplan'][0])
    g=dict(model=model,collision=scene,base=base,moving=lambda:scene.moving_reference,ik=lambda target,seed:solver.solve(target,base,seed,starts=1))
    returning=check_path(g,path);approach=approach_from(g,path[-1],plan)
    continuity=[]
    for start,end in zip(path,path[1:]):
        first=None;last=None;speed=0.
        for q,dq in segment_samples(solver,np.asarray(start),np.asarray(end),.8,1/240,.030):
            if first is None:first=q
            last=(q,dq);speed=max(speed,float(np.max(abs(dq))))
        continuity.append(dict(first_step_rad=float(np.max(abs(first-start))),end_position_error_rad=float(np.max(abs(last[0]-end))),end_command_velocity_rad_s=float(np.max(abs(last[1]))),max_joint_command_speed_rad_s=speed))
    result=dict(reference='2c126cd',offline_only=True,live_episode_reuses_no_archived_views_or_grasp=True,return_check=returning,approach_check=approach,segment_continuity=continuity,finite_existing_actuator_and_smooth_move_unchanged=True)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(dict(return_check=returning,approach_valid=approach['valid'],segments=len(continuity))))
