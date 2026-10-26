"""Closed-state collision/timed-route replay, without Isaac or new physics.

Native geometry is used ONLY for safety. No new grasp, scene restoration,
object law, executed release/park, or success of the closed episode is inferred.
"""
import argparse,hashlib,json,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np


def replay(episode,sequence):
    ep=Path(episode)
    if json.loads((ep/'PROCESS_COMPLETION.json').read_text()).get('actor_exited') is not True:
        raise ValueError('CLOSED_EPISODE_REQUIRED')
    response=ep/'agent_responses'/('%03d.json'%sequence)
    outcome=json.loads(response.read_text())['result']
    if not outcome.get('release',{}).get('confirmed') or not outcome.get('hold_confirmed'):
        raise ValueError('MEASURED_RELEASED_HOLD_REQUIRED')
    state=outcome['stages'][-1]['state']
    if state['latch'] is not None:raise ValueError('NO_HARD_STOP_REPLAY_CONTINUATION')
    job=json.loads((ep/'job.json').read_text());effective=json.loads((ep/'EFFECTIVE_RUNTIME.json').read_text())
    config=json.loads((Path(job['v8_robot_io'])/'NATIVE_CONFIG.json').read_text())
    import xarm7_native_gripper
    xarm7_native_gripper.OUT=Path(job['v8_robot_io'])  # same explicit binding as V8 entry/inspection
    from interaction_runtime.kinematics import FastNativeModel
    from cross_robot_adapter import RobotPinkIK
    from agentic_interaction_v81.collision import MobileScene
    from revolute10_fixed import authored_handle_colliders
    from interaction_runtime.policy import Policy
    from agentic_interaction_v81.exit_routes import timed_returns
    from .staged_exit import parking_targets
    import native_g1_safe_park as native
    root=Path(__file__).resolve().parents[1]
    source=json.loads((Path(job['source'])/'report.json').read_text())
    model=FastNativeModel(None,Path(job['asset_root']),source)
    export_path=ep/'cooked_initial.json'
    if not export_path.exists():export_path=ep/'planning_current_scene.json'
    export=json.loads(export_path.read_text())
    allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])
    q=np.asarray(state['robot_q']);arm=np.array(effective['arm_indices']);fingers=np.array(effective['finger_indices'])
    base=np.asarray(state['base']);moving=np.asarray(state['moving_pose'])
    scene=MobileScene(export,model,allowed,np.asarray(source['robot_base_pose']))
    r=NS(model=model,robot=NS(get_joint_positions=lambda:q.copy()),arm=arm,fingers=fingers,
        base=base,collision=scene,solver=RobotPinkIK(config),c=config,dt=effective['physics_dt'],
        policy=Policy(effective['policy'],effective['policy_sha256']))
    g=dict(model=model,robot=r.robot,arm=arm,fingers=fingers,base=base,collision=scene,
        moving=lambda:moving.copy(),qtarget=np.array(state['retained_command']),
        ROOT=root,wrist_config=json.loads((ep/'WRIST_CALIBRATION.json').read_text())['configuration'])
    r.g=g;ops=NS(r=r,g=g,articulation=lambda:NS(joint_type=job['articulation_v7']['joint_type']))
    checks=[];began=time.monotonic();selected=None
    with timed_returns(r) as audit:
        for region,target in parking_targets(ops):
            path,reason=native.route_to_joint(g,q[arm],target)
            row=dict(region=region,target_q=np.asarray(target).tolist(),valid=path is not None,reason=reason)
            checks.append(row)
            print('PARK_REPLAY',region,reason,flush=True)
            if path is not None:
                selected=row;break
    return dict(scope='Static closed measured state, original full native collision and timed path; no Isaac/new contact or measured park',
        episode=ep.name,source_sequence=sequence,response_sha256=hashlib.sha256(response.read_bytes()).hexdigest(),
        geometry_file=export_path.name,geometry_sha256=hashlib.sha256(export_path.read_bytes()).hexdigest(),
        current_coordinate_native=state['coordinate_native'],original_targets=3,
        checks=checks,timed_audit=audit,selected=selected,wall_s=time.monotonic()-began,
        old_episode_status_unchanged=True,physical_parking_verified=False,hardware_or_Isaac_motion_executed=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path)
    parser.add_argument('output',type=Path);parser.add_argument('--sequence',type=int,required=True)
    args=parser.parse_args();result=replay(args.episode,args.sequence)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print('NEW_ROUTE_FOUND',result['selected'] is not None,flush=True)
