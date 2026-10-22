"""Read-only command/measurement/cooked-collider diagnosis after actor exit."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def run(episode,job_path,output):
    import fcl
    from interaction_runtime.kinematics import FastNativeModel
    from interaction_runtime.prismatic_planning_v73 import native_precision_ik_class
    from xarm7_native_execution import mapped_export
    from interactive_twin_recovery.mobile import physical_scene_class
    from revolute10_fixed import authored_handle_colliders
    from diagnose_probing_v3 import columns
    from update_probing_v2_model import matrix
    p=Path(episode);job=json.loads(Path(job_path).read_text());cfg=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text())
    src=json.loads((Path(job['source'])/'report.json').read_text());base=src['robot_base_pose']
    model=FastNativeModel(None,Path(job['asset_root']),src);solver=native_precision_ik_class()(cfg['gripper_config'])
    export=json.loads((p/'planning_current_scene.json').read_text());allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection'])
    scene=physical_scene_class(ROOT)(mapped_export(export),model,allowed);a,index=columns(p)
    arm=np.asarray(cfg['arm_indices']);fingers=np.asarray(cfg['finger_indices']);segment=index['segments'][-1]
    ix=np.flatnonzero((a['step']>=segment['first_step'])&(a['step']<=segment.get('last_step',a['step'][-1])))
    frames=[]
    for i in ix[np.unique(np.linspace(0,len(ix)-1,min(14,len(ix))).astype(int))]:
        q=a['q'][i,arm];finger=a['q'][i,fingers];command=a['command_q'][i];M=matrix(a['moving_link_pose'][i]);poses=model.poses(q,base,finger_q=finger)
        measured=scene.check(poses,M,True);command_check=scene.check(model.poses(command,base,finger_q=finger),M,True)
        nearest=[]
        for r in scene.robot:
            if r.body not in ('gripper_link1','gripper_link2'):continue
            for s in scene.scene:
                if s.path in allowed:continue
                d=float(fcl.distance(r.object,s.object))
                nearest.append(dict(finger=r.body,path=s.path,body=s.body,distance_m=d))
        nearest=sorted(nearest,key=lambda v:v['distance_m'])[:3]
        frames.append(dict(step=int(a['step'][i]),t=float(a['t'][i]),drawer_m=float(a['joint_q'][i]),
            measured_check=measured,command_check=command_check,nearest_command_geometry=nearest,
            finger_q=finger.tolist(),command_minus_measured_arm_rad=(command-q).tolist(),
            command_velocity_rad_s=a.get('command_dq',a.get('command_qd',np.zeros((len(a['t']),7))))[i].tolist()))
    # Re-run the nominal successor prediction with the actual post-pull fingers.
    first=ix[0];T=matrix(a['tcp_pose'][first]);M=matrix(a['moving_link_pose'][first]);q=a['q'][first,arm];finger=a['q'][first,fingers]
    start=float(a['joint_q'][first]);target=float(segment['action']['target_q'])if 'action'in segment else None
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text())
    actions=[s for e in trace for s in e.get('command',{}).get('segments',[])if s['role']==segment['role']]
    if actions:target=actions[-1]['target_q']
    prior=json.loads((p/'identification_input.json').read_text())['prior'];axis=np.asarray(prior['axis_world']);axis/=np.linalg.norm(axis)
    replay=[]
    if target is not None:
        bias=a['command_q'][first]-q
        for value in np.linspace(start,target,30):
            D=np.eye(4);D[:3,3]=axis*(value-start);seed=solver.solve(D@T,base,q,starts=1)
            if seed is None:replay.append(dict(value_m=float(value),valid=False,reason='IK'));break
            ok,why=scene.check(model.poses(seed+bias,base,finger_q=finger),D@M,True)
            replay.append(dict(value_m=float(value),valid=bool(ok),reason=why));q=seed
    result=dict(kind='offline native safety geometry only; no executable GT grasp correction',
        physics_started=False,hidden_dynamics_read=False,role=segment['role'],recorded_result=segment.get('result'),
        frames=frames,constant_first_state_prediction=replay,allowed_handle_collider_count=len(allowed),
        native_moving_link=scene.moving_link)
    Path(output).parent.mkdir(parents=True,exist_ok=True);Path(output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(role=segment['role'],last=frames[-1],nominal_replay_rejected=any(not x['valid']for x in replay))))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.episode,a.job,a.output)
