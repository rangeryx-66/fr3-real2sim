"""Closed-record FK replay. This does not simulate contact or claim new motion."""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
from .command_governor import CommandGovernor


def replay(episode,output,segment):
    episode=Path(episode);output=Path(output)
    closed=json.loads((episode/'PROCESS_COMPLETION.json').read_text())
    if not closed.get('actor_exited'):raise ValueError('CLOSED_SOURCE_EPISODE_REQUIRED')
    if output.exists():raise ValueError('PRESERVE_PREVIOUS_REPLAY')
    output.mkdir(parents=True)
    index=json.loads((episode/'telemetry/index.json').read_text())
    columns={k:[]for k in ['t','phase','segment','command_q','command_TCP_pose','tcp_pose']}
    for chunk in index['chunks']:
        p=episode/'telemetry'/chunk['file']
        if hashlib.sha256(p.read_bytes()).hexdigest()!=chunk['sha256']:raise ValueError('CORRUPTED_SOURCE_CHUNK')
        with np.load(p)as a:
            for k in columns:columns[k].append(a[k])
    a={k:np.concatenate(v)for k,v in columns.items()}
    rows=np.flatnonzero((a['segment']==segment)&(a['phase']==index['metadata']['phases'].index('OPEN_CONTINUE')))
    if len(rows)<3:raise ValueError('NO_MATCHING_TRAJECTORY')
    job=json.loads((episode/'job.json').read_text())
    runtime=json.loads((episode/'EFFECTIVE_RUNTIME.json').read_text())
    from cross_robot_adapter import RobotPinkIK
    from interaction_reference_handover import seed_pose
    config=json.loads((Path(job['v8_robot_io'])/'NATIVE_CONFIG.json').read_text())
    solver=RobotPinkIK(config);dt=runtime['physics_dt']
    # Constant rigid base transforms leave speed/acceleration norms invariant.
    fk=lambda q:seed_pose(solver,[0,0,0,0],q)
    governor=CommandGovernor(fk,dt)
    declaration=next(s['declared_action']for s in index['segments']if s['id']==segment)
    motion=dict(runtime['policy']['phases']['LOADED'])
    motion['speed_m_s']=min(motion['speed_m_s'],declaration['reference_speed_m_s'])
    motion['acceleration_m_s2']=min(motion['acceleration_m_s2'],declaration['reference_acceleration_m_s2'])
    q=np.array(a['command_q'][rows[0]],float);v=np.zeros_like(q)
    records=[];began=time.perf_counter();error=None
    for offset,row in enumerate(rows[1:]):
        try:
            wanted=(a['command_q'][row]-q)/dt
            v=governor.velocity(q,wanted,v,np.asarray(config['arm_speed_limits'])/3,
                runtime['policy']['arm_command_acceleration_rad_s2']['LOADED'],motion)
            q=q+v*dt
        except (RuntimeError,ValueError)as failure:
            error=dict(offset=offset,error=repr(failure));break
        records.append(dict(t=float(a['t'][row]),**governor.latest))
        if offset%1000==999:print('REPLAY_PROGRESS',offset+1,len(rows)-1,flush=True)
    braking=[];braking_error=None
    if error is not None:
        for step in range(int(5/dt)):
            if np.max(abs(v))<=1e-12:break
            try:
                v=governor.velocity(q,np.zeros_like(v),v,np.asarray(config['arm_speed_limits'])/3,
                    runtime['policy']['arm_command_acceleration_rad_s2']['LOADED'],motion)
                q=q+v*dt
                braking.append(dict(step=step,**governor.latest))
            except (RuntimeError,ValueError)as failure:
                braking_error=repr(failure);break
    old=a['command_TCP_pose'][rows,:3];actual=a['tcp_pose'][rows,:3]
    old_speed=np.linalg.norm(np.diff(old,axis=0),axis=1)/np.diff(a['t'][rows])
    actual_speed=np.linalg.norm(np.diff(actual,axis=0),axis=1)/np.diff(a['t'][rows])
    result=dict(source_episode=episode.name,source_segment=segment,source_rows=len(rows),
        scope='Counterfactual final-command FK only; recorded goals replayed, no new contact/physics/object motion',
        original_command_TCP_speed_max_m_s=float(old_speed.max()),
        original_measured_TCP_speed_max_m_s=float(actual_speed.max()),
        new_command_speed_max_m_s=max((x['linear_speed_m_s']for x in records),default=None),
        new_command_acceleration_max_m_s2=max((x['linear_acceleration_m_s2']for x in records),default=None),
        governed_frames=len(records),limited_frames=governor.limited,failure=error,
        braking=dict(steps=len(braking),error=braking_error,command_stopped=bool(np.max(abs(v))<=1e-12),
            speed_max_m_s=max((x['linear_speed_m_s']for x in braking),default=None),
            acceleration_max_m_s2=max((x['linear_acceleration_m_s2']for x in braking),default=None),
            physical_stop_confirmed=False),
        wall_s=time.perf_counter()-began,declared_limits=motion,
        source_module_sha256=hashlib.sha256((Path(__file__).parent/'command_governor.py').read_bytes()).hexdigest(),
        GT_read=False,hardware_or_Isaac_motion_executed=False)
    (output/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(output/'command_diagnostics.npz',**{k:np.array([r[k]for r in records])for k in records[0]}if records else {})
    print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--segment',type=int,default=4);args=p.parse_args()
    replay(args.episode,args.output,args.segment)
