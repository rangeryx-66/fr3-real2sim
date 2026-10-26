"""Closed telemetry load/speed counterfactual. Never predicts new physical motion."""
import argparse, hashlib, json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
from interaction_runtime.policy import Policy, ForceMonitor
from .constraint_feedback import POLICY, ActionForceMonitor


def replay(episode):
    episode=Path(episode)
    if not json.loads((episode/'PROCESS_COMPLETION.json').read_text()).get('actor_exited'):
        raise ValueError('CLOSED_EPISODE_REQUIRED')
    index=json.loads((episode/'telemetry/index.json').read_text())
    effective=json.loads((episode/'EFFECTIVE_RUNTIME.json').read_text())
    job=json.loads((episode/'job.json').read_text())
    chunk=index['chunks'][-1];path=episode/'telemetry'/chunk['file']
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    if digest!=chunk['sha256']:raise ValueError('CORRUPT_SOURCE_CHUNK')
    policy=Policy(effective['policy'],effective['policy_sha256'])
    c=effective['gripper_config'];arm=effective['arm_indices']
    r=NS(job={'v87_constraint_policy':POLICY},policy=policy,stopped=False,
        rows=[{'wrist_ft':{'valid':True,'raw_wrench_sensor':[0.]*6}}],g={'phase':'OPEN_CONTINUE'},
        qtarget=np.zeros(len(effective['native_joint_order'])),arm=np.asarray(arm),
        actual=lambda:0.,_decelerating=False)
    r.robot=NS(get_joint_positions=lambda:r.q,get_joint_velocities=lambda:r.dq)
    r.filtered=lambda:r.pad
    limits=np.array([[float(x['limit']['lower']),float(x['limit']['upper'])]
                     for x in c['joint_parameters']])
    r.model=NS(margin=lambda q:float(np.min(np.minimum(q-limits[:,0],limits[:,1]-q))))
    r.supervisor=NS(latched=None)
    old=ForceMonitor(policy)
    new=ActionForceMonitor(policy,r);r.supervisor.force=new
    first_old=None;first_new=None;scales=[];count=0
    props=index['metadata']['body_properties']
    with np.load(path) as a:
        stop_t=float(a['t'][-1]);rows=np.flatnonzero(a['t']>=stop_t-2.)
        for i in rows:
            T=np.eye(4);T[:3,3]=a['sensor_pose'][i,:3]
            T[:3,:3]=Rotation.from_quat(a['sensor_pose'][i,3:]).as_matrix()
            raw=a['raw_ft'][i]-np.asarray(job['ft_calibration']['bias_sensor'])
            wrench=np.r_[T[:3,:3]@raw[:3],T[:3,:3]@raw[3:]]
            for b,pose in zip(props,a['tool_poses'][i]):
                com=Rotation.from_quat(pose[3:]).apply(b['com_body_m'])+pose[:3]
                force=np.array([0.,0.,-9.81*b['mass_kg']])
                wrench+=np.r_[force,np.cross(com-T[:3,3],force)]
            r.q=a['q'][i];r.dq=a['dq'][i];r.qtarget=r.q.copy()
            r.qtarget[r.arm]=a['command_q'][i];r.pad=a['pad_load'][i]
            r.rows[-1]['wrist_ft']['raw_wrench_sensor']=a['raw_ft'][i]
            r.rows[-1]['wrist_ft']['valid']=bool(a['ft_valid'][i])
            pads=dict(enumerate(a['pad_load'][i].tolist()));t=float(a['t'][i]);dt=effective['physics_dt']
            before=old.update(pads,dt,t,wrench)
            after=new.update(pads,dt,t,wrench)
            scales.append(new.feedback['scale']);count+=1
            if before['stop'] and first_old is None:
                first_old=dict(t=t,status=before['status'],angle_deg=float(np.degrees(a['joint_q'][i])))
            if after['status']=='CONTINUOUS_LOAD_REGULATION' and first_new is None:
                first_new=dict(t=t,status=after['status'],authority='CONTINUOUS_REGULATION',
                    trailing_torque_nm=new.feedback['trailing_torque_nm'],
                    trailing_pad_n=new.feedback['trailing_pad_n'])
    return dict(scope='Closed measured wrench/pad classification only. No new motion, no collision/IK replay, no counterfactual contact success.',
        source_episode=episode.name,source_chunk_sha256=digest,policy_sha256=policy.sha256,
        source_rows=count,original_first_stop=first_old,new_first_continuous_regulation=first_new,
        minimum_reference_scale=min(scales),new_requests_hard_stop=bool(after['stop']),
        old_episode_latch_untouched=True,physical_stop_or_unload_verified=False,
        no_physics_started=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();result=replay(a.episode)
    with a.output.open('x')as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result))
