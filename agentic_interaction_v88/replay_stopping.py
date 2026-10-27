"""Offline stopping replay of a CLOSED episode; never starts a simulator.

Uses measured command history and known robot kinematics, not object dynamics.
Float32 arm telemetry is an approximate reconstruction, not a bit-exact dump.
"""
import argparse,hashlib,inspect,json,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def replay(episode):
    p=Path(episode)
    if json.loads((p/'PROCESS_COMPLETION.json').read_text()).get('actor_exited') is not True:
        raise ValueError('CLOSED_RECORDED_EPISODE_ONLY')
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'scripts'))
    import xarm7_native_gripper as hand
    job=json.loads((p/'job.json').read_text());hand.OUT=Path(job['v8_robot_io'])
    from cross_robot_adapter import RobotPinkIK
    from .kinematics import CoupledKinematics
    from agentic_interaction_v87.command_governor import CommandGovernor
    from interaction_runtime.probing_profiles_v3 import speed_fraction
    c=hand.configuration();solver=RobotPinkIK(c)
    base=json.loads((Path(job['source'])/'report.json').read_text())['robot_base_pose']
    coupled=CoupledKinematics(solver,base,[0.,0.],np.vstack([np.tile([-1.,1.],(2,1)),solver.original_limits]),
        np.r_[.08,.08,np.asarray(c['arm_speed_limits'])/3])
    index=json.loads((p/'telemetry/index.json').read_text())
    chunks=[np.load(p/'telemetry'/d['file']) for d in index['chunks']]
    keys=['t','segment','command_q','command_dq','v88_base_command_m',
        'v88_base_command_velocity_m_s','v88_command_linear_speed_m_s']
    a={k:np.concatenate([d[k] for d in chunks]) for k in keys}
    segment=next(s for s in index['segments'] if s['id']==int(a['segment'][-1]))
    events=json.loads((p/'events.json').read_text())
    starts=[e['t'] for e in events if e['event']=='PRISMATIC_BREAKAWAY_CONFIRMED'
        and e['t']<=float(a['t'][-1])]
    if not starts:raise ValueError('RECORDED_PROFILE_START_REQUIRED')
    action=segment['declared_action'];out=[];dt=1/240
    original=json.loads((root/'configs/interaction_runtime_v1.json').read_text())
    for offset in [.3,.1,0.]:
        j=int(np.argmin(abs(a['t']-(a['t'][-1]-offset))))
        q=np.r_[a['v88_base_command_m'][j],a['command_q'][j]]
        previous=np.r_[a['v88_base_command_m'][j-1],a['command_q'][j-1]]
        v=np.r_[a['v88_base_command_velocity_m_s'][j],a['command_dq'][j]]
        P=coupled.fk(previous);T=coupled.fk(previous+v*dt)
        twist=np.r_[(T[:3,3]-P[:3,3])/dt,
            Rotation.from_matrix(T[:3,:3]@P[:3,:3].T).as_rotvec()/dt]
        motion=dict(original['phases']['LOADED'])
        motion['speed_m_s']=min(motion['speed_m_s'],action['reference_speed_m_s']*
            speed_fraction(float(a['t'][j])-starts[-1],action))
        motion['acceleration_m_s2']=min(motion['acceleration_m_s2'],action['reference_acceleration_m_s2'])
        gov=CommandGovernor(coupled.fk,dt,preserve_direction=True);gov.previous_twist=twist.copy()
        command=q.copy();velocity=v.copy();error=None;steps=0;start=time.perf_counter()
        peaks=np.zeros(4)
        try:
            for _ in range(int(5/dt)):
                if np.max(abs(velocity))<=1e-12:break
                velocity=gov.velocity(command,np.zeros(9),velocity,coupled.speeds,
                    np.r_[motion['acceleration_m_s2'],motion['acceleration_m_s2'],
                        np.full(7,original['arm_command_acceleration_rad_s2']['LOADED'])],motion)
                command+=velocity*dt;steps+=1
                peaks=np.maximum(peaks,[gov.latest[k] for k in ['linear_speed_m_s',
                    'angular_speed_rad_s','linear_acceleration_m_s2','angular_acceleration_rad_s2']])
        except RuntimeError as e:error=str(e)
        out.append(dict(t=float(a['t'][j]),motion_limits=motion,stop_steps=steps,error=error,
            max_remaining_velocity=float(np.max(abs(velocity))),wall_s=time.perf_counter()-start,
            maximum_command_norms=peaks.tolist(),
            reconstructed_linear_twist=float(np.linalg.norm(twist[:3])),
            recorded_linear_twist=float(a['v88_command_linear_speed_m_s'][j])))
    file=Path(inspect.getfile(CommandGovernor))
    return dict(schema='closed-command-stopping-replay/v1',episode=p.name,
        governor_source_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),cases=out,
        physical_motion_or_verified_exit_claimed=False,object_dynamics_GT_read=False,
        reconstruction='recorded float32 arm commands/velocities, float64 base commands; approximate, not bit-exact live state')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('episode',type=Path)
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    result=replay(args.episode);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
