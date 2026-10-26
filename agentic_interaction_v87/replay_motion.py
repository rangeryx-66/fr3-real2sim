"""Read-only reproduction of a CLOSED episode's monitor/stop-proof failure.

Original q/TCP/F/T evidence and outcome are never overwritten. Counterfactual
classification cannot certify a new physical release, parking or full target.
"""
import argparse,hashlib,json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
from interaction_runtime.articulation_v7 import Articulation
from interaction_runtime.progress import LOOKAHEAD_RAD
from agentic_interaction_v81.motion_dwell import MotionDwell
from agentic_interaction_v84.monitor import factory
from .motion_policy import PhysicalMotionDwell
from .constraint_feedback import POLICY


def matrix(pose):
    T=np.eye(4);T[:3,3]=pose[:3];T[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix()
    return T


def replay(episode):
    p=Path(episode);completion=json.loads((p/'PROCESS_COMPLETION.json').read_text())
    if completion.get('actor_exited') is not True:raise ValueError('CLOSED_EPISODE_REQUIRED')
    index=json.loads((p/'telemetry/index.json').read_text())
    effective=json.loads((p/'EFFECTIVE_RUNTIME.json').read_text())
    prior=json.loads((p/'identification_input.json').read_text())['prior']
    job=json.loads((p/'job.json').read_text())
    articulation=Articulation(prior['joint_type'],tuple(prior['axis_world']),
        tuple(prior['origin_world_m']),tuple(job['articulation_v7']['limits_native']))
    keys=['t','q','tcp_pose','joint_q','phase','segment','reference_q','pad_load','safety_stop']
    columns={k:[] for k in keys};commitments=[];missing_diagnostics=[]
    for chunk in index['chunks']:
        f=p/'telemetry'/chunk['file'];digest=hashlib.sha256(f.read_bytes()).hexdigest()
        if digest!=chunk['sha256']:raise ValueError('CORRUPTED_SOURCE_TELEMETRY')
        commitments.append(dict(file=chunk['file'],sha256=digest))
        with np.load(f)as a:
            for k in keys:columns[k].append(a[k])
            if 'v87_command_governor_valid' not in a.files:missing_diagnostics.append(chunk['file'])
    a={k:np.concatenate(v)for k,v in columns.items()}
    segment=index['segments'][0];action=segment['declared_action'];dt=effective['physics_dt']
    begin=segment['first_step']*dt;phases=index['metadata']['phases']
    rows=np.flatnonzero((a['segment']==segment['id']) &
        (a['phase']==phases.index('OPEN_CONTINUE')))
    pose=[matrix(a['tcp_pose'][rows[0]])];r=NS(job={'v87_constraint_policy':POLICY},tcp=lambda:pose[0])
    # No per-frame wall timestamps exist: do NOT invent resource forecasts.
    captures=[];monitor=factory(r,articulation,captures)(float(a['joint_q'][rows[0]]),1,action)
    failures=[]
    from interaction_runtime.probing_profiles_v3 import speed_fraction
    for i in rows:
        pose[0]=matrix(a['tcp_pose'][i]);t=float(a['t'][i])-begin
        scale=articulation.path_scale(pose[0][:3,3])
        state=dict(requested_rate_rad_s=min(action['reference_speed_m_s']/scale,.08)*speed_fraction(t,action),
            lag_rad=float(a['reference_q'][i]-a['joint_q'][i]))
        failure=monitor.update(t,float(a['joint_q'][i]),state,dt,
            action['target_q']-float(a['joint_q'][i]))
        if failure:failures.append(failure)
    last=index['segments'][0]['result'];stop_start=begin+last['elapsed_sim_s']
    hold_rows=np.flatnonzero(a['t']>=stop_start-dt)
    old=MotionDwell(dt);new=PhysicalMotionDwell(dt,lambda:articulation);arm=effective['arm_indices']
    old_ready=[];new_ready=[];speeds=[]
    for i in hold_rows:
        T=matrix(a['tcp_pose'][i]);q=a['q'][i,arm];coordinate=float(a['joint_q'][i]);t=float(a['t'][i])
        before=old.update(t,q,T,coordinate);after=new.update(t,q,T,coordinate)
        old_ready.append(before['ready']);new_ready.append(after['ready'])
        if after['object_path_speed_m_s'] is not None:speeds.append(after['object_path_speed_m_s'])
    return dict(scope='Same recorded motion classifications only; no new physical command, collision sweep, release or parking claimed',
        source_episode=p.name,source_commit=json.loads((p/'V81_PRELAUNCH.json').read_text()).get('source_commit'),
        source_chunks=commitments,source_outcome=completion['physical_outcome'],
        original_segment_status=last['status'],original_segment_reason=last['reason'],
        actual_maximum_deg=float(np.degrees(a['joint_q'].max())),
        original_low_speed_suspect_windows=[w for w in last['abc_feedback']['windows'] if w['suspect']],
        new_transmission_failure_windows=failures,
        reconstruction_scope='recorded reference/coordinate window timing; original nominal state inferred from saved reference. Resource forecast intentionally not replayed.',
        stop_proof=dict(rows=len(hold_rows),original_any_ready=any(old_ready),new_any_ready=any(new_ready),
            new_last_ready=new_ready[-1],object_path_speed_max_m_s=max(speeds),
            threshold_source='existing stationary TCP linear/angular cells, through known articulation; scalar native-rate cutoff removed for this release proof'),
        diagnostic_columns_missing_in_initial_chunks=missing_diagnostics,
        raw_FT_joint_TCP_preserved=True,new_physics_started=False,old_terminal_record_unchanged=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();result=replay(a.episode)
    with a.output.open('x')as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:result[k]for k in ['actual_maximum_deg','stop_proof','diagnostic_columns_missing_in_initial_chunks']}))
    print('NEW_TRANSMISSION_FAILURE_WINDOWS',len(result['new_transmission_failure_windows']))
