"""Closed native controller comparison. Never reads plant/hidden GT files."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def analyze(episode, output):
    p, out = Path(episode), Path(output)
    complete = json.loads((p/'PROCESS_COMPLETION.json').read_text())
    if complete.get('actor_exited') is not True:
        raise ValueError('CLOSED_EPISODE_REQUIRED')
    from diagnose_probing_v3 import columns, rows_at
    from xarm_wrist_ft import wrench_at
    from xarm_ft_v31_estimator import common_filter
    from interaction_runtime.serialization import result_packet
    a, index = columns(p)
    job = json.loads((p/'job.json').read_text())
    if job.get('articulation_v7',{}).get('joint_type') != 'prismatic':
        raise ValueError('PRISMATIC_METRIC_ANALYSIS_ONLY; native radians must not be labelled metres')
    config = json.loads((p/'EFFECTIVE_RUNTIME.json').read_text())
    arm = config['arm_indices']
    base = [config['native_joint_order'].index(n) for n in ('mobile_x','mobile_y')]
    phase = index['metadata']['phases']
    t, tcp = a['t'], a['tcp_pose'][:,:3]
    adjacent = (np.diff(a['step'])==1) & (abs(np.diff(t)-1/240)<1e-7)
    tcp_speed = np.linalg.norm(np.diff(tcp,axis=0),axis=1)/np.diff(t)
    events = json.loads((p/'events.json').read_text())
    trace = json.loads((p/'AGENT_ACTION_TRACE.json').read_text()) if (p/'AGENT_ACTION_TRACE.json').exists() else []
    waits = json.loads((p/'ABC_WAITS.json').read_text()).get('waits',[]) if (p/'ABC_WAITS.json').exists() else []
    profile = json.loads((p/'RUNTIME_RESULT.json').read_text())['profile']
    summary = dict(episode=p.name, source_commit=json.loads((p/'PROCESS.json').read_text()).get('source_commit'),
        wall_s=complete['wall_s'], native_record_span_s=float(t[-1]-t[0]), physics_s=profile['simulation_s'],
        wall_per_sim=complete['wall_s']/profile['simulation_s'],
        maximum_actual_m=float(a['joint_q'].max()), final_actual_m=float(a['joint_q'][-1]),
        recorded_stop_marker_samples=int(a['safety_stop'].sum()),
        measured_TCP_max_m_s=float(tcp_speed[adjacent].max()) if adjacent.any() else None,
        actual_base_net_m=(a['q'][-1,base]-a['q'][0,base]).tolist(),
        actual_arm_net_rad=(a['q'][-1,arm]-a['q'][0,arm]).tolist(),
        physical_grasps=sum(bool(e.get('result',{}).get('physical_grasp_started')) for e in trace),
        high_level_commands=len(trace), high_level_count_is_not_LLM_calls=True,
        Agent_wait_wall_s=sum(float(w.get('wall_s',0.)) for w in waits),
        candidate_rejections=[e for e in events if e.get('event')=='V89_CANDIDATE_REJECTED'],
        stops=[e for e in events if e.get('event') in ('V89_STOP_CONFIRMED','V89_STOP_PENDING')],
        terminal=json.loads((p/'TERMINAL_SAFETY.json').read_text()), profile=profile,
        raw_stop_marker_is_not_hard_classification=True,
        no_hidden_dynamics_read=True, segments=[])
    summary['wrench_scope']='diagnostic world wrench at wrist origin, original bias and tool-gravity compensation; dynamic tool inertia remains in these curves, original estimator applies its full correction separately'
    wrench_full=np.full((len(t),6),np.nan)
    summary['verified_grasps_from_events']=sum(e.get('event')=='GRASP_VERIFIED' for e in events)
    summary['physical_grasps_from_events'] = sum(e.get('event')=='PHYSICAL_GRASP_START' for e in events)
    summary['regrasp_attempts'] = max(0, summary['physical_grasps_from_events']-1)
    summary['phase_measurements'] = []
    for k, label in enumerate(phase):
        ids = np.flatnonzero((a['phase']==k) & a['ft_valid'].astype(bool))
        if not len(ids): continue
        wrench = []
        for row in rows_at(a, ids, index['metadata'], 'DIAGNOSTIC_NOT_FIT'):
            ft=row['wrist_ft']; T=ft['T_world_sensor']
            wrench.append(wrench_at(ft['raw_wrench_sensor']-job['ft_calibration']['bias_sensor'], T, T[:3,3])+ft['tool_gravity_wrench_world'])
        wrench=np.asarray(wrench);wrench_full[ids]=wrench
        normals=np.linalg.norm(a['normal_force_on_fingers_world_N'][ids],axis=2)
        db=a['q'][ids[-1],base]-a['q'][ids[0],base]
        dx=tcp[ids[-1]]-tcp[ids[0]]
        measurement=dict(phase=label, records=len(ids), native_record_span_s=float(t[ids[-1]]-t[ids[0]]),
            pad_normal_peak_N=normals.max(axis=0).tolist(), pad_normal_median_N=np.median(normals,axis=0).tolist(),
            finger_handle_resultant_peak_N=a['pad_load'][ids].max(axis=0).tolist(),
            wrist_compensated_force_peak_N=float(np.linalg.norm(wrench[:,:3],axis=1).max()),
            wrist_compensated_torque_peak_Nm=float(np.linalg.norm(wrench[:,3:],axis=1).max()),
            TCP_nominal_command_tracking_peak_m=float(np.linalg.norm(tcp[ids]-a['command_TCP_pose'][ids,:3],axis=1).max()),
            actual_base_net_m=db.tolist(), actual_TCP_net_m=dx.tolist(), arm_TCP_compensation_net_m=(dx-np.r_[db,0.]).tolist(),
            measured_object_min_m=float(a['joint_q'][ids].min()), measured_object_max_m=float(a['joint_q'][ids].max()),
            diagnostic_aggregation_is_not_FIT=True, repeated_phase_span_includes_gaps=True)
        summary['phase_measurements'].append(measurement)
    if 'v89_command_position' in a:
        changed=np.r_[True,np.max(abs(np.diff(a['v89_command_position'],axis=0)),axis=1)>0]
        running=a['v89_stop_state']==0
        mask=changed & running
        summary['accepted_command_timing_observed']=dict(samples=int(mask.sum()),
            solve_total_s=float(a['v89_solve_s'][mask].sum()),
            stop_check_total_s=float(a['v89_stop_check_s'][mask].sum()),
            stop_check_p95_s=float(np.quantile(a['v89_stop_check_s'][mask],.95)) if mask.any() else None,
            scope='unique changed RUNNING command samples; initial/zero commands and brake ticks excluded; see inclusive collision profile')
        summary['TCP_design_ratio_peaks']=np.nanmax(a['v89_tcp_design_ratios'],axis=0).tolist() if 'v89_tcp_design_ratios' in a else None
    curves=[]
    for s in index['segments']:
        ids=np.flatnonzero((a['segment']==s['id']) & a['ft_valid'].astype(bool))
        if not len(ids):continue
        rows=rows_at(a,ids,index['metadata'],s['role'])
        w=[]
        for row in rows:
            ft=row['wrist_ft'];T=ft['T_world_sensor']
            w.append(wrench_at(ft['raw_wrench_sensor']-job['ft_calibration']['bias_sensor'],T,T[:3,3])+ft['tool_gravity_wrench_world'])
        w=np.asarray(w);q=a['joint_q'][ids];v=a['joint_dq'][ids]
        normals=np.linalg.norm(a['normal_force_on_fingers_world_N'][ids],axis=2)
        dbase=a['q'][ids[-1],base]-a['q'][ids[0],base]
        dtcp=tcp[ids[-1]]-tcp[ids[0]]
        result=s.get('result',{})
        m=dict(id=s['id'],role=s['role'],status=result.get('status'),reason=result.get('reason'),
            requested_target_m=s.get('target_q'),target_reached=result.get('requested_target_reached',False),
            start_m=float(q[0]),end_m=float(q[-1]),min_m=float(q.min()),max_m=float(q.max()),
            measured_reverse_m=float(max(0.,q[0]-q[-1])),
            native_duration_s=float(t[ids[-1]]-t[ids[0]]),
            native_pad_normal_median_N=np.median(normals,axis=0).tolist(),
            native_pad_normal_peak_N=normals.max(axis=0).tolist(),
            finger_handle_resultant_peak_N=a['pad_load'][ids].max(axis=0).tolist(),
            wrist_compensated_force_peak_N=float(np.linalg.norm(w[:,:3],axis=1).max()),
            wrist_compensated_torque_peak_Nm=float(np.linalg.norm(w[:,3:],axis=1).max()),
            wrench_component_std=np.std(w,axis=0).tolist(),
            wrench_scope='world axes at wrist origin; original bias/gravity compensation, dynamic tool loading retained',
            TCP_handle_relative_translation_peak_m=float(a['relative_slip_m'][ids].max()),
            TCP_relative_drift_is_not_material_slip=True,
            base_net_m=dbase.tolist(),TCP_net_m=dtcp.tolist(),
            arm_cartesian_compensation_net_m=(dtcp-np.r_[dbase,0.]).tolist(),
            arm_compensation_peak_rad=float(abs(a['q'][ids][:,arm]-a['q'][ids[0],arm]).max()),
            TCP_command_tracking_peak_m=float(np.linalg.norm(tcp[ids]-a['command_TCP_pose'][ids,:3],axis=1).max()),
            recorded_stop_marker_samples=int(a['safety_stop'][ids].sum()))
        moving=np.isin(a['phase'][ids],[phase.index(n) for n in ('OPEN_CONTINUE','CONTACT_PRELOAD') if n in phase])
        ii=ids[moving]
        if len(ii)>247 and np.all(np.diff(a['step'][ii])==1):
            acc=common_filter(np.gradient(a['joint_dq'][ii],1/240));jerk=np.gradient(acc,1/240)
            m['motion_kinematics']=dict(speed_mean_m_s=float(np.mean(abs(a['joint_dq'][ii]))),
                speed_p95_m_s=float(np.quantile(abs(a['joint_dq'][ii]),.95)),
                acceleration_rms_m_s2=float(np.sqrt(np.mean(acc[121:-121]**2))),
                jerk_rms_m_s3=float(np.sqrt(np.mean(jerk[121:-121]**2))),
                processing='segment-local original SG121-equivalent common filter,121sample edge exclusion',
                scope='selected object rail DOF, not TCP or motor jerk',
                matched_speed_or_time_advantage_claimed=False)
        summary['segments'].append(m)
        curves.append(dict(ids=ids,w=w,q=q,v=v,normals=normals,role=s['role']))
    model=p/'model/estimate.json'
    summary['Physics_ID']=json.loads(model.read_text()) if model.exists() else None
    try:
        from agentic_interaction_v81.contact_diagnosis import analyze as diagnose
        summary['native_material_contact_diagnosis']=diagnose(p)
    except (FileNotFoundError,KeyError,ValueError) as error:
        summary['native_material_contact_diagnosis']=dict(available=False,reason=str(error))
    out.mkdir(parents=True,exist_ok=True)
    (out/'METRICS.json').write_text(json.dumps(result_packet(summary),indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,1,figsize=(10,11),sharex=True)
    for c in curves:
        ii=c['ids'];tt=t[ii]
        axes[0].plot(tt,1000*c['q'],label=c['role']);axes[1].plot(tt,1000*c['v'])
        axes[2].plot(tt,np.linalg.norm(c['w'][:,:3],axis=1));axes[2].plot(tt,np.linalg.norm(c['w'][:,3:],axis=1),ls='--')
        axes[3].plot(tt,c['normals'][:,0]);axes[3].plot(tt,c['normals'][:,1],ls='--')
        axes[4].plot(tt,1000*a['relative_slip_m'][ii])
    if not curves:
        axes[0].plot(t,1000*a['joint_q']);axes[1].plot(t,np.linalg.norm(a['dq'][:,arm],axis=1))
        axes[2].plot(t,np.linalg.norm(wrench_full[:,:3],axis=1));axes[2].plot(t,np.linalg.norm(wrench_full[:,3:],axis=1),ls='--')
        normals_all=np.linalg.norm(a['normal_force_on_fingers_world_N'],axis=2)
        axes[3].plot(t,normals_all)
        axes[4].text(.5,.5,'No B segment: no qualified motion/slip comparison',
            transform=axes[4].transAxes,ha='center',va='center')
    velocity_label='Rail m/s' if curves else 'Arm projected dq norm rad/s'
    for ax,label in zip(axes,['Drawer mm',velocity_label,'Wrist N / Nm','Pad normal N','TCP/handle drift mm']):ax.set_ylabel(label);ax.grid(alpha=.2)
    if curves:axes[0].legend(fontsize=7)
    axes[-1].set_xlabel('Recorded native simulation time s');fig.suptitle(p.name+' — measured, no GT')
    fig.tight_layout();fig.savefig(out/'MOTION_LOAD_CURVES.png',dpi=130);plt.close(fig)
    fig,axes=plt.subplots(6,1,figsize=(10,10),sharex=True)
    for j,ax in enumerate(axes):
        ax.plot(t,wrench_full[:,j],lw=.6)
        ax.set_ylabel(('F' if j<3 else 'T')+'XYZ'[j%3]+(' N' if j<3 else ' Nm'));ax.grid(alpha=.2)
    axes[0].set_title(p.name+' — measured world wrist6D\nBias/gravity compensated; dynamic tool inertia retained',fontsize=10)
    axes[-1].set_xlabel('Native simulation time s');fig.tight_layout();fig.savefig(out/'WRIST_6D_CURVES.png',dpi=130);plt.close(fig)
    manifest={str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest() for f in p.rglob('*') if f.is_file()}
    (out/'RAW_SEAL.json').write_text(json.dumps(dict(episode=p.name,files=manifest,actor_exited=True),indent=2)+'\n')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--episode',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    sys.path[:0]=[str(args.source),str(args.source/'scripts')]
    r=analyze(args.episode,args.output)
    print(json.dumps({k:r[k] for k in ('episode','wall_s','physics_s','maximum_actual_m','final_actual_m','recorded_stop_marker_samples')},indent=2))
