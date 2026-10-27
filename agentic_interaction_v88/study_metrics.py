"""Closed-trial metrics/curves from native telemetry; no plant/GT IO."""
import argparse,json,sys
from pathlib import Path
import numpy as np


def collect(episode,output):
    p=Path(episode);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    if json.loads((p/'PROCESS_COMPLETION.json').read_text()).get('actor_exited') is not True:
        raise ValueError('CLOSED_EXPERIMENT_REQUIRED')
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'scripts'))
    from diagnose_probing_v3 import columns,rows_at
    from xarm_wrist_ft import wrench_at
    from xarm_ft_v31_estimator import common_filter
    from agentic_interaction_v81.contact_diagnosis import analyze
    contact=analyze(p)
    a,index=columns(p);settings=json.loads((p/'identification_input.json').read_text())
    arm=index['metadata']['v83_load_telemetry']['arm_indices']
    phase=index['metadata']['phases'];curves=[];segments=[]
    for s in index['segments']:
        ix=np.flatnonzero((a['segment']==s['id'])&a['ft_valid'].astype(bool))
        if not len(ix):continue
        rows=rows_at(a,ix,index['metadata'],s['role']);w=[]
        for row in rows:
            f=row['wrist_ft'];T=f['T_world_sensor']
            w.append(wrench_at(f['raw_wrench_sensor']-settings['bias'],T,T[:3,3])+
                f['tool_gravity_wrench_world'])
        w=np.asarray(w);t=a['t'][ix];q=a['joint_q'][ix];v=a['joint_dq'][ix]
        moving=np.isin(a['phase'][ix],[phase.index(n) for n in ['OPEN_CONTINUE','CONTACT_PRELOAD'] if n in phase])
        tt=t[moving];qq=q[moving];vv=v[moving];kinematics=None
        if len(tt)>247 and np.max(abs(np.diff(tt)-1/240))<1e-7:
            acceleration=common_filter(np.gradient(vv,1/240));jerk=np.gradient(acceleration,1/240)
            inside=slice(121,-121)
            reference=a['reference_q'][ix][moving]
            kinematics=dict(actual_speed_mean_m_s=float(np.mean(abs(vv))),
                actual_speed_p95_m_s=float(np.percentile(abs(vv),95)),
                reference_velocity_RMSE_m_s=float(np.sqrt(np.mean((np.gradient(reference,1/240)-vv)**2))),
                acceleration_RMS_m_s2=float(np.sqrt(np.mean(acceleration[inside]**2))),
                acceleration_abs95_m_s2=float(np.percentile(abs(acceleration[inside]),95)),
                jerk_RMS_m_s3=float(np.sqrt(np.mean(jerk[inside]**2))),
                filter='unchanged SG121-equivalent common filter; segment-local 121-edge exclusion',
                smoother_operation_advantage_claimed=False,
                comparison_requires_actual_matched_speed_or_time=True)
        base=a['v88_base_measured_xyzyaw'][ix,:3];tcp=a['tcp_pose'][ix,:3]
        base_delta=base[-1]-base[0];tcp_delta=tcp[-1]-tcp[0]
        ratio=float(base_delta@tcp_delta/(tcp_delta@tcp_delta)) if tcp_delta@tcp_delta>1e-16 else None
        startup=s.get('result',{}).get('prismatic_startup',{})
        slip=any('KINEMATIC_TANGENTIAL_SLIP_EVIDENCE' in str(v) for v in startup.get('observed_mechanisms',[]))
        contacts=np.any(np.linalg.norm(a['normal_force_on_fingers_world_N'][ix],axis=2)>0,axis=1)
        record=dict(id=s['id'],role=s['role'],status=s.get('result',{}).get('status'),
            reason=s.get('result',{}).get('reason'),actual_start_m=float(q[0]),actual_end_m=float(q[-1]),
            actual_min_m=float(q.min()),actual_max_m=float(q.max()),native_operation_s=float(t[-1]-t[0]),
            measured_pad_normal_median_n=np.median(a['pad_load'][ix],axis=0).tolist(),
            measured_pad_normal_peak_n=np.max(a['pad_load'][ix],axis=0).tolist(),
            wrist_gravity_comp_force_peak_n=float(np.linalg.norm(w[:,:3],axis=1).max()),
            wrist_gravity_comp_torque_peak_nm=float(np.linalg.norm(w[:,3:],axis=1).max()),
            wrist_gravity_comp_component_std=np.std(w,axis=0).tolist(),
            wrist_scope='at measured sensor origin, world axes, bias/gravity compensated; tool inertia retained as in live guard',
            relative_TCP_handle_translation_peak_m=float(np.max(a['relative_slip_m'][ix])),
            relative_translation_is_not_finger_slip_distance=True,
            measured_kinematic_sliding_evidence=slip,
            native_positive_contact_fraction=float(contacts.mean()),
            complete_contact_separation_observed=False if contacts.all() else None,
            normal_absence_alone_not_a_grasp_loss=True,
            base_net_m=base_delta.tolist(),TCP_net_m=tcp_delta.tolist(),
            arm_cartesian_compensation_net_m=(tcp_delta-base_delta).tolist(),
            base_projected_net_share=ratio,
            arm_compensation_max_rad=float(abs(a['q'][ix][:,arm]-a['q'][ix[0],arm]).max()),
            TCP_command_tracking_peak_m=float(np.linalg.norm(tcp-a['command_TCP_pose'][ix,:3],axis=1).max()),
            kinematics=kinematics,hard_stop_samples=int(a['safety_stop'][ix].sum()))
        segments.append(record)
        curves.append(dict(id=s['id'],t=t,q=q,v=v,w=w,pads=a['pad_load'][ix],
            relative=a['relative_slip_m'][ix],base=base,comp=tcp-base))
    terminal=json.loads((p/'TERMINAL_SAFETY.json').read_text())
    owner=json.loads((p.parent.parent/'LAUNCH_OWNER.json').read_text())
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text())
    waits=json.loads((p/'ABC_WAITS.json').read_text()).get('waits',[])
    diagnostic=0.
    for event in (p/'safety_events').glob('hard_stop_*.json'):
        finishes=[f.stat().st_mtime for f in (p/'agent_commands').glob('*.json')
            if json.loads(f.read_text()).get('operation')=='finish' and f.stat().st_mtime>=event.stat().st_mtime]
        if finishes:diagnostic+=min(finishes)-event.stat().st_mtime
    result=dict(schema='physics-aware-closed-trial-metrics/v1',episode=p.name,
        source_sha=owner['source_commit'],segments=segments,
        maximum_actual_coordinate_m=float(a['joint_q'].max()),
        minimum_actual_coordinate_m=float(a['joint_q'].min()),
        physical_acquisitions=sum(bool(e.get('result',{}).get('physical_grasp_started')) for e in trace),
        high_level_commands=len(trace),Agent_wait_wall_s=sum(float(w.get('wall_s',0)) for w in waits),
        separately_recorded_latched_diagnostic_hold_wall_s=diagnostic,
        total_wall_s=float((p/'PROCESS_COMPLETION.json').stat().st_mtime-owner['start_unix_s']),
        timing_scope='owned launcher start to original process-completion file timestamp; includes initialization/observation/planning/operation/exit',
        runtime_profile=json.loads((p/'RUNTIME_RESULT.json').read_text())['profile'],
        terminal_safety=terminal,hidden_object_dynamics_used=False,
        material_contact_diagnosis=contact,
        physics_identification=json.loads((p/'model/estimate.json').read_text()) if (p/'model/estimate.json').exists() else None)
    (out/'METRICS.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,1,figsize=(10,11),sharex=True)
    for c in curves:
        label='segment '+str(c['id']);axes[0].plot(c['t'],1000*c['q'],label=label)
        axes[1].plot(c['t'],1000*c['v']);axes[2].plot(c['t'],np.linalg.norm(c['w'][:,:3],axis=1))
        axes[3].plot(c['t'],c['pads'][:,0]);axes[3].plot(c['t'],c['pads'][:,1],ls='--')
        axes[4].plot(c['t'],1000*c['relative'])
    for ax,label in zip(axes,['Drawer mm','Actual mm/s','Wrist force N','Pad reports N','TCP/handle drift mm']):ax.set_ylabel(label);ax.grid(alpha=.2)
    axes[0].legend();axes[-1].set_xlabel('Native simulation time s')
    fig.suptitle(p.name+' — measured motion/contact; no cross-method smoothness claim')
    fig.tight_layout();fig.savefig(out/'MOTION_LOAD_CURVES.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for c in curves:
        for j,name in enumerate(('x','y','z')):
            axes[0].plot(c['t'],c['w'][:,j],label=f"s{c['id']} F{name}")
            axes[1].plot(c['t'],c['w'][:,j+3],label=f"s{c['id']} T{name}")
    for ax,label in zip(axes,['Wrist force N','Wrist torque Nm']):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=6,ncol=3)
    axes[-1].set_xlabel('Native simulation time s')
    fig.suptitle(p.name+' — six-axis wrist, measured sensor origin / world axes')
    fig.tight_layout();fig.savefig(out/'WRIST_SIX_AXIS.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for c in curves:
        for j,name in enumerate(('x','y','z')):
            axes[0].plot(c['t'],1000*(c['base'][:,j]-c['base'][0,j]),label=f"s{c['id']} {name}")
            axes[1].plot(c['t'],1000*(c['comp'][:,j]-c['comp'][0,j]),label=f"s{c['id']} {name}")
    for ax,label in zip(axes,['Base displacement mm','Arm Cartesian compensation mm']):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=6,ncol=3)
    axes[-1].set_xlabel('Native simulation time s')
    fig.suptitle(p.name+' — actual simultaneous base / arm motion')
    fig.tight_layout();fig.savefig(out/'COORDINATION_CURVES.png',dpi=140);plt.close(fig)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('episode',type=Path)
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    r=collect(args.episode,args.output);print(json.dumps(dict(episode=r['episode'],segments=len(r['segments']),
        maximum_m=r['maximum_actual_coordinate_m'],output=str(args.output))))
