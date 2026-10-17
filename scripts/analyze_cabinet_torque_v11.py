"""Post-run private evidence and independent truth evaluation, never runtime input."""
import argparse,json
from pathlib import Path
import numpy as np
from summarize_hybrid_episode import rows_at,main as summarize


def main(root):
    root=Path(root);ep=root/'episode';pid=json.loads((root/'PROCESS.json').read_text())['pid']
    if Path('/proc/%d'%pid).exists() or not (ep/'EXPORT_COMPLETE.json').exists():raise RuntimeError('Actor must exit/export before posthoc evaluation')
    summarize(root,root/'PRIVATE_EPISODE_SUMMARY.json');rows=list(rows_at(ep))
    from evaluate_large_range_torque_v1 import evaluate
    # Reuse independent geometry/mass evaluator but compare three, not five,
    # coefficients. The versioned estimator never sees these authored values.
    import inspect
    import evaluate_large_range_torque_v1 as evaluator
    source=inspect.getsource(evaluate).replace('gs,gc])','gs,gc])[:3]').replace("'Coulomb_Nm','gravity_sin_Nm','gravity_cos_Nm'","'Coulomb_Nm'")
    ns=dict(evaluator.__dict__);exec(compile(source,__file__+'::posthoc_only','exec'),ns);truth=ns['evaluate'](root)
    summary=json.loads((root/'PRIVATE_EPISODE_SUMMARY.json').read_text());updates=[]
    for path in sorted((ep/'hybrid_identification').glob('*/TORQUE_COMPONENTS.json')):updates.append(json.loads(path.read_text()))
    summary['effective_parameter_updates']=updates;summary['posthoc_truth_access']=True
    summary['recovery_acquisitions']=max(0,summary['handle_acquisitions']-1)
    summary['verified_safe_park']=bool(summary['attempts'] and summary['attempts'][-1].get('safe_exit',{}).get('safe_park_reached'))
    summary['segment_endpoint_opening_travel_deg']=sum(max(0,s['result'].get('end_deg',0)-s['result'].get('start_deg',0)) for s in json.loads((ep/'HYBRID_SEGMENTS.json').read_text()) if s.get('result')) if (ep/'HYBRID_SEGMENTS.json').exists() else 0.
    summary['segment_endpoint_closing_travel_deg']=sum(max(0,s['result'].get('start_deg',0)-s['result'].get('end_deg',0)) for s in json.loads((ep/'HYBRID_SEGMENTS.json').read_text()) if s.get('result')) if (ep/'HYBRID_SEGMENTS.json').exists() else 0.
    (root/'PRIVATE_EPISODE_SUMMARY.json').write_text(json.dumps(summary,indent=2))
    out=root/'private_analysis';out.mkdir(exist_ok=True)
    if rows:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        t=np.array([r['t'] for r in rows]);angle=np.array([r['door_angle_deg'] for r in rows]);ap=np.array([r['aperture_m'] for r in rows])*1000
        fig,axes=plt.subplots(5,1,figsize=(14,13),sharex=True)
        axes[0].plot(t,angle);axes[0].set_ylabel('actual door deg')
        for k in ['gripper_link1','gripper_link2']:axes[1].plot(t,[r['forces_n'].get(k,0) for r in rows],lw=.5,label=k)
        axes[1].legend();axes[1].set_ylabel('raw normal N')
        axes[2].plot(t,ap,label='measured');axes[2].plot(t,[r.get('telemetry',{}).get('command_aperture_m',np.nan)*1000 for r in rows],lw=.5,label='command');axes[2].legend();axes[2].set_ylabel('aperture mm')
        axes[3].plot(t,[1000*r.get('relative_translation_slip_m',0) for r in rows]);axes[3].set_ylabel('relative drift mm')
        speed=np.r_[0,np.linalg.norm(np.diff(np.array([r['T_tcp'] for r in rows])[:,:3,3],axis=0),axis=1)/np.diff(t)*1000]
        axes[4].plot(t,speed,lw=.5);axes[4].axhline(5,color='r',ls='--',label='loaded limit; free motion differs');axes[4].legend();axes[4].set_ylabel('measured TCP mm/s');axes[4].set_xlabel('simulation s')
        for ax in axes:ax.grid(alpha=.25)
        fig.tight_layout();fig.savefig(out/'EPISODE_CONTACT_MOTION.png',dpi=140);plt.close(fig)
    # Plot only already partitioned/compensated segment arrays; never smooth
    # across fit/validation boundaries just to make a prettier figure.
    blocks=[]
    for file in sorted((ep/'hybrid_identification').glob('*/TORQUE_BLOCK.npz')):
        with np.load(file,allow_pickle=True) as z:blocks.append({k:z[k] for k in z.files})
    if blocks:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(4,1,figsize=(14,11),sharex=True)
        wrench_fig,wrench_axes=plt.subplots(3,2,figsize=(14,10),sharex=True)
        for block in blocks:
            axes[0].plot(block['t'],np.rad2deg(block['q']),lw=.6)
            axes[1].plot(block['t'],block['torque'],lw=.5)
            axes[2].plot(block['t'],block['qd'],lw=.5)
            axes[3].plot(block['t'],block['qdd'],lw=.5)
            for j,ax in enumerate(wrench_axes.flat):
                ax.plot(block['t'],block['raw'][:,j],lw=.4,label='raw SENSOR')
                ax.plot(block['t'],block['compensated_world'][:,j],lw=.4,label='tool-compensated WORLD')
        for ax,label in zip(axes,['actual angle deg','wrist-derived hinge Nm','qd rad/s','qdd rad/s2']):ax.set_ylabel(label);ax.grid(alpha=.25)
        axes[-1].set_xlabel('simulation s');fig.tight_layout();fig.savefig(out/'SEGMENT_EFFORT_MOTION.png',dpi=140);plt.close(fig)
        for j,ax in enumerate(wrench_axes.flat):ax.set_ylabel(['Fx N','Fy N','Fz N','Tx Nm','Ty Nm','Tz Nm'][j]);ax.grid(alpha=.25)
        wrench_fig.suptitle('Native-rate raw sensor versus compensated world wrench; different frames explicitly labeled')
        wrench_fig.tight_layout();wrench_fig.savefig(out/'WRIST_WRENCHES.png',dpi=140);plt.close(wrench_fig)
    events=json.loads((ep/'events.json').read_text())
    verified=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
    opening=next((e for e in events if e.get('event')=='SHORT_OPENING_START'),None)
    extra=dict(completed_closing_segment=False,parser_failure_after_safe_park=summary['verified_safe_park'],nominal_parent_3deg_fields_are_legacy=True)
    if verified and opening:
        hold=[r for r in rows if verified['t']<=r['t']<=opening['t']]
        def state(r):return dict(t=r['t'],angle_deg=r['door_angle_deg'],aperture_mm=r['aperture_m']*1000,command_aperture_mm=r['telemetry']['command_aperture_m']*1000,raw_normal_N=r['forces_n'],filtered_normal_N=r['telemetry']['pad_load_filtered_n'])
        first_zero=next((r for r in hold if r['forces_n']['gripper_link1']==0),None)
        extra.update(hold_duration_s=opening['t']-verified['t'],hold_start=state(hold[0]),hold_end=state(hold[-1]),left_first_zero_normal_after_verification_s=None if first_zero is None else first_zero['t']-verified['t'],zero_normal_is_not_proven_separation=True)
    extra['handover_events']=[e for e in events if e.get('event')=='REFERENCE_HANDOVER']
    extra['LOCAL_CENTERING_handover_physically_exercised']=any('LOCAL_CENTERING' in e.get('reason','') for e in extra['handover_events'])
    extra['maximum_reported_relative_slip_proxy_m']=max((r.get('relative_translation_slip_m',0.) for r in rows),default=0.)
    (root/'PRIVATE_CONTACT_AND_HANDOVER_DIAGNOSTICS.json').write_text(json.dumps(extra,indent=2))
    completed_closing=any(v.get('result',{}).get('ok') and v.get('result',{}).get('end_deg',0)<v.get('result',{}).get('start_deg',0) for v in json.loads((ep/'HYBRID_SEGMENTS.json').read_text())) if (ep/'HYBRID_SEGMENTS.json').exists() else False
    models=[]
    for i,m in enumerate(updates):
        formal=m.get('identifiability',{}).get('supported',[False]*3);supported=list(formal);fit=m.get('fit',{})
        if not completed_closing:supported[1:]=[False,False] # failed tiny reversal is not completed bidirectional validation; report-only
        models.append(dict(update=i,model='I*qdd+b*qd+C*sign(qd)',parameters={n:(fit.get('parameters',[None]*3)[j] if supported[j] else None) for j,n in enumerate(['I_eff','b','Coulomb'])},
            conditional_fit_not_identified=fit,provisional_positive_direction_resistance_Nm=fit.get('parameters',[None,None,None])[2],not_bidirectional_friction_validation=True,formal_linear_support=formal,completed_closing_validation=completed_closing,observability=supported,valid_range=m.get('coverage'),sensor_provenance=str(ep/'WRIST_FT_MAPPING.json'),gravity='vertical hinge; negligible projected torque',raw_evidence_private=True))
    (root/'ESTIMATED_PHYSICS_MODELS.json').write_text(json.dumps(models or [dict(parameters=dict(I_eff=None,b=None,Coulomb=None),status='no informative contact data',observability=[False]*3)],indent=2))
    print(json.dumps({k:summary[k] for k in ['maximum_actual_angle_deg','segment_endpoint_closing_travel_deg','handle_acquisitions','verified_safe_park','native_records','simulation_s','wall_s']}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);main(p.parse_args().root)
