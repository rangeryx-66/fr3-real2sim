"""Compact post-exit metrics/figure only; no refitting or simulation."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def report(root):
    root=Path(root);episode=root/'episode_00';r=json.loads((root/'RESULT.json').read_text())
    assert r['completion']['actor_exited'] and r['episode_result']['status']=='SESSION_COMPLETE'
    assert r['online_model_preserved'] and r['all_numeric_chunks_intact']
    out=root/'public_derived';out.mkdir(exist_ok=False)
    from update_probing_v31_model import load_segments
    from diagnose_probing_v3 import columns
    pieces,train,test,quality=load_segments(episode,allow_interrupted_fit=True);a,index=columns(episode)
    model=r['online_model']['constant_model'];beta=np.array(model['parameters']);truth=np.array(r['authored_reference'])
    validation=[s for s in r['actual']['segments'] if s['role'].startswith('VALIDATION')]
    assert validation and all(s['result']['status']=='TARGET_REACHED' and s['continuous_240Hz'] for s in validation)
    assert r['online_model']['complete_validation_excluded_from_fit_uncertainty_and_selection']
    assert r['prevalidation_frozen_model']['validation_bins']==0 and r['prevalidation_frozen_model']['parameters']==model['parameters']
    segments=[]
    for s in r['actual']['segments']:
        segments.append(dict(id=s['id'],role=s['role'],status=s['result']['status'],requested_target_deg=index['segments'][s['id']-1]['target_deg'],
            start_deg=s['start_deg'],end_deg=s['end_deg'],gain_deg=s['result']['gain_deg'],duration_s=s['duration_s'],records=s['records'],
            continuous_240Hz=s['continuous_240Hz'],max_TCP_mm_s=s['max_TCP_mm_s'],minimum_joint_margin_rad=s['minimum_joint_margin_rad'],
            max_handle_relative_slip_mm=s['max_handle_relative_slip_mm'],pad_peaks_n=s['pad_peaks_n'],normal_report_below_0p05N_fraction=s['positive_normal_unreported_fraction_at_0p05N']))
    comparison={name:{k:c.get(k) for k in ['parameters','conditional_std','absolute_error','relative_error_percent','common_V31_heldout_torque_rmse_nm','common_V31_heldout_motion','full_segment_boundary_sensitivity']} for name,c in r['comparison'].items()}
    plan=json.loads((episode/'visual_plan.json').read_text());checks=json.loads((episode/'EXECUTION_CANDIDATE_AUDIT.json').read_text())
    metrics=dict(experiment='V4.5',reference='bcf5dda',registration='b76148b',assessment=dict(status='PASS_LIMITED_BLIND_VALIDATION',
        basis='Complete fresh physical FIT/reverse FIT/independent validation; interpretable full-rank estimates with small post-exit authored errors and free held-out prediction',
        limitations=['One development cabinet and one new physical configuration, 0–5.57deg only','Authored inertia is not independently measured effective PhysX inertia','Conditional statistical uncertainty misses damping/friction systematic bias','Legacy V3 slightly lower short-angle RMSE; no uniform superiority claim','Ideal virtual F/T, approximate actuators, registered station and known hinge/state']),
        authored_reference_I_b_C=truth.tolist(),authored_configuration=r['authored_configuration'],segments=segments,
        opening_travel_deg=sum(max(0,s['gain_deg']) for s in segments),closing_travel_deg=-sum(min(0,s['gain_deg']) for s in segments),
        comparison=comparison,online_model=dict(parameters=beta.tolist(),conditional_std=model['conditional_std'],supported=model['supported'],information=model['information'],
            training_bins=r['online_model']['training_bins'],validation_bins=r['online_model']['validation_bins'],version=r['online_model']['estimator_version'],
            validation_excluded=True,unchanged_after_validation=True,tool_compensation_count=r['online_model']['tool_compensation_count'],worker_wall_s=r['online_model']['worker_wall_s'],
            posthoc_error_over_conditional_std=((beta-truth)/np.array(model['conditional_std'])).tolist(),total_uncertainty_calibration='not established'),
        quality=quality,excitation=r['excitation'],first_grasp=True,physical_acquisitions=r['actual']['physical_handle_acquisitions'],recovery_acquisitions=0,
        automatic_family=[plan[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']],execution_candidate_checks=checks,
        safe_park=r['episode_result']['trace'][6]['result']['safe_exit']['park'],hard_stop_samples=r['actual']['hard_stop_samples'],load_warning_samples=r['actual']['load_warning_samples'],
        Agent_trace=r['actual']['operations'],completion=r['completion'],runtime_profile=r['runtime']['profile'],storage=r['storage'],evidence_issues=r['runtime']['issues'],
        prevalidation_snapshot=r['prevalidation_frozen_model'],truth_access=json.loads((root/'TRUTH_ACCESS.json').read_text()),
        online_model_sha256=r['online_model_sha256'],numeric_integrity=True,raw_published=False)
    atomic_json(out/'METRICS.json',result_packet(metrics))
    # Publish the exact now-revealed small configuration, never raw measurements.
    blob=(root/'HIDDEN_CONFIGURATIONS.json').read_bytes();commit=json.loads((root/'CONFIGURATION_COMMITMENT.json').read_text())
    assert hashlib.sha256(blob).hexdigest()==commit['configuration_sha256']
    (out/'REVEALED_CONFIGURATIONS.json').write_bytes(blob)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(12,9),constrained_layout=True);t0=pieces[0]['t'][0]
    colors=['#2478b4','#e38b2c','#27945c']
    for d,color in zip(pieces,colors):
        t=d['t']-t0;label=str(d['segments'][0]);good=d['valid'];tau=np.c_[d['qdd'],d['qd'],d['direction']]@beta
        axes[0,0].plot(t,np.rad2deg(d['q']),c=color,label=label)
        axes[0,1].plot(t,d['native_qd'],c=color,lw=.35,alpha=.35);axes[0,1].plot(t,d['qd'],c=color,label=label)
        axes[1,0].plot(t[good],d['qdd'][good],c=color,label=label)
        axes[1,1].plot(t[good],d['torque'][good]*1000,c=color,label=label);axes[1,1].plot(t[good],tau[good]*1000,c='black',ls='--',lw=.8)
        if label.startswith('VALIDATION'):
            axes[2,0].plot(t[good],(d['torque'][good]-tau[good])*1000,c=color,label='held-out residual')
    positions=np.arange(3);offset=.15
    for name,shift,color in [('V31_online',-offset,'#2478b4'),('V3_offline_same_intervals',offset,'#d46e45')]:
        c=r['comparison'][name];axes[2,1].bar(positions+shift,c['relative_error_percent'],width=.28,label=name,color=color)
    axes[2,1].set_xticks(positions,['I','b','C']);axes[2,1].axhline(0,c='gray',lw=.7)
    labels=[('Actual door angle (deg)','interaction time (s)'),('Measured velocity (rad/s)','interaction time (s)'),('Common-filtered acceleration (rad/s²)','interaction time (s)'),('Wrist-derived / predicted torque (mN m)','interaction time (s)'),('Validation torque residual (mN m)','interaction time (s)'),('Post-exit authored relative error (%)','parameter')]
    for ax,(y,x) in zip(axes.flat,labels):ax.set(ylabel=y,xlabel=x);ax.grid(alpha=.2);ax.legend(fontsize=6)
    fig.suptitle('V4.5 — fresh blind physical episode; FIT-only V3.1 and independent validation',fontsize=12);fig.savefig(out/'probing_validation.png',dpi=145);plt.close(fig)
    print(out)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);report(p.parse_args().root)
