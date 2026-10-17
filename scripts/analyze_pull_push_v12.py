"""Post-export V1.2 evaluation only. Never imported by the physics owner.

Legacy SHORT_OPENING_SUCCESS is reported separately from the V1.2 objective.
No estimator is refit, no truth is opened until the actor has exited/exported.
"""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import numpy as np
from summarize_hybrid_episode import main as summarize, rows_at


def main(root):
    root=Path(root);ep=root/'episode';pid=json.loads((root/'PROCESS.json').read_text())['pid']
    if Path('/proc/%d'%pid).exists() or not (ep/'EXPORT_COMPLETE.json').exists():
        raise RuntimeError('Post-export evaluator only; keep hidden dynamics sealed')
    summarize(root,root/'PRIVATE_EPISODE_SUMMARY.json')
    summary=json.loads((root/'PRIVATE_EPISODE_SUMMARY.json').read_text())
    rows=list(rows_at(ep));events=json.loads((ep/'events.json').read_text())
    push=summary['panel_push'];attempts=summary['attempts']
    segments=json.loads((ep/'HYBRID_SEGMENTS.json').read_text())
    t=np.array([r['t'] for r in rows]);T=np.array([r['T_tcp'] for r in rows])
    speeds=np.r_[0.,np.linalg.norm(np.diff(T[:,:3,3],axis=0),axis=1)/np.diff(t)*1000]
    def maximum(mask):return float(max(speeds[np.asarray(mask,dtype=bool)],default=0))
    actual_contacts=[any(c['force_n']>0 for c in r.get('contacts',[])) for r in rows]
    protected_panel=[r.get('panel_wrist_guard') is not None for r in rows]
    protocol=[]
    for p in sorted((ep/'agent_commands').glob('[0-9]*.json')):
        index=int(p.stem);accepted=ep/'agent_responses'/('%03d.accepted.json'%index)
        completed=ep/'agent_responses'/('%03d.json'%index)
        command=json.loads(p.read_text());ack=json.loads(accepted.read_text()) if accepted.exists() else {}
        result=json.loads(completed.read_text()) if completed.exists() else {}
        protocol.append(dict(sequence=index,operation=command['operation'],accepted=ack.get('status'),
            completed=result.get('status'),same_payload_hash=ack.get('command_sha256')==hashlib.sha256(p.read_bytes()).hexdigest()))
    models=[]
    for p in sorted((ep/'hybrid_identification').glob('*/TORQUE_COMPONENTS.json')):
        value=json.loads(p.read_text());models.append(dict(segment=p.parent.name,result=value))
    # Reuse the independent authored-inertia evaluator, restricted to the same
    # three coefficients as the frozen estimator. It also records truth access.
    import evaluate_large_range_torque_v1 as evaluator
    source=inspect.getsource(evaluator.evaluate)
    source=source.replace('gs,gc])','gs,gc])[:3]').replace("'Coulomb_Nm','gravity_sin_Nm','gravity_cos_Nm'","'Coulomb_Nm'")
    namespace=dict(evaluator.__dict__);exec(compile(source,__file__+'::posthoc_three_components','exec'),namespace)
    truth=namespace['evaluate'](root)
    verified=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
    opening=next((e for e in events if e.get('event')=='SHORT_OPENING_START'),None)
    hold=[]
    if verified and opening:hold=[r for r in rows if verified['t']<=r['t']<=opening['t']]
    def state(r):return dict(t=r['t'],actual_angle_deg=r['door_angle_deg'],aperture_mm=r['aperture_m']*1000,
        command_aperture_mm=r['telemetry']['command_aperture_m']*1000,normal_loads=r['forces_n'],
        filtered_normal_loads=r['telemetry']['pad_load_filtered_n'])
    source_hashes=json.loads((root/'FROZEN.json').read_text())['source_sha256']
    repo=Path(__file__).resolve().parents[1]
    mismatch=[n for n,h in source_hashes.items() if hashlib.sha256((repo/n).read_bytes()).hexdigest()!=h]
    raw_endpoint_difference=push.get('started_angle_deg',summary['maximum_actual_angle_deg'])-push.get('final_angle_deg',summary['final_actual_angle_deg'])
    panel_rows=[r for r in rows if r.get('phase')=='PANEL_PUSH']
    closing=max(0.,panel_rows[0]['door_angle_deg']-panel_rows[-1]['door_angle_deg']) if panel_rows else 0.
    result=dict(schema='pull-push-v12-posthoc',primary_bidirectional_objective=False,
        primary_reason='No panel contact or actual closing before original TCP speed stop',
        initial_pull_success=bool(attempts and attempts[0]['controller_target_reached']),
        legacy_short_success_not_V12_pass=summary['hybrid_result'].get('success'),
        max_actual_angle_deg=summary['maximum_actual_angle_deg'],final_actual_angle_deg=summary['final_actual_angle_deg'],
        actual_panel_closing_deg=closing,raw_result_endpoint_difference_deg=raw_endpoint_difference,
        closing_note='Native angle samples, not a subtraction mixing float64-rad and float32-degree representations.',
        physical_handle_acquisitions=summary['handle_acquisitions'],
        intentional_handle_release_completions=summary['handle_release_completions'],
        panel_attempts=len([s for s in segments if s['label']=='PANEL_PUSH']),
        positive_panel_contact_samples=summary['panel_contact_samples'],planned_regrasps=0,
        final_safe_park=bool(push.get('separation',{}).get('safe_park_reached')),
        final_safe_exit=push.get('separation'),final_joint_q=rows[-1]['q'],final_loads=rows[-1]['forces_n'],
        raw_force_events=summary['force_events'],peak_raw_finger_normal_n=summary['peak_raw_pad_n'],
        minimum_joint_margin_rad=summary['minimum_joint_margin_rad'],
        maximum_TCP_all_phase_mm_s=maximum([True]*len(rows)),maximum_TCP_positive_contact_mm_s=maximum(actual_contacts),
        maximum_TCP_protected_panel_mm_s=maximum(protected_panel),
        first_positive_normal_contacts=summary['first_positive_normal_contacts'],
        acquisition=verified,one_second_hold=dict(duration_s=opening['t']-verified['t'],start=state(hold[0]),end=state(hold[-1])) if hold else None,
        handover_events=[e for e in events if e.get('event')=='REFERENCE_HANDOVER'],
        segment_records=segments,parameter_updates=models,authored_truth_posthoc=truth,
        model_boundary='Only opening data support local derivatives; no completed held-out closing segment. I/b and bidirectional friction remain unvalidated.',
        protocol_acknowledgments=protocol,protocol_payload_hash_mismatches=[p['sequence'] for p in protocol if not p['same_payload_hash']],
        frozen_source_mismatches=mismatch,simulation_s=summary['simulation_s'],wall_s=summary['wall_s'],
        native_records=summary['native_records'],wrist_ft_valid=summary['wrist_ft_valid'],
        physics_step_dt_range_s=summary['measured_step_dt_range_s'],private_raw_evidence=True)
    (root/'PRIVATE_V12_RESULT.json').write_text(json.dumps(result,indent=2))
    estimated=[]
    for entry in models:
        model=entry['result'];fit=model.get('fit',{})
        estimated.append(dict(segment=entry['segment'],parameters=dict(I_eff=None,b=None,Coulomb=None),
            status=model.get('status','positive-opening conditional fit only'),
            provisional_positive_direction_resistance_Nm=fit.get('parameters',[None,None,None])[2],
            conditional_fit_not_identified=fit,completed_closing_validation=False,
            sensor_provenance='ideal fixed-interface wrist F/T; raw and compensated private streams',
            valid_motion_coverage=model.get('coverage'),observability=[False,False,False]))
    (root/'ESTIMATED_PHYSICS_MODELS.json').write_text(json.dumps(estimated,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True)
    axs[0].plot(t,[r['door_angle_deg'] for r in rows],lw=.7);axs[0].set_ylabel('actual angle deg')
    for n in ['gripper_link1','gripper_link2']:axs[1].plot(t,[r['forces_n'][n] for r in rows],lw=.4,label=n)
    axs[1].set_ylabel('native normal N');axs[1].legend()
    axs[2].plot(t,[r['aperture_m']*1000 for r in rows],label='measured');axs[2].plot(t,[r['telemetry']['command_aperture_m']*1000 for r in rows],label='command',lw=.5);axs[2].legend();axs[2].set_ylabel('aperture mm')
    axs[3].plot(t,speeds,lw=.4);axs[3].axhline(5,c='r',ls='--',label='original protected-mode limit');axs[3].legend();axs[3].set_ylabel('TCP mm/s');axs[3].set_xlabel('simulation s')
    for ax in axs:ax.grid(alpha=.2)
    fig.suptitle('V1.2: initial pull completed; panel entry speed stop; no closing\nAll native-rate samples plotted; pre-contact entry separately zoomed in private diagnostic')
    fig.tight_layout();fig.savefig(root/'PRIVATE_EPISODE_OVERVIEW.png',dpi=140);plt.close(fig)
    print(json.dumps({k:result[k] for k in ['primary_bidirectional_objective','max_actual_angle_deg','actual_panel_closing_deg','final_safe_park','native_records','wrist_ft_valid','simulation_s','wall_s','frozen_source_mismatches']}))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);main(p.parse_args().root)
