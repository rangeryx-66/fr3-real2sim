"""Compact post-run engineering metrics. No actor, hidden physics or refitting."""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_probing_v3 import columns
from interaction_runtime.serialization import result_packet


def summarize(folder,output):
    from interaction_runtime.kinematics import FastNativeModel
    a,index=columns(folder);segments=[];job=json.loads((folder/'job.json').read_text());config=json.loads((folder/'EFFECTIVE_RUNTIME.json').read_text())
    model=FastNativeModel(None,Path(job['asset_root']),json.loads((Path(job['source'])/'report.json').read_text()))
    for s in index['segments']:
        ix=np.flatnonzero(a['segment']==s['id']);t=a['t'][ix];dt=np.diff(t)
        speed=np.linalg.norm(np.diff(a['tcp_pose'][ix,:3],axis=0),axis=1)/dt
        segments.append(dict(id=s['id'],role=s['role'],declared_action=s['declared_action'],result=s['result'],records=len(ix),
            contiguous_240Hz=bool(np.all(np.diff(a['step'][ix])==1)),
            pad_peak_n=np.max(a['pad_load'][ix],axis=0).tolist(),pad_mean_n=np.mean(a['pad_load'][ix],axis=0).tolist(),
            pad_zero_report_fraction=np.mean(a['pad_load'][ix]<=0,axis=0).tolist(),normal_absence_is_not_proven_separation=True,
            relative_translation_range_mm=(1000*np.quantile(a['relative_slip_m'][ix],[0,1])).tolist(),
            TCP_speed_max_mm_s=float(1000*np.max(speed)),raw_wrist_force_max_n=float(np.max(np.linalg.norm(a['raw_ft'][ix,:3],axis=1))),
            measured_aperture_range_mm=(1000*np.quantile(a['aperture'][ix],[0,1])).tolist(),
            command_aperture_range_mm=(1000*np.quantile(a['command_aperture'][ix],[0,1])).tolist(),
            minimum_measured_arm_joint_margin_rad=float(np.min(np.minimum(a['q'][ix][:,config['arm_indices']]-model.limits[:,0],model.limits[:,1]-a['q'][ix][:,config['arm_indices']]))),
            protection_samples=int(np.sum(a['safety_stop'][ix]))))
    recovery=json.loads((folder/'agent_responses/006.json').read_text())['result'];model=json.loads((folder/'model/estimate.json').read_text());runtime=json.loads((folder/'RUNTIME_RESULT.json').read_text())
    selected=json.loads((folder/'visual_plan.json').read_text());audit=json.loads((folder/'EXECUTION_CANDIDATE_AUDIT.json').read_text())
    observations=json.loads((folder/'AGENT_OBSERVATIONS.json').read_text());trace=json.loads((folder/'AGENT_ACTION_TRACE.json').read_text())
    r=dict(scope='V4.4 native-contact engineering regression; not new blind dynamics validation',segments=segments,
        recovery_acquisition=recovery.get('acquisition'),recovery_error=recovery.get('error'),safe_exit=recovery['safe_exit']['park'],
        grasp_verifications=[x for x in json.loads((folder/'events.json').read_text()) if x.get('event')=='GRASP_VERIFIED'],
        automatic_family=[selected[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']],
        planned_joint_margin_rad=selected['minimum_approach_arc_margin_rad'],execution_candidates=audit,
        observations=[{k:v for k,v in o.items() if k in ['index','t_sim','accepted_for_geometry','sensing_and_segmentation_wall_s']} for o in observations],
        agent_trace=[dict(sequence=x['command']['sequence'],operation=x['command']['operation'],reason=x['command'].get('reason'),wall_s=x['wall_s'],unloaded_wait_s=x['unloaded_wait_s']) for x in trace],
        native_safety=json.loads((folder/'NATIVE_GUARD_REGRESSION.json').read_text()),
        model=dict(estimator_version=model['estimator_version'],training_bins=model['training_bins'],validation_bins=model['validation_bins'],
            validation_excluded=model['complete_validation_excluded_from_fit_uncertainty_and_selection'],worker_wall_s=model['worker_wall_s'],
            heldout_torque_RMSE_Nm=model.get('constant_model',{}).get('heldout_torque_rmse_nm'),
            forward=model.get('constant_model',{}).get('heldout_motion'),physical_accuracy_claim=False),
        runtime_profile=runtime['profile'],evidence_issues=runtime['issues'],storage=json.loads((folder/'STORAGE_SUMMARY.json').read_text()),
        completion=json.loads((folder/'PROCESS_COMPLETION.json').read_text()))
    output.write_text(json.dumps(result_packet(r),indent=2));print(output)

if __name__=='__main__':summarize(*map(Path,sys.argv[1:3]))
