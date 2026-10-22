"""Sealed, read-only engineering evaluation. Never updates the online model."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from summarize_prismatic_v75 import summarize
from diagnose_probing_v3 import columns,rows_at
from interaction_runtime.prismatic_unloading_v751 import causal_world_wrench


def run(episode,output):
    p=Path(episode);seal=json.loads((p/'ONLINE_SEAL_V751.json').read_text())
    if not seal['actor_exited']or not seal['online_actions_finished']:raise ValueError('ONLINE_SEAL_REQUIRED')
    for name,digest in seal['sha256'].items():
        if hashlib.sha256((p/name).read_bytes()).hexdigest()!=digest:raise ValueError('SEALED_EVIDENCE_CHANGED:'+name)
    result=summarize(p);a,index=columns(p)
    settings=json.loads((p/'identification_input.json').read_text());axis=np.asarray(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    model=json.loads((p/'model/estimate.json').read_text());selected=model.get('selected')or{}
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text())
    result['attempts']=[]
    for row in trace:
        if row['command']['operation']!='attempt_grasp_and_probe':continue
        r=row['result'];result['attempts'].append(dict(requested_preload_N=row['command'].get('native_g1_preload_request_n'),
            family=r.get('family'),physical_grasp_started=r.get('physical_grasp_started'),acquisition=r.get('acquisition'),
            segments=[{k:s.get(k)for k in ['status','reason','gain_native','joint_coordinate','elapsed_sim_s','physical_transmission_valid']}for s in r.get('segments',[])],
            safe_exit=(r.get('safe_exit')or{}).get('safe'),verified_park=(r.get('safe_exit')or{}).get('park',{}).get('reached'),
            error=r.get('error')))
    result.update(maximum_coordinate_mm=float(a['joint_q'].max()*1000),
        simulation_s=index['physics_steps_seen']/240,
        observations_all_generations=len(list((p/'rgbd').glob('generation_*/rgb.png'))),
        intentional_Agent_view_motions=sum(x['command']['operation']=='move_and_observe'and'operation_stopped'not in x.get('result',{})for x in trace),
        safety_stop_recorded=bool(np.any(a['safety_stop'])),
        raw_normal_peak_N=a['pad_load'].max(0).tolist(),
        frozen_online_selection=selected.get('intervals'),
        model_status=model.get('status'),FIT_bins=model.get('training_bins'),VALIDATION_bins=model.get('validation_bins'),
        heldout_unfiltered_input_sensitivity=selected.get('heldout_unfiltered_input_sensitivity'),
        uncertainty=selected.get('uncertainty'),FIT_trajectory_CV=selected.get('FIT_only_trajectory_CV'))
    result['segment_loads']=[]
    for s in index['segments']:
        ix=np.flatnonzero((a['step']>=s['first_step'])&(a['step']<=s['last_step']))
        rr=rows_at(a,ix,index['metadata'],'SEALED_CAUSAL_LOAD_DIAGNOSTIC')
        w=np.array([causal_world_wrench(r,settings['bias'])for r in rr]);rail=w[:,:3]@axis
        transverse=np.linalg.norm(w[:,:3]-rail[:,None]*axis,axis=1)
        moving=abs(a['joint_dq'][ix])>1e-4
        view=dict(role=s['role'],status=s['result'].get('status'),moving_samples=int(moving.sum()),
                  rail_RMS_N=float(np.sqrt(np.mean(rail[moving]**2)))if moving.any()else None,
                  moving_transverse_mean_N=float(transverse[moving].mean())if moving.any()else None,
                  maximum_causal_sensor_moment_Nm=float(np.linalg.norm(w[:,3:],axis=1).max()),
                  raw_pad_peak_N=a['pad_load'][ix].max(0).tolist(),
                  load_compensation='same causal bias + tool gravity; full dynamic compensation remains in the estimator')
        result['segment_loads'].append(view)
    events=json.loads((p/'events.json').read_text())
    unloading=[e for e in events if e.get('event')=='V751_TRANSVERSE_UNLOADING']
    result['unloading']=dict(updates=len(unloading),
        maximum_offset_mm=max((np.linalg.norm(e['offset_world_m'])*1000 for e in unloading),default=0.),
        maximum_abs_rail_correction_m=max((abs(e['rail_component_m'])for e in unloading),default=0.),
        normal_load_symmetry_is_not_a_gate=True)
    # Only now, after the immutable online seal, may the evaluator read truth.
    truth=json.loads((p/'HIDDEN_AUTHORED_PHYSICS.json').read_text())
    reference=dict(mass_kg=sum(x['mass_kg']for x in truth['bodies']),
                   damping_N_s_m=truth['configuration']['b'],friction_N=truth['configuration']['tau_c'],law=truth['law'])
    result['post_episode_authored_reference']=reference
    result['hidden_truth_read']=True
    result['hidden_truth_read_only_after_online_seal']=True
    result['independently_verified_effective_PhysX_parameters']=False
    result['parameter_relative_errors_percent']=None
    if selected:
        beta=selected['parameters'];ref=[reference['mass_kg'],reference['damping_N_s_m'],reference['friction_N']]
        result['parameter_relative_errors_percent']=[100*(x/y-1)for x,y in zip(beta,ref)]if selected['intervals']==0 else None
        result['mass_damping_relative_errors_percent']=[100*(x/y-1)for x,y in zip(beta[:2],ref[:2])]
        support=selected.get('support_intervals_native',[])
        if support:
            grid=np.concatenate([np.linspace(lo,hi,201)for lo,hi in support])
            if selected['intervals']==0:pred=np.full_like(grid,beta[2])
            else:
                from xarm_ft_v7_estimator import basis
                pred=basis(grid,selected['domain_native'],selected['intervals'])[:,selected['active_controls']]@np.asarray(beta[2:])
            law=reference['law'];actual=law['base']+law.get('amplitude',0.)*np.exp(-.5*((grid-law.get('center',0.))/law.get('width',1.))**2)
            result['resistance_error_observed_support']=dict(rmse_N=float(np.sqrt(np.mean((pred-actual)**2))),maximum_abs_N=float(abs(pred-actual).max()),
                evaluation='uniform grid within each frozen FIT-supported interval only; post-seal diagnostic, no refit',full_stroke_or_unseen_peak_validated=False)
    for name,digest in seal['sha256'].items():assert hashlib.sha256((p/name).read_bytes()).hexdigest()==digest
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=run(a.episode,a.output);print(json.dumps({k:r[k]for k in ['episode','status','maximum_coordinate_mm','verified_park','parameter_relative_errors_percent','unloading','total_bytes']}))
