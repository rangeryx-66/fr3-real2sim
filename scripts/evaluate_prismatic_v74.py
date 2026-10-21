"""Independent post-batch evaluation. Never fits or alters an online model.

All eight online seals/exits precede every hidden-value read. No raw data export.
"""
import argparse,datetime,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from prismatic_registration_v74 import read,sha
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet

def storage_categories(folder):
    counts={}
    for path in folder.rglob('*'):
        if not path.is_file():continue
        top=path.relative_to(folder).parts[0]
        category=('motion_FT' if top=='telemetry' else 'native_contact' if 'contact' in top.lower()
            else 'checkpoint_RGBD' if top=='rgbd' else 'model' if top=='model'
            else 'geometry_and_plans' if top in ('agent_geometry','incremental_feedback','planning')
            else 'protocol' if top.startswith('agent_') else 'metadata_logs_other')
        counts[category]=counts.get(category,0)+path.stat().st_size
    return counts

def heldout_metrics(piece,model):
    """Add reporting denominators to the unchanged production prediction."""
    from xarm_ft_v7_estimator import prediction
    result=prediction(piece,model)
    if result.get('status')!='predicted':return result
    edge=piece['processing']['boundary_exclusion_samples']
    ix=np.arange(edge,len(piece['t'])-edge)
    effort=float(np.sqrt(np.mean(piece['torque'][ix]**2)))
    span=float(np.ptp(piece['q'][ix]))
    velocity=float(np.sqrt(np.mean(piece['native_qd'][ix]**2)))
    result.update(measured_effort_RMS_N=effort,measured_displacement_m=span,
        measured_velocity_RMS_m_s=velocity,
        normalized_effort_RMSE=result['effort_rmse']/effort if effort else None,
        normalized_position_RMSE=result['coordinate_rmse']/span if span else None,
        normalized_velocity_RMSE=result['velocity_rmse']/velocity if velocity else None,
        full_piece_duration_s=float(piece['t'][-1]-piece['t'][0]),
        retained_prediction_horizon_s=result['duration_s'],
        physical_completion_is_separately_reported=True)
    return result

def evaluate(batch,registration,private):
    batch=Path(batch);reg=read(registration)
    if len(reg['episodes'])!=8:raise ValueError('All eight original slots required')
    for e in reg['episodes']:
        p=batch/'episodes'/e['episode_id']
        if not (p/'ONLINE_SEAL.json').exists() or not read(p/'PROCESS_COMPLETION.json')['actor_exited']:
            raise ValueError('Batch still online or unexecuted:'+e['episode_id'])
        seal=read(p/'ONLINE_SEAL.json')
        if any(not (p/f).exists() or sha(p/f)!=h for f,h in seal['files'].items()):raise ValueError('Online output changed after seal')
        pid=read(p/'PROCESS.json')['pid']
        if Path('/proc') .joinpath(str(pid)).exists():raise ValueError('Owned actor still alive')
    if sha(private)!=reg['hidden_configuration_sha256']:raise ValueError('Hidden commitment changed')
    # First access to parameter values occurs below, after the full-batch guard.
    access=batch/'TRUTH_ACCESS.json'
    reveal_UTC=(read(access)['accessed_UTC'] if access.exists() else datetime.datetime.now(datetime.timezone.utc).isoformat())
    atomic_json(access,dict(accessed_UTC=reveal_UTC,all_eight_online_seals_verified=True,
        hidden_truth_used_for_online_decisions=False,evaluation_complete=False))
    cases=read(private)
    from evaluate_cross_object_v7 import authored_inertial_reference
    from diagnose_probing_v3 import columns
    from update_probing_v7_model import load_segments
    from update_prismatic_v73_model import qualify
    from xarm_ft_v7_estimator import basis,samples,prediction
    rows=[]
    for e in reg['episodes']:
        p=batch/'episodes'/e['episode_id'];case=cases[e['configuration_index']];trace=read(p/'AGENT_ACTION_TRACE.json') if (p/'AGENT_ACTION_TRACE.json').exists() else []
        # Registered launchers keep jobs in the batch, not inside each episode.
        job_path=batch/'jobs'/(e['episode_id']+'.json');job=read(job_path)
        if sha(job_path)!=read(p/'PROCESS.json')['job_sha256']:raise ValueError('Executed job changed')
        attempts=[x for x in trace if x.get('command',{}).get('operation')=='attempt_grasp_and_probe']
        physical=[x for x in attempts if x['result'].get('physical_grasp_started')]
        results=[x['result'] for x in attempts];segments=[dict(s,role=a['command']['segments'][i]['role']) for a in attempts for i,s in enumerate(a['result'].get('segments',[]))]
        last_exit=next((x.get('result',{}).get('safe_exit') for x in reversed(trace) if x.get('result',{}).get('safe_exit')), {})
        row=dict(episode_id=e['episode_id'],asset_id=e['asset_id'],status=read(p/'EPISODE_RESULT.json').get('status'),
            wall_s=read(p/'PROCESS_COMPLETION.json')['wall_s'],storage_bytes=sum(f.stat().st_size for f in p.rglob('*') if f.is_file()),storage_by_category_bytes=storage_categories(p),
            initial_grasp=bool(physical and (physical[0]['result'].get('acquisition') or {}).get('acquired')),
            initial_proposal_success=bool(attempts and (attempts[0]['result'].get('acquisition') or {}).get('acquired')),
            physical_acquisitions=len(physical),successful_acquisitions=sum(bool((x['result'].get('acquisition') or {}).get('acquired')) for x in physical),
            recovery_acquisitions=max(0,len(physical)-1),planning_proposals=len(attempts),
            observations=len(list((p/'rgbd').glob('*/camera.json'))),Agent_view_moves=sum(x.get('command',{}).get('operation')=='move_and_observe' and 'observation' in x.get('result',{}) for x in trace),
            Agent_terminal_reason=next((x['command'].get('reason') for x in reversed(trace) if x.get('command',{}).get('operation')=='finish'),None),
            runner_exit_status_is_not_physical_PASS=True,
            safe_park=bool(last_exit.get('safe') and (last_exit.get('park') or {}).get('reached')),
            complete_validation=any(s.get('status')=='TARGET_REACHED' and s.get('requested_target_reached') and s['role'].startswith('VALIDATION') for s in segments),
            pull_target_complete=any(s.get('status')=='TARGET_REACHED' and s.get('gain_native',0)>0 for s in segments),
            reverse_target_complete=any(s.get('status')=='TARGET_REACHED' and s.get('gain_native',0)<0 for s in segments),
            pull_travel_m=sum(max(0,s.get('gain_native') or 0) for s in segments),reverse_travel_m=sum(max(0,-(s.get('gain_native') or 0)) for s in segments),
            segments=[{k:s.get(k) for k in ('role','status','reason','gain_native','joint_coordinate','elapsed_sim_s','requested_target_reached','physical_transmission_valid')} for s in segments],
            attempts=[dict(requested_preload_N=x['command'].get('native_g1_preload_request_n'),acquisition=x['result'].get('acquisition'),family=x['result'].get('family'),error=x['result'].get('error'),safe_exit=(x['result'].get('safe_exit') or {}).get('safe')) for x in attempts],
            safety_events=[v for v in read(p/'events.json') if any(k in v['event'] for k in ('SAFETY','WARNING','OVERLOAD','TRANSMISSION','BUDGET'))] if (p/'events.json').exists() else [],
            authored=None,online_model=None,excitation=[],comparison={})
        index=read(p/'telemetry/index.json') if (p/'telemetry/index.json').exists() else {}
        if index.get('chunks'):
            a,_=columns(p);row.update(maximum_drawer_m=float(a['joint_q'].max()),final_drawer_m=float(a['joint_q'][-1]),simulation_s=index['physics_steps_seen']/240,
                numeric_chunks_intact=all(sha(p/'telemetry'/c['file'])==c['sha256'] for c in index['chunks']),raw_pad_peak_N=a['pad_load'].max(0).tolist(),
                maximum_relative_translation_m=float(np.nanmax(a['relative_slip_m'])),raw_sensor_peak_abs=np.max(abs(a['raw_ft']),axis=0).tolist(),warning_samples=int(a['load_warning'].sum()),hard_stop_samples=int(a['safety_stop'].sum()))
            phases=np.asarray(index['metadata']['phases'])[a['phase']]
            for attempt,summary in zip(attempts,row['attempts']):
                result=attempt['result'];lo=result.get('first_record');hi=result.get('last_record')
                ix=np.flatnonzero((a['step']>=lo)&(a['step']<=hi)&(phases=='FORCE_HOLD')) if lo is not None and hi is not None else np.array([],dtype=int)
                ix=ix[-240:]  # Last native second of hold; report only, no FIT filtering.
                summary['last_hold_positive_normal_reports_N']=(dict(samples=len(ix),duration_s=float(a['t'][ix[-1]]-a['t'][ix[0]]),
                    median=np.median(a['pad_load'][ix].astype(float),axis=0).tolist(),peak=a['pad_load'][ix].max(0).tolist(),
                    commanded_aperture_m=float(np.median(a['command_aperture'][ix])),measured_aperture_m=float(np.median(a['aperture'][ix])),
                    missing_normal_report_is_not_proven_separation=True) if len(ix) else None)
            # A thrown hard stop may omit gain_native from the operation reply.
            # Always recover actual travel from measured native-coordinate data,
            # never count Recorder.completed (export finished) as target success.
            recorded=[]
            for s in index['segments']:
                ix=np.flatnonzero(a['segment']==s['id']);r=s.get('result',{})
                gain=float(a['joint_q'][ix[-1]]-a['joint_q'][ix[0]]) if len(ix) else 0.
                recorded.append(dict(role=s['role'],status=r.get('status'),reason=r.get('reason'),gain_native=gain,
                    joint_coordinate=float(a['joint_q'][ix[-1]]) if len(ix) else None,elapsed_sim_s=r.get('elapsed_sim_s'),
                    measured_record_duration_s=float(a['t'][ix[-1]]-a['t'][ix[0]]) if len(ix) else None,
                    requested_target_reached=bool(r.get('status')=='TARGET_REACHED' and r.get('ok')),
                    physical_transmission_valid=r.get('physical_transmission_valid'),
                    sliding_observed=(r.get('prismatic_startup') or {}).get('sliding_started_s') is not None))
            row['segments']=recorded
            row['pull_travel_m']=sum(max(0,s['gain_native']) for s in recorded);row['reverse_travel_m']=sum(max(0,-s['gain_native']) for s in recorded)
            row['pull_target_complete']=any(s['requested_target_reached'] and s['gain_native']>0 for s in recorded)
            row['reverse_target_complete']=any(s['requested_target_reached'] and s['gain_native']<0 for s in recorded)
            row['complete_validation']=any(s['requested_target_reached'] and s['role'].startswith('VALIDATION') for s in recorded)
            row['physical_pull_observed']=any(s['gain_native']>5e-5 and s['sliding_observed'] for s in recorded)
            row['physical_reverse_observed']=any(s['gain_native']< -5e-5 and s['sliding_observed'] for s in recorded)
        f=p/'model/estimate.json'
        row['online_model_status']=read(f).get('status') if f.exists() else 'unavailable'
        if (p/'model/FIT_CONSISTENCY_V73.json').exists():
            c=read(p/'model/FIT_CONSISTENCY_V73.json')
            row['cross_FIT_consistency']={k:c.get(k) for k in ('between_FIT_parameter_range','between_FIT_parameter_std','leave_one_FIT_out','meaning')}
            row['cross_FIT_consistency']['groups']=[{k:g.get(k) for k in ('segment','parameters','conditional_std','condition','velocity_rms_m_s','acceleration_rms_m_s2')} for g in c.get('groups',[])]
        if f.exists() and read(f).get('selected'):
            model=read(f);authored=read(p/'HIDDEN_AUTHORED_PHYSICS.json')
            if authored['configuration']!=case:raise ValueError('Scene used a different registered configuration')
            mass,extra=authored_inertial_reference(authored,job['asset_root'],'prismatic')
            row['authored']=dict(mass_kg=mass,damping_N_s_m=case['b'],resistance_law=case['law'],unrandomized_moving_bodies=extra,reference='authored rigid-body combination; not independently measured PhysX effective mass')
            row['online_model_sha256']=sha(f);row['online_model']=dict(selected=model['selected'],FIT_qualification=model.get('FIT_qualification_v73'),worker_source_sha256=model.get('source_sha256'),conditional_uncertainty_is_not_total_calibrated_accuracy=True)
            pieces,settings,quality=load_segments(p,True)
            for d in pieces:
                qualify(d);samples(d,settings['estimation']['domain_native'],settings['estimation']['intervals']);v=d['valid']
                row['excitation'].append(dict(block=d['block_id'],qualified_samples=int(v.sum()),range_m=[float(d['q'].min()),float(d['q'].max())],velocity_RMS_m_s=float(np.sqrt(np.mean(d['native_qd'][v]**2))) if v.any() else None,acceleration_RMS_m_s2=float(np.sqrt(np.mean(d['qdd'][v]**2))) if v.any() else None))
            row['FIT_eligibility_provenance']=quality
            FIT_labels={d['block_id'].rsplit('#',1)[0] for d in pieces if d['segments'][0].startswith('FIT') and d['valid'].any()}
            row['eligible_FIT_trajectory_count']=len(FIT_labels)
            row['state_dependence_selected']=bool(model['selected']['intervals'] and len(FIT_labels)>=3)
            for name in ('constant_model','extended_model','selected'):
                m=model.get(name)
                if not m:continue
                grid=np.concatenate([np.linspace(lo,hi,101) for lo,hi in m['support_intervals_native']]);law=case['law']
                estimated=basis(grid,m['domain_native'],m['intervals'])[:,m['active_controls']]@m['parameters'][2:] if m['intervals'] else np.full(len(grid),m['parameters'][2])
                actual=law['base']+law['amplitude']*np.exp(-.5*((grid-law['center'])/law['width'])**2)
                row['comparison'][name]=dict(parameters=m['parameters'],conditional_std=m['conditional_std'],conditionally_supported=m['conditionally_supported'],scaled_condition=m['scaled_condition'],
                    selected_online=(m==model['selected']),extension_minimum_FIT_count_satisfied=(len(FIT_labels)>=3),
                    mass_error_percent=100*(m['parameters'][0]/mass-1) if not extra else None,damping_error_percent=100*(m['parameters'][1]/case['b']-1),
                    resistance_RMSE_observed_support_N=float(np.sqrt(np.mean((estimated-actual)**2))),support_intervals_m=m['support_intervals_native'],
                    heldout_motion={d['block_id']:heldout_metrics(d,m) for d in pieces if d['segments'][0].startswith('VALIDATION')})
            if sha(f)!=row['online_model_sha256']:raise ValueError('Evaluator modified online result')
        elif (p/'HIDDEN_AUTHORED_PHYSICS.json').exists():
            truth=read(p/'HIDDEN_AUTHORED_PHYSICS.json');mass,extra=authored_inertial_reference(truth,job['asset_root'],'prismatic');row['authored']=dict(mass_kg=mass,damping_N_s_m=case['b'],resistance_law=case['law'],unrandomized_moving_bodies=extra)
        rows.append(row)
    summary={k:dict(successes=sum(bool(r.get(k)) for r in rows),denominator=8) for k in ('initial_grasp','physical_pull_observed','physical_reverse_observed','pull_target_complete','reverse_target_complete','complete_validation','safe_park')}
    out=dict(schema='v74-sealed-blind-evaluation',registered_episodes=8,configuration_commitment=reg['hidden_configuration_sha256'],truth_reveal_UTC=reveal_UTC,online_models_modified=False,rows=rows,stage_rates=summary,scope='four development objects, fresh independent dynamics; not unseen-object transfer or a V7.2 rerun')
    (batch/'public_derived').mkdir(exist_ok=True);atomic_json(batch/'public_derived/METRICS.json',result_packet(out));atomic_json(access,dict(accessed_UTC=reveal_UTC,all_eight_online_seals_verified=True,hidden_truth_used_for_online_decisions=False,evaluation_complete=True));return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--batch',type=Path,required=True);p.add_argument('--registration',type=Path,required=True);p.add_argument('--private',type=Path,required=True);a=p.parse_args();r=evaluate(a.batch,a.registration,a.private);print(json.dumps(r['stage_rates']))
