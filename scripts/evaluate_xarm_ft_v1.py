"""Independent post-episode truth/behavior evaluator. Never imported online."""
import argparse,json,pickle,gzip,hashlib,datetime
from pathlib import Path
import numpy as np
from xarm_ft_v1_estimator import local_process,analyze,resolution_sensitivity,forward,KEYS
from evaluate_xarm_ft_identification import effective_inertia


def curve(q,model):
    c=model['parameters'][2:]
    if model['kind']=='constant':return np.full_like(q,c[0])
    x=np.clip(q/np.deg2rad(20),0,1);return c[0]*(1-x)**2+c[1]*2*x*(1-x)+c[2]*x*x


def evaluate(root):
    root=Path(root);checks=[]
    for i in range(2):
        c=root/('case_%02d'%i);pid=json.loads((c/'PROCESS.json').read_text())['pid']
        checks.append(dict(case=i,pid=pid,alive=Path('/proc/%d'%pid).exists(),exported=(c/'episode/EXPORT_COMPLETE.json').exists()))
    if not all(not c['alive'] and c['exported'] for c in checks):raise RuntimeError('Both registered actions must end before hidden truth is opened')
    boundary=dict(UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,all_actions_finished=True)
    if not (root/'TRUTH_ACCESS_BOUNDARY.json').exists():
        (root/'TRUTH_ACCESS_BOUNDARY.json').write_text(json.dumps(boundary,indent=2))
    with (root/'INDEPENDENT_EVALUATION_RUNS.jsonl').open('a') as log:log.write(json.dumps(boundary)+'\n')
    cases=[]
    for i in range(2):
        case=root/('case_%02d'%i);episode=case/'episode';job=json.loads((case/'job.json').read_text());truth=json.loads((episode/'HIDDEN_AUTHORED_PHYSICS.json').read_text())
        packets=sorted((episode/'identification_v1').glob('attempt_*/block_*/IDENTIFIED_MODEL.json'));updates=[]
        law=truth['law'];gt=dict(I_eff_kg_m2=effective_inertia(truth,job['asset_root'],{}),b_nm_s_rad=truth['configuration']['b'],Coulomb=law)
        for path in packets:
            model=json.loads(path.read_text());entry=dict(path=str(path.relative_to(episode)),classification='original in-episode numeric update',models={})
            if not model.get('models'):continue
            data=dict(np.load(path.parent/'CUMULATIVE_DATA.npz',allow_pickle=True));qrange=[float(np.min(data['q'])),float(np.max(data['q']))];q=np.linspace(*qrange,101)
            samples=json.loads((path.parent/'CUMULATIVE_SAMPLES.json').read_text());training=[s['q'] for s in samples if not s['segment'].startswith('VALIDATION')]
            true_curve=law['base_nm']+law['amplitude_nm']*np.exp(-.5*((q-law['center_rad'])/law['width_rad'])**2)
            for name,m in model['models'].items():
                estimate=curve(q,m);valid_rank=m['algebraic_rank']==len(m['parameters']);supported=[bool(v and valid_rank) for v in m['supported']]
                entry['models'][name]=dict(model=m,supported_after_rank_audit=supported,I_absolute_error=float(abs(m['parameters'][0]-gt['I_eff_kg_m2'])),b_absolute_error=float(abs(m['parameters'][1]-gt['b_nm_s_rad'])),
                    c_observed_range_rmse_nm=float(np.sqrt(np.mean((estimate-true_curve)**2))),c_observed_range_max_error_nm=float(np.max(abs(estimate-true_curve))),
                    curve_angle_deg=np.rad2deg(q).tolist(),estimated_c_nm=estimate.tolist(),authored_c_nm=true_curve.tolist(),
                    no_state_extrapolation_validation_claim=True)
            entry.update(coverage=model['coverage'],selected_model=model['selected_model'],candidate_information=model.get('candidate_information'),observed_range_deg=np.rad2deg(qrange).tolist(),training_range_deg=np.rad2deg([min(training),max(training)]).tolist() if training else None);updates.append(entry)
        # Preserve partial/failed interaction data too; label separate from online.
        rows=[]
        for f in sorted((episode/'observations_chunks').glob('*.gz')):
            with gzip.open(f,'rb') as stream:rows.extend(pickle.load(stream))
        measured_by_tick={round(r['t']*240):r for r in rows}
        rgbd=[]
        for f in sorted((episode/'rgbd').rglob('camera.json')):
            camera=json.loads(f.read_text());tick=round(camera['t_sim']*240);row=measured_by_tick.get(tick)
            rgbd.append(dict(folder=str(f.parent.relative_to(episode)),camera_timestamp_s=camera['t_sim'],capture_role='information-block endpoint' if f.parent.name.startswith('identification_v1') else 'actual wrist observation/pregrasp refinement',physics_step_index=row.get('telemetry',{}).get('physics_step_index') if row else None,
                actual_angle_deg=row['door_angle_deg'] if row else None,hinge_velocity_rad_s=row.get('telemetry',{}).get('hinge_velocity_rad_s') if row else None,
                wrist_wrench_valid=row.get('wrist_ft',{}).get('valid') if row else None,
                source='same physics tick association to measured robot-side sensor and allowed known articulation state',GT_dynamics_used=False))
        (case/'MULTISTATE_RGBD_INDEX.json').write_text(json.dumps(rgbd,indent=2))
        end=json.loads((episode/'RECOVERY_EPISODE_RESULT.json').read_text())
        posthoc=[]
        from xarm_ft_known_joint_prior import native_joint_prior
        for j,attempt in enumerate(end['attempts']):
            selected=rows[attempt['first_row']:attempt['last_row']]
            if not selected or not any(r.get('identification_segment') for r in selected):continue
            prior=native_joint_prior(job['asset_root'],selected[0]['T_moving_link']);data,provenance=local_process(selected,prior,job['ft_calibration']['bias_sensor'])
            if data is None:continue
            aligned=[measured_by_tick[round(t*240)] for t in data['t']]
            contact=dict(pad_loads=np.array([[r['forces_n'].get(n,np.nan) for n in ['gripper_link1','gripper_link2']] for r in aligned]),
                relative_translation_proxy_m=np.array([r.get('relative_translation_slip_m',np.nan) for r in aligned]),
                measured_aperture_m=np.array([r['aperture_m'] for r in aligned]),
                command_aperture_m=np.array([r.get('telemetry',{}).get('command_aperture_m',np.nan) for r in aligned],dtype=float))
            summary=analyze(data);quant=resolution_sensitivity(data)
            outcomes_path=episode/('MULTISTATE_SEGMENTS_%02d.json'%j)
            outcomes=json.loads(outcomes_path.read_text()) if outcomes_path.exists() else []
            summary['executed_segment_outcomes']=outcomes
            summary['poor_transmission_or_protected_segments']=[r['label'] for r in outcomes if not r['ok']]
            summary['segment_status_semantics']='ok means its configured local target, never an automatic30deg claim; failed-segment tags do not retrospectively alter frozen weights'
            summary['data_quality_audit']=dict(
                native_records=len(selected),local_processed_records=len(data['t']),retained_valid_native_records=int(data['valid'].sum()),
                excluded_native_records=int((~data['valid']).sum()),
                low_velocity_or_reversal_records=int(np.sum(abs(data['qd'])<1e-4)),
                original_force_event_records=sum(r.get('force_event',{}).get('status') not in [None,'NORMAL'] for r in aligned),
                relative_translation_above_original_effort_validity_5mm_records=int(np.sum(contact['relative_translation_proxy_m']>.005)),
                zero_positive_normal_both_pads_records=int(np.sum(np.nanmax(contact['pad_loads'],axis=1)==0)),
                zero_positive_normal_is_not_proven_contact_separation=True,
                derivatives_and_tool_compensation_segment_local=True,raw_native_records_unchanged=True,
                definition='posthoc quality counts only; no new sample filtering, fit weights or execution rules')
            native={round(r['t']*240):r.get('telemetry',{}).get('hinge_velocity_rad_s') for r in selected}
            velocity=np.array([native.get(round(t*240),np.nan) for t in data['t']],dtype=float)
            if np.isfinite(velocity).all():
                # Fixed fitted models; replace only the measured initial-state
                # velocity and velocity-reference metric, never refit to GT.
                alternate=dict(data,qd=velocity)
                for m in summary.get('models',{}).values():
                    m['heldout_native_velocity_initial_state']={}
                    for label in dict.fromkeys(data['segments']):
                        if not str(label).startswith('VALIDATION'):continue
                        ids=np.flatnonzero(data['segments']==label)
                        runs=np.split(ids,np.flatnonzero(np.diff(data['t'][ids])>.01)+1)
                        m['heldout_native_velocity_initial_state'][str(label)]=[forward(alternate,run,m) for run in runs if len(run)>1]
                summary['native_vs_segment_local_velocity_rmse_rad_s']=float(np.sqrt(np.mean((velocity-data['qd'])**2)))
            c_truth=law['base_nm']+law['amplitude_nm']*np.exp(-.5*((data['q']-law['center_rad'])/law['width_rad'])**2)
            reference=gt['I_eff_kg_m2']*data['qdd']+gt['b_nm_s_rad']*data['qd']+c_truth*np.sign(data['qd'])
            summary['authored_inverse_dynamics_reference']={}
            for label in dict.fromkeys(data['segments']):
                ix=(data['segments']==label)&data['valid']
                if ix.any():summary['authored_inverse_dynamics_reference'][str(label)]=dict(sensor_vs_authored_reference_rmse_nm=float(np.sqrt(np.mean((data['torque'][ix]-reference[ix])**2))),independently_measured_GT_torque=False)
            out=episode/'posthoc_v1'/('attempt_%02d'%j);out.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(out/'MEASUREMENTS.npz',**{k:data[k] for k in KEYS},native_hinge_velocity=velocity,**contact);(out/'MODEL.json').write_text(json.dumps(summary,indent=2));(out/'RESOLUTION_SENSITIVITY.json').write_text(json.dumps(quant,indent=2))
            posthoc.append(dict(attempt=j,classification='post-hoc frozen estimator on complete measured attempt including partial last segment',model=summary,provenance=provenance))
        audit=[];native_b=[];log=episode/'PRIVATE_NATIVE_RESISTANCE.jsonl'
        if log.exists():
            for line in log.read_text().splitlines():
                d=json.loads(line);expected=law['base_nm']+law['amplitude_nm']*np.exp(-.5*((d['q']-law['center_rad'])/law['width_rad'])**2)
                audit.append(max(abs(d['native_properties'][k]-expected) for k in [0,1]));native_b.append(d['native_properties'][2])
        result=dict(case=i,seed=truth['configuration']['seed'],authored_truth=gt,truth_semantics='authored native passive friction and coherent mass model; effective solver dynamics may differ',
            hidden_truth_not_used_online=True,updates=updates,posthoc=posthoc,physical_result=end,
            native_property_readback_count=len(audit),max_native_Coulomb_readback_error_nm=float(max(audit)) if audit else None,
            native_viscous_readback_range=[float(min(native_b)),float(max(native_b))] if native_b else None,
            native_viscous_vs_authored_b_max_error=float(max(abs(np.asarray(native_b)-gt['b_nm_s_rad']))) if native_b else None,
            complete_GT_joint_effort_channel='not available; no contact-sum substitute',
            physical_twin_success_not_inferred_from_torque_fit=True)
        (case/'INDEPENDENT_EVALUATION_V1.json').write_text(json.dumps(result,indent=2));cases.append(result)
        if updates:
            last=updates[-1];selected_model=last['models'][last['selected_model']]['model'];supported=last['models'][last['selected_model']]['supported_after_rank_audit'];prior=json.loads((packets[-1].parent/'INPUT.json').read_text())['prior']
            model=dict(schema='articulated_physics_model_v1',geometry_reference=job['asset_root'],observed_geometry_reference=str(episode/'agent_geometry/observed_geometry.json'),kinematic_prior=prior,
                current_motion_state=json.loads((episode/'progress.json').read_text()),selected_family=last['selected_model'],
                I_eff_kg_m2=selected_model['parameters'][0] if supported[0] else None,b_nm_s_rad=selected_model['parameters'][1] if supported[1] else None,
                Coulomb_function=dict(basis='constant' if last['selected_model']=='constant' else 'three nonnegative quadratic Bernstein controls, domain0..20deg',provisional_coefficients_nm=selected_model['parameters'][2:],conditional_supported_controls=supported[2:]),
                conditional_covariance=selected_model['conditional_covariance'],valid_observed_range_deg=last['observed_range_deg'],training_range_deg=last['training_range_deg'],unfitted_interior_region_deg=[7,9],
                parameters_not_individually_identified=['mass','full COM','full inertia tensor','spring vs gravity'],
                model_status='provisional; independent held-out motion and calibration limitations govern interpretation',
                sensor=dict(type='ideal native fixed-interface load cell',native_hz=240,calibration=str(case.parent.parent/'xarm_wrist_ft_id_v0_20261017/sensor_smoke_00/SENSOR_ANALYSIS.json'),raw_mapping=str(episode/'WRIST_FT_MAPPING.json')),
                source_freeze=str(root/'FROZEN.json'),supported_segments=[u['path'] for u in updates],wrist_RGBD_index=str(case/'MULTISTATE_RGBD_INDEX.json'),wrist_RGBD=[str(p.relative_to(episode)) for p in (episode/'rgbd').glob('identification_v1*')],
                executed_grasp_geometry=[str(Path(a['folder'])/'geometry/observed_geometry.json') for a in end['attempts'] if a.get('physical_grasp_started')],
                remaining_known_dependencies=['hinge geometry','actual dynamic articulation state','registered station','native safety collision model','simulated wrist mounting','approximate actuators'],hidden_dynamics_excluded=True)
            (case/'ESTIMATED_PHYSICS_MODEL_V1.json').write_text(json.dumps(model,indent=2))
        if not updates:
            # A failed/incomplete block must not masquerade as an online
            # parameter update. Still export a usable explicit unknown model
            # with references to the separately labelled partial-data analysis.
            candidates=[p for p in posthoc if p['model'].get('models')]
            provisional=candidates[-1]['model'] if candidates else None
            selected=provisional['models'][provisional['selected_model']] if provisional else None
            support=[bool(v and selected['algebraic_rank']==len(selected['parameters'])) for v in selected['supported']] if selected else []
            regions=[c['measured_angle_extent_deg'] for c in provisional.get('coverage',[])] if provisional else []
            model=dict(schema='articulated_physics_model_v1',estimation_stage='posthoc incomplete-block diagnosis, not an in-episode model update',
                kinematic_prior=native_joint_prior(job['asset_root'],rows[-1]['T_moving_link']) if rows else None,
                current_motion_state=json.loads((episode/'progress.json').read_text()),
                valid_observed_range_deg=[min(r[0] for r in regions),max(r[1] for r in regions)] if regions else None,
                Coulomb_function=dict(basis=selected['kind'],provisional_coefficients_nm=selected['parameters'][2:],conditional_supported_controls=support[2:],function_domain_deg=[0,20],no_unobserved_range_validation_claim=True) if selected else None,
                conditional_covariance=selected['conditional_covariance'] if selected else None,
                physical_identification_validated=False,
                geometry_reference=job['asset_root'],observed_geometry_reference=str(episode/'agent_geometry/observed_geometry.json'),
                I_eff_kg_m2=selected['parameters'][0] if selected and support[0] else None,
                b_nm_s_rad=selected['parameters'][1] if selected and support[1] else None,
                provisional_fitted_model=selected,conditional_supported_parameters=support,
                model_status='multi-state identification incomplete; no physical-twin success claim',
                supported_measured_segments=[p['provenance'] for p in posthoc],
                executed_grasp_geometry=[str(Path(a['folder'])/'geometry/observed_geometry.json') for a in end['attempts'] if a.get('physical_grasp_started')],
                unknown_terms=['mass','full COM','full inertia tensor','unvisited resistance profile','gravity vs spring'],
                sensor=dict(type='ideal native fixed-interface load cell',native_hz=240,raw_mapping=str(episode/'WRIST_FT_MAPPING.json')),
                wrist_RGBD_index=str(case/'MULTISTATE_RGBD_INDEX.json'),source_freeze=str(root/'FROZEN.json'),
                known_dependencies=['hinge geometry','actual dynamic articulation state','registered station','native safety collision model','simulated wrist mounting','approximate actuators'],
                hidden_dynamics_excluded_from_fitting=True)
            (case/'ESTIMATED_PHYSICS_MODEL_V1.json').write_text(json.dumps(model,indent=2))
    (root/'INDEPENDENT_EVALUATION_ALL.json').write_text(json.dumps(cases,indent=2));return cases

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root');evaluate(p.parse_args().root)
