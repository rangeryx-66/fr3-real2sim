"""Compact public derivatives + private evidence hashes after the blind episode."""
import json,sys,hashlib,time,fcntl
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json

def run(root):
    p=root/'episode_00';r=json.loads((root/'RESULT.json').read_text());assert r['completion']['actor_exited'];out=root/'public_derived';out.mkdir(exist_ok=True);a=r['actual'];trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text());idx=json.loads((p/'telemetry/index.json').read_text());clock=json.loads((root/'passive_geometry_dataset/CLOCK_DIAGNOSTICS.json').read_text());patch=json.loads((root/'METADATA_PATCH_REGRESSION.json').read_text());coverage=json.loads((root/'fixed_camera_evaluation/COVERAGE.json').read_text());online=json.loads((p/'model/estimate.json').read_text());pre=json.loads((p/'PREVALIDATION_ONLINE_MODEL.json').read_text());observations=[]
    for view in sorted((p/'rgbd').iterdir()):
        if view.is_dir() and (view/'camera.json').exists():
            m=json.loads((view/'camera.json').read_text());observations.append(dict(checkpoint=view.name,t_sim=m['t_sim'],FK_error_m=m['FK_camera_translation_error_m'],rigid_error_m=m['rigid_camera_translation_error_m']))
    segments=[]
    for s in idx['segments']:
        v=s.get('result',{});segments.append(dict(id=s['id'],role=s['role'],requested_deg=s['target_deg'],actual_end_deg=v.get('angle_deg'),gain_deg=v.get('gain_deg'),status=v.get('status'),reason=v.get('reason'),sim_duration_s=v.get('elapsed_sim_s'),physical_transmission_valid=v.get('physical_transmission_valid'),FIT_quality=next((z['quality']['label'] for z in r['quality'] if z['id']==s['id']),None)))
    commands=[]
    for i,t in enumerate(trace):
        c=t['command'];v=t['result'];s=dict(sequence=i,operation=c['operation'],Agent_reasoning=c.get('agent_reasoning'),inspected_image_sha256=c.get('inspected_image_sha256'),target=c.get('target'),operation_wall_s=t['wall_s'])
        if 'feasible' in v:s['feasible']=v['feasible'];s['route_reasons']=[z.get('reason') for z in v.get('audit',[])]
        if c['operation']=='build_geometry':s.update(plan=v.get('plan'),family=v.get('candidate_family'))
        if c['operation']=='attempt_grasp_and_probe':s.update(acquired=v.get('acquisition',{}).get('acquired'),family=v['family'],segments=[dict(status=z['status'],angle_deg=z.get('angle_deg')) for z in v['segments']],safe_park=v.get('safe_exit',{}).get('safe'))
        commands.append({k:v for k,v in s.items() if v is not None})
    # Camera coverage/angles are evaluation-only; never copy this summary into
    # the future geometry Agent's clean observation package.
    cams=[]
    for row in coverage['rows']:
        cams.append(dict(sequence=row['sequence'],actual_angle_deg=row['angle_deg'],moving_sample_depth_agreement=row['groups']['moving']['joint_depth_agreement_fraction'],stationary_sample_depth_agreement=row['groups']['stationary']['joint_depth_agreement_fraction']))
    store={}
    for f in root.rglob('*'):
        if not f.is_file():continue
        parts=f.relative_to(root).parts
        category='passive_RGBD_dataset' if parts[0]=='passive_geometry_dataset' else 'numeric_FT_motion' if len(parts)>1 and parts[1]=='telemetry' else 'wrist_RGBD' if len(parts)>1 and parts[1]=='rgbd' else 'event_video' if f.suffix=='.mp4' else 'derived_source_metadata'
        store[category]=store.get(category,0)+f.stat().st_size
    summary=dict(experiment='V6',assessment='PASS_WITH_LIMITATIONS_SINGLE_BLIND_DEVELOPMENT_EPISODE',reference='fe69627',actual_executed_commit='97f9364',case='45936',episodes=1,acquisitions=2,recovery_regrasps=1,Agent_view_adjustments=4,wrist_checkpoints=observations,
        maximum_actual_deg=a['maximum_actual_deg'],final_deg=a['final_deg'],opening_segment_travel_deg=sum(max(s['gain_deg'] or 0,0) for s in segments),closing_segment_travel_deg=-sum(min(s['gain_deg'] or 0,0) for s in segments),segments=segments,
        blind_commitment=json.loads((root/'CONFIGURATION_COMMITMENT.json').read_text())['configuration_sha256'],truth_reveal_UTC=r['truth_access_UTC'],authored_reference=dict(inertia_kg_m2=r['authored_inertia'],**r['authored_configuration']),authored_not_independent_effective_PhysX_measurement=True,
        prevalidation_model=dict(training_bins=pre['training_bins'],validation_bins=pre['validation_bins'],parameters=pre['selected']['parameters'],conditional_std=pre['selected']['conditional_std'],scaled_condition=pre['selected']['scaled_condition']),primary_prevalidation_comparison=r['comparison'],final_online_comparison=r['final_online_comparison'],qualified_FIT_support_intervals_deg=r['qualified_FIT_support_intervals_deg'],excitation=r['excitation'],postvalidation_FIT_was_predeclared=True,online_model_sha256=patch['original_online_sha256'],online_model_unchanged=r['online_model_preserved'],full_basis_conditioning_is_poor=True,unmeasured_resistance_gaps_deg=[[a[1],b[0]] for a,b in zip(r['qualified_FIT_support_intervals_deg'],r['qualified_FIT_support_intervals_deg'][1:])],no_claim_of_measured_peak_in_gap=True,
        safety={k:a[k] for k in ['peak_pads_n','raw_sensor_peak_abs','interaction_minimum_joint_margin_rad','sampled_all_phase_minimum_joint_margin_rad','measured_loaded_TCP_max_mm_s','load_warning_samples','hard_stop_samples','positive_normal_report_loss_events']},final_safe_park=trace[-1]['result']['safe_exit']['safe'],
        time=dict(wall_including_init_cleanup_s=r['completion']['wall_including_init_cleanup_s'],simulation_s=a['physics_steps']/240,Agent_unloaded_wait_s=a['Agent_wait_s'],physical_operations_wall_s=sum(t['wall_s'] for t in trace if t['command']['operation']=='attempt_grasp_and_probe'),automatic_geometry_planning_wall_s=sum(t['wall_s'] for t in trace if t['command']['operation']=='build_geometry'),passive_capture_wall_s=r['passive_camera_dataset']['capture_wall_s'],worker_final_wall_s=online['worker_wall_s']),
        storage=dict(bytes_by_category=store,total_bytes=sum(store.values()),target_bytes=200000000),camera=dict(pairs=15,cameras=2,raw_files=45,calibrated_fixed_poses=True,clock_repair=clock,evaluation_only_coverage=cams,geometry_observation_package_excludes_GT=True,online_fixed_image_inspections=0),integrity=dict(numeric_chunks_intact=r['numeric_chunks_intact'],unique_stored_steps=a['unique_stored_steps'],all_FT_valid=a['all_FT_samples_valid'],stored_records=a['stored_records'],physics_steps=a['physics_steps'],recording_issues=a['recording_issues'],metadata_patch_regression=patch),native_passive_law=r['native_law_application'],limitations=['known hinge/state and registered station','simulated wrist/fixed cameras and ideal load cell','approximate actuators; not hardware safety validation','conditional covariance excludes systematic/contact and model-selection errors','no45deg or hinge reconstruction result','geometry/model observations have a qualified FIT gap; no measured peak there'])
    assert summary['maximum_actual_deg']>=30 and any(s['role'].startswith('VALIDATION') and s['status']=='TARGET_REACHED' for s in segments)
    atomic_json(out/'SUMMARY.json',summary);atomic_json(out/'AGENT_TRACE.json',commands)
    hashes={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [p/'model/estimate.json',p/'telemetry/index.json',p/'SOURCE_PROVENANCE.json',root/'HIDDEN_CONFIGURATIONS.json',root/'passive_geometry_dataset/index.json',root/'passive_geometry_dataset/calibration.json',root/'passive_geometry_dataset/integrity.json']}
    atomic_json(root/'PRIVATE_EVIDENCE_INDEX.json',dict(private_root=str(root.resolve()),files=hashes,geometry_Agent_input_only=str((root/'passive_geometry_dataset').resolve()),evaluation_only_siblings=['fixed_camera_evaluation','HIDDEN_CONFIGURATIONS.json','episode_00'],raw_not_uploaded=True))
    print(json.dumps(dict(maximum_actual_deg=summary['maximum_actual_deg'],closing_travel_deg=summary['closing_segment_travel_deg'],wall_s=summary['time']['wall_including_init_cleanup_s'],storage=summary['storage']),indent=2))
if __name__=='__main__':run(Path(sys.argv[1]))
