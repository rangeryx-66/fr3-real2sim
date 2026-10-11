from pathlib import Path
import json,hashlib,shutil,math,datetime
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/benchmarkA_30deg_20261011';registered=json.loads((out/'REGISTERED_OBJECTS.json').read_text());ranking=json.loads((out/'DISCOVERY_GEOMETRY_RANKING.json').read_text())['ranking'];oldroot=ROOT/'results/revolute10_fixed_20261011';oldjob=json.loads((oldroot/'45621/setup_job.json').read_text());oldsource=json.loads((oldroot/'45621/source/report.json').read_text())
shared=out/'shared_sources';shared.mkdir(exist_ok=True)
for name in ['frozen_run_7320_speed.py','run_7320_pink_fixed.py']:
 shutil.copy2(oldroot/'shared_sources'/name,shared/name)
for name in ['plan_benchmark_a30.py','run_benchmark_a30.py','run_benchmark_a30_batch.py','revolute10_fixed.py','run_revolute10_fixed.py','piper_pink_fixed.py','rank_prepared_handle_assets.py','prepare_physx_microwave.py']:
 shutil.copy2(ROOT/'scripts'/name,shared/name)
(out/'SHARED_SOURCE_HASHES.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in shared.glob('*.py')},indent=2))
for row in registered['objects']:
 i=row['asset_id'];original=Path(row['asset_root']);case=out/i;case.mkdir(exist_ok=True);asset=case/'asset';asset.mkdir(exist_ok=True)
 for f in original.iterdir():
  if f.is_dir() and f.name not in ['visual_compatibility','usd'] and not (asset/f.name).exists():(asset/f.name).symlink_to(f,target_is_directory=True)
 meta=json.loads((original/'manifest.json').read_text());candidates=[x for x in ranking if x['asset_id']==i and x['joint_name']==meta['joint_name'] and x['sections']];candidates=sorted(candidates,key=lambda x:x['score'],reverse=True)[:3]
 meta['interaction_geometry']={'method':'unchanged geometry candidate generator; original prepared colliders','selection':candidates[0] if candidates else None,'pieces':[],'original_asset_root':str(original),'eligibility_handle_mesh_used_for_selection_only':row['eligibility_handle_mesh']}
 (asset/'manifest.json').write_text(json.dumps(meta,indent=2))
 src=case/'source';src.mkdir(exist_ok=True);s={k:oldsource[k] for k in ['asset_installation','home_q','robot_base_pose','support_bottom_z_m'] if k in oldsource};s.update(asset_id=i,schema='known-model-benchmark-source/v1',saved_grasp_poses=[],task='Benchmark A: closed to >=30 degrees; one grasp');(src/'report.json').write_text(json.dumps(s,indent=2))
 setup=case/'setup';setup.mkdir(exist_ok=True)
 j={k:oldjob[k] for k in ['camera_calibration','wrist_experiment','timing_experiment']};j.update(episode_id='benchmarkA_'+i,source=str(src),asset_root=str(asset),output=str(setup),gpu=1,plan=str(case/'planning/grasp_plan.json'),whole_task_plan=str(case/'planning/whole_plan.json'),collision_export=str(setup/'cooked_initial.json'),deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=12)).isoformat(),skill={'joint_type':'revolute'},plant={},initial_articulation_rad=0.,baseline_runner=str(shared/'frozen_run_7320_speed.py'),pink_runner=str(shared/'run_7320_pink_fixed.py'),pink_motion_scale=1.,grasp_geometry_candidates=candidates,benchmark_goal_rad=math.pi/6,authored_target_rad=meta['source_joint_limits_rad']['upper'],official_attempt=1,benchmark_root=str(out))
 (case/'setup_job.json').write_text(json.dumps(j,indent=2))
 if not candidates:(case/'planning_failure.json').write_text(json.dumps({'asset_id':i,'failure_reason':'NO_AUTOMATIC_GEOMETRIC_CONTACT_SECTIONS','stage':'planning','official_physical_attempts':0},indent=2))
# Frozen baseline and original benchmark are read-only checks, never execution inputs.
checks=json.loads((oldroot/'frozen_baseline_before.json').read_text())['files']
for f in oldroot.rglob('*'):
 if f.is_file() and not f.is_symlink() and f.name not in ['EVIDENCE_SHA256.json']:checks[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
for f in [ROOT/'results/45746_frozen_prefix_20261008/source_config_hashes.json',ROOT/'results/45746_frozen_fullrange_20261008/source_config_hashes.json']:
 for name,digest in json.loads(f.read_text()).items():checks[str(f.parent/name)]=digest
(out/'FROZEN_BASELINES_AND_OLD_BENCHMARK_BEFORE.json').write_text(json.dumps(checks,indent=2))
print('prepared',registered['asset_ids'],'frozen_files',len(checks))
