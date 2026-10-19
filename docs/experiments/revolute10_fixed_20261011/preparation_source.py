import json,hashlib,os,shutil
from pathlib import Path
from datetime import datetime,timedelta,timezone
root=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=root/'results/revolute10_fixed_20261011'
registered=json.loads((out/'REGISTERED_OBJECTS.json').read_text())
ranking=json.loads(Path('/data1/home/rangeryx/fr3_real2sim_piper_mobile/results/interactive_twin_asset_preparation/ranking.json').read_text())
old=json.loads((root/'results/7320_pink_fixed_20261010/physical_03/job.json').read_text());oldsource=json.loads((Path(old['source'])/'report.json').read_text())
(out/'shared_sources').mkdir(exist_ok=True)
for name in ['revolute10_fixed.py','run_revolute10_fixed.py','piper_pink_fixed.py','run_7320_pink_fixed.py']:
 shutil.copy2(root/'scripts'/name,out/'shared_sources'/name)
shutil.copy2(old['baseline_runner'],out/'shared_sources/frozen_run_7320_speed.py')
hashes={str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (out/'shared_sources').glob('*.py')};(out/'SHARED_SOURCE_HASHES.json').write_text(json.dumps(hashes,indent=2))
for index,row in enumerate(registered['objects']):
 i=row['asset_id'];case=out/i;case.mkdir(exist_ok=True);(case/'source').mkdir(exist_ok=True)
 original=Path(row['asset_root']);m=json.loads((original/'manifest.json').read_text())
 candidates=[x for x in ranking['ranking'] if x['asset_id']==i and x['joint_name']==m['joint_name'] and x['sections']]
 # Score ranks candidates only. No old screen_rejections become fatal gates.
 candidates=sorted(candidates,key=lambda x:x['score'],reverse=True)[:3]
 if not candidates:
  (case/'planning_failure.json').write_text(json.dumps({'asset_id':i,'classification':'KNOWN_MODEL_DIAGNOSTIC','failure_category':'perception/grasp','failure_reason':'EXISTING_GEOMETRIC_GRASP_GENERATOR_RETURNED_NO_CONTACT_SECTIONS','official_physical_attempts':0,'counted_as_failure':True,'target_deg':row['target_deg'],'maximum_robot_caused_angle_deg':0,'raw_geometry_ranking':[x for x in ranking['ranking'] if x['asset_id']==i]},indent=2));continue
 asset=case/'asset';asset.mkdir(exist_ok=True)
 if not (asset/'urdf').exists():(asset/'urdf').symlink_to(original/'urdf',target_is_directory=True)
 selection=candidates[0];m['door_link']=selection['handle_link'];m['interaction_geometry']={'method':'read-only current-asset authored mesh selection; original collision geometry unchanged','selection':selection,'pieces':[],'original_asset_root':str(original),'no_saved_successful_GT_grasp_template':True}
 (asset/'manifest.json').write_text(json.dumps(m,indent=2))
 source={'schema':'known-model-benchmark-source/v1','asset_id':i,'asset_installation':oldsource['asset_installation'],'home_q':[0,1.15,-1.35,0,.2,0],'camera_offset_m':oldsource.get('camera_offset_m'),'robot_base_pose':oldsource['robot_base_pose'],'support_bottom_z_m':-.56,'task':'known-model fixed-base authored-range benchmark; automatic current-asset grasp geometry','saved_grasp_poses':[]}
 (case/'source/report.json').write_text(json.dumps(source,indent=2))
 setup=case/'setup';setup.mkdir(exist_ok=True)
 job={'episode_id':'revolute10_'+i,'source':str(case/'source'),'asset_root':str(asset),'output':str(setup),'gpu':1+index%3,'plan':str(case/'planning/grasp_plan.json'),'whole_task_plan':str(case/'planning/whole_plan.json'),'deadline_shanghai':(datetime.now(timezone(timedelta(hours=8)))+timedelta(hours=12)).isoformat(),'skill':{'joint_type':'revolute'},'plant':{},'initial_articulation_rad':0.,'camera_calibration':old['camera_calibration'],'wrist_experiment':{'force_policy':old['wrist_experiment']['force_policy'],'mobile_scan':old['wrist_experiment']['mobile_scan']},'timing_experiment':old['timing_experiment'],'baseline_runner':str(out/'shared_sources/frozen_run_7320_speed.py'),'pink_runner':str(out/'shared_sources/run_7320_pink_fixed.py'),'pink_motion_scale':1.,'grasp_geometry_candidates':candidates,'collision_export':str(setup/'cooked_initial.json'),'official_attempt':1,'authored_target_rad':row['target_rad'],'benchmark_root':str(out)}
 (case/'setup_job.json').write_text(json.dumps(job,indent=2))
print('registered',registered['asset_ids']);print('prepared',[p.parent.name for p in out.glob('*/setup_job.json')]);print('geometric failures',[p.parent.name for p in out.glob('*/planning_failure.json')])
