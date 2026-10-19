from pathlib import Path
import json,hashlib,shutil,datetime,numpy as np
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');old=ROOT/'results/oracle_door_development_20261011/45176';out=ROOT/'results/45176_contact_transmission_20261011';out.mkdir(exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();job=json.loads((old/'job.json').read_text());asset=out/'asset';asset.mkdir(exist_ok=True)
for p in Path(job['asset_root']).iterdir():
 if p.is_dir() and p.name not in ['usd','visual_compatibility'] and not (asset/p.name).exists():(asset/p.name).symlink_to(p.resolve(),target_is_directory=True)
shutil.copy2(Path(job['asset_root'])/'manifest.json',asset/'manifest.json');source=out/'source';source.mkdir(exist_ok=True);s=json.loads((Path(job['source'])/'report.json').read_text());export=json.loads(Path(job['collision_export']).read_text());e=next(e for e in export['shapes'] if e.get('rigid_body_path','').endswith('/base_link'));T=np.array(e['rigid_body_world_transform']);s['robot_base_pose']=[*T[:3,3],float(np.rad2deg(np.arctan2(T[1,0],T[0,0])))];(source/'report.json').write_text(json.dumps(s,indent=2))
prior=json.loads((old/'planning/whole_plan.json').read_text());runner=ROOT/'scripts/run_oracle_door_precontact_continuity.py';closure=ROOT/'results/45415_closure_verification_20261011/provenance/active_closure.py'
job.update(source=str(source),asset_root=str(asset),output=str(out/'planning'),plan=str(out/'grasp_plan.json'),whole_task_plan=str(out/'whole_plan.json'),planning_base_seeds=[prior['base']],gpu=6,active_closure_source=str(closure),expected_runner_sha256=sha(runner),expected_active_closure_sha256=sha(closure),expected_trajectory_helper_sha256=sha(ROOT/'scripts/precontact_trajectory.py'),episode_id='45176_one_contact_transmission_trial',deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat(),maximum_physical_attempts=1)
(out/'planning_job.json').write_text(json.dumps(job,indent=2))
paths=list(json.loads((ROOT/'results/45415_precontact_continuity_20261011/physical_01/source_config_hashes.json').read_text()))+[ROOT/'scripts/plan_oracle_door.py',ROOT/'scripts/run_oracle_door_closure_continuity.py',Path(old/'job.json')]
(out/'FROZEN_INPUT_HASHES.json').write_text(json.dumps({str(p):sha(Path(p)) for p in paths},indent=2))
print(out)
