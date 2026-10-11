"""Copy exact e1e534c inputs; no simulator or new plan generation."""
from pathlib import Path
import json,hashlib,shutil,datetime
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');old=ROOT/'results/45415_closure_verification_20261011';out=ROOT/'results/45415_precontact_continuity_20261011';out.mkdir(exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
runner=ROOT/'scripts/run_oracle_door_precontact_continuity.py';oldrunner=ROOT/'scripts/run_oracle_door_closure_continuity.py'
assert sha(oldrunner)=='5fb2530b0b637f266be51cc2025b5fae7f7f8b139358ee5a59ae57108ef02638'
for n in ['whole_plan.json','grasp_plan.json']:shutil.copy2(old/n,out/n)
for name in ['source','asset','provenance']:
 p=out/name
 if not p.exists():p.symlink_to((old/name).resolve(),target_is_directory=True)
job=json.loads((old/'job.json').read_text());job.update(source=str(out/'source'),asset_root=str(out/'asset'),plan=str(out/'grasp_plan.json'),whole_task_plan=str(out/'whole_plan.json'),output=str(out/'physical_01'),gpu=3,expected_runner_sha256=sha(runner),expected_trajectory_helper_sha256=sha(ROOT/'scripts/precontact_trajectory.py'),episode_id='45415_one_precontact_continuity_trial',deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
(out/'job.json').write_text(json.dumps(job,indent=2))
paths=[oldrunner,Path(job['active_closure_source']),Path(job['baseline_runner']),Path(job['pink_runner']),ROOT/'scripts/piper_pink_fixed.py',ROOT/'articulated_interaction/control.py',ROOT/'articulated_interaction/physical_baseline.py',ROOT/'config/piper.urdf',ROOT/'config/semantic_interaction.json',ROOT/'wrist_reconstruction/force_policy.py',old/'whole_plan.json',old/'grasp_plan.json']
(out/'FROZEN_INPUT_HASHES.json').write_text(json.dumps({str(p):sha(p) for p in paths},indent=2))
shutil.copy2(old/'EXPORT_LIFECYCLE_VERIFIED.json',out/'EXPORT_LIFECYCLE_VERIFIED.json')
print(out)
