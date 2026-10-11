"""No simulator: pin the exact previous plan and generate the one-line closure."""
from pathlib import Path
import ast,json,hashlib,shutil,datetime
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');old=ROOT/'results/oracle_door_development_20261011/45415';out=ROOT/'results/45415_closure_verification_20261011';out.mkdir(exist_ok=True);provenance=out/'provenance';provenance.mkdir(exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
prior=json.loads((old/'development_02/source_config_hashes.json').read_text())
inputs=[old/'planning/whole_plan.json',old/'planning/grasp_plan.json',old/'source/report.json',old/'asset/manifest.json']
for p in inputs:
 if sha(p)!=prior[str(p)]:raise RuntimeError('PRELAUNCH_PREVIOUS_INPUT_CHANGED:'+str(p))
for n in ['whole_plan.json','grasp_plan.json']:shutil.copy2(old/'planning'/n,out/n)
source=out/'source';source.mkdir(exist_ok=True);shutil.copy2(old/'source/report.json',source/'report.json')
asset=out/'asset';asset.mkdir(exist_ok=True)
for f in (old/'asset').iterdir():
 if f.is_dir() and f.name not in ['visual_compatibility','usd'] and not (asset/f.name).exists():(asset/f.name).symlink_to(f.resolve(),target_is_directory=True)
shutil.copy2(old/'asset/manifest.json',asset/'manifest.json')
control=ROOT/'articulated_interaction/control.py';tree=ast.parse(control.read_text());node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='JawCenteredClosure');original=ast.get_source_segment(control.read_text(),node)+'\n'
needle='self.opening=float(opening) # pause further squeeze';replacement='self.opening=min(self.opening,float(opening)) # retain closing command while centering'
if original.count(needle)!=1:raise RuntimeError('PRELAUNCH_CLOSURE_SOURCE_NOT_THE_PINNED_ONE_LINE')
fixed=original.replace(needle,replacement);active=provenance/'active_closure.py';active.write_text('import numpy as np\n\n'+fixed);(provenance/'original_closure.py').write_text('import numpy as np\n\n'+original)
shutil.copy2(ROOT/'scripts/run_oracle_door_closure_continuity.py',provenance/'runner_snapshot.py')
job=json.loads((old/'development_02/job.json').read_text());job.update(source=str(source),asset_root=str(asset),plan=str(out/'grasp_plan.json'),whole_task_plan=str(out/'whole_plan.json'),output=str(out/'physical_01'),gpu=3,active_closure_source=str(active),expected_active_closure_sha256=sha(active),expected_runner_sha256=sha(ROOT/'scripts/run_oracle_door_closure_continuity.py'),deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=2)).isoformat(),episode_id='45415_one_closure_fix_verification',classification='ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT',maximum_physical_attempts=1)
(out/'job.json').write_text(json.dumps(job,indent=2));(out/'INPUT_PROVENANCE.json').write_text(json.dumps({'base':json.loads((out/'whole_plan.json').read_text())['base'],'previous_commit':'93d6706','unchanged_plan_source_manifest_hashes':{str(p):sha(p) for p in inputs},'copied_whole_plan_sha256':sha(out/'whole_plan.json'),'copied_grasp_plan_sha256':sha(out/'grasp_plan.json'),'original_control_file_sha256':sha(control),'runner_path':str(ROOT/'scripts/run_oracle_door_closure_continuity.py'),'runner_sha256':job['expected_runner_sha256'],'active_closure_sha256':sha(active),'physical_trial_cap':1},indent=2))
print(out)
