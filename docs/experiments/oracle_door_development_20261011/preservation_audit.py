from pathlib import Path
import json,hashlib,os
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/oracle_door_development_20261011';old=ROOT/'results/benchmarkA_30deg_20261011';expected=json.loads((out/'FROZEN_BEFORE.json').read_text());expected.update(json.loads((old/'FROZEN_BASELINES_AND_OLD_BENCHMARK_BEFORE.json').read_text()));changes=[]
for name,digest in expected.items():
 p=Path(name)
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:changes.append(name)
actors=[]
for f in out.glob('*/development_*/actor_process.json'):
 x=json.loads(f.read_text());x['trial']=str(f.parent.relative_to(out));x['still_running']=Path('/proc/'+str(x['pid'])).exists();actors.append(x)
result={'classification':'PRESERVATION_AND_SCOPE_AUDIT','frozen_and_old_result_files_checked':len(expected),'changed_or_missing_frozen_files':changes,'physical_trials_started':len(actors),'maximum_initial_trials_per_object':1,'additional_trials_started':1,'physical_trial_hard_cap':4,'actors':actors,'active_experiment_actors':sum(x['still_running'] for x in actors),'new_physical_successes_30_deg':0,'closure_fix_physically_tested':False,'raw_240Hz_missing_trials':['45176/development_01'],'raw_240Hz_saved_trials':['45415/development_01','45415/development_02'],'known_model_and_oracle_only':True};(out/'FINAL_AUDIT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
