"""Immutable four-case owner: one closed-start attempt each, never retry."""
from pathlib import Path
import json,hashlib,subprocess,os,time
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/door_transfer4_20261012';PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();freeze=json.loads((out/'BATCH_SOURCE_FREEZE.json').read_text());reg=json.loads((out/'REGISTERED_CASES.json').read_text());runner=ROOT/'scripts/run_shared_door_skill.py'
for p,h in freeze['hashes'].items():
 if sha(p)!=h:raise RuntimeError('PRELAUNCH_BATCH_SOURCE_CHANGED_NO_EPISODE:'+p)
infra=json.loads((out/'EXPORT_LIFECYCLE_VERIFIED.json').read_text())
if infra['process_returncode']!=0 or not all(x['lossless'] for x in infra['checks'].values()):raise RuntimeError('PRELAUNCH_EXPORT_NOT_VERIFIED')
with (out/'BATCH_RESERVED.json').open('x') as f:json.dump({'cases':reg['asset_ids'],'maximum':4,'no_extra_attempts':True,'start_wall':time.time()},f,indent=2)
env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1');env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH'];active={};done=[]
for i in reg['asset_ids']:
 case=out/i
 if not (case/'job.json').exists():done.append({'asset_id':i,'planning_failure':True,'physical_attempts':0});continue
 job=json.loads((case/'job.json').read_text());pre=json.loads((case/'PREFLIGHT.json').read_text());trial=Path(job['output']);trial.mkdir(exist_ok=True)
 if sha(runner)!=job['expected_runner_sha256'] or sha(ROOT/'scripts/shared_door_execution.py')!=job['shared_policy_sha256']:raise RuntimeError('PRELAUNCH_EXECUTABLE_CHANGED')
 for p,h in pre['source_config_hashes'].items():
  if sha(p)!=h:raise RuntimeError('PRELAUNCH_CASE_CONFIG_CHANGED:'+i)
 with (trial/'TRIAL_RESERVED.json').open('x') as f:json.dump({'asset_id':i,'physical_attempt':1,'maximum':1,'normal_closed_start':True,'runner':str(runner),'runner_sha256':sha(runner)},f,indent=2)
 paths=list(freeze['hashes'])+[str(case/'job.json'),str(case/'whole_plan.json'),str(case/'grasp_plan.json'),str(Path(job['asset_root'])/'manifest.json'),str(Path(job['source'])/'report.json')];(trial/'source_config_hashes.json').write_text(json.dumps({p:sha(p) for p in paths},indent=2));cmd=[PY,'-u',str(runner),'--job',str(case/'job.json')];log=(trial/'actor.log').open('w');start=time.time();p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(trial/'actor_process.json').write_text(json.dumps({'pid':p.pid,'actual_argv':cmd,'runner_sha256':sha(runner),'gpu':job['gpu'],'started_wall':start,'physical_attempt':1},indent=2));active[i]=(p,log,start,trial);print('PHYSICAL_START',i,p.pid,flush=True)
while active:
 for i,(p,log,start,trial) in list(active.items()):
  if (trial/'scene_preserved_after_result.json').exists():(trial/'close_completed_scene').touch()
  code=p.poll()
  if code is not None:
   log.close();cleanup={'pid':p.pid,'actor_exited':True,'returncode':code,'wall_s':time.time()-start,'automatic_retry':False};(trial/'actor_cleanup.json').write_text(json.dumps(cleanup,indent=2));done.append({'asset_id':i,'physical_attempts':1,'cleanup':cleanup,'report_available':(trial/'report.json').exists(),'export_available':(trial/'EXPORT_COMPLETE.json').exists()});del active[i];print('PHYSICAL_END',i,code,flush=True)
 (out/'BATCH_PROGRESS.json').write_text(json.dumps({'active_cases':list(active),'completed':done,'no_tuning':True,'no_automatic_retry':True},indent=2));time.sleep(1.)
changes=[p for p,h in freeze['hashes'].items() if sha(p)!=h];(out/'BATCH_COMPLETE.json').write_text(json.dumps({'completed':done,'batch_source_changes':changes,'physical_attempts':sum(x['physical_attempts'] for x in done),'no_replacement':True},indent=2));print('BATCH_COMPLETE',len(done),flush=True)
