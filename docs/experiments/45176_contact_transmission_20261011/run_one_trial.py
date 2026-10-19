"""Explicit executable, exclusive single-trial reservation; no retry loop."""
from pathlib import Path
import json,hashlib,subprocess,os,time
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45176_contact_transmission_20261011';runner=ROOT/'scripts/run_oracle_door_precontact_continuity.py';python='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
job=json.loads((out/'job.json').read_text());preflight=json.loads((out/'PREFLIGHT.json').read_text());infra=json.loads((out/'EXPORT_LIFECYCLE_VERIFIED.json').read_text())
if str(runner)!=preflight['runner_path'] or sha(runner)!=job['expected_runner_sha256'] or sha(Path(job['active_closure_source']))!=job['expected_active_closure_sha256']:raise RuntimeError('PRELAUNCH_EXECUTABLE_MISMATCH_NO_ISAAC')
if preflight['runner_sha256']!=sha(runner) or not preflight['offline_supported']:raise RuntimeError('PRELAUNCH_PROOF_MISMATCH_NO_ISAAC')
if not all(all(x['lossless'] for x in c['checks'].values()) and c['process_returncode']==0 for c in infra['cases']):raise RuntimeError('PRELAUNCH_EXPORT_NOT_VERIFIED_NO_ISAAC')

for name in ['whole_plan.json','grasp_plan.json']:
 if sha(out/name)!=preflight['source_config_hashes'][str(out/name)]:raise RuntimeError('PRELAUNCH_PLAN_CHANGED_NO_ISAAC')
trial=Path(job['output']);trial.mkdir(exist_ok=True)
with (out/'PHYSICAL_TRIAL_RESERVED.json').open('x') as f:json.dump({'asset_id':'45176','attempt':1,'maximum':1,'no_retry':True,'reserved_wall':time.time(),'runner':str(runner),'runner_sha256':sha(runner)},f,indent=2)
paths=[runner,Path(job['active_closure_source']),out/'job.json',out/'whole_plan.json',out/'grasp_plan.json',Path(job['source'])/'report.json',Path(job['asset_root'])/'manifest.json',Path(job['baseline_runner']),Path(job['pink_runner']),ROOT/'scripts/plan_contact_transmission_door.py',ROOT/'scripts/handle_contact_quality.py',ROOT/'scripts/precontact_trajectory.py',ROOT/'scripts/piper_pink_fixed.py',ROOT/'articulated_interaction/control.py',ROOT/'articulated_interaction/physical_baseline.py',ROOT/'config/piper.urdf',ROOT/'config/semantic_interaction.json',ROOT/'wrist_reconstruction/force_policy.py']
(trial/'source_config_hashes.json').write_text(json.dumps({str(p):sha(p) for p in paths},indent=2))
env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1');env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
cmd=[python,'-u',str(runner),'--job',str(out/'job.json')];start=time.time();log=(trial/'actor.log').open('w');process=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(trial/'actor_process.json').write_text(json.dumps({'pid':process.pid,'started_wall':start,'gpu':job['gpu'],'actual_argv':cmd,'runner_sha256':sha(runner),'physical_attempt':1},indent=2));print('ONE_PHYSICAL_START',process.pid,cmd,flush=True)
while process.poll() is None:
 if (trial/'scene_preserved_after_result.json').exists():
  (trial/'owner_cleanup_request.json').write_text(json.dumps({'pid':process.pid,'after_report_and_export':True,'wall':time.time()},indent=2));(trial/'close_completed_scene').touch()
 time.sleep(.5)
log.close();(trial/'actor_cleanup.json').write_text(json.dumps({'pid':process.pid,'actor_exited':True,'returncode':process.returncode,'wall_s':time.time()-start,'automatic_retry':False},indent=2));print('ONE_PHYSICAL_END',process.returncode,flush=True)
