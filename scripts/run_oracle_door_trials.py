"""Owner for at most one initial normal-closed physical trial per registered object."""
import json,hashlib,subprocess,os,time,datetime,concurrent.futures
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'results/oracle_door_development_20261011';PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

def execute(i):
 case=out/i;planning=case/'planning';job=json.loads((case/'job.json').read_text())
 if not (planning/'candidate.json').exists():print('PLANNING_FAILURE_NO_PHYSICAL_ATTEMPT',i,flush=True);return
 c=json.loads((planning/'candidate.json').read_text());meta_path=Path(job['asset_root'])/'manifest.json';meta=json.loads(meta_path.read_text());meta['interaction_geometry']['selection']=c['grasp_geometry_selection'];meta['door_link']=c['grasp_geometry_selection']['handle_link'];meta_path.write_text(json.dumps(meta,indent=2))
 source=Path(job['source'])/'report.json';s=json.loads(source.read_text());s['robot_base_pose']=c['base'];source.write_text(json.dumps(s,indent=2))
 (planning/'whole_plan.json').write_text(json.dumps(c,indent=2));(planning/'grasp_plan.json').write_text(json.dumps({'trial_candidates':[{'base':c['base'],'T':c['T_grasp'],'preplan':c['preplan'],'approach':c['approach']}]},indent=2))
 trial=case/'development_01';trial.mkdir(exist_ok=True)
 with (trial/'TRIAL_RESERVED.json').open('x') as f:json.dump({'asset_id':i,'trial':1,'classification':'ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT','normal_closed_start':True,'reserved_wall':time.time()},f,indent=2)
 job['output']=str(trial);job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=2)).isoformat();jf=trial/'job.json';jf.write_text(json.dumps(job,indent=2))
 paths=[jf,source,meta_path,planning/'whole_plan.json',planning/'grasp_plan.json',ROOT/'scripts/run_oracle_door.py',ROOT/'scripts/plan_oracle_door.py',ROOT/'scripts/run_benchmark_a30.py',ROOT/'scripts/run_revolute10_fixed.py',ROOT/'scripts/piper_pink_fixed.py',Path(job['baseline_runner']),Path(job['pink_runner'])]
 (trial/'source_config_hashes.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},indent=2))
 env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1');env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
 start=time.time();log=(trial/'actor.log').open('w');p=subprocess.Popen([PY,'-u',str(ROOT/'scripts/run_oracle_door.py'),'--job',str(jf)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 (trial/'actor_process.json').write_text(json.dumps({'pid':p.pid,'started_wall':start,'gpu':job['gpu']},indent=2));print('PHYSICAL_START',i,p.pid,flush=True)
 while p.poll() is None:
  if (trial/'scene_preserved_after_result.json').exists():
   (trial/'owner_cleanup_request.json').write_text(json.dumps({'pid':p.pid,'after_export':True,'wall':time.time()},indent=2));(trial/'close_completed_scene').touch()
  time.sleep(.5)
 log.close();(trial/'actor_cleanup.json').write_text(json.dumps({'pid':p.pid,'actor_exited':True,'returncode':p.returncode,'wall_s':time.time()-start},indent=2));print('PHYSICAL_END',i,p.returncode,flush=True)

if __name__=='__main__':
 # Initial batch only. This file has no re-test loop.
 (out/'PHYSICAL_BATCH_START.json').write_text(json.dumps({'pid':os.getpid(),'wall':time.time(),'maximum_initial_trials':3,'additional_retests_not_automatic':True},indent=2))
 with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
  for future in [pool.submit(execute,i) for i in ['45176','45203','45415']]:future.result()
 (out/'PHYSICAL_BATCH_COMPLETE.json').write_text(json.dumps({'wall':time.time(),'initial_trials_complete':True},indent=2))
