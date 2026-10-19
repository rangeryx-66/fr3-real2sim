"""No physical trials: a fixture export test and an empty-Isaac SIGINT test."""
from pathlib import Path
import json,subprocess,os,time,signal
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45415_precontact_continuity_20261011';runner=ROOT/'scripts/run_oracle_door_precontact_continuity.py';job=json.loads((out/'job.json').read_text());env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'));env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH'];python='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python';fixture=ROOT/'results/oracle_door_development_20261011/45415/development_02'
results=[]
for name,empty,mechanism in [('infra_marker_python',False,'STOP_AND_EXPORT'),('infra_sigint_python',False,'SIGINT')]:
 folder=out/name;folder.mkdir(exist_ok=True);cmd=[python,'-u',str(runner),'--lifecycle-selftest',str(folder),'--fixture',str(fixture),'--baseline',job['baseline_runner'],'--gpu','3']
 if empty:cmd.append('--isaac-empty')
 started=time.time();log=(folder/'process.log').open('w');p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(folder/'process.json').write_text(json.dumps({'pid':p.pid,'cmd':cmd,'started_wall':started,'physical_trials':0},indent=2));print('INFRA_START',name,p.pid,flush=True)
 while not (folder/'SELFTEST_READY.json').exists() and p.poll() is None:
  if time.time()-started>180:raise RuntimeError('INFRA_STARTUP_TIMEOUT_NO_PHYSICAL_TRIAL')
  time.sleep(.25)
 if p.poll() is not None:raise RuntimeError('INFRA_EXIT_BEFORE_READY:'+name)
 if mechanism=='SIGINT':os.kill(p.pid,signal.SIGINT)
 else:(folder/'STOP_AND_EXPORT').touch()
 code=p.wait(timeout=120);log.close();result=json.loads((folder/'SELFTEST_RESULT.json').read_text());result.update(process_returncode=code,owner_mechanism=mechanism,wall_s=time.time()-started);(folder/'verified_result.json').write_text(json.dumps(result,indent=2));results.append(result)
 if code!=0 or not result['pause_called'] or result['video_returncode']!=0 or not all(x['lossless'] for x in result['checks'].values()):raise RuntimeError('INFRA_SELFTEST_FAILED_NO_PHYSICAL_TRIAL')
 print('INFRA_PASS',name,json.dumps(result),flush=True)
(out/'EXPORT_LIFECYCLE_VERIFIED.json').write_text(json.dumps({'physical_trials_consumed':0,'cases':results},indent=2))
