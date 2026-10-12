"""Two registered single attempts; exclusive owner lock, no retries."""
from pathlib import Path
import json,os,fcntl,subprocess,time,hashlib
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/xarm7_native_transfer_repair_20261012'
lock=open('/tmp/xarm7-native-isaac-owner.lock','a+');fcntl.flock(lock,fcntl.LOCK_EX);freeze=json.loads((O/'PHYSICAL_BATCH_FREEZE.json').read_text());env=os.environ.copy();env.update(PYTHONPATH='.:scripts',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env.get('PATH',''))
ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg';fp=str(Path(ff).with_name('ffprobe'))
fixture=O/'ENCODER_PREFLIGHT.mp4';subprocess.run([ff,'-y','-f','lavfi','-i','color=c=black:s=64x64:r=30','-frames:v','10','-c:v','libx264',str(fixture)],stdout=subprocess.DEVNULL,stderr=(O/'encoder_preflight.log').open('w'),check=True);probe=json.loads(subprocess.check_output([fp,'-v','error','-show_entries','stream=nb_frames,width,height','-of','json',str(fixture)]));(O/'ENCODER_PREFLIGHT.json').write_text(json.dumps({'video_probe':probe,'not_physical_evidence':True,'ffmpeg_sha256':hashlib.sha256(Path(ff).read_bytes()).hexdigest()}))
exits=[]
for asset in freeze['physical_cases']:
 d=O/asset;changed=[p for p,h in freeze['source_hashes'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
 if changed:raise RuntimeError('PRELAUNCH_FROZEN_SOURCE_CHANGED:'+str(changed))
 inventory=subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.used,memory.total,utilization.gpu','--format=csv,noheader'],text=True);(d/'GPU_PRELAUNCH.csv').write_text(inventory)
 # No other owned actor may overlap. Unrelated jobs are never killed.
 actors=subprocess.check_output(['ps','-u','rangeryx','-o','pid,args'],text=True);owned=[s for s in actors.splitlines() if 'scripts/run_xarm7' in s or 'scripts/probe_xarm7' in s]
 if owned:raise RuntimeError('PRELAUNCH_OWNED_ACTOR_ALREADY_ACTIVE:'+str(owned))
 with (d/'SINGLE_ATTEMPT_RESERVED.json').open('x') as out:json.dump({'time':time.time(),'sequential':True,'maximum_attempts':1},out)
 command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts/run_xarm7_native_transfer_repair.py'),'--job',str(d/'job.json')]
 with (d/'physical_01.log').open('w') as log:
  child=subprocess.Popen(command,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
  (d/'PROCESS.json').write_text(json.dumps({'pid':child.pid,'command':command,'runner_sha256':freeze['source_hashes'][str(ROOT/'scripts/run_xarm7_native_transfer_repair.py')],'gpu':4,'previous_actor_exited':True},indent=2));finished_seen=None
  while child.poll() is None:
   complete=(d/'physical_01/report.json').exists() and (d/'physical_01/EXPORT_COMPLETE.json').exists()
   if complete:
    if finished_seen is None:finished_seen=time.monotonic()
    if time.monotonic()-finished_seen>45:
     cmd=Path('/proc/%d/cmdline'%child.pid).read_bytes().replace(b'\x00',b' ').decode()
     if str(d/'job.json') in cmd and 'run_xarm7_native_transfer_repair.py' in cmd:
      child.terminate()
      try:child.wait(timeout=10)
      except subprocess.TimeoutExpired:child.kill();child.wait()
      (d/'FINISHED_ACTOR_CLEANUP.json').write_text(json.dumps({'pid':child.pid,'command':cmd,'after_completed_report_and_export':True}))
   time.sleep(2)
  code=child.returncode
  result={'asset':asset,'pid':child.pid,'returncode':code,'automatic_retry':False};(d/'PROCESS_EXIT.json').write_text(json.dumps(result));exits.append(result)
(O/'BATCH_COMPLETE.json').write_text(json.dumps({'results':exits,'no_concurrent_owned_actors':True,'retries':0},indent=2))
