"""Six pre-registered independent attempts; frozen skill, sequential ownership."""
from pathlib import Path
import argparse,json,os,fcntl,subprocess,time,hashlib
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/native_g1_heldout_20261013'
parser=argparse.ArgumentParser();parser.add_argument('--gpu-ownership',choices=['exclusive','shared-authorized'],default='shared-authorized');parser.add_argument('--freeze',type=Path,default=O/'PHYSICAL_BATCH_FREEZE.json');parser.add_argument('--resume-completed',action='store_true');parser.add_argument('--gpu',type=int,help='optional GPU override; otherwise select by free VRAM, not idleness');a=parser.parse_args()
freeze=json.loads(a.freeze.read_text());lock=open('/tmp/xarm7-native-isaac-owner.lock','a+');fcntl.flock(lock,fcntl.LOCK_EX)
env=dict(os.environ,PYTHONPATH='.:scripts',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1');env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env.get('PATH','')
ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg';probe=str(Path(ff).with_name('ffprobe'));test=O/'ENCODER_PREFLIGHT.mp4'
subprocess.run([ff,'-y','-f','lavfi','-i','color=c=black:s=64x64:r=30','-frames:v','10','-c:v','libx264',str(test)],stdout=subprocess.DEVNULL,stderr=(O/'encoder_preflight.log').open('w'),check=True)
(O/'ENCODER_PREFLIGHT.json').write_text(json.dumps({'not_physical_evidence':True,'probe':json.loads(subprocess.check_output([probe,'-v','error','-show_entries','stream=nb_frames,width,height','-of','json',str(test)])),'ffmpeg_sha256':hashlib.sha256(Path(ff).read_bytes()).hexdigest()}))
results=[]
for asset in freeze['physical_cases']:
 d=O/asset
 if a.resume_completed and (d/'SINGLE_ATTEMPT_RESERVED.json').exists():
  if not (d/'PROCESS_EXIT.json').exists():raise RuntimeError('RESUME_REQUIRES_PREVIOUS_ACTOR_EXIT')
  results.append(json.loads((d/'PROCESS_EXIT.json').read_text()));print('PRESERVE_ALREADY_ATTEMPTED_CASE_NO_RETRY',asset,flush=True);continue
 if asset not in freeze['case_input_hashes']:
  results.append({'asset_id':asset,'status':'PLANNING_FAILURE','physical_attempts':0,'physical_grasp':None,'actual_angle_deg':None});print('SKIP_REGISTERED_PLANNING_FAILURE',asset,flush=True);continue
 job=json.loads((d/'job.json').read_text());gpu=job['gpu'];allhashes=freeze['source_hashes']|freeze['case_input_hashes'][asset]
 changed=[p for p,h in allhashes.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
 if changed:raise RuntimeError('PRELAUNCH_FROZEN_INPUT_CHANGED:'+str(changed))
 # Operational GPU selection does not change manipulation timing or parameters.
 resources=subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free,memory.used,memory.total,utilization.gpu','--format=csv,noheader,nounits'],text=True)
 rows=[[int(v.strip()) for v in line.split(',')] for line in resources.splitlines()]
 gpu=a.gpu if a.gpu is not None else max(rows,key=lambda row:row[1])[0]
 if job['gpu']!=gpu:
  history=d/'GPU_JOB_HISTORY';history.mkdir(exist_ok=True)
  (history/('%d.json'%time.time_ns())).write_text(json.dumps(job,indent=2))
  job['gpu']=gpu;(d/'job.json').write_text(json.dumps(job,indent=2))
 uuid=subprocess.check_output(['nvidia-smi','--id='+str(gpu),'--query-gpu=uuid','--format=csv,noheader'],text=True).strip()
 apps=subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid,used_gpu_memory','--format=csv,noheader'],text=True)
 busy=[line for line in apps.splitlines() if line.startswith(uuid)]
 (d/'GPU_PRELAUNCH.csv').write_text(resources)
 (d/'GPU_OWNERSHIP.json').write_text(json.dumps({'mode':'capacity-based/shared-authorized','gpu':gpu,'other_compute_process_count':len(busy),'hardware_exclusive':not busy,'one_owned_actor':True,'selection':'largest free VRAM, utilization and other processes are not blockers','resources_csv':resources}))
 actors=subprocess.check_output(['ps','-u','rangeryx','-o','pid,args'],text=True);owned=[line for line in actors.splitlines() if 'scripts/run_xarm' in line or 'scripts/probe_xarm' in line]
 if owned:raise RuntimeError('PRELAUNCH_OWNED_ACTOR_ACTIVE:'+str(owned))
 with (d/'SINGLE_ATTEMPT_RESERVED.json').open('x') as f:json.dump({'time':time.time(),'maximum_attempts':1,'registered_asset':asset,'sequential':True},f)
 command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(R/'scripts/run_xarm7_native_transfer_repair.py'),'--job',str(d/'job.json')]
 with (d/'physical_01.log').open('w') as log:
  child=subprocess.Popen(command,cwd=R,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT);(d/'PROCESS.json').write_text(json.dumps({'pid':child.pid,'command':command,'entry_sha256':job['expected_runner_sha256'],'gpu':gpu,'previous_owned_actor_exited':True}));seen=None
  print('PHYSICAL_START',asset,child.pid,flush=True)
  while child.poll() is None:
   if (d/'physical_01/report.json').exists() and (d/'physical_01/EXPORT_COMPLETE.json').exists():
    if seen is None:seen=time.monotonic()
    if time.monotonic()-seen>45:
     cmd=Path('/proc/%d/cmdline'%child.pid).read_bytes().replace(b'\0',b' ').decode()
     if str(d/'job.json') in cmd and 'run_xarm7_native_transfer_repair.py' in cmd:
      child.terminate()
      try:child.wait(timeout=10)
      except subprocess.TimeoutExpired:child.kill();child.wait()
      (d/'FINISHED_ACTOR_CLEANUP.json').write_text(json.dumps({'pid':child.pid,'after_completed_export':True}))
   time.sleep(2)
  item={'asset_id':asset,'pid':child.pid,'returncode':child.returncode,'physical_attempts':1,'retry':False};(d/'PROCESS_EXIT.json').write_text(json.dumps(item));results.append(item);print('PHYSICAL_EXIT',json.dumps(item),flush=True)
(O/'BATCH_COMPLETE.json').write_text(json.dumps({'results':results,'registered_first_six_only':True,'retries':0,'reserved_four_not_executed':freeze['reserved_cases']},indent=2));print('BATCH_COMPLETE',flush=True)
