"""Launch the third registered case once, after the current actor has exited."""
import os,time,json,subprocess,hashlib,signal
from pathlib import Path
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/xarm7_native_gripper_20261012';first=1470593;third='45249'
def alive(pid):
 try:
  state=Path('/proc/%d/stat'%pid).read_text().split()[2]
  return state!='Z'
 except FileNotFoundError:return False
while alive(first):
 if (O/'45134/physical_01/report.json').exists():
  finished=time.time()
  while alive(first) and time.time()-finished<45:time.sleep(1)
  if alive(first):
   cmd=Path('/proc/%d/cmdline'%first).read_bytes().replace(b'\x00',b' ').decode()
   if 'run_xarm7_native_door.py' in cmd and '/45134/job.json' in cmd:
    os.kill(first,signal.SIGTERM);time.sleep(10)
    if alive(first):os.kill(first,signal.SIGKILL)
    (O/'45134/FINISHED_ACTOR_CLEANUP.json').write_text(json.dumps({'pid':first,'command':cmd,'after_report_and_pause_export':True}))
  break
 time.sleep(2)
f=json.loads((O/'CONTACT_BATCH_FINAL_INPUTS_FREEZE.json').read_text());changed=[s for s,h in f['sources'].items() if hashlib.sha256(Path(s).read_bytes()).hexdigest()!=h]
if changed:
 (O/'45249/PRELAUNCH_SOURCE_CHANGE.json').write_text(json.dumps(changed));raise SystemExit('changed source; no scene launched')
reservation=O/'45249/SINGLE_ATTEMPT_RESERVED.json'
with reservation.open('x') as out:json.dump({'one_attempt_only':True,'previous_45134_actor_exited':not alive(first),'time':time.time()},out)
env=os.environ.copy();env.update(PYTHONPATH='.:scripts',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env.get('PATH',''))
command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','scripts/run_xarm7_native_door.py','--job',str(O/'45249/job.json')]
with (O/'45249/physical_01.log').open('w') as log:
 p=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL)
 (O/'45249/PROCESS.json').write_text(json.dumps({'pid':p.pid,'command':command,'shared_source_changes':changed,'no_automatic_retries':True},indent=2));code=p.wait()
 (O/'45249/PROCESS_EXIT.json').write_text(json.dumps({'pid':p.pid,'returncode':code}))
(O/'REGISTERED_BATCH_CLOSED.json').write_text(json.dumps({'contact_launches':{'45134':1470593,'45443':2012887,'45249':p.pid},'automatic_retries':0,'returncode_last':code}))
