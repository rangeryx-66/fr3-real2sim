"""Own-process serialization and bounded native short tests; shared GPUs are allowed."""
import argparse,datetime,fcntl,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 os.umask(0o077)
 p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--test',choices=['free','fixture','session','guards'],default='free');p.add_argument('--cycles',type=int,default=4);p.add_argument('--gpu',type=int,default=3);a=p.parse_args()
 root=ROOT/'results/fast_runtime_v1_20261018';root.mkdir(exist_ok=True)
 with (root/'owned_actor.lock').open('a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  for proc in Path('/proc').iterdir():
   if not proc.name.isdigit():continue
   try:
    if proc.stat().st_uid!=os.getuid():continue
    argv=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
   except OSError:continue
   if proc.name!=str(os.getpid()) and ('scripts/run_fast_interaction_v1.py --job' in argv or 'scripts/run_legacy_runtime_short_control.py --job' in argv):raise RuntimeError('Owned actor already running: '+proc.name)
  out=root/a.label
  if (out/'PROCESS.json').exists():raise RuntimeError('Preserve previous test directory; select a new engineering label')
  out.mkdir(exist_ok=True);job=json.loads((ROOT/'results/agent_pull_push_v12_20261018/job.json').read_text())
  # Carry scene/perception inputs only; no legacy controller, generated-owner
  # or old force/timing policy is admitted into the new runtime job.
  keep={'source','asset_root','gpu','visual_gpu','skill','initial_articulation_rad','camera_calibration','wrist_calibration','robust_uncertainty_config','visual_prompt','visual_reobservations','visual_planning_wall_s','benchmark_goal_rad','authored_target_rad','robot_start_q','active_closure_source','expected_active_closure_sha256','private_physics_configuration','identification_case','ft_calibration','shared_execution'}
  sam=job['wrist_experiment']['sam3'];job={k:v for k,v in job.items() if k in keep};job['wrist_experiment']={'sam3':sam};job['runtime_policy']='configs/interaction_runtime_v1.json'
  job.update(output=str(out),gpu=a.gpu,engineering_fixture=str(root/'contact_fixture'),engineering_rgbd_reference=str(ROOT/'results/agent_pull_push_v12_20261018/episode/rgbd/generation_00_agent_01'))
  job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=1)).isoformat()
  (out/'job.json').write_text(json.dumps(job,indent=2))
  env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
  cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','scripts/run_fast_interaction_v1.py','--job',str(out/'job.json'),'--test',a.test,'--cycles',str(a.cycles)]
  start=time.perf_counter()
  with (out/'actor.log').open('wb') as log:
   child=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   (out/'PROCESS.json').write_text(json.dumps(dict(pid=child.pid,command=cmd),indent=2))
   try:rc=child.wait(timeout=900 if a.test!='free' else 300)
   except subprocess.TimeoutExpired:
    (out/'STOP_AND_EXPORT').touch()
    try:rc=child.wait(timeout=30)
    except subprocess.TimeoutExpired:
     child.terminate()
     try:rc=child.wait(timeout=15)
     except subprocess.TimeoutExpired:child.kill();rc=child.wait(timeout=10)
  (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_including_initialization_and_cleanup_s=time.perf_counter()-start,actor_exited=True,only_owned_process_managed=True),indent=2))
if __name__=='__main__':main()
