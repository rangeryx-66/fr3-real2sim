"""Single registered episode; no automatic physical retries or backup archives."""
import argparse,datetime,fcntl,json,os,subprocess,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
def main():
    os.umask(0o077);p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--gpu',type=int,default=3);p.add_argument('--visual-gpu',type=int,default=5);p.add_argument('--check',action='store_true');a=p.parse_args()
    lock=(ROOT/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=ROOT/'results/efficient_probing_v2_20261018'/a.label
    if out.exists():raise RuntimeError('Preserve existing episode; no overwrite or automatic retry')
    out.mkdir(parents=True);config=json.loads((ROOT/'configs/efficient_probing_v2.json').read_text());old=json.loads((ROOT/'results/agent_pull_push_v12_20261018/job.json').read_text())
    keep={'source','asset_root','skill','camera_calibration','wrist_calibration','robust_uncertainty_config','visual_prompt','visual_reobservations','visual_planning_wall_s','benchmark_goal_rad','authored_target_rad','robot_start_q','active_closure_source','expected_active_closure_sha256','private_physics_configuration','identification_case','ft_calibration','shared_execution'}
    job={k:v for k,v in old.items() if k in keep};job.update(output=str(out),gpu=a.gpu,visual_gpu=a.visual_gpu,wrist_experiment={'sam3':old['wrist_experiment']['sam3']},runtime_acquisition_budget=2)
    job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(seconds=config['episode_wall_limit_s'])).isoformat();(out/'job.json').write_text(json.dumps(job,indent=2))
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','scripts/run_efficient_probing_v2.py','--job',str(out/'job.json')]
    if a.check:cmd.append('--check')
    start=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        child=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(out/'PROCESS.json').write_text(json.dumps(dict(pid=child.pid,command=cmd),indent=2))
        try:rc=child.wait(timeout=config['episode_wall_limit_s'])
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=child.wait(timeout=45)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:rc=child.wait(timeout=30)
                except subprocess.TimeoutExpired:child.kill();rc=child.wait(timeout=10)
    (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_including_init_cleanup_s=time.perf_counter()-start,actor_exited=True),indent=2))
    from interaction_runtime.compact_v2 import storage
    (out/'STORAGE_SUMMARY.json').write_text(json.dumps(storage(out),indent=2))
if __name__=='__main__':main()
