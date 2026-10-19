"""One serialized short contact regression, with finite exit-reserved resources."""
import datetime,fcntl,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--gpu',type=int,default=6);parser.add_argument('--label',default='native_contact_00');parser.add_argument('--guard',action='store_true');parser.add_argument('--normal',action='store_true');a=parser.parse_args()
    lock=(ROOT/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=ROOT/'results/adaptive_probing_v42_20261019'/a.label;out.mkdir(parents=True,exist_ok=False)
    old=json.loads((ROOT/'results/agent_pull_push_v12_20261018/job.json').read_text());keep={'source','asset_root','skill','camera_calibration','wrist_calibration','robust_uncertainty_config','robot_start_q','active_closure_source','expected_active_closure_sha256','ft_calibration','shared_execution','visual_prompt','visual_reobservations','visual_planning_wall_s','benchmark_goal_rad','authored_target_rad','private_physics_configuration','identification_case'}
    job={k:v for k,v in old.items() if k in keep};job.update(engineering_normal_profile=a.normal,output=str(out),gpu=a.gpu,visual_gpu=a.gpu,wrist_experiment={'sam3':old['wrist_experiment']['sam3']},research_reference='a2ea768',allow_interrupted_fit=True,episode_wall_budget_s=1200.,episode_sim_budget_s=600.,safe_exit_wall_reserve_s=120.,safe_exit_sim_reserve_s=60.)
    job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(seconds=1200)).isoformat();(out/'job.json').write_text(json.dumps(job,indent=2))
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',('scripts/run_adaptive_guard_v42.py' if a.guard else 'scripts/run_adaptive_contact_v42.py'),'--job',str(out/'job.json')];began=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(out/'PROCESS.json').write_text(json.dumps(dict(pid=child.pid,command=cmd),indent=2))
        try:rc=child.wait(timeout=1230)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=child.wait(timeout=45)
            except subprocess.TimeoutExpired:child.terminate();rc=child.wait(timeout=30)
    (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True),indent=2))
if __name__=='__main__':main()
