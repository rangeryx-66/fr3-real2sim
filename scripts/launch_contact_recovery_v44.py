"""One serialized short native engineering chain; no independent research run."""
import argparse,datetime,fcntl,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--gpu',type=int,default=6);parser.add_argument('--visual-gpu',type=int,default=5);parser.add_argument('--label',default='native_00');a=parser.parse_args()
    lock=(ROOT/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=ROOT/'results/contact_recovery_v44_20261019'/a.label;out.mkdir(parents=True,exist_ok=False)
    old=json.loads((ROOT/'results/independent_probing_v43_20261019/episode_00/job.json').read_text())
    job=dict(old,output=str(out),gpu=a.gpu,visual_gpu=a.visual_gpu,engineering_fixture=str(ROOT/'results/contact_recovery_v44_20261019/initial_fixture'),
             research_reference='169263f',runtime_acquisition_budget=2,agent_wait_timeout_s=360,episode_wall_budget_s=1800.,episode_sim_budget_s=900.,safe_exit_wall_reserve_s=180.,safe_exit_sim_reserve_s=60.,
             classification='V4.4_NATIVE_CONTACT_ENGINEERING; archived initial failure fixture, fresh Agent recovery; not blind research validation')
    job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(seconds=1980)).isoformat();(out/'job.json').write_text(json.dumps(job,indent=2))
    sources=list((ROOT/'interaction_runtime').glob('*.py'))+[ROOT/'scripts'/n for n in ['native_g1_execution_candidates.py','native_g1_agent_execution.py','native_g1_hybrid_contact.py','native_g1_panel_support.py','plan_shared_door_skill.py','xarm7_native_planning.py','native_g1_visual_input.py','run_contact_recovery_v44.py','native_g1_wrist_input.py','update_probing_v31_model.py','xarm_ft_v31_estimator.py']]
    (out/'REGISTERED_SOURCE.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2))
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','scripts/run_contact_recovery_v44.py','--job',str(out/'job.json')];began=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(out/'PROCESS.json').write_text(json.dumps(dict(pid=child.pid,command=cmd),indent=2))
        try:rc=child.wait(timeout=1980)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=child.wait(timeout=60)
            except subprocess.TimeoutExpired:child.terminate();rc=child.wait(timeout=30)
    (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True),indent=2))

if __name__=='__main__':main()
