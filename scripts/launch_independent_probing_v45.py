"""Single blind V4.5 registration/launch; reuse the frozen V4.4 research entry."""
import argparse,datetime,fcntl,hashlib,json,os,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
ENTRY='scripts/run_informative_probing_v44.py'


def main():
    os.umask(0o077);p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--prepare',action='store_true');p.add_argument('--gpu',type=int,default=6);p.add_argument('--visual-gpu',type=int,default=5);a=p.parse_args()
    if a.prepare:
        # Existing positive-density mass/inertia and passive-resistance family.
        # This isolated generator prints the commitment/priors, never its seed/truth.
        from launch_independent_probing_v43 import prepare
        public=prepare(a.root);public.update(experiment='V4.5',reference='bcf5dda',entry=ENTRY)
        (a.root/'CONFIGURATION_COMMITMENT.json').write_text(json.dumps(public,indent=2));print(json.dumps(public));return
    registration=json.loads((ROOT/'configs/probing_v45_registration.json').read_text())
    for f,h in registration['frozen_sources'].items():
        if hashlib.sha256((ROOT/f).read_bytes()).hexdigest()!=h:raise RuntimeError('Frozen implementation mismatch: '+f)
    commitment=json.loads((a.root/'CONFIGURATION_COMMITMENT.json').read_text())
    if hashlib.sha256((a.root/'HIDDEN_CONFIGURATIONS.json').read_bytes()).hexdigest()!=commitment['configuration_sha256']:raise RuntimeError('Blind configuration commitment mismatch')
    lock=(ROOT/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=a.root/'episode_00';out.mkdir(parents=True,exist_ok=False) # no automatic valid-episode retry
    old=json.loads((ROOT/'results/agent_pull_push_v12_20261018/job.json').read_text())
    keep={'source','asset_root','skill','camera_calibration','wrist_calibration','robust_uncertainty_config','visual_prompt','visual_reobservations','visual_planning_wall_s','benchmark_goal_rad','authored_target_rad','robot_start_q','active_closure_source','expected_active_closure_sha256','ft_calibration','shared_execution'}
    job={k:v for k,v in old.items() if k in keep}
    job.update(output=str(out),gpu=a.gpu,visual_gpu=a.visual_gpu,wrist_experiment={'sam3':old['wrist_experiment']['sam3']},runtime_acquisition_budget=2,
        private_physics_configuration=str(a.root/'HIDDEN_CONFIGURATIONS.json'),identification_case=0,research_reference='bcf5dda',experiment='V4.5',
        storage_target_bytes=100000000,agent_wait_timeout_s=360,allow_interrupted_fit=True,
        episode_wall_budget_s=3000.,episode_sim_budget_s=1500.,safe_exit_wall_reserve_s=300.,safe_exit_sim_reserve_s=90.,
        scientific_blinding='scene-generation-only truth; no hidden seed/dynamics/authored audit in Agent or estimator inputs until actor and online decisions finish')
    assert not any(k in job for k in ['engineering_fixture','saved_grasp','T_grasp','saved_viewpoints'])
    deadline_s=3300
    job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(seconds=deadline_s)).isoformat()
    (out/'job.json').write_text(json.dumps(job,indent=2))
    frozen=dict(reference='bcf5dda',entry=ENTRY,sources=dict(registration['frozen_sources']))
    for f in [str(Path(__file__).relative_to(ROOT)),'configs/probing_v45_registration.json','scripts/watch_probing_v43_estimates.py']:
        frozen['sources'][f]=hashlib.sha256((ROOT/f).read_bytes()).hexdigest()
    (a.root/'FROZEN_SOURCE.json').write_text(json.dumps(frozen,indent=2))
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',ENTRY,'--job',str(out/'job.json')];began=time.perf_counter()
    with (out/'actor.log').open('wb') as log,(out/'estimate_capture.log').open('wb') as capture_log:
        actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (out/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=command),indent=2))
        capture=subprocess.Popen([command[0],'scripts/watch_probing_v43_estimates.py',str(out)],cwd=ROOT,env=env,stdout=capture_log,stderr=subprocess.STDOUT)
        try:rc=actor.wait(timeout=deadline_s)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=actor.wait(timeout=90)
            except subprocess.TimeoutExpired:
                actor.terminate()
                try:rc=actor.wait(timeout=30)
                except subprocess.TimeoutExpired:actor.kill();rc=actor.wait(timeout=10)
        (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_including_init_cleanup_s=time.perf_counter()-began,actor_exited=True),indent=2))
        try:capture.wait(timeout=15)
        except subprocess.TimeoutExpired:capture.terminate();capture.wait(timeout=10)
    from interaction_runtime.compact_v2 import storage
    (out/'STORAGE_SUMMARY.json').write_text(json.dumps(storage(out),indent=2))

if __name__=='__main__':main()
