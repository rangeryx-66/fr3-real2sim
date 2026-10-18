"""One blind V4.3 research episode; frozen V4.2 runtime and V3.1 estimator."""
import argparse,datetime,fcntl,hashlib,json,os,secrets,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def prepare(root):
    import numpy as np
    from xarm_ft_id_scene import PRIORS
    root.mkdir(parents=True,exist_ok=False);seed=secrets.randbits(32);rng=np.random.default_rng(seed)
    case=dict(case=0,asset='45936',seed=seed,mass_scale=float(rng.uniform(*PRIORS['moving_mass_scale'])),density_gradient=rng.uniform(*PRIORS['density_gradient'],3).tolist(),
        tau_c=float(rng.uniform(*PRIORS['coulomb_nm'])),b=float(rng.uniform(*PRIORS['viscous_nm_s_rad'])),spring=False)
    case['law']=dict(family='constant',base_nm=case['tau_c'],amplitude_nm=0.,center_rad=0.,width_rad=1.)
    private=root/'HIDDEN_CONFIGURATIONS.json';private.write_text(json.dumps([case],indent=2));private.chmod(0o400)
    # Only a commitment is returned. No true dynamics in Agent/worker packets.
    return dict(asset='45936',cases=1,configuration_sha256=hashlib.sha256(private.read_bytes()).hexdigest(),family=PRIORS,truth_reveal='only after actor exit and all Agent decisions finish')


def main():
    os.umask(0o077);p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--prepare',action='store_true');p.add_argument('--gpu',type=int,default=3);p.add_argument('--visual-gpu',type=int,default=5);a=p.parse_args()
    if a.prepare:
        public=prepare(a.root);(a.root/'CONFIGURATION_COMMITMENT.json').write_text(json.dumps(public,indent=2));print(json.dumps(public));return
    commitment=json.loads((a.root/'CONFIGURATION_COMMITMENT.json').read_text())
    if hashlib.sha256((a.root/'HIDDEN_CONFIGURATIONS.json').read_bytes()).hexdigest()!=commitment['configuration_sha256']:raise RuntimeError('Frozen blind configuration commitment mismatch')
    lock=(ROOT/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=a.root/'episode_00'
    if out.exists():raise RuntimeError('Episode root already exists; inspect any infrastructure failure explicitly, never auto-retry')
    out.mkdir(parents=True);old=json.loads((ROOT/'results/agent_pull_push_v12_20261018/job.json').read_text())
    keep={'source','asset_root','skill','camera_calibration','wrist_calibration','robust_uncertainty_config','visual_prompt','visual_reobservations','visual_planning_wall_s','benchmark_goal_rad','authored_target_rad','robot_start_q','active_closure_source','expected_active_closure_sha256','ft_calibration','shared_execution'}
    job={k:v for k,v in old.items() if k in keep};job.update(output=str(out),gpu=a.gpu,visual_gpu=a.visual_gpu,wrist_experiment={'sam3':old['wrist_experiment']['sam3']},runtime_acquisition_budget=2,
        private_physics_configuration=str(a.root/'HIDDEN_CONFIGURATIONS.json'),identification_case=0,research_reference='5fa0f98',storage_target_bytes=100000000,agent_wait_timeout_s=300,allow_interrupted_fit=True,
        episode_wall_budget_s=3000.,episode_sim_budget_s=1500.,safe_exit_wall_reserve_s=300.,safe_exit_sim_reserve_s=90.,
        scientific_blinding='scene-generation-only truth; Agent reads only observation/plan/contact/model packets; never private job, seed or authored audits before exit')
    deadline_s=3300 # 50-min runtime includes 5-min guarded exit; owner adds 5 min export/shutdown allowance.
    job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(seconds=deadline_s)).isoformat();(out/'job.json').write_text(json.dumps(job,indent=2))
    sources=['interaction_runtime/segments.py','interaction_runtime/checkpoints.py','scripts/native_g1_wrist_input.py','interaction_runtime/progress.py','interaction_runtime/segment_quality.py','scripts/agent_command_protocol_v12.py','scripts/run_informative_probing_v31.py','scripts/update_probing_v31_model.py','scripts/xarm_ft_v31_estimator.py','interaction_runtime/estimator_v31.py','interaction_runtime/runtime.py','interaction_runtime/probing_v2.py','interaction_runtime/probing_v3.py','interaction_runtime/operations.py','configs/interaction_runtime_v1.json',str(Path(__file__).relative_to(ROOT))]
    (a.root/'FROZEN_SOURCE.json').write_text(json.dumps(dict(reference='5fa0f98',sources={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in sources}),indent=2))
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','scripts/run_informative_probing_v31.py','--job',str(out/'job.json')];began=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        child=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(out/'PROCESS.json').write_text(json.dumps(dict(pid=child.pid,command=cmd),indent=2))
        try:rc=child.wait(timeout=deadline_s)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=child.wait(timeout=90)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:rc=child.wait(timeout=30)
                except subprocess.TimeoutExpired:child.kill();rc=child.wait(timeout=10)
    (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_including_init_cleanup_s=time.perf_counter()-began,actor_exited=True),indent=2))
    from interaction_runtime.compact_v2 import storage
    (out/'STORAGE_SUMMARY.json').write_text(json.dumps(storage(out),indent=2))


if __name__=='__main__':main()
