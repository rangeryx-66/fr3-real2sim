"""Sequential engineering-only launcher; same owned lock and free-VRAM policy."""
import argparse,fcntl,hashlib,json,os,subprocess,time
from pathlib import Path


def main(runner='scripts/run_prismatic_transmission_v7.py',registered=False,joint_type='prismatic'):
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args()
    root=Path(__file__).resolve().parents[1];job=json.loads(a.job.read_text())
    if registered:
        from launch_job_v7 import verify_registration
        entries={'prismatic':'scripts/run_registered_prismatic_v71.py','revolute':'scripts/run_registered_revolute_v71.py'}
        if job.get('experiment')!='V7_BENCHMARK' or runner!=entries.get(joint_type):
            raise ValueError('Explicit registered joint-specific entry required')
        verify_registration(root,job)
    elif job.get('experiment')!='V7' or job.get('run_classification')!='PRISMATIC_ENGINEERING' or job.get('benchmark_registration'):
        raise ValueError('Only explicitly labeled prismatic engineering jobs')
    if job['articulation_v7']['joint_type']!=joint_type or (not registered and joint_type!='prismatic'):
        raise ValueError('Wrong joint-specific launcher')
    m=json.loads((root/'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_FREEZE.json').read_text())
    bad=[n for n,h in m['source_sha256'].items() if hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]
    if bad:raise ValueError('Frozen revolute drift: '+str(bad))
    if registered:
        external=json.loads((root/'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_EXTERNAL_FREEZE.json').read_text())
        bad=[n for n,h in external['external_robot_camera_closure_uncertainty_sha256'].items() if hashlib.sha256(Path(n).read_bytes()).hexdigest()!=h]
        if bad:raise ValueError('Frozen external model/calibration drift: '+str(bad))
    out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    if (out/'PROCESS.json').exists():raise ValueError('Existing run cannot be silently retried')
    lock=(root/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    free={int(r.split(',')[0]):int(r.split(',')[1]) for r in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()}
    if free[job['gpu']]<12000:raise ValueError('Insufficient current free VRAM; no exclusivity required')
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    if runner not in ('scripts/run_prismatic_transmission_v7.py','scripts/run_prismatic_preload_v71.py','scripts/run_registered_prismatic_v71.py','scripts/run_registered_revolute_v71.py'):
        raise ValueError('Unregistered prismatic engineering entry')
    command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',runner,'--job',str(a.job.resolve())]
    began=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        actor=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (out/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=command,free_VRAM_MiB=free,job_sha256=hashlib.sha256(a.job.read_bytes()).hexdigest(),engineering_only=not registered,initialization_is_not_a_physical_attempt=True),indent=2))
        try:rc=actor.wait(timeout=float(job['episode_wall_budget_s'])+180)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=actor.wait(timeout=90)
            except subprocess.TimeoutExpired:
                actor.terminate()
                try:rc=actor.wait(timeout=30)
                except subprocess.TimeoutExpired:actor.kill();rc=actor.wait(timeout=10)
    (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True),indent=2))
if __name__=='__main__':main()
