"""One owned native actor, no automatic retry or batch action policy."""
import argparse,fcntl,hashlib,json,os,subprocess,time
from pathlib import Path


def verify_registration(root,job):
    """Fail before scene creation if the registered implementation drifts.

    Engineering jobs have no registration. Research jobs must explicitly carry
    the commitment path and their immutable episode identifier. This reads only
    a digest of the private generator file, never its parameters.
    """
    if job.get('experiment')!='V7_BENCHMARK':return
    registration=Path(job['benchmark_registration']);data=json.loads(registration.read_text())
    matches=[e for e in data['episodes'] if e['episode_id']==job['registered_episode_id']]
    if len(matches)!=1:raise ValueError('Unregistered research episode')
    e=matches[0]
    if not e.get('execution_authorized',True):raise ValueError('Registered episode deferred: '+str(e.get('deferral_reason')))
    if e['asset_id']!=job['asset_id'] or e['configuration_index']!=job['identification_case']:
        raise ValueError('Registered object/configuration mapping changed')
    if any(job.get(k)!=v for k,v in data.get('resource_budget',{}).items()):raise ValueError('Registered finite resource budget changed')
    errors=[p for p,h in data['source_sha256'].items() if hashlib.sha256((root/p).read_bytes()).hexdigest()!=h]
    if errors:raise ValueError('Frozen source mismatch: '+', '.join(errors))
    if hashlib.sha256(Path(job['private_physics_configuration']).read_bytes()).hexdigest()!=data['hidden_configuration_sha256']:
        raise ValueError('Blind configuration commitment mismatch')


def main():
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path);p.add_argument('--check',action='store_true');p.add_argument('--breakaway-check',action='store_true');a=p.parse_args()
    root=Path(__file__).resolve().parents[1];job=json.loads(a.job.read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    verify_registration(root,job)
    if (out/'PROCESS.json').exists():raise RuntimeError('Already launched: preserve completed attempts; no implicit retry')
    lock=(root/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    free={int(line.split(',')[0]):int(line.split(',')[1]) for line in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()}
    if free[job['gpu']]<12000:raise RuntimeError('Insufficient free VRAM now; exclusive GPU is not required')
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    if a.breakaway_check and (a.check or job.get('engineering_mode')!='independent_breakaway' or job.get('experiment')=='V7_BENCHMARK'):
        raise ValueError('Independent force fixture must be explicitly engineering-only')
    entry='scripts/check_prismatic_breakaway_v7.py' if a.breakaway_check else 'scripts/run_cross_object_v7.py'
    command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',entry,'--job',str(a.job.resolve())]+(['--check'] if a.check else [])
    began=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        actor=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (out/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=command,free_VRAM_MiB=free,job_sha256=hashlib.sha256(a.job.read_bytes()).hexdigest(),initialization_is_not_a_physical_attempt=True),indent=2))
        deadline=float(job['episode_wall_budget_s'])+180
        try:rc=actor.wait(timeout=deadline)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=actor.wait(timeout=90)
            except subprocess.TimeoutExpired:
                actor.terminate()
                try:rc=actor.wait(timeout=30)
                except subprocess.TimeoutExpired:actor.kill();rc=actor.wait(timeout=10)
    (out/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True),indent=2))


if __name__=='__main__':main()
