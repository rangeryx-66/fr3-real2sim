"""Single-slot sequential launcher; no automatic action policy or physical retry."""
import argparse,fcntl,json,os,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from prismatic_registration_v74 import verify_job,sha,read
from interaction_runtime.prismatic_v73 import audit_frozen
from agent_command_protocol_v12 import atomic_json

def main():
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args();job=read(a.job)
    m,e=verify_job(ROOT,job);frozen=audit_frozen(ROOT);out=Path(job['output']);batch=out.parent
    for prior in m['episodes'][:e['configuration_index']]:
        d=batch/prior['episode_id']
        if not (d/'ONLINE_SEAL.json').exists() or not read(d/'PROCESS_COMPLETION.json').get('actor_exited'):
            raise ValueError('Previous registered episode not sealed/exited:'+prior['episode_id'])
    out.mkdir(parents=True,exist_ok=True)
    if (out/'PROCESS.json').exists():raise ValueError('Registered episode already launched; never automatically retry')
    lock=(ROOT/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    free={int(x.split(',')[0]):int(x.split(',')[1]) for x in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()}
    if free[job['gpu']]<12000:raise ValueError('Insufficient free VRAM; shared GPUs permitted')
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'))
    cmd=[sys.executable,'-u','scripts/run_registered_prismatic_v74.py','--job',str(a.job.resolve())];began=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        actor=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        atomic_json(out/'PROCESS.json',dict(pid=actor.pid,command=cmd,free_VRAM_MiB=free,job_sha256=sha(a.job),registered_episode_id=e['episode_id'],engineering_only=False,frozen_revolute=frozen))
        try:rc=actor.wait(timeout=float(job['episode_wall_budget_s'])+180)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=actor.wait(timeout=90)
            except subprocess.TimeoutExpired:
                actor.terminate()
                try:rc=actor.wait(timeout=30)
                except subprocess.TimeoutExpired:actor.kill();rc=actor.wait(timeout=10)
    atomic_json(out/'PROCESS_COMPLETION.json',dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True))
    # Seal actual online output before any independent hidden evaluation.
    paths=[f for f in out.rglob('*') if f.is_file() and (str(f.relative_to(out)).startswith(('model/','agent_commands/','agent_responses/')) or f.name in ('EPISODE_RESULT.json','SOURCE_PROVENANCE.json','AGENT_ACTION_TRACE.json','CODEX_DECISIONS.jsonl'))]
    atomic_json(out/'ONLINE_SEAL.json',dict(registered_episode_id=e['episode_id'],files={str(f.relative_to(out)):sha(f) for f in paths},actor_exited=True,hidden_values_revealed=False,all_online_actions_finished=True))

if __name__=='__main__':main()
