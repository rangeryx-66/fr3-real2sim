"""Bounded V7.5 engineering launcher. Frozen benchmark slots remain inaccessible."""
import argparse,fcntl,hashlib,json,os,subprocess,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.prismatic_v73 import audit_frozen
from agent_command_protocol_v12 import atomic_json

def main():
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args();job=json.loads(a.job.read_text())
    if job.get('experiment')!='V7' or job.get('run_classification')!='PRISMATIC_ENGINEERING' or job.get('benchmark_registration') or job.get('preload_fixture'):raise ValueError('New engineering job required')
    if job['articulation_v7']['joint_type']!='prismatic' or job.get('prismatic_version')!='V7.5':raise ValueError('Explicit V7.5 prismatic engineering only')
    frozen=audit_frozen(ROOT);out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    if (out/'PROCESS.json').exists():raise ValueError('Preserve attempt, use a newly registered engineering ID')
    lock=(ROOT/'results/fast_runtime_v1_20261018/owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    free={int(x.split(',')[0]):int(x.split(',')[1]) for x in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()}
    if free[job['gpu']]<12000:raise ValueError('Insufficient free VRAM; another partially occupied GPU may be used')
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','scripts/run_prismatic_v75.py','--job',str(a.job.resolve())]
    began=time.perf_counter()
    with (out/'actor.log').open('wb') as log:
        actor=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        atomic_json(out/'PROCESS.json',dict(pid=actor.pid,command=cmd,free_VRAM_MiB=free,job_sha256=hashlib.sha256(a.job.read_bytes()).hexdigest(),engineering_only=True,frozen_revolute=frozen))
        try:rc=actor.wait(timeout=float(job['episode_wall_budget_s'])+180)
        except subprocess.TimeoutExpired:
            (out/'STOP_AND_EXPORT').touch()
            try:rc=actor.wait(timeout=90)
            except subprocess.TimeoutExpired:
                actor.terminate()
                try:rc=actor.wait(timeout=30)
                except subprocess.TimeoutExpired:actor.kill();rc=actor.wait(timeout=10)
    atomic_json(out/'PROCESS_COMPLETION.json',dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True))
if __name__=='__main__':main()
