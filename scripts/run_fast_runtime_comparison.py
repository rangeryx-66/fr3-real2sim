"""Sequential short native before/after control with parent-observed cleanup timing."""
import json,os,subprocess,time,datetime,argparse,fcntl
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--label',required=True);parser.add_argument('--cycles',type=int,default=4);args=parser.parse_args()
    if args.cycles<1:raise ValueError('Positive finite cycle count required')
    parent=ROOT/'results/fast_runtime_v1_20261018';parent.mkdir(exist_ok=True)
    lock=(parent/'owned_actor.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=parent/args.label
    if out.exists():raise RuntimeError('Preserve prior comparison; use a new label')
    out.mkdir()
    base=json.loads((ROOT/'results/agent_pull_push_v12_20261018/job.json').read_text())
    env=dict(os.environ,PYTHONPATH='.:scripts',PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH'])
    results=[]
    for name,runner in [('legacy_short','run_legacy_runtime_short_control.py'),('fast_short','run_fast_interaction_v1.py')]:
        folder=out/name;folder.mkdir();job=dict(base,output=str(folder),gpu=3,engineering_cycles=args.cycles,legacy_executable=str(ROOT/'results/agent_pull_push_v12_20261018/episode/ACTUAL_LARGE_RANGE_TORQUE_EXECUTABLE.py'))
        job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=2)).isoformat()
        (folder/'job.json').write_text(json.dumps(job,indent=2));cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts'/runner),'--job',str(folder/'job.json')]
        if name=='fast_short':cmd+=['--test','free','--cycles',str(args.cycles)]
        began=time.perf_counter()
        with (folder/'actor.log').open('wb') as log:
            p=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            (folder/'PROCESS.json').write_text(json.dumps(dict(pid=p.pid,command=cmd)))
            try:rc=p.wait(timeout=600)
            except subprocess.TimeoutExpired:
                (folder/'STOP_AND_EXPORT').touch()
                try:rc=p.wait(timeout=30)
                except subprocess.TimeoutExpired:p.terminate();rc=p.wait(timeout=15)
        results.append(dict(case=name,returncode=rc,wall_including_initialization_and_cleanup_s=time.perf_counter()-began,actor_exited=True))
        (out/'SHORT_COMPARISON_PROCESS.json').write_text(json.dumps(results,indent=2))
        if rc!=0:break
        evidence=folder/('report.json' if name=='legacy_short' else 'TEST_RESULT.json')
        if not evidence.exists():break
        status=json.loads(evidence.read_text())
        if name=='legacy_short' and not status.get('success'):break
if __name__=='__main__':main()
