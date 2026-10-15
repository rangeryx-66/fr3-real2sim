"""One fresh registered 45936 actor; shared GPU capacity, no auto-retry."""
import datetime,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'results/native_g1_incremental_activation_20261016/live/45936'
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

if __name__=='__main__':
    BASE.mkdir(parents=True,exist_ok=True)
    if (BASE/'PROCESS.json').exists():raise SystemExit('Existing actor record; inspect it instead of duplicate launch')
    frozen=json.loads((ROOT/'docs/experiments/native_g1_incremental_activation_20261016/FROZEN.json').read_text())
    for p,h in frozen['source_sha256'].items():
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h:raise SystemExit('Active source mismatch: '+p)
    capacity=sorted([(int(r.split(',')[1]),int(r.split(',')[0])) for r in subprocess.check_output(
        ['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
    job=json.loads((ROOT/'results/native_g1_agent_transfer_20261015/45936/job.json').read_text())
    runner=ROOT/'scripts/run_native_g1_incremental_activation.py'
    job.update(output=str(BASE/'episode'),gpu=capacity[0][1],visual_gpu=capacity[1][1],
        episode_id='incremental_activation_45936_observation_only',wrist_observation_only=True,
        capture_shadow_only=False,agent_wait_timeout_s=900,activation_source_files=list(frozen['source_sha256']),
        expected_runner_sha256=hashlib.sha256(runner.read_bytes()).hexdigest(),
        deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=2)).isoformat())
    path=BASE/'job.json';path.write_text(json.dumps(job,indent=2))
    (BASE/'GPU_CAPACITY.json').write_text(json.dumps(dict(free_mib=capacity,exclusive_GPU_required=False),indent=2))
    command=[PYTHON,'-u',str(runner),'--job',str(path)]
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',
        MPLCONFIGDIR='/tmp/activate_mpl',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    with (BASE/'actor.log').open('w') as log:
        actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        record=dict(pid=actor.pid,command=command,start_unix_s=time.time())
        (BASE/'PROCESS.json').write_text(json.dumps(record,indent=2));print('START',json.dumps(record),flush=True)
        record.update(returncode=actor.wait(),end_unix_s=time.time())
    (BASE/'PROCESS_EXIT.json').write_text(json.dumps(record,indent=2));print('END',json.dumps(record),flush=True)
