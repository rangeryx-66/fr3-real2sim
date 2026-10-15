"""Single registered actor; sequential ownership and unchanged frozen runner."""
import datetime,hashlib,json,os,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'
if __name__=='__main__':
    aid=sys.argv[1];registration=json.loads((ROOT/'docs/experiments/native_g1_agent_transfer_20261015/REGISTERED_CASES.json').read_text())
    if aid not in registration['execution_order']:raise SystemExit('Not registered')
    base=ROOT/'results/native_g1_agent_transfer_20261015'/aid
    if (base/'PROCESS.json').exists():raise SystemExit('Existing episode; inspect instead of duplicate')
    for p in registration['frozen_source_sha256']:
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=registration['frozen_source_sha256'][p]:raise SystemExit('Frozen source changed: '+p)
    for other in registration['execution_order']:
        folder=base.parent/other
        if (folder/'PROCESS.json').exists() and not (folder/'PROCESS_EXIT.json').exists():raise SystemExit('Prior owned actor has not exited: '+other)
    capacity=sorted([(int(r.split(',')[1]),int(r.split(',')[0])) for r in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).strip().splitlines()],reverse=True)
    job=json.loads((base/'job.json').read_text());job.update(gpu=capacity[0][1],visual_gpu=capacity[1][1],deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
    (base/'job.json').write_text(json.dumps(job,indent=2));(base/'GPU_CAPACITY.json').write_text(json.dumps(dict(free_mib=capacity,exclusive_GPU_required=False),indent=2))
    command=[PYTHON,'-u',str(ROOT/'scripts/run_native_g1_agent_manipulation.py'),'--job',str(base/'job.json')]
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    with (base/'actor.log').open('w') as log:
        actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        record=dict(pid=actor.pid,command=command,start_unix_s=time.time(),runner_sha256=hashlib.sha256((ROOT/'scripts/run_native_g1_agent_manipulation.py').read_bytes()).hexdigest());(base/'PROCESS.json').write_text(json.dumps(record,indent=2));print('START',json.dumps(record),flush=True)
        record.update(returncode=actor.wait(),end_unix_s=time.time())
    (base/'PROCESS_EXIT.json').write_text(json.dumps(record,indent=2));print('END',json.dumps(record),flush=True)
