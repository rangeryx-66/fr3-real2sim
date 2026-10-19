"""One sequential registered actor; shared GPU capacity, no physical retries."""
import argparse,datetime,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--asset',choices=['45936','46427'],required=True);a=p.parse_args()
    root=ROOT/'results/native_g1_contact_recovery_v1_20261016';root.mkdir(exist_ok=True)
    for proc in root.glob('*/PROCESS.json'):
        if not (proc.parent/'PROCESS_EXIT.json').exists():raise SystemExit('Inspect existing registered actor before launching another: '+str(proc))
    base=root/a.asset;base.mkdir(exist_ok=True)
    if (base/'PROCESS.json').exists():raise SystemExit('Registered case already launched; inspect its actual result, never auto-retry')
    freeze=json.loads((ROOT/'docs/experiments/native_g1_contact_recovery_v1_20261016/FROZEN.json').read_text())
    for f,h in freeze['source_sha256'].items():
        if hashlib.sha256((ROOT/f).read_bytes()).hexdigest()!=h:raise SystemExit('Prelaunch source mismatch: '+f)
    capacity=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
    job=json.loads((ROOT/'results/native_g1_agent_transfer_20261015'/a.asset/'job.json').read_text());runner=ROOT/'scripts/run_native_g1_contact_recovery.py'
    job.update(output=str(base/'episode'),gpu=capacity[0][1],visual_gpu=capacity[1][1],episode_id='contact_recovery_v1_'+a.asset,agent_wait_timeout_s=900,activation_source_files=list(freeze['source_sha256']),expected_runner_sha256=hashlib.sha256(runner.read_bytes()).hexdigest(),deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
    job['maximum_physical_attempts']=2
    job['skill_inputs']['permitted_recovery']=dict(local_centering_attempts=1,release=True,regrasp=True,move_base=False)
    job['contact_recovery_budget']=dict(first_grasp=1,recovery_grasp=1,Agent_view_adjustments=4,force_history_reset_under_load=False)
    path=base/'job.json';path.write_text(json.dumps(job,indent=2));(base/'GPU_CAPACITY.json').write_text(json.dumps(dict(free_MiB=capacity,exclusive_GPU_required=False),indent=2))
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/g1_contact_recovery_mpl',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    command=[PYTHON,'-u',str(runner),'--job',str(path)]
    with (base/'actor.log').open('w') as log:
        actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);record=dict(pid=actor.pid,command=command,start_unix_s=time.time());(base/'PROCESS.json').write_text(json.dumps(record,indent=2));print('START',json.dumps(record),flush=True);record.update(returncode=actor.wait(),end_unix_s=time.time())
    (base/'PROCESS_EXIT.json').write_text(json.dumps(record,indent=2));print('END',json.dumps(record),flush=True)
