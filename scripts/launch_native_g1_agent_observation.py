"""Launch exactly one persistent observation-only actor; never auto-retry."""
import datetime,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'results/native_g1_agent_wrist_20261015'
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

if __name__=='__main__':
    BASE.mkdir(parents=True,exist_ok=True)
    if (BASE/'PROCESS.json').exists():raise SystemExit('Existing episode retained; do not duplicate')
    job=json.loads((ROOT/'results/native_g1_wrist_rgbd_20261015/observation/job.json').read_text())
    capacity=sorted([(int(r.split(',')[1]),int(r.split(',')[0])) for r in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).strip().splitlines()],reverse=True)
    job.update(output=str(BASE/'episode'),gpu=capacity[0][1],visual_gpu=capacity[1][1],episode_id='native_g1_agent_wrist_45600',visual_input_version='Codex_agent_rigid_wrist_RGBD',
               expected_runner_sha256=hashlib.sha256((ROOT/'scripts/run_native_g1_agent_observation.py').read_bytes()).hexdigest(),
               wrist_observation_only=True,capture_shadow_only=False,
               deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
    jobpath=BASE/'job.json';jobpath.write_text(json.dumps(job,indent=2));(BASE/'GPU_CAPACITY.json').write_text(json.dumps(dict(free_mib=capacity,exclusive_GPU_required=False),indent=2))
    command=[PYTHON,'-u',str(ROOT/'scripts/run_native_g1_agent_observation.py'),'--job',str(jobpath)]
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    with (BASE/'actor.log').open('w') as log:
        actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        record=dict(pid=actor.pid,command=command,start_unix_s=time.time());(BASE/'PROCESS.json').write_text(json.dumps(record,indent=2));print('START',json.dumps(record),flush=True)
        record.update(returncode=actor.wait(),end_unix_s=time.time())
    (BASE/'PROCESS_EXIT.json').write_text(json.dumps(record,indent=2));print('END',json.dumps(record),flush=True)
