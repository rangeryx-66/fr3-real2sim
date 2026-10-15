"""One observation run, then at most one separately requested physical run."""
import argparse,datetime,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'results/native_g1_wrist_rgbd_20261015'
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['observation','physical'],required=True);a=p.parse_args()
    directory=BASE/a.mode;directory.mkdir(parents=True,exist_ok=True)
    if (directory/'PROCESS.json').exists():raise SystemExit('Existing launch retained; inspect instead of retrying a valid episode')
    job=json.loads((ROOT/'results/native_g1_multiview_skill_20261015/45600/job.json').read_text())
    capacity=sorted([(int(r.split(',')[1]),int(r.split(',')[0])) for r in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).strip().splitlines()],reverse=True)
    job.update(output=str(directory/'episode'),gpu=capacity[0][1],visual_gpu=capacity[1][1],episode_id='native_g1_wrist_'+a.mode+'_45600',
               expected_runner_sha256=hashlib.sha256((ROOT/'scripts/run_native_g1_wrist_short.py').read_bytes()).hexdigest(),
               wrist_calibration=str(ROOT/'configs/xarm7_wrist_rgbd_sim.json'),wrist_observation_only=a.mode=='observation',capture_shadow_only=False,
               deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
    path=directory/'job.json';path.write_text(json.dumps(job,indent=2))
    (directory/'GPU_CAPACITY.json').write_text(json.dumps(dict(free_mib=capacity,exclusive_GPU_required=False),indent=2))
    command=[PYTHON,'-u',str(ROOT/'scripts/run_native_g1_wrist_short.py'),'--job',str(path)]
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    with (directory/'actor.log').open('w') as log:
        actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        record=dict(mode=a.mode,asset='45600',pid=actor.pid,command=command,start_unix_s=time.time())
        (directory/'PROCESS.json').write_text(json.dumps(record,indent=2));print('START',json.dumps(record),flush=True)
        record.update(returncode=actor.wait(),end_unix_s=time.time())
    (directory/'PROCESS_EXIT.json').write_text(json.dumps(record,indent=2));print('END',json.dumps(record),flush=True)
