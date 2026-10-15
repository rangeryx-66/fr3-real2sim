"""Two predeclared development trials, sequentially, no physical retries."""
from pathlib import Path
import datetime,hashlib,json,os,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'results/native_g1_multiview_skill_20261015'
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'
CASES=['45600','45606']

if __name__=='__main__':
    records=[]
    for asset in CASES:
        directory=BASE/asset;directory.mkdir(parents=True,exist_ok=True)
        out=directory/'physical_01'
        # Never repeat a launched case automatically. A pre-physics infrastructure
        # repair requires explicit inspection of the saved process/scene evidence.
        if (directory/'PROCESS.json').exists():
            records.append(dict(asset=asset,existing_launch_preserved=True));continue
        old=ROOT/'results/native_g1_visual_reliability_20261015'/asset/'job.json'
        job=json.loads(old.read_text())
        free=subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True)
        capacity=sorted([(int(r.split(',')[1]),int(r.split(',')[0])) for r in free.strip().splitlines()],reverse=True)
        job.update(output=str(out),gpu=capacity[0][1],visual_gpu=capacity[1][1],episode_id='native_g1_multiview_'+asset,
                   expected_runner_sha256=hashlib.sha256((ROOT/'scripts/run_native_g1_multiview_short.py').read_bytes()).hexdigest(),
                   visual_planning_wall_s=1200,visual_input_version='open_multiview_rgbd_rear_hypotheses',
                   deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
        jobpath=directory/'job.json';jobpath.write_text(json.dumps(job,indent=2))
        (directory/'GPU_CAPACITY.json').write_text(json.dumps(dict(free_mib=capacity,selected_isaac=job['gpu'],selected_sam3=job['visual_gpu'],exclusive_GPU_required=False),indent=2))
        command=[PYTHON,'-u',str(ROOT/'scripts/run_native_g1_multiview_short.py'),'--job',str(jobpath)]
        env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
        with (directory/'physical.log').open('w') as log:
            actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            record=dict(asset=asset,pid=actor.pid,command=command,start_unix_s=time.time())
            (directory/'PROCESS.json').write_text(json.dumps(record,indent=2));print('START',asset,actor.pid,flush=True)
            record.update(returncode=actor.wait(),end_unix_s=time.time())
        (directory/'PROCESS_EXIT.json').write_text(json.dumps(record,indent=2));records.append(record)
        (BASE/'BATCH_STATUS.json').write_text(json.dumps(dict(records=records),indent=2))
        if not (out/'report.json').exists():
            print('INFRASTRUCTURE_INSPECT_REQUIRED',asset,flush=True);break
        print('END',asset,json.loads((out/'report.json').read_text())['status'],flush=True)
