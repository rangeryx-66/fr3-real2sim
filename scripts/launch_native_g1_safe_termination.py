"""Single registered V1.1 episode. No valid physical retry."""
import datetime,hashlib,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'


if __name__=='__main__':
    root=ROOT/'results/native_g1_safe_termination_v11_20261016';base=root/'45936';base.mkdir(parents=True,exist_ok=True)
    if (base/'PROCESS.json').exists():raise SystemExit('Existing registered launch: inspect evidence, no automatic physical retry')
    freeze=json.loads((ROOT/'docs/experiments/native_g1_safe_termination_v11_20261016/FROZEN.json').read_text())
    for name,sha in freeze['source_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=sha:raise SystemExit('Source mismatch before initialization: '+name)
    # Do not require an idle GPU. This only reports free capacity and picks
    # the largest available devices; never terminates another user's process.
    capacity=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
    job=json.loads((ROOT/'results/native_g1_contact_recovery_v1_20261016/45936/job.json').read_text())
    runner=ROOT/'scripts/run_native_g1_safe_termination.py'
    job.update(output=str(base/'episode'),gpu=capacity[0][1],visual_gpu=capacity[1][1],episode_id='safe_termination_v11_45936',
        activation_source_files=list(freeze['source_sha256']),expected_runner_sha256=hashlib.sha256(runner.read_bytes()).hexdigest(),
        deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
    job['safe_termination_v11']=freeze['parking_definition']
    path=base/'job.json';path.write_text(json.dumps(job,indent=2));(base/'GPU_CAPACITY.json').write_text(json.dumps(dict(free_MiB=capacity,exclusive_required=False),indent=2))
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/g1_safe_park_mpl',
        PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    command=[PYTHON,'-u',str(runner),'--job',str(path)]
    with (base/'actor.log').open('w') as log:
        actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        record=dict(pid=actor.pid,command=command,start_unix_s=time.time());(base/'PROCESS.json').write_text(json.dumps(record,indent=2));print('START',json.dumps(record),flush=True)
        record.update(returncode=actor.wait(),end_unix_s=time.time())
    (base/'PROCESS_EXIT.json').write_text(json.dumps(record,indent=2));print('END',json.dumps(record),flush=True)
