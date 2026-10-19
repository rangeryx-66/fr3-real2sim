"""Sequential launcher for the three user-registered visual reliability cases."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,subprocess,time
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'results/native_g1_visual_reliability_20261015';CASES=['45600','45606','45671']
p=argparse.ArgumentParser();p.add_argument('--phase',choices=['shadow','physical'],required=True);a=p.parse_args()
records=[]
for asset in CASES:
    d=BASE/asset;jobpath=d/('shadow_job.json' if a.phase=='shadow' else 'job.json');job=json.loads(jobpath.read_text());out=Path(job['output'])
    if (out/'report.json').exists():
        records.append(dict(asset=asset,existing_outcome_preserved=True,report=str(out/'report.json')));continue
    free=subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True)
    capacities=sorted([(int(row.split(',')[1]),int(row.split(',')[0])) for row in free.strip().splitlines()],reverse=True)
    job['gpu']=capacities[0][1];job['visual_gpu']=capacities[1][1];job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=2)).isoformat();jobpath.write_text(json.dumps(job,indent=2));(d/(a.phase+'_GPU_CAPACITY.json')).write_text(json.dumps(dict(free_mib=capacities,selected_isaac=job['gpu'],selected_sam3=job['visual_gpu'],exclusive_GPU_required=False),indent=2))
    command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts/run_native_g1_rgbd_short.py'),'--job',str(jobpath)]
    env=dict(os.environ,PYTHONPATH=str(ROOT/'scripts')+':'+str(ROOT),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1');log=(d/(a.phase+'.log')).open('w')
    actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rec=dict(asset=asset,phase=a.phase,pid=actor.pid,command=command,start_unix_s=time.time());(d/(a.phase+'_PROCESS.json')).write_text(json.dumps(rec,indent=2));print('START',asset,a.phase,actor.pid,flush=True)
    code=actor.wait();log.close();rec.update(returncode=code,end_unix_s=time.time());(d/(a.phase+'_PROCESS_EXIT.json')).write_text(json.dumps(rec,indent=2));records.append(rec)
    if not (out/'report.json').exists():
        (BASE/(a.phase+'_BATCH_STATUS.json')).write_text(json.dumps(dict(state='INITIALIZATION_OR_EXPORT_ERROR_REQUIRES_REPAIR',records=records),indent=2));print('INFRASTRUCTURE_STOP',asset,code,flush=True);break
    report=json.loads((out/'report.json').read_text());print('END',asset,report['status'],flush=True)
else:
    (BASE/(a.phase+'_BATCH_STATUS.json')).write_text(json.dumps(dict(state='COMPLETED',records=records),indent=2))
