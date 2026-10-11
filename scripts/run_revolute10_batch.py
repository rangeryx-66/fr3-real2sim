"""Registered once-only batch. Setup/export is separate from official contact.

One queue per GPU; no failed-object replacement or physical retry. Finished
actors close after exporting. Frozen controller and dataset files are read-only.
"""
import argparse, concurrent.futures, datetime, hashlib, json, os, shutil, subprocess, sys, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'


def write(path, value):
    path.write_text(json.dumps(value,indent=2))


def execute_case(case, gpu):
    job=json.loads((case/'setup_job.json').read_text());job['gpu']=gpu
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
    def run(script,jobfile,folder,stage,extra=()):
        started=time.time()
        with (folder/(stage+'.log')).open('w') as log:
            process=subprocess.Popen([PYTHON,'-u',str(ROOT/'scripts'/script),'--job',str(jobfile),*extra],
                                     cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            write(folder/(stage+'_process.json'),{'pid':process.pid,'started_wall':started,'gpu':gpu,'stage':stage})
            code=process.wait()
        write(folder/(stage+'_cleanup.json'),{'pid':process.pid,'actor_exited':True,'returncode':code,'wall_s':time.time()-started})
        return code
    (case/'setup_job.json').write_text(json.dumps(job,indent=2))
    setup=case/'setup'
    if (setup/'cooked_initial.json').exists() and (setup/'setup_cleanup.json').exists():
        code=json.loads((setup/'setup_cleanup.json').read_text())['returncode']
        print('REUSE_READ_ONLY_SETUP_EXPORT',case.name,gpu,flush=True)
    else:
        print('SETUP_START',case.name,gpu,flush=True)
        code=run('export_revolute_transfer_scene.py',case/'setup_job.json',setup,'setup')
    if code!=0 or not (setup/'cooked_initial.json').exists():
        write(case/'planning_failure.json',{'asset_id':case.name,'classification':'KNOWN_MODEL_DIAGNOSTIC',
              'failure_category':'other','failure_reason':'NORMAL_SCENE_SETUP_FAILED','official_physical_attempts':0,
              'target_deg':job['authored_target_rad']*180/3.141592653589793,'setup_exit_code':code});return
    planning=case/'planning';planning.mkdir(exist_ok=True)
    print('PLANNING_START',case.name,flush=True)
    code=run('revolute10_fixed.py',case/'setup_job.json',planning,'planning',('--output',str(planning),'--wall-s','300'))
    if code!=0 or not (planning/'candidate.json').exists():
        summary=json.loads((planning/'planning_summary.json').read_text()) if (planning/'planning_summary.json').exists() else {}
        write(case/'planning_failure.json',{'asset_id':case.name,'classification':'KNOWN_MODEL_DIAGNOSTIC',
              'failure_category':'IK/collision/grasp' if code==0 else 'other','failure_reason':summary.get('blocker','PLANNER_EXCEPTION'),
              'official_physical_attempts':0,'target_deg':job['authored_target_rad']*180/3.141592653589793,'planning':summary});return
    candidate=json.loads((planning/'candidate.json').read_text())
    manifest=Path(job['asset_root'])/'manifest.json';meta=json.loads(manifest.read_text())
    meta['door_link']=candidate['grasp_geometry_selection']['handle_link']
    meta['interaction_geometry']['selection']=candidate['grasp_geometry_selection'];write(manifest,meta)
    source=Path(job['source'])/'report.json';src=json.loads(source.read_text());src['robot_base_pose']=candidate['base'];write(source,src)
    write(planning/'whole_plan.json',candidate)
    write(planning/'grasp_plan.json',{'trial_candidates':[{'base':candidate['base'],'T':candidate['T_grasp'],
          'preplan':candidate['preplan'],'approach':candidate['approach']}]})
    output=case/'official_01';output.mkdir(exist_ok=True)
    job['output']=str(output);job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=4)).isoformat()
    jobfile=output/'job.json';write(jobfile,job)
    # Durable launch reservation: a crash never silently creates a second attempt.
    ledger=output/'OFFICIAL_ATTEMPT_RESERVED.json'
    with ledger.open('x') as f:json.dump({'attempt':1,'asset_id':case.name,'reserved_wall':time.time(),'no_retry':True},f,indent=2)
    write(output/'source_config_hashes.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest()
          for p in [jobfile,source,manifest,planning/'whole_plan.json',planning/'grasp_plan.json',
                    ROOT/'scripts/run_revolute10_fixed.py',ROOT/'scripts/revolute10_fixed.py',ROOT/'scripts/piper_pink_fixed.py',
                    Path(job['baseline_runner']),Path(job['pink_runner'])]})
    print('OFFICIAL_START',case.name,gpu,flush=True)
    code=run('run_revolute10_fixed.py',jobfile,output,'actor')
    if not (output/'report.json').exists():
        write(output/'execution_failure.json',{'asset_id':case.name,'classification':'KNOWN_MODEL_DIAGNOSTIC','official_attempt':1,
              'failure_reason':'EXECUTION_SETUP_OR_SIMULATOR_EXCEPTION','actor_exit_code':code,'evidence':'actor.log'})
    print('OFFICIAL_END',case.name,code,flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--registered',type=Path,required=True);a=p.parse_args()
    root=a.registered.parent;data=json.loads(a.registered.read_text());queues=[[],[],[]]
    for index,i in enumerate(data['asset_ids']):
        case=root/i
        if (case/'setup_job.json').exists():queues[index%3].append(case)
    write(root/'BATCH_STARTED.json',{'pid':os.getpid(),'started_wall':time.time(),'registered_sha256':hashlib.sha256(a.registered.read_bytes()).hexdigest(),
                                   'gpu_queues':[[p.name for p in q] for q in queues]})
    def worker(queue,gpu):
        for case in queue:
            try:execute_case(case,gpu)
            except Exception as error:
                write(case/'batch_exception.json',{'asset_id':case.name,'error':repr(error),'no_retry':True});print('BATCH_EXCEPTION',case.name,repr(error),flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        futures=[pool.submit(worker,q,gpu) for q,gpu in zip(queues,[1,2,3])]
        for future in futures:future.result()
    write(root/'BATCH_COMPLETE.json',{'completed_wall':time.time(),'pid':os.getpid(),'physical_retry':False})


if __name__=='__main__':main()
