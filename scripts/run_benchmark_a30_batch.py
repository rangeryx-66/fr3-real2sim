"""Explicit benchmark order: plan every registered case, then attempt once.

No object-specific controllers or outcome-driven tuning. Physics and planner
sources are shared with the frozen fixed-base benchmark, with a 30 degree goal.
"""
import argparse, concurrent.futures, datetime, hashlib, json, os, subprocess, time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PYTHON='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

def write(path,value):path.write_text(json.dumps(value,indent=2))

def run(script,jobfile,folder,label,gpu,extra=()):
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
    started=time.time();closed=False
    with (folder/(label+'.log')).open('w') as log:
        process=subprocess.Popen([PYTHON,'-u',str(ROOT/'scripts'/script),'--job',str(jobfile),*extra],
                                 cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        write(folder/(label+'_process.json'),{'pid':process.pid,'started_wall':started,'gpu':gpu,'stage':label})
        while process.poll() is None:
            if label=='actor' and not closed and (folder/'scene_preserved_after_result.json').exists():
                # The physical attempt has ended and exported its evidence. The
                # scene survived the failure; cleanup is an explicit owner action.
                write(folder/'owner_cleanup_request.json',{'pid':process.pid,'after_physical_result':True,
                       'no_retry_or_continuation':True,'requested_wall':time.time()})
                (folder/'close_completed_scene').touch();closed=True
            time.sleep(.25)
        code=process.returncode
    write(folder/(label+'_cleanup.json'),{'pid':process.pid,'actor_exited':True,'returncode':code,'wall_s':time.time()-started})
    return code

def plan_case(case,gpu):
    job=json.loads((case/'setup_job.json').read_text());job['gpu']=gpu;write(case/'setup_job.json',job)
    setup=case/'setup';setup.mkdir(exist_ok=True)
    code=run('export_revolute_transfer_scene.py',case/'setup_job.json',setup,'setup',gpu)
    if code!=0 or not (setup/'cooked_initial.json').exists():
        write(case/'planning_failure.json',{'asset_id':case.name,'failure_reason':'SCENE_SETUP_FAILED','stage':'planning',
              'official_physical_attempts':0,'target_deg':30.0,'setup_exit_code':code});return
    planning=case/'planning';planning.mkdir(exist_ok=True)
    code=run('plan_benchmark_a30.py',case/'setup_job.json',planning,'planning',gpu,
             ('--output',str(planning),'--wall-s','300'))
    if code!=0 or not (planning/'candidate.json').exists():
        write(case/'planning_failure.json',{'asset_id':case.name,'failure_reason':'NO_EXECUTABLE_SHARED_PLAN' if code==0 else 'PLANNER_EXCEPTION',
              'stage':'planning','official_physical_attempts':0,'target_deg':30.0,'planning_exit_code':code});return
    write(case/'PLANNING_FEASIBLE.json',{'asset_id':case.name,'target_deg':30.0,'planning_finished_wall':time.time()})
    print('PLANNED',case.name,flush=True)

def execute_case(case,gpu):
    job=json.loads((case/'setup_job.json').read_text());job['gpu']=gpu
    planning=case/'planning';candidate=json.loads((planning/'candidate.json').read_text())
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
    with (output/'OFFICIAL_ATTEMPT_RESERVED.json').open('x') as stream:
        json.dump({'attempt':1,'asset_id':case.name,'reserved_wall':time.time(),'no_retry':True},stream,indent=2)
    files=[jobfile,source,manifest,planning/'whole_plan.json',planning/'grasp_plan.json',
           ROOT/'scripts/run_benchmark_a30.py',ROOT/'scripts/plan_benchmark_a30.py',
           ROOT/'scripts/run_revolute10_fixed.py',ROOT/'scripts/revolute10_fixed.py',ROOT/'scripts/piper_pink_fixed.py',
           Path(job['baseline_runner']),Path(job['pink_runner'])]
    write(output/'source_config_hashes.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    print('OFFICIAL_START',case.name,gpu,flush=True)
    code=run('run_benchmark_a30.py',jobfile,output,'actor',gpu)
    if not (output/'report.json').exists():
        write(output/'execution_failure.json',{'asset_id':case.name,'official_attempt':1,'failure_reason':'EXECUTION_OR_SIMULATOR_EXCEPTION','actor_exit_code':code})
    print('OFFICIAL_END',case.name,code,flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--registered',type=Path,required=True);a=p.parse_args()
    root=a.registered.parent;data=json.loads(a.registered.read_text());cases=[root/i for i in data['asset_ids']]
    write(root/'BATCH_STARTED.json',{'pid':os.getpid(),'started_wall':time.time(),
                                   'registered_sha256':hashlib.sha256(a.registered.read_bytes()).hexdigest()})
    def phase(action,items):
        def worker(queue,gpu):
            for case in queue:
                try:action(case,gpu)
                except Exception as error:
                    write(case/(action.__name__+'_exception.json'),{'asset_id':case.name,'error':repr(error),'no_retry':True})
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
            futures=[pool.submit(worker,items[index::3],index+1) for index in range(3)]
            for future in futures:future.result()
    phase(plan_case,cases)
    feasible=[case for case in cases if (case/'PLANNING_FEASIBLE.json').exists()]
    write(root/'ALL_PLANNING_COMPLETE.json',{'finished_wall':time.time(),'coverage':len(feasible),'denominator':10})
    phase(execute_case,feasible)
    write(root/'BATCH_COMPLETE.json',{'completed_wall':time.time(),'pid':os.getpid(),'physical_retry':False})

if __name__=='__main__':main()
