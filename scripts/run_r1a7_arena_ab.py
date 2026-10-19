"""Run R1 Raw/Adapted on the 40 sealed Isaac Lab Arena FR3 inputs.

The orchestrator stops starting work at 05:00 Asia/Shanghai and kills a stage
before that deadline. Completed stages are reused unchanged on restart.
"""
import argparse
import json
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from run_r1a7_generalization import ROOT, SIM, start, stop, ready

OUT=ROOT/'results/r1a7_arena_ab'
ASSETS='/data1/home/rangeryx/fr3_moveit_grasp/assets/arena_complex'

def seconds_left():
    now=datetime.now(ZoneInfo('Asia/Shanghai'))
    return max(0.,(now.replace(hour=5,minute=0,second=0,microsecond=0)-now).total_seconds())

def stage(target,mode,env):
    dest=OUT/target/mode;dest.mkdir(parents=True,exist_ok=True)
    summary=dest/'summary.json'
    if summary.exists() and all((dest/f'trial_{i:02d}.json').exists() for i in range(1,6)):
        previous=json.loads(summary.read_text())
        if previous.get('trials')==5 and not previous.get('categories',{}).get('SYSTEM_ERROR'):
            return dict(exit_code=0,summary=previous,reused_completed_stage=True)
    if seconds_left()<180:return dict(status='CUTOFF_05_00')
    inputs=OUT/'inputs'/target
    cmd=[sys.executable,'-u',str(ROOT/'src/r1a7_backend.py'),'--mode',mode,'--trials','5',
         '--grasps-dir',str(inputs),'--reference-manifest',str(inputs/'reference_manifest.json'),
         '--scenarios-json',str(inputs/'scenarios.json')]
    with (dest/'run.log').open('w') as log:
        try:
            done=subprocess.run(cmd,cwd=ROOT,env={**env,'R1A7_RUN_DIR':str(dest)},
                                stdout=log,stderr=subprocess.STDOUT,timeout=min(3600,seconds_left()-10))
            result=dict(exit_code=done.returncode)
        except subprocess.TimeoutExpired:
            result=dict(status='CUTOFF_05_00')
    if summary.exists():
        result['summary']=json.loads(summary.read_text())
        if any(result['summary'].get('categories',{}).get(k,0) for k in ('SYSTEM_ERROR','TF_ERROR')):
            result['exit_code']=1
    result['completed_trials']=sum((dest/f'trial_{i:02d}.json').exists() for i in range(1,6))
    return result

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--objects',nargs='*')
    parser.add_argument('--gpu',type=int,default=1)
    parser.add_argument('--port',type=int,default=18781)
    parser.add_argument('--ros-domain',type=int,default=217)
    a=parser.parse_args()
    protocol=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())
    objects=a.objects or protocol['classes']
    if any(obj not in protocol['classes'] for obj in objects):parser.error('unknown Arena object')
    if not (OUT/'input_manifest.json').exists():raise FileNotFoundError('run build_r1a7_arena_manifest.py first')
    OUT.mkdir(parents=True,exist_ok=True)
    run_config=dict(ik_random_seeds=6,manifold_limit=24,base_pose='0.329,-0.175,0.237,56.295',
                    pedestal_size='0.10,0.10,0.20',cutoff_local='05:00 Asia/Shanghai',
                    input_manifest=str(OUT/'input_manifest.json'),gpu=a.gpu,port=a.port,ros_domain=a.ros_domain,
                    objects=objects)
    (OUT/f'run_config_gpu{a.gpu}.json').write_text(json.dumps(run_config,indent=2)+'\n')
    report={}
    for target in objects:
        if seconds_left()<180:
            report[target]={'status':'CUTOFF_05_00'};break
        env={**os.environ,'R1A7_BASE_POSE':'0.329,-0.175,0.237,56.295',
             'R1A7_PEDESTAL_SIZE':'0.10,0.10,0.20','R1A7_PLANT_PORT':str(a.port),
             'R1A7_IK_RANDOM_SEEDS':'6','R1A7_MANIFOLD_LIMIT':'24',
             'R1A7_ARENA_ASSET_DIR':ASSETS,'ROS_DOMAIN_ID':str(a.ros_domain),'ROS_LOCALHOST_ONLY':'1',
             'NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost',
             'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
        procs=[];report[target]={}
        try:
            logs=OUT/target
            print('ARENA_START',target,flush=True)
            procs.append(start([SIM,'-u',str(ROOT/'src/r1a7_sim_server.py'),'--gpu',str(a.gpu),
                                '--port',str(a.port),'--arena-target',target],logs/'sim.log',env))
            procs.append(start(['ros2','launch',str(ROOT/'src/r1a7_moveit.launch.py')],logs/'moveit.log',env))
            procs.append(start([sys.executable,'-u',str(ROOT/'src/r1a7_ros_bridge.py')],logs/'bridge.log',env))
            ready(a.port,target,procs)
            for mode in ('raw','adapted'):
                report[target][mode]=stage(target,mode,env)
                print('ARENA_STAGE',target,mode,json.dumps(report[target][mode]),flush=True)
                (OUT/f'orchestration_gpu{a.gpu}.json').write_text(json.dumps(report,indent=2)+'\n')
                if report[target][mode].get('status')=='CUTOFF_05_00':break
                if report[target][mode].get('exit_code')!=0:raise RuntimeError(target+' '+mode+' failed')
        except Exception as error:
            report[target]['error']=repr(error)
            print('ARENA_ERROR',target,repr(error),flush=True)
        finally:
            stop(procs)
            (OUT/f'orchestration_gpu{a.gpu}.json').write_text(json.dumps(report,indent=2)+'\n')
        if any(value.get('status')=='CUTOFF_05_00' for value in report[target].values() if isinstance(value,dict)):
            break
    print('ARENA_AB_DONE',json.dumps(report),flush=True)

if __name__=='__main__':main()
