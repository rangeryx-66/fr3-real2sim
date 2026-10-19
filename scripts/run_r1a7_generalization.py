"""Run fresh AnyGrasp capture and paired raw/adapted Isaac benchmarks."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[1]
SIM='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

def start(cmd,log,env):
    log.parent.mkdir(parents=True,exist_ok=True)
    stream=open(log,'w')
    proc=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    stream.close()
    return proc

def stop(procs):
    for proc in reversed(procs):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGTERM)
    for proc in reversed(procs):
        try:proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=10)

def ready(port,object_id,procs):
    deadline=time.monotonic()+150
    while time.monotonic()<deadline:
        if any(proc.poll() is not None for proc in procs):raise RuntimeError('server process exited during startup')
        try:
            with urlopen(f'http://127.0.0.1:{port}',timeout=1) as stream:
                state=json.load(stream)
            if state.get('object',{}).get('id')==object_id:return
        except Exception:pass
        time.sleep(1)
    raise TimeoutError('Isaac plant did not become ready')

def run_stage(object_id,mode,scenarios,root,env):
    dest=root/object_id/mode
    dest.mkdir(parents=True,exist_ok=True)
    summary=dest/'summary.json'
    if summary.exists() and all((dest/f'trial_{i:02d}.json').exists() for i in range(1,6)):
        previous=json.loads(summary.read_text())
        if not any(previous.get('categories',{}).get(error,0) for error in ('SYSTEM_ERROR','TF_ERROR')):
            return dict(exit_code=0,summary=previous,reused_completed_stage=True)
    call_env={**env,'R1A7_RUN_DIR':str(dest)}
    cmd=[sys.executable,'-u',str(ROOT/'src/r1a7_backend.py'),'--mode',mode,'--trials','5',
         '--scenarios-json',str(scenarios)]
    if mode!='capture':cmd.extend(['--grasps-dir',str(root/object_id/'capture')])
    with open(dest/'run.log','w') as stream:
        done=subprocess.run(cmd,cwd=ROOT,env=call_env,stdout=stream,stderr=subprocess.STDOUT,timeout=2400)
    return dict(exit_code=done.returncode,summary=json.loads(summary.read_text()) if summary.exists() else None)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--objects',nargs='*',default=None)
    p.add_argument('--port',type=int,default=18778)
    p.add_argument('--ros-domain',type=int,default=214)
    a=p.parse_args()
    specs=json.loads((ROOT/'config/r1a7_generalization_objects.json').read_text())
    ids=a.objects or [obj['id'] for obj in specs]
    if any(object_id not in {obj['id'] for obj in specs} for object_id in ids):p.error('unknown object')
    root=ROOT/'results/r1a7_generalization'
    env={**os.environ,'R1A7_BASE_POSE':'0.329,-0.175,0.237,56.295',
         'R1A7_PEDESTAL_SIZE':'0.10,0.10,0.20','R1A7_PLANT_PORT':str(a.port),
         'ROS_DOMAIN_ID':str(a.ros_domain),'ROS_LOCALHOST_ONLY':'1',
         'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
    report={}
    for object_id in ids:
        print('OBJECT_START',object_id,flush=True)
        procs=[]
        try:
            logs=root/object_id
            procs.append(start([SIM,'-u',str(ROOT/'src/r1a7_sim_server.py'),'--gpu','1','--port',str(a.port),'--object-id',object_id],logs/'sim.log',env))
            procs.append(start(['ros2','launch',str(ROOT/'src/r1a7_moveit.launch.py')],logs/'moveit.log',env))
            procs.append(start([sys.executable,'-u',str(ROOT/'src/r1a7_ros_bridge.py')],logs/'bridge.log',env))
            ready(a.port,object_id,procs)
            scenarios=root/'scenarios'/f'{object_id}.json'
            report[object_id]={}
            for mode in ('capture','raw','adapted'):
                print('STAGE_START',object_id,mode,flush=True)
                report[object_id][mode]=run_stage(object_id,mode,scenarios,root,env)
                print('STAGE_DONE',object_id,mode,json.dumps(report[object_id][mode]),flush=True)
                (root/'orchestration.json').write_text(json.dumps(report,indent=2))
        except Exception as error:
            report.setdefault(object_id,{})['error']=repr(error)
            (root/'orchestration.json').write_text(json.dumps(report,indent=2))
            print('OBJECT_ERROR',object_id,repr(error),flush=True)
        finally:
            stop(procs)
            time.sleep(3)
    print('GENERALIZATION_DONE',json.dumps(report),flush=True)

if __name__=='__main__':main()
