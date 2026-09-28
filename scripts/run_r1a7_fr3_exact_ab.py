"""Replay the original FR3 benchmark inputs for paired R1 raw/recovery execution."""
import json
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from run_r1a7_generalization import ROOT, SIM, start, stop, ready

FR3=Path('/data1/home/rangeryx/fr3_moveit_grasp')
REFERENCE=FR3/'results/run_1788943864166403373'
OUT=ROOT/'results/r1a7_fr3_exact_ab'

def deadline_seconds():
    now=datetime.now(ZoneInfo('Asia/Shanghai'))
    deadline=now.replace(hour=5,minute=0,second=0,microsecond=0)
    if now>=deadline: return 0.
    return (deadline-now).total_seconds()

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    env={**os.environ,'R1A7_BASE_POSE':'0.329,-0.175,0.237,56.295',
         'R1A7_PEDESTAL_SIZE':'0.10,0.10,0.20','R1A7_PLANT_PORT':'18779',
         'ROS_DOMAIN_ID':'215','ROS_LOCALHOST_ONLY':'1',
         'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
    procs=[];report={}
    try:
        if deadline_seconds()<60:raise TimeoutError('05:00 Asia/Shanghai cutoff reached')
        procs.append(start([SIM,'-u',str(ROOT/'src/r1a7_sim_server.py'),'--gpu','1','--port','18779'],OUT/'sim.log',env))
        procs.append(start(['ros2','launch',str(ROOT/'src/r1a7_moveit.launch.py')],OUT/'moveit.log',env))
        procs.append(start([sys.executable,'-u',str(ROOT/'src/r1a7_ros_bridge.py')],OUT/'bridge.log',env))
        ready(18779,'benchmark_box',procs)
        for mode in ('raw','adapted'):
            dest=OUT/mode;dest.mkdir(exist_ok=True)
            summary=dest/'summary.json'
            if summary.exists() and all((dest/f'trial_{i:02d}.json').exists() for i in range(1,11)):
                previous=json.loads(summary.read_text())
                if previous.get('trials')==10 and not previous.get('categories',{}).get('SYSTEM_ERROR'):
                    report[mode]={'exit_code':0,'summary':previous,'reused_completed_stage':True}
                    continue
            remaining=deadline_seconds()
            if remaining<60:
                report[mode]={'status':'CUTOFF_05_00'}
                break
            cmd=[sys.executable,'-u',str(ROOT/'src/r1a7_backend.py'),'--mode',mode,
                 '--trials','10','--grasps-dir',str(REFERENCE),
                 '--reference-manifest',str(ROOT/'config/r1a7_fr3_exact_reference.json')]
            with (dest/'run.log').open('w') as log:
                try:
                    done=subprocess.run(cmd,cwd=ROOT,env={**env,'R1A7_RUN_DIR':str(dest)},
                                        stdout=log,stderr=subprocess.STDOUT,
                                        timeout=max(1,remaining-10))
                    report[mode]={'exit_code':done.returncode}
                except subprocess.TimeoutExpired:
                    report[mode]={'status':'CUTOFF_05_00'}
            summary=dest/'summary.json'
            if summary.exists():report[mode]['summary']=json.loads(summary.read_text())
            (OUT/'orchestration.json').write_text(json.dumps(report,indent=2)+'\n')
            if report[mode].get('status')=='CUTOFF_05_00':break
        if deadline_seconds()>60 and all(report.get(m,{}).get('exit_code')==0 for m in ('raw','adapted')):
            with (OUT/'r1_audit.log').open('w') as log:
                done=subprocess.run([sys.executable,'-u',str(ROOT/'scripts/audit_r1a7_exact_candidates.py')],
                                    cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,
                                    timeout=min(1800,deadline_seconds()-10))
            report['candidate_audit_exit_code']=done.returncode
    except Exception as error:
        report['error']=repr(error)
    finally:
        stop(procs)
        (OUT/'orchestration.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
