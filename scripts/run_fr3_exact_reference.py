"""Recheck the original FR3 pipeline and audit all original top-K candidates."""
import hashlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen
from zoneinfo import ZoneInfo

from run_r1a7_generalization import ROOT, SIM, start, stop

FR3=Path('/data1/home/rangeryx/fr3_moveit_grasp')
OUT=ROOT/'results/r1a7_fr3_exact_ab'
REFERENCE=json.loads((ROOT/'config/r1a7_fr3_exact_reference.json').read_text())

def remaining():
    now=datetime.now(ZoneInfo('Asia/Shanghai'))
    return max(0.,(now.replace(hour=5,minute=0,second=0,microsecond=0)-now).total_seconds())

def ready(procs):
    end=time.monotonic()+150
    while time.monotonic()<end:
        if any(p.poll() is not None for p in procs):raise RuntimeError('FR3 process exited during startup')
        try:
            with urlopen('http://127.0.0.1:18765',timeout=1) as stream:
                if 'fr3_joint1' in json.load(stream).get('names',[]):return
        except Exception:pass
        time.sleep(1)
    raise TimeoutError('FR3 Isaac plant did not become ready')

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    env={**os.environ,'ROS_DOMAIN_ID':'216','ROS_LOCALHOST_ONLY':'1',
         'NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost',
         'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
    procs=[];report={}
    try:
        if remaining()<60:raise TimeoutError('05:00 Asia/Shanghai cutoff reached')
        procs.append(start([SIM,'-u',str(FR3/'src/sim_server.py'),'--gpu','1','--port','18765'],OUT/'fr3_sim.log',env))
        procs.append(start(['ros2','launch',str(FR3/'src/moveit.launch.py')],OUT/'fr3_moveit.log',env))
        procs.append(start([sys.executable,'-u',str(FR3/'src/ros_bridge.py')],OUT/'fr3_bridge.log',env))
        ready(procs)
        with (OUT/'fr3_audit.log').open('w') as log:
            done=subprocess.run([sys.executable,'-u',str(ROOT/'scripts/audit_fr3_exact_candidates.py')],
                                cwd=FR3,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=min(1800,remaining()-10))
        report['candidate_audit_exit_code']=done.returncode
        if remaining()<60:
            report['trial_run']='CUTOFF_05_00';return
        before={p for p in (FR3/'results').glob('run_*') if p.is_dir()}
        with (OUT/'fr3_trial.log').open('w') as log:
            try:
                done=subprocess.run([sys.executable,'-u',str(FR3/'src/backend.py'),'--trials','10'],
                                    cwd=FR3,env=env,stdout=log,stderr=subprocess.STDOUT,
                                    timeout=min(2400,remaining()-10))
                report['trial_exit_code']=done.returncode
            except subprocess.TimeoutExpired:
                report['trial_run']='CUTOFF_05_00'
        created=sorted((p for p in (FR3/'results').glob('run_*') if p.is_dir() and p not in before),
                       key=lambda p:p.stat().st_mtime)
        if created:
            run=created[-1];report['trial_directory']=str(run)
            summary=run/'summary.json'
            if summary.exists():report['summary']=json.loads(summary.read_text())
            report['grasp_hash_match']=[hashlib.sha256((run/f'trial_{i:02d}_grasps.json').read_bytes()).hexdigest()==row['grasps_sha256']
                                        for i,row in enumerate(REFERENCE,1) if (run/f'trial_{i:02d}_grasps.json').exists()]
    except Exception as error:
        report['error']=repr(error)
    finally:
        stop(procs)
        (OUT/'fr3_orchestration.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
