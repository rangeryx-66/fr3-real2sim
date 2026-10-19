"""Independent fixed -> kinematic recovery -> reposition/lock door experiment.

Runs in simulation only. Contact failures never trigger base relocation.
All child processes share a hard Shanghai deadline and preserve baseline files.
"""
import argparse
from datetime import datetime,timedelta
from zoneinfo import ZoneInfo
import hashlib,json,subprocess,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','asset-root','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--geometry-python',required=True);p.add_argument('--isaac-python',required=True)
    p.add_argument('--gpu',type=int,default=6);p.add_argument('--max-real-trials',type=int,default=5)
    p.add_argument('--deadline-shanghai');p.add_argument('--baseline-report',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    zone=ZoneInfo('Asia/Shanghai');now=datetime.now(zone)
    deadline=datetime.fromisoformat(a.deadline_shanghai) if a.deadline_shanghai else now.replace(hour=5,minute=0,second=0,microsecond=0)
    if deadline.tzinfo is None:deadline=deadline.replace(tzinfo=zone)
    if not a.deadline_shanghai and deadline<=now:deadline+=timedelta(days=1)
    common=['--source',str(a.source),'--asset-root',str(a.asset_root),'--deadline-shanghai',deadline.isoformat()]
    out={'deadline':deadline.isoformat(),'real_trials':[],'input_sha256':{n:hashlib.sha256((a.source/n).read_bytes()).hexdigest() for n in ['capture.npz','handle_mask.npy','graspgenx_native.json']},'mobile_status':'NOT_TRIGGERED'}
    if a.baseline_report:
        out['r1_baseline']={'provenance':'archived; not rerun by this experiment','path':str(a.baseline_report),'sha256':hashlib.sha256(a.baseline_report.read_bytes()).hexdigest(),'report':json.loads(a.baseline_report.read_text())}
    def run(python,script,args,folder):
        left=(deadline-datetime.now(zone)).total_seconds()
        if left<=0:raise TimeoutError('CUTOFF_05_00')
        folder.mkdir(parents=True,exist_ok=True)
        with (folder/'run.log').open('w') as log:
            subprocess.run(['env','-u','PYTHONPATH','-u','CUDA_VISIBLE_DEVICES',python,str(ROOT/'scripts'/script),*args],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=left)
        return json.loads((folder/'report.json').read_text())
    try:
        subprocess.run([sys.executable,str(ROOT/'scripts/prepare_piper_description.py')],check=True)
        fixed=a.output/'fixed_preflight';r=run(a.geometry_python,'piper_mobile_preflight.py',[*common,'--output',str(fixed)],fixed)
        out['fixed_preflight']={'path':str(fixed/'report.json'),'counts':r['counts'],'status':r['status']}
        selected=fixed
        if any(v['status']=='FULL_PATH_PLANNED' for v in r['rows']):
            centered=a.output/'fixed_centering';r=run(a.geometry_python,'piper_mobile_preflight.py',[*common,'--output',str(centered),'--center-from',str(fixed/'report.json'),'--max-candidates','20'],centered)
            selected=centered
            out['fixed_centering']={'path':str(centered/'report.json'),'counts':r['counts'],'status':r['status']}
        if not any(v['status']=='FULL_PATH_PLANNED' for v in r['rows']):
            recovered=a.output/'mobile_preflight';r=run(a.geometry_python,'piper_mobile_preflight.py',[*common,'--output',str(recovered),'--recover-from',str(fixed/'report.json')],recovered)
            out['mobile_status']=r['status'];selected=recovered
        feasible=[(i,v) for i,v in enumerate(r['rows']) if v['status']=='FULL_PATH_PLANNED']
        feasible.sort(key=lambda item:(item[1]['minimum_joint_margin_rad'],item[1].get('minimum_collision_clearance_m') or 0),reverse=True)
        if not feasible:out['status']='NO_FULL_OPENING_PATH'
        else:
            for number,(index,_) in enumerate(feasible[:a.max_real_trials]):
                trial=a.output/f'real_trial_{number:02d}'
                physical=run(a.isaac_python,'piper_mobile_execute.py',[*common,'--plan',str(selected/'report.json'),'--output',str(trial),'--candidate-index',str(index),'--gpu',str(a.gpu),'--geometry-python',a.geometry_python],trial)
                out['real_trials'].append({'report':str(trial/'report.json'),'status':physical['status'],'legal_real_grasp':physical['legal_real_grasp'],'max_actual_door_angle_deg':physical['max_actual_door_angle_deg'],'minimum_joint_margin_rad':physical['minimum_joint_margin_rad']})
                (a.output/'report.json').write_text(json.dumps(out,indent=2))
                if physical['status']=='SUCCESS':break
            out['status']='SUCCESS' if any(v['status']=='SUCCESS' for v in out['real_trials']) else 'REAL_CONTACT_OR_EXECUTION_FAILED'
            if r['mode']=='fixed':out['mobile_status']='NOT_NEEDED_FOR_KINEMATICS; contact failure is not a reposition trigger'
    except subprocess.TimeoutExpired:out['status']='CUTOFF_05_00'
    except Exception as e:out.update(status='BLOCKED',error=str(e))
    finally:
        (a.output/'report.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))


if __name__=='__main__':main()
