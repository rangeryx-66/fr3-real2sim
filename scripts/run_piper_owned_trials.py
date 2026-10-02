"""Fresh physical episode per candidate; contact failure never enables a base move."""
import argparse,json,subprocess,sys,shutil,time
from pathlib import Path
from datetime import datetime,timedelta
from zoneinfo import ZoneInfo
from collections import Counter
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['search','source','asset-root','fixed-base-report','canonical-validation','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--sim-python',default='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python');p.add_argument('--gpu',type=int,default=7);p.add_argument('--max-trials',type=int,default=96);p.add_argument('--deadline-shanghai');p.add_argument('--follow-search',action='store_true')
    a=p.parse_args();now=datetime.now(ZoneInfo('Asia/Shanghai'));deadline=datetime.fromisoformat(a.deadline_shanghai) if a.deadline_shanghai else now.replace(hour=5,minute=0,second=0,microsecond=0)
    if deadline<=now and not a.deadline_shanghai:deadline+=timedelta(days=1)
    a.output.mkdir(parents=True,exist_ok=True);plan=a.output/'plan.json';shutil.copy(a.search.parent/'target_points.npy',a.output/'target_points.npy')
    summary_path=a.output/'summary.json';summary=json.loads(summary_path.read_text()) if summary_path.exists() else {'fixed_base':json.loads(a.fixed_base_report.read_text())['base_final'],'deadline_shanghai':deadline.isoformat(),'opening_goals_deg':[1,5,10,22],'physical_trials':[],'mobile_base_enabled':False,'raw_perception_unchanged':True}
    # A retried startup error is historical, not the status of the resumed run.
    summary['status']='RUNNING'
    for key in ('blocked_trial','returncode'):summary.pop(key,None)
    done={r['variant'] for r in summary['physical_trials']}
    while True:
        if datetime.now(deadline.tzinfo)>=deadline:summary['status']='CUTOFF_05_00';break
        try:search=json.loads(a.search.read_text())
        except json.JSONDecodeError:
            if not a.follow_search:raise
            time.sleep(.25);continue
        candidates=search.get('trial_candidates',[])[:a.max_trials]
        pending=[(i,c) for i,c in enumerate(candidates) if c['variant'] not in done]
        if not pending:
            if a.follow_search and not search.get('planning_complete',False):time.sleep(5);continue
            summary['status']='BOUNDED_SEARCH_EXHAUSTED';break
        i,candidate=pending[0]
        plan.write_text(json.dumps({'mode':'fixed','rows':candidates,'best':None},indent=2))
        out=a.output/candidate['variant'];out.mkdir(exist_ok=True);cmd=['env','-u','PYTHONPATH','-u','CUDA_VISIBLE_DEVICES',a.sim_python,str(ROOT/'scripts/piper_owned_grasp_trial.py'),'--source',str(a.source),'--asset-root',str(a.asset_root),'--fixed-base-report',str(a.fixed_base_report),'--canonical-validation',str(a.canonical_validation),'--plan',str(plan),'--candidate-index',str(i),'--output',str(out),'--gpu',str(a.gpu),'--deadline-shanghai',deadline.isoformat()]
        print('TRIAL',i,candidate['variant'],candidate['offset_handle_m'],candidate['rpy_handle_deg'],flush=True)
        with (out/'run.log').open('w') as log:process=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        if (out/'report.json').exists():
            r=json.loads((out/'report.json').read_text());row={k:r.get(k) for k in ['status','legal_closure','legal_pull','legal_real_grasp','measured_pull_displacement_m','max_actual_door_angle_deg','minimum_joint_margin_rad','maximum_relative_slip_m','native_metal_contact_samples','stage_tests','stop_state']}
            row.update(variant=candidate['variant'],output=str(out),offset_handle_m=candidate['offset_handle_m'],rpy_handle_deg=candidate['rpy_handle_deg']);summary['physical_trials'].append(row)
            done.add(candidate['variant']);summary['counts']=dict(Counter(r['status'].split(':',1)[0] for r in summary['physical_trials']))
            print('RESULT',candidate['variant'],r['status'],flush=True)
            if r['status']=='REAL_PAD_ONLY_OPENING_COMPLETE':summary.update(status='COMPLETE',best=row);summary_path.write_text(json.dumps(summary,indent=2));return
        else:
            summary.update(status='IMPLEMENTATION_ERROR',blocked_trial=candidate['variant'],returncode=process.returncode);summary_path.write_text(json.dumps(summary,indent=2));raise RuntimeError('trial did not produce a measured report; inspect '+str(out/'run.log'))
        summary['counts']=dict(Counter(r['status'].split(':',1)[0] for r in summary['physical_trials']));summary_path.write_text(json.dumps(summary,indent=2))
        if process.returncode or r.get('exception_type') not in [None,'RuntimeError'] or 'Traceback' in r['status']:
            raise RuntimeError('implementation failure, not a contact/kinematic outcome: '+r['status'])
    summary_path.write_text(json.dumps(summary,indent=2))

if __name__=='__main__':main()
