"""Run fresh closed-setup maximum-range only after regression and old actors exit.

This coordinates the existing episode runner; it never replays failed prefixes
or changes an actor's loaded code. All outputs share the original absolute clock.
"""
import argparse,json,os,subprocess,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def alive(pid):
    try:
        os.kill(pid,0)
        return Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()[0]!='Z'
    except (ProcessLookupError,FileNotFoundError):return False


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--previous-output',type=Path,required=True);a=p.parse_args()
    out=a.output.resolve();clock=json.loads((out/'run_clock.json').read_text());deadline=clock['capture_deadline_wall_s']
    lease=(out/'refactored_owner.lock').open('a')
    import fcntl
    fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    baseline=out/'baseline_regression/summary.json';previous=None
    while time.time()<deadline:
        reg=json.loads(baseline.read_text()) if baseline.exists() else {}
        live=[]
        for obj in ('7320','45746'):
            path=a.previous_output/(obj+'_component.json')
            row=json.loads(path.read_text()) if path.exists() else {}
            pid=row.get('pid')
            if pid and alive(pid):live.append({'object':obj,'pid':pid})
        state={'phase':'WAIT_FOR_FROZEN_REGRESSION_AND_OLD_ACTORS','old_live':live,'baseline_runs':len(reg.get('runs',[])),'baseline_complete':reg.get('complete',False),'fixed_capture_ceiling':deadline}
        encoded=json.dumps(state,sort_keys=True)
        if encoded!=previous:
            (out/'dispatch_status.json').write_text(json.dumps(state,indent=2));print(encoded,flush=True);previous=encoded
        if reg.get('complete'):
            if reg.get('actual_contact_success_count')!=4 or reg.get('grasp_run_count')!=4:
                raise RuntimeError('FROZEN_PHYSICAL_BASELINE_REGRESSION_FAILED')
            control=next((r['report'] for r in reg['runs'] if r['name']=='no_operation'),{})
            if abs(control.get('actual_door_displacement_deg',float('inf')))>.5:
                raise RuntimeError('FROZEN_BASELINE_CONTROL_FAILED')
            if not live:break
        time.sleep(10)
    else:
        (out/'dispatch_status.json').write_text(json.dumps({'phase':'WALL_BUDGET_EXHAUSTED_BEFORE_DISPATCH','clock':clock},indent=2));return
    cmd=[sys.executable,str(ROOT/'scripts/run_wrist_reconstruction.py'),'--config',str(out/'refactored_config.json'),'--output',str(out),'--stage','capture','--mode','maximum-range','--resume']
    print('FRESH_CLOSED_SETUP_CAPTURE_START '+json.dumps(cmd),flush=True)
    with (out/'capture_supervisor.log').open('a') as log:child=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    (out/'dispatch_status.json').write_text(json.dumps({'phase':'FRESH_CLOSED_SETUP_CAPTURE','pid':child.pid,'command':cmd,'clock':clock},indent=2))
    code=child.wait()
    from wrist_reconstruction.readiness import both_ready
    c=json.loads((out/'refactored_config.json').read_text());ready,details=both_ready(out,c)
    (out/'backend_data_gate.json').write_text(json.dumps({'ready':ready,'objects':details},indent=2))
    (out/'dispatch_status.json').write_text(json.dumps({'phase':'CAPTURE_ACTORS_STOPPED','returncode':code,'backend_ready':ready,'clock':clock},indent=2))
    print('CAPTURE_ACTORS_STOPPED '+json.dumps({'returncode':code,'backend_ready':ready,'objects':details}),flush=True)
    # Reconstruction and final REPORT follow physical verification in the
    # owning chat. Missing capture never triggers a backend or physics fit.
if __name__=='__main__':main()
