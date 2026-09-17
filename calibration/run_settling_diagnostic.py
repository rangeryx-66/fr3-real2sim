"""Uninterrupted fixed-GT settling diagnostics at 30 N / mu=0.7."""
import argparse, json, time, sys, hashlib, os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from settling_gate import GateThresholds
import plant
plant.URL='http://127.0.0.1:'+os.environ.get('CALIBRATION_PORT','18765')

def phase(name): plant.command(dict(op='calibration_phase',phase=name))

def scaled(plan, minimum_duration):
    result=json.loads(json.dumps(plan)); current=result['points'][-1]['t']
    scale=max(1.,minimum_duration/current)
    for p in result['points']: p['t']*=scale
    return result

def trajectory(plan):
    answer=plant.command(dict(op='trajectory',**plan),timeout=240)
    if not answer['ok']: raise RuntimeError('ARM_TRACKING_ERROR '+str(answer))

def close_force(total=30.):
    answer=plant.command(dict(op='calibration_force',force_N=total))
    if not answer['ok']: raise RuntimeError(str(answer))
    start=plant.state()['t']; stable=None
    while True:
        s=plant.state('calibration'); f=np.asarray(s['calibration']['filtered_force_N'])
        good=abs(float(f.sum())-total)<=max(2.,total*.2) and min(s['forces'])>.1
        stable=s['t'] if good and stable is None else stable if good else None
        if stable is not None and s['t']-stable>=.3:return
        if s['t']-start>5.:raise RuntimeError('FORCE_UNREACHED')
        time.sleep(.01)

def episode(target,repeat,out,thresholds):
    path=ROOT/f'results/hand_calibration/gt_path_{target}.json'; plan=json.loads(path.read_text())
    label=f'{target}_settling_r{repeat:02d}'; destination=out/f'{label}.json'
    if destination.exists():return json.loads(destination.read_text())
    trace=out/f'{label}.trace.json.gz'; r=dict(id=label,target=target,repeat=repeat,
        force_total_N=30.,mu=.7,success=False,category='SYSTEM_ERROR',
        gt_path_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),thresholds=thresholds.__dict__)
    stage='RESET'; wall=time.monotonic()
    try:
        plant.command(dict(op='reset'));plant.command(dict(op='calibration_set_joints',q=plan['pre_q']));plant.settle(.8)
        plant.command(dict(op='calibration_material',mu=.7));plant.settle(.1)
        phase('RESET');initial=plant.state();r['initial']=initial['box'];r['initial_quaternion']=initial['box_quat']
        plant.command(dict(op='trace_start',path=str(trace)))
        stage='APPROACH';phase(stage);trajectory(plan['approach'])
        stage='CLOSE';phase(stage);plant.command(dict(op='arm_gain_scale',scale=2.));close_force();
        stage='CLOSE_HOLD';phase(stage);plant.settle(.5);s=plant.state()
        recent=[v for v in s['history'] if v['t']>=s['t']-.2]
        if not recent or not all(min(v['forces'])>.1 for v in recent):r['category']='BAD_CONTACT';return r
        plant.command(dict(op='calibration_gate_start'))
        stage='MICRO_LIFT';phase(stage);trajectory(plan['micro']);plant.settle(.5)
        old=plant.command(dict(op='calibration_gate_finish'))['gate'];r['old_gate']=old
        r['new_gate']=plant.command(dict(op='calibration_stability',thresholds=thresholds.__dict__,old_gate=old))['stability']
        # This diagnostic intentionally continues past either gate while physical
        # bilateral contact remains, establishing the gate's real false decisions.
        if not r['new_gate']['metrics']['terminal_bilateral']:
            r['category']='CONTACT_LOSS';return r
        trajectory(scaled(plan['lift'],10.0))
        r['post_slow_lift']=plant.command(dict(op='calibration_stability',thresholds=thresholds.__dict__,old_gate=old))['stability']
        stage='HOLD';phase(stage);start=plant.state()['t'];plant.settle(2.2);s=plant.state()
        samples=[v for v in s['history'] if start<=v['t']<=start+2.1]
        r['hold_samples']=samples
        heights=[v['z']-r['initial'][2] for v in samples];duration=samples[-1]['t']-samples[0]['t'] if samples else 0
        r.update(hold_duration_s=duration,max_lift_m=max(heights,default=0),min_hold_lift_m=min(heights,default=0))
        contact_ok=bool(samples) and all(min(v['forces'])>.1 for v in samples)
        peak=max(heights,default=0);floor=min(heights,default=0)
        if not contact_ok:
            r['category']='CONTACT_LOSS'
        elif duration<2:
            r.update(category='INSUFFICIENT_HOLD',physical_failure='INSUFFICIENT_HOLD')
        elif peak<.08:
            # An object can remain visibly clamped while slipping relative to
            # the TCP and therefore never reach the required physical height.
            # Preserve that measured cause instead of labelling it as a drop.
            post=r.get('post_slow_lift',{});metric=post.get('metrics',{});window=metric.get('windows',{}).get('200ms',{})
            converged=(metric.get('terminal_bilateral',False) and
                window.get('relative_translation_velocity_m_s',float('inf'))<=thresholds.translation_velocity_m_s and
                window.get('relative_angular_velocity_deg_s',float('inf'))<=thresholds.angular_velocity_deg_s)
            if converged:
                r.update(category='INSUFFICIENT_LIFT',physical_failure='INSUFFICIENT_LIFT',
                         stability_outcome='SETTLING_THEN_STABLE')
            else:
                cause=post.get('category','CONTINUOUS_SLIP')
                if cause not in {'ROTATIONAL_INSTABILITY','CONTINUOUS_SLIP'}:cause='CONTINUOUS_SLIP'
                r.update(category=cause,physical_failure='INSUFFICIENT_LIFT')
        elif floor<.08:
            r.update(category='DROP',physical_failure='DROP_AFTER_LIFT')
        elif peak-floor>.01:
            r['category']='CONTINUOUS_SLIP'
        else:
            r['success']=True
            r['category']='SETTLING_THEN_STABLE' if not old['passed'] else 'STABLE'
    except Exception as e:
        r.update(category='ARM_TRACKING_ERROR' if 'ARM_TRACKING_ERROR' in str(e) else 'SYSTEM_ERROR',detail=str(e))
    finally:
        r.update(stage=stage,wall_seconds=time.monotonic()-wall)
        try:r['trace']=plant.command(dict(op='trace_stop'))
        except Exception as e:r['trace_error']=str(e)
        destination.write_text(json.dumps(r,indent=2))
        print(json.dumps({k:r.get(k) for k in ['id','success','category','wall_seconds']}),flush=True)
    return r

def main():
    p=argparse.ArgumentParser();p.add_argument('--target',required=True,choices=['soup','banana','bowl','mug'])
    p.add_argument('--repeats',type=int,required=True);p.add_argument('--output',required=True);p.add_argument('--thresholds')
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    th=GateThresholds(**json.loads(Path(a.thresholds).read_text())) if a.thresholds else GateThresholds()
    for i in range(a.repeats):
        r=episode(a.target,i,out,th)
        if r['category']=='SYSTEM_ERROR':raise RuntimeError(r.get('detail',r['category']))

if __name__=='__main__':main()
