"""Fixed-GT-grasp force/friction trials. No perception, selection or weld."""
import argparse,json,time,sys,hashlib,gzip,os,random
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));import plant
plant.URL='http://127.0.0.1:'+os.environ.get('CALIBRATION_PORT','18765')

def relative(s):
    H=Rotation.from_quat(np.roll(s['tcp_quat'],-1)).as_matrix();O=Rotation.from_quat(np.roll(s['box_quat'],-1)).as_matrix()
    T=np.eye(4);T[:3,:3]=H.T@O;T[:3,3]=H.T@(np.array(s['box'])-s['tcp']);return T
def phase(name):plant.command(dict(op='calibration_phase',phase=name))
def trajectory(plan):
    r=plant.command(dict(op='trajectory',**plan))
    if not r['ok']:raise RuntimeError('ARM_TRACKING_ERROR '+str(r))
def episode(name,label,force,mu,controller,out):
    planpath=ROOT/f'results/hand_calibration/gt_path_{name}.json';plan=json.loads(planpath.read_text())
    dest=out/f'{label}.json'
    if dest.exists():return json.loads(dest.read_text())
    r=dict(id=label,target=name,force_total_N=force,mu=mu,controller=controller,success=False,category='SYSTEM_ERROR',gt_path_sha256=hashlib.sha256(planpath.read_bytes()).hexdigest())
    start=time.monotonic();tracepath=out/f'{label}.trace.json.gz';stage='RESET'
    try:
        plant.command(dict(op='reset'));plant.command(dict(op='calibration_set_joints',q=plan['pre_q']));plant.settle(.8)
        material=plant.command(dict(op='calibration_material',mu=mu))
        if not material['ok']:raise RuntimeError(str(material))
        plant.settle(.1);r['runtime_material']=plant.command(dict(op='calibration_audit'))
        phase('RESET');initial=plant.state();r['initial']=initial['box'];r['initial_quaternion']=initial['box_quat']
        plant.command(dict(op='trace_start',path=str(tracepath)))
        stage='APPROACH';phase(stage);trajectory(plan['approach']);s=plant.state();r['approach_actual']=dict(tcp=s['tcp'],tcp_quat=s['tcp_quat'],box=s['box'],box_quat=s['box_quat'])
        if any(max(x['forces'])>.1 for x in s['history'] if x['t']>=initial['t']):r['category']='PRECONTACT';return r
        stage='CLOSE';phase(stage);plant.command(dict(op='arm_gain_scale',scale=2.))
        if controller=='position':
            p=dict(names=['fr3_finger_joint1','fr3_finger_joint2'],points=[dict(t=1.,q=[0.,0.])],gripper=True)
            plant.command(dict(op='trajectory',**p));plant.settle(.5)
        else:
            cmd=plant.command(dict(op='calibration_force',force_N=force))
            if not cmd['ok']:raise RuntimeError(str(cmd))
            t0=plant.state()['t'];stable=None
            while True:
                s=plant.state('calibration');f=np.array(s['calibration']['filtered_force_N']);good=abs(float(f.sum())-force)<=max(2.,force*.2) and min(s['forces'])>.1
                stable=s['t'] if good and stable is None else stable if good else None
                if stable is not None and s['t']-stable>=.3:break
                if s['t']-t0>5.:r['category']='FORCE_UNREACHED';return r
                time.sleep(.01)
        stage='CLOSE_HOLD';phase(stage);plant.settle(.5);s=plant.state();r['close_T_TCP_target']=relative(s).tolist()
        recent=[v for v in s['history'] if v['t']>=s['t']-.2]
        r['bilateral_close']=bool(recent) and all(min(v['forces'])>.1 for v in recent)
        if not r['bilateral_close']:r['category']='BAD_CONTACT';return r
        plant.command(dict(op='calibration_gate_start'));stage='MICRO_LIFT';phase(stage);trajectory(plan['micro']);plant.settle(.2)
        answer=plant.command(dict(op='calibration_gate_finish'))
        if not answer['ok']:raise RuntimeError(str(answer))
        r['micro_gate']=answer['gate']
        if not r['micro_gate']['passed']:r['category']='UNSTABLE_GRASP';return r
        stage='LIFT';phase(stage);trajectory(plan['lift'])
        stage='HOLD';phase(stage);t0=plant.state()['t'];plant.settle(2.2);s=plant.state();samples=[v for v in s['history'] if t0<=v['t']<=t0+2.1]
        r['hold_samples']=samples;r['final_T_TCP_target']=relative(s).tolist()
        heights=[v['z']-r['initial'][2] for v in samples];r['hold_duration_s']=samples[-1]['t']-samples[0]['t'] if samples else 0
        if r['hold_duration_s']<2.:r['category']='DROP'
        elif max(heights)<.08:r['category']='INSUFFICIENT_LIFT'
        elif min(heights)<.08:r['category']='DROP'
        elif max(heights)-min(heights)>.01 or not all(min(v['forces'])>.1 for v in samples):r['category']='SLIP'
        else:r.update(success=True,category='SUCCESS')
    except Exception as e:r.update(category='ARM_TRACKING_ERROR' if 'ARM_TRACKING_ERROR' in str(e) else 'SYSTEM_ERROR',detail=str(e))
    finally:
        r['stage']=stage;r['wall_seconds']=time.monotonic()-start
        try:r['trace']=plant.command(dict(op='trace_stop'))
        except Exception as e:r['trace_error']=str(e)
        dest.write_text(json.dumps(r,indent=2));print(json.dumps({k:r.get(k) for k in ['id','success','category','stage','wall_seconds']}),flush=True)
    return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--target',required=True);p.add_argument('--output',required=True);p.add_argument('--preflight',action='store_true');p.add_argument('--mu',type=float);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    cases=[('force',f,mu,rep) for mu in [.3,.5,.7,1.] for f in [10,20,30,40,60] for rep in range(3)]
    cases=[('position',0,.7,rep) for rep in range(3)]+cases
    random.Random(20260910).shuffle(cases)
    if a.mu is not None:cases=[c for c in cases if c[2]==a.mu]
    if a.preflight:cases=[('force',30,.7,0)]
    for controller,f,mu,rep in cases:
        label=f'{a.target}_{controller}_F{f:02d}_mu{mu:.1f}_r{rep}'
        r=episode(a.target,label,f,mu,controller,out)
        if r['category'] in ['SYSTEM_ERROR','PRECONTACT']:raise RuntimeError('Invalid calibration setup: '+str(r.get('detail',r['category'])))
    (out/f'{a.target}_mu{a.mu}_COMPLETE.json').write_text(json.dumps(dict(target=a.target,n=len(cases))))
