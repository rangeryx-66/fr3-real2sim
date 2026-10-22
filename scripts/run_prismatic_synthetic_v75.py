"""Analytical raw-wrench fixtures; NOT an Isaac or hardware experiment."""
import argparse,copy,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from prismatic_estimator_audit_v75 import perturb_raw,compare_qualification,stability,applicability,momentum_fit
from xarm_ft_v31_estimator import process
from xarm_ft_v7_estimator import samples,fit,prediction
from update_prismatic_v73_model import qualify
from interaction_runtime.serialization import result_packet


CONFIG=dict(domain_native=[0.,.15],intervals=[2],joint_type='prismatic',
    units=dict(coordinate='m',effort='N',inertial='kg',damping='N s/m'),
    parameter_scales=[3.,5.,.5],ridge=1e-8)
PRIOR=dict(joint_type='prismatic',axis_world=[1.,0.,0.],origin_world_m=[0.,0.,0.])


def raw_fixture(role,direction=1,speed=.0025,period=3.1,plant='constant',duration=14.):
    dt=1/240;t=np.arange(0,duration,dt);w=2*np.pi/period
    v=direction*speed*(1+.45*np.sin(w*t));q=.03+direction*speed*(t+.45*(1-np.cos(w*t))/w)
    acc=direction*speed*.45*w*np.cos(w*t)
    if plant=='static_kinetic':
        v[(t>=5)&(t<=7)]=0.;q=.03+np.cumsum(v)*dt;acc=np.gradient(v,dt)
    mass,b,C=2.,3.,.3
    kinetic=np.full(len(t),C if plant!='asymmetric' or direction>0 else .42)
    force=mass*acc+b*v+kinetic*np.sign(v)
    if plant=='static_kinetic':force[(t>=5)&(t<=7)]+=.12
    theta=.15*np.sin(.5*t);omega=np.c_[np.zeros((len(t),2)),.075*np.cos(.5*t)]
    alpha=np.c_[np.zeros((len(t),2)),-.0375*np.sin(.5*t)]
    rotations=Rotation.from_rotvec(np.c_[np.zeros((len(t),2)),theta]).as_matrix()
    tool_mass=.8;com=np.array([.01,0.,.07]);I=np.diag([.001,.002,.0015]);rows=[]
    for i,time in enumerate(t):
        R=rotations[i];p=np.array([q[i],0.,.5]);r=R@com
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=p
        gravity=np.r_[tool_mass*np.array([0.,0.,-9.81]),np.cross(r,tool_mass*np.array([0.,0.,-9.81]))]
        linear_acc=np.array([acc[i],0.,0.])+np.cross(alpha[i],r)+np.cross(omega[i],np.cross(omega[i],r))
        inertia=R@I@R.T;dynamic_force=tool_mass*linear_acc
        dynamic=np.r_[dynamic_force,inertia@alpha[i]+np.cross(omega[i],inertia@omega[i])+np.cross(r,dynamic_force)]
        interaction=np.r_[np.array([force[i],0.,0.]),np.cross([0.,0.,.15],[force[i],0.,0.])]
        world=interaction-gravity+dynamic;raw=np.r_[R.T@world[:3],R.T@world[3:]]
        body=dict(name='analytical_tool',mass_kg=tool_mass,T_world_body=T.copy(),com_world_m=p+r,inertia_world_kg_m2=inertia)
        rows.append(dict(t=float(time),phase='OPEN_CONTINUE',identification_segment=role,
            force_event=dict(status='NORMAL'),relative_translation_slip_m=0.,
            wrist_ft=dict(valid=True,raw_wrench_sensor=raw,T_world_sensor=T.copy(),
                          body_states=[body],tool_gravity_wrench_world=gravity)))
    return rows,q,v,dict(mass_kg=mass,damping_N_s_m=b,kinetic_open_N=C,
        kinetic_close_N=.42 if plant=='asymmetric'else C,static_hold_force_N=.12 if plant=='static_kinetic'else 0.)


SCENARIOS={
 'ideal':{},
 'bias_force_plus_10mN':dict(bias_sensor=[.01,0,0,0,0,0]),
 'bias_force_minus_10mN':dict(bias_sensor=[-.01,0,0,0,0,0]),
 'bias_torque_plus_1mNm':dict(bias_sensor=[0,0,0,0,.001,0]),
 'AI1500_stated_resolution_quantization':dict(quantum_sensor=[.4]*3+[.01]*3),
 'assumed_noise_1mN':dict(noise_sd_sensor=[.001]*3+[.0001]*3),
 'assumed_noise_10mN':dict(noise_sd_sensor=[.01]*3+[.001]*3),
 'assumed_noise_50mN':dict(noise_sd_sensor=[.05]*3+[.005]*3),
 'assumed_delay_one_240Hz_sample':dict(delay_samples=1),
 'assumed_delay_two_240Hz_samples':dict(delay_samples=2),
 'tool_mass_plus_one_percent':dict(tool_mass_scale=1.01),
 'tool_mass_minus_one_percent':dict(tool_mass_scale=.99),
 'tool_COM_plus_one_mm':dict(tool_com_offset_body_m=[.001,0,0]),
 'tool_COM_minus_one_mm':dict(tool_com_offset_body_m=[-.001,0,0]),
 'assumed_position_quantization_100um':dict(q_quantum_m=.0001,velocity_from_position=True),
 'assumed_position_noise_10um':dict(q_noise_sd_m=.00001,velocity_from_position=True),
}


def processed(fixture,role,scenario=None):
    rows,q,v,_=fixture
    rr,qq,vv=perturb_raw(rows,q,v,scenario or {})
    d=process(rr,PRIOR,[0.]*6,vv,native_coordinate=qq)
    d['block_id']=role+'#0'
    return d


def fit_pieces(pieces):
    rr=[]
    for d in pieces:
        if not d['segments'][0].startswith('FIT'):continue
        qualify(d);rr.extend(samples(d,CONFIG['domain_native'],CONFIG['intervals']))
    if len(rr)<8:return None,rr
    return fit(rr,CONFIG),rr


def run(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);report=dict(kind='analytical raw-input diagnostic, NOT native physics',
        hidden_real_dynamics_used=False,scenario_seed=7501,specification='AI1500 stated quantization is not Gaussian SD or full hardware noise',
        spec_source='https://docs.accessories.ufactory.cc/6_Axis_Force_Torque_Sensor/5.Technical_Specifications.html',
        control_or_production_estimator_changed=False,plants={})
    report['diagnostic_estimation_configuration']=copy.deepcopy(CONFIG)
    roles=[('FIT_ACCEL',1,.0025,3.1),('FIT_REVERSE',-1,.002,2.5),('FIT_SLOW',1,.0015,4.3),('VALIDATION_DISTINCT',-1,.0022,2.1)]
    for plant in ('constant','static_kinetic','asymmetric'):
        raw=[raw_fixture(role,sign,speed,period,plant)for role,sign,speed,period in roles]
        pp=[processed(f,roles[i][0])for i,f in enumerate(raw)];model,train=fit_pieces(copy.deepcopy(pp))
        p=dict(analytical_parameters_evaluation_only=raw[0][3],qualification=compare_qualification(pp,CONFIG,model),
               model=model,applicability=applicability(train,model),stability=stability(train,CONFIG,model),
               integral=momentum_fit([qualify_copy(d)for d in pp],CONFIG,model),scenarios={})
        for name,scenario in SCENARIOS.items():
            dd=[processed(f,roles[i][0],scenario)for i,f in enumerate(raw)];m,rr=fit_pieces(dd)
            r=dict(assumptions=scenario,bins=len(rr),qualified_samples=sum(int(d['valid'].sum())for d in dd if d['segments'][0].startswith('FIT')))
            if m:
                v=dd[-1];samples(v,CONFIG['domain_native'],CONFIG['intervals'])
                r.update(model=m,heldout=prediction(v,m),applicability=applicability(rr,m),
                         relative_parameter_error_percent=[100*(m['parameters'][i]/[2.,3.,.3][i]-1)for i in range(3)] if plant!='asymmetric'else None)
            else:r['status']='unknown: insufficient qualified samples'
            p['scenarios'][name]=r
        report['plants'][plant]=p
    (out/'SYNTHETIC_AUDIT.json').write_text(json.dumps(result_packet(report),indent=2)+'\n')
    print(json.dumps({k:dict(parameters=v['model']['parameters'],applicability=v['applicability']['status'],integral=v['integral'].get('parameters'),quantized_parameters=v['scenarios']['AI1500_stated_resolution_quantization'].get('model',{}).get('parameters'),noisy_state_bins=v['scenarios']['assumed_position_noise_10um']['bins'])for k,v in report['plants'].items()}))
    return report


def qualify_copy(d):
    result=copy.deepcopy(d);qualify(result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path)
    p.add_argument('--production-regularization',action='store_true',help='Use existing prismatic ridge and parameter scales; no truth-based tuning')
    a=p.parse_args()
    if a.production_regularization:CONFIG.update(ridge=.001,parameter_scales=[5.,10.,1.])
    run(a.output)
