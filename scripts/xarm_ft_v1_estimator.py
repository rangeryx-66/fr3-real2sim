"""Segment-local measurement processing and small passive models, no hidden plant input."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import lsq_linear
from xarm_ft_id_estimator import process as frozen_process, DERIVATIVE_WINDOW

CONFIG=dict(derivative_window_samples=121,edge_exclusion_samples=121,fit_bin_s=.1,
    interior_holdout_deg=[7.,9.],model_domain_deg=[0.,20.],
    effort_floor_nm=.004644,empirical_unloaded_100ms_bin_rms_nm=.00021057,empirical_unloaded_100ms_bin_abs_P95_nm=.00049417,
    scales=[1.,.25,.045],regularization=.001,
    state_basis='quadratic Bernstein, three nonnegative Coulomb controls on 0..20deg',
    conditional_uncertainty_not_calibrated=True,quantization_only_steps=[.4,.4,.4,.01,.01,.01])
KEYS=['t','q','qd','qdd','raw','compensated_world','gravity_world','inertial_world','torque','valid','segments','raw_to_joint']


def partition(row):
    label=row.get('identification_segment') or ''
    if not label:return ''
    if label.startswith('VALIDATION'):return label
    lo,hi=CONFIG['interior_holdout_deg']
    return 'VALIDATION_INTERIOR_'+label if lo<=row['door_angle_deg']<=hi else label


def local_process(rows,prior,bias):
    """Partition BEFORE *any* smoothing, including all tool derivatives.

    Two cascaded SG operations require a full 121-step edge exclusion. Nothing
    from the other side of a segment/holdout/gap enters this process call.
    """
    groups=[];current=[];last=None;label=None
    for row in rows:
        now=partition(row);valid=row.get('wrist_ft',{}).get('valid',False)
        gap=last is not None and abs(row['t']-last-1/240)>1e-6
        if now!=label or gap or not valid:
            if current:groups.append((label,current))
            current=[]
        if now and valid:current.append(row)
        last=row['t'];label=now if valid else None
    if current:groups.append((label,current))
    arrays=[];samples=[];provenance=[]
    for label,block in groups:
        if len(block)<2*CONFIG['edge_exclusion_samples']+5:
            provenance.append(dict(segment=label,records=len(block),status='insufficient local derivative support; data retained, not execution failure'));continue
        data=frozen_process(block,prior,bias)
        poses=np.asarray([r['wrist_ft']['T_world_sensor'] for r in block]);R=poses[:,:3,:3]
        axis=np.asarray(prior['axis_world']);axis=axis/np.linalg.norm(axis)
        force_coefficient=np.cross(axis,poses[:,:3,3]-np.asarray(prior['origin_world_m']))
        data['raw_to_joint']=np.c_[np.einsum('nji,nj->ni',R,force_coefficient),np.einsum('nji,j->ni',R,axis)]
        edge=CONFIG['edge_exclusion_samples'];data['valid'][:edge]=False;data['valid'][-edge:]=False
        data['segments']=np.full(len(block),label)
        samples.extend(regression_samples(data))
        arrays.append(data);provenance.append(dict(segment=label,records=len(block),start=block[0]['t'],end=block[-1]['t'],valid=int(data['valid'].sum()),source_interval_only=True))
    if not arrays:return None,provenance
    result={k:np.concatenate([v[k] for v in arrays],axis=0) for k in KEYS};result['samples']=samples
    return result,provenance


def regression_samples(data):
    result=[]
    for segment in dict.fromkeys(data['segments']):
        ids=np.flatnonzero((data['segments']==segment)&data['valid'])
        for group in np.unique(np.floor(data['t'][ids]/CONFIG['fit_bin_s']).astype(int)):
            ix=ids[np.floor(data['t'][ids]/CONFIG['fit_bin_s']).astype(int)==group]
            if len(ix)<5:continue
            result.append(dict(segment=str(segment),t=float(data['t'][ix].mean()),q=float(data['q'][ix].mean()),
                qd=float(data['qd'][ix].mean()),qdd=float(data['qdd'][ix].mean()),direction=float(np.sign(data['qd'][ix]).mean()),tau=float(data['torque'][ix].mean())))
    return result


def features(samples,kind):
    q=np.array([s['q'] for s in samples]);v=np.array([s['qd'] for s in samples]);acc=np.array([s['qdd'] for s in samples]);direction=np.array([s['direction'] for s in samples])
    if kind=='constant':return np.c_[acc,v,direction]
    x=np.clip(q/np.deg2rad(CONFIG['model_domain_deg'][1]),0,1)
    return np.c_[acc,v,direction*(1-x)**2,direction*2*x*(1-x),direction*x*x]


def fit(samples,kind):
    X=features(samples,kind);y=np.array([s['tau'] for s in samples]);scale=np.array([1.,.25]+[.045]*(X.shape[1]-2));A=X*scale
    ridge=CONFIG['regularization']*np.eye(X.shape[1])
    # Same magnitude regularization, plus smooth Coulomb-control differences.
    if kind!='constant':
        D=np.zeros((2,5));D[0,2:4]=[-1,1];D[1,3:5]=[-1,1];ridge=np.vstack([ridge,.001*D])
    estimate=lsq_linear(np.vstack([A,ridge]),np.r_[y,np.zeros(len(ridge))],bounds=(0,np.inf)).x*scale
    error=X@estimate-y;floor=CONFIG['effort_floor_nm'];sigma2=max(float(np.mean(error**2)),floor**2)
    std=np.sqrt(np.maximum(np.diag(sigma2*np.linalg.pinv(X.T@X)),0));singular=np.linalg.svd(A,compute_uv=False)
    return dict(kind=kind,parameters=estimate.tolist(),conditional_std=std.tolist(),training_rmse_nm=float(np.sqrt(np.mean(error**2))),
        conditional_covariance=(sigma2*np.linalg.pinv(X.T@X)).tolist(),
        scaled_regressor_singular_values=singular.tolist(),scaled_condition=float(singular[0]/max(singular[-1],1e-30)),
        scaled_fisher_information=(A.T@A/floor**2).tolist(),algebraic_rank=int(np.linalg.matrix_rank(A)),
        supported=[bool(b>2*u) for b,u in zip(estimate,std)],samples=len(samples),
        uncertainty='conditional, correlated bins/solver residual not calibrated confidence; no execution gate')


def forward(data,indices,model):
    I,b=model['parameters'][:2]
    if I<=1e-8:return dict(status='unavailable: degenerate inertia; no GT/prior substitution')
    t=data['t'][indices];q=data['q'][indices];v=data['qd'][indices];tau=data['torque'][indices];qp=[float(q[0])];vp=[float(v[0])]
    for j in range(1,len(t)):
        dt=float(t[j]-t[j-1]);z=qp[-1];x=np.clip(z/np.deg2rad(20),0,1)
        c=model['parameters'][2] if model['kind']=='constant' else np.array([(1-x)**2,2*x*(1-x),x*x])@model['parameters'][2:]
        momentum=I*vp[-1]+dt*tau[j-1];vel=float(np.sign(momentum)*max(abs(momentum)-dt*c,0)/(I+dt*b))
        vp.append(vel);qp.append(qp[-1]+dt*vel)
    qp=np.asarray(qp);vp=np.asarray(vp)
    return dict(status='predicted',samples=len(t),duration_s=float(t[-1]-t[0]),angle_rmse_deg=float(np.rad2deg(np.sqrt(np.mean((qp-q)**2)))),
        final_error_deg=float(np.rad2deg(qp[-1]-q[-1])),velocity_rmse_rad_s=float(np.sqrt(np.mean((vp-v)**2))),
        state_trajectory_forcing=False,supported_inertia=bool(model['supported'][0]),prediction_outside_observed_domain=bool(np.any((qp<0)|(qp>np.deg2rad(20)))))


def coverage(data):
    output=[]
    for lo,hi in [(0,3),(3,6),(6,10),(10,15),(15,20),(20,30)]:
        ix=(np.rad2deg(data['q'])>=lo)&(np.rad2deg(data['q'])<hi)&data['valid']
        if not ix.any():continue
        output.append(dict(angle_region_deg=[lo,hi],samples=int(ix.sum()),positive_velocity_samples=int(np.sum(data['qd'][ix]>0)),negative_velocity_samples=int(np.sum(data['qd'][ix]<0)),
            measured_angle_extent_deg=np.rad2deg([data['q'][ix].min(),data['q'][ix].max()]).tolist(),velocity_range_rad_s=[float(data['qd'][ix].min()),float(data['qd'][ix].max())],
            acceleration_range_rad_s2=[float(data['qdd'][ix].min()),float(data['qdd'][ix].max())],velocity_rms=float(np.sqrt(np.mean(data['qd'][ix]**2))),acceleration_rms=float(np.sqrt(np.mean(data['qdd'][ix]**2))),
            inertia_effort_sensitivity_for_prior_I_0p1_to_0p7_nm=(np.array([.1,.7])*np.sqrt(np.mean(data['qdd'][ix]**2))).tolist(),
            effort_floor_nm=CONFIG['effort_floor_nm'],observed_100ms_bin_abs_P95_nm=CONFIG['empirical_unloaded_100ms_bin_abs_P95_nm']))
    return output


def analyze(data):
    samples=data['samples'];train=[s for s in samples if not s['segment'].startswith('VALIDATION')];test=[s for s in samples if s['segment'].startswith('VALIDATION')]
    result=dict(configuration=CONFIG,training_samples=len(train),validation_samples=len(test),coverage=coverage(data),models={},
        entire_validation_segments_excluded=True,derivatives_computed_inside_each_partition_only=True,
        unidentifiable=['mass/full COM/full inertia tensor','gravity vs spring with vertical axis/narrow range'],
        sensor_model='ideal fixed-interface load cell; hardware accuracy not reproduced')
    if len(train)<8:return dict(result,status='insufficient informative fitting samples')
    for kind in ['constant','state_coulomb']:
        model=fit(train,kind)
        if test:model['heldout_torque_rmse_nm']=float(np.sqrt(np.mean((features(test,kind)@model['parameters']-np.array([s['tau'] for s in test]))**2)))
        cv=[]
        for label in dict.fromkeys(s['segment'] for s in train):
            a=[s for s in train if s['segment']!=label];b=[s for s in train if s['segment']==label]
            if len(a)<8:continue
            m=fit(a,kind);cv.append(dict(segment=label,rmse_nm=float(np.sqrt(np.mean((features(b,kind)@m['parameters']-np.array([s['tau'] for s in b]))**2)))))
        model['FIT_only_leave_segment_out']=cv
        model['heldout_motion']={}
        for label in dict.fromkeys(data['segments']):
            if not str(label).startswith('VALIDATION'):continue
            ids=np.flatnonzero(data['segments']==label)
            # Split disjoint traversals, never bridge omitted motion/time gaps.
            runs=np.split(ids,np.flatnonzero(np.diff(data['t'][ids])>.01)+1)
            model['heldout_motion'][str(label)]=[forward(data,run,model) for run in runs if len(run)>1]
        result['models'][kind]=model
    cvs={k:np.mean([c['rmse_nm'] for c in v['FIT_only_leave_segment_out']]) if v['FIT_only_leave_segment_out'] else float('inf') for k,v in result['models'].items()}
    # Model choice uses FIT-only errors, never validation values or hidden truth.
    result['selected_model']='state_coulomb' if len(result['coverage'])>=3 and cvs['state_coulomb']<.9*cvs['constant'] else 'constant'
    result['selection_basis']='at least 3 observed regions + >10% FIT-only leave-segment-out improvement; ranking/model choice only, not motion admission'
    return result


def candidate_information(data,result,blocks):
    """Measured-excitation surrogate for Agent inspection, never a safety gate."""
    if not result.get('models'):return []
    kind=result['selected_model'];model=result['models'][kind]
    train=[s for s in data['samples'] if not s['segment'].startswith('VALIDATION')]
    X=features(train,kind);scale=np.array([1.,.25]+[.045]*(X.shape[1]-2))
    F=(X*scale).T@(X*scale)/CONFIG['effort_floor_nm']**2
    v=max(1e-5,float(np.quantile(abs(data['qd'][data['valid']]),.9)))
    acceleration=max(1e-6,float(np.quantile(abs(data['qdd'][data['valid']]),.9)))
    current=float(data['q'][-1]);outputs=[]
    for name,actions in blocks.items():
        if not isinstance(actions,list):continue
        predicted=[];q0=current
        for action in actions:
            q1=float(np.deg2rad(action['target_deg']));sign=np.sign(q1-q0)
            speed=v*float(action.get('reference_speed_m_s',.0015*action['speed_scale']))/.0015
            for j,q in enumerate(np.linspace(q0,q1,32)):
                modulation=1. if action.get('profile')=='constant' else .65+.35*np.sin(j*np.pi/8)**2
                predicted.append(dict(q=q,qd=sign*speed*modulation,qdd=sign*acceleration*np.cos(j*np.pi/8),direction=sign,tau=0))
            q0=q1
        A=features(predicted,kind)*scale;future=F+A.T@A/CONFIG['effort_floor_nm']**2
        gain=float(np.linalg.slogdet(future+np.eye(len(F))*.001)[1]-np.linalg.slogdet(F+np.eye(len(F))*.001)[1])
        torque=features(predicted,kind)@model['parameters']
        outputs.append(dict(menu=name,scaled_logdet_information_gain_surrogate=gain,
            predicted_torque_range_nm=[float(torque.min()),float(torque.max())],
            predicted_angle_range_deg=np.rad2deg([min(p['q'] for p in predicted),max(p['q'] for p in predicted)]).tolist(),
            surrogate_uses_measured_velocity_acceleration=True,not_physical_feasibility_or_success_probability=True))
    return outputs


def resolution_sensitivity(data):
    """Quantization ONLY. No hardware noise/precision/calibration claim."""
    # projection coefficients can be persisted by the numeric worker, so raw
    # ADC rounding is transformed and moment-shifted before refitting.
    if 'raw_to_joint' not in data:return dict(status='projection coefficients unavailable')
    results=[];steps=np.asarray(CONFIG['quantization_only_steps'])
    for offset in [0.,.25,.5,.75]:
        rounded=(np.round(data['raw']/steps-offset)+offset)*steps
        delta=np.einsum('ij,ij->i',rounded-data['raw'],data['raw_to_joint'])
        clone=dict(data,torque=data['torque']+delta);clone['samples']=regression_samples(clone)
        m=analyze(clone)
        results.append(dict(offset_LSB=offset,torque_change_rms_nm=float(np.sqrt(np.mean(delta**2))),models={k:dict(parameters=v['parameters'],heldout_torque_rmse_nm=v.get('heldout_torque_rmse_nm')) for k,v in m.get('models',{}).items()}))
    return dict(method='sensor-local component rounding, declared resolution only; no real accuracy/noise reproduction',results=results)
