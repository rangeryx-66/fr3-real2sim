"""Measurement-only constant-parameter ID. No scene generator or hidden truth import."""
import json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from xarm_wrist_ft import generalized_effort,tool_inertial_world

DERIVATIVE_WINDOW=121  # 0.504s, centered updates have 0.25s latency


def process(rows,prior,bias):
    usable=[r for r in rows if r.get('wrist_ft',{}).get('valid')]
    if len(usable)<DERIVATIVE_WINDOW:return None
    ft=[r['wrist_ft'] for r in usable];t=np.array([r['t'] for r in usable]);dt=float(np.median(np.diff(t)))
    q=np.deg2rad([r['door_angle_deg'] for r in usable]);win=DERIVATIVE_WINDOW
    velocity=savgol_filter(q,win,3,deriv=1,delta=dt);acceleration=savgol_filter(q,win,3,deriv=2,delta=dt)
    bodies=[[] for _ in ft]
    for j in range(len(ft[0]['body_states'])):
        b=[r['body_states'][j] for r in ft];p=np.array([v['com_world_m'] for v in b]);rot=np.array([v['T_world_body'] for v in b])[:,:3,:3]
        a=savgol_filter(p,win,3,deriv=2,delta=dt,axis=0)
        omega=np.vstack([np.zeros(3),Rotation.from_matrix(rot[1:]@rot[:-1].transpose(0,2,1)).as_rotvec()/dt])
        omega=savgol_filter(omega,win,3,axis=0);alpha=savgol_filter(omega,win,3,deriv=1,delta=dt,axis=0)
        for i,v in enumerate(b):bodies[i].append(dict(**v,com_acceleration_world_m_s2=a[i],omega_world_rad_s=omega[i],alpha_world_rad_s2=alpha[i]))
    raw=np.array([f['raw_wrench_sensor'] for f in ft]);T=np.array([f['T_world_sensor'] for f in ft]);R=T[:,:3,:3]
    unbiased=raw-np.asarray(bias);w=np.concatenate([np.einsum('nij,nj->ni',R,unbiased[:,:3]),np.einsum('nij,nj->ni',R,unbiased[:,3:])],axis=1)
    gravity=np.array([f['tool_gravity_wrench_world'] for f in ft]);inertial=np.array([tool_inertial_world(b,T[i,:3,3]) for i,b in enumerate(bodies)])
    compensated=w+gravity-inertial
    axis=np.asarray(prior['axis_world']);origin=np.asarray(prior['origin_world_m'])
    torque=np.array([axis@(v[3:]+np.cross(T[i,:3,3]-origin,v[:3])) for i,v in enumerate(compensated)])
    phase=np.array([r['phase'] for r in usable]);tags=[];valid=np.ones(len(usable),bool)
    for i,r in enumerate(usable):
        reasons=[]
        if not r.get('identification_segment'):reasons.append('not an interaction segment')
        if i<win//2 or i>=len(usable)-win//2:reasons.append('derivative edge/latency')
        if r.get('force_event',{}).get('status') not in [None,'NORMAL']:reasons.append('original force event')
        if r.get('relative_translation_slip_m',0)>.005:reasons.append('relative motion: invalid for effort; manipulation unchanged')
        if abs(velocity[i])<1e-4:reasons.append('stiction/reversal/derivative direction uncertain')
        if max(r.get('forces_n',{}).values(),default=0)==0 and abs(velocity[i])<1e-4:reasons.append('no positive normal report and no progress')
        tags.append(reasons);valid[i]=not reasons
    # Native rate retained; fit one mean per nonoverlapping 0.1s bin to avoid
    # treating correlated 240Hz/filtered samples as independent evidence.
    segments=np.array([r.get('identification_segment','') for r in usable]);samples=[]
    for name in dict.fromkeys(segments):
        if not name:continue
        ids=np.flatnonzero((segments==name)&valid)
        for group in np.unique(np.floor(t[ids]/.1).astype(int)):
            ix=ids[np.floor(t[ids]/.1).astype(int)==group]
            if len(ix)<5:continue
            samples.append(dict(segment=name,t=float(t[ix].mean()),x=[float(acceleration[ix].mean()),float(velocity[ix].mean()),float(np.sign(velocity[ix]).mean())],tau=float(torque[ix].mean()),q=float(q[ix].mean())))
    return dict(t=t,q=q,qd=velocity,qdd=acceleration,raw=raw,compensated_world=compensated,gravity_world=gravity,inertial_world=inertial,torque=torque,valid=valid,tags=tags,segments=segments,samples=samples)


def estimate(processed,output,prior):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);samples=processed['samples']
    train=[s for s in samples if not s['segment'].startswith('VALIDATION')];test=[s for s in samples if s['segment'].startswith('VALIDATION')]
    result=dict(supported_samples=len(train),heldout_samples=len(test),kinematic_prior=prior,
        model='tau=I_eff*qdd+b*qd+tau_c*sign(qd)',terms_not_identified=['mass','full COM','full inertia tensor','angle-dependent friction','spring vs gravity'],
        gravity_axis_projection=float(abs(np.asarray(prior['axis_world'])@np.array([0,0,1]))),
        measurement='fixed-joint wrist F/T minus bias plus tool gravity minus tool inertia; no GT object effort',
        derivative_window_samples=DERIVATIVE_WINDOW,update_latency_s=.25,data_tags='not execution gates',
        uncertainty_note='conditional linear covariance, not calibrated hardware confidence; correlated sensor/solver residuals remain')
    if len(train)>=4:
        X=np.array([s['x'] for s in train]);y=np.array([s['tau'] for s in train]);scale=np.array([1.,.25,.045])
        A=X*scale;singular=np.linalg.svd(A,compute_uv=False);rank=int(np.linalg.matrix_rank(A));regularization=1e-3
        fit=lsq_linear(np.vstack([A,regularization*np.eye(3)]),np.r_[y,np.zeros(3)],bounds=(0,np.inf));beta=fit.x*scale
        residual=X@beta-y;sigma2=max(float(np.mean(residual**2)),.0037**2)
        covariance=sigma2*np.linalg.pinv(X.T@X);uncertainty=np.sqrt(np.maximum(np.diag(covariance),0))
        result.update(parameters=dict(zip(['I_eff_kg_m2','b_nm_s_rad','tau_c_nm'],beta.tolist())),conditional_std=dict(zip(['I_eff_kg_m2','b_nm_s_rad','tau_c_nm'],uncertainty.tolist())),
            rank=rank,scaled_singular_values=singular.tolist(),scaled_condition=float(singular[0]/max(singular[-1],1e-30)),training_rmse_nm=float(np.sqrt(np.mean(residual**2))),regularization=regularization,
            observability={n:bool(rank==3 and b>2*u) for n,b,u in zip(['I_eff','b','tau_c'],beta,uncertainty)},valid_range_rad=[float(min(s['q'] for s in train)),float(max(s['q'] for s in train))])
        if test:
            xx=np.array([s['x'] for s in test]);yy=np.array([s['tau'] for s in test]);result['heldout_torque_rmse_nm']=float(np.sqrt(np.mean((xx@beta-yy)**2)))
    else:result['status']='insufficient informative interaction data'
    (output/'IDENTIFIED_MODEL.json').write_text(json.dumps(result,indent=2))
    np.savez_compressed(output/'measured_effort.npz',**{k:processed[k] for k in ['t','q','qd','qdd','raw','compensated_world','gravity_world','inertial_world','torque','valid','segments']})
    (output/'VALIDITY_TAGS.json').write_text(json.dumps(processed['tags']))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True)
    ax[0].plot(processed['t'],np.rad2deg(processed['q']));ax[0].set_ylabel('known hinge angle deg')
    ax[1].plot(processed['t'],processed['torque'],lw=.5);ax[1].set_ylabel('F/T-derived effort Nm')
    ax[2].plot(processed['t'],processed['qd'],label='qd rad/s');ax[2].plot(processed['t'],processed['qdd'],label='qdd rad/s2');ax[2].legend()
    for a in ax:a.grid(alpha=.3)
    ax[-1].set_xlabel('simulation seconds');fig.tight_layout();fig.savefig(output/'FT_MOTION.png',dpi=140);plt.close(fig)
    return result
