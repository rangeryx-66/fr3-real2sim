"""Wrist-measurement-only five-parameter hinge model, segment-local holdouts.

Resisting torque: I*qdd + b*qd + c*sign(qd) + gs*sin(q) + gc*cos(q).
The two gravity coefficients represent effective projected first moments, not
individual mass/COM or a full inertia tensor. No scene-generator import.
"""
import numpy as np
from scipy.optimize import lsq_linear
from xarm_ft_v1_estimator import local_process, coverage

NAMES=['I_eff_kg_m2','b_Nm_s_rad','c_Nm','gravity_sin_Nm','gravity_cos_Nm']
SCALE=np.array([.4,.25,.04,.5,.5])
FLOOR=.004644


def features(samples):
    return np.array([[s['qdd'],s['qd'],s['direction'],np.sin(s['q']),np.cos(s['q'])] for s in samples])


def fit(samples):
    X=features(samples);y=np.array([s['tau'] for s in samples]);A=X*SCALE
    # Prior regularization stabilizes a provisional fit, never creates evidence.
    r=.001*np.eye(5)
    p=lsq_linear(np.vstack([A,r]),np.r_[y,np.zeros(5)],bounds=([0,0,0,-np.inf,-np.inf],np.full(5,np.inf))).x*SCALE
    residual=X@p-y;sigma=max(FLOOR,float(np.sqrt(np.mean(residual**2))))
    _,s,V=np.linalg.svd(A,full_matrices=False);rank=int(np.linalg.matrix_rank(A));cov=sigma**2*np.linalg.pinv(X.T@X)
    # Truncated/null directions must not look infinitely precise under pinv.
    null=V[s<max(s[0]*1e-8,1e-15)] if len(s) else V
    std=np.sqrt(np.maximum(np.diag(cov),0));std[np.any(abs(null)>.01,axis=0) if len(null) else np.zeros(5,dtype=bool)]=np.inf
    return dict(parameter_names=NAMES,parameters=p.tolist(),conditional_std=std.tolist(),scaled_singular_values=s.tolist(),algebraic_rank=rank,
        scaled_condition=float(s[0]/max(s[-1],1e-30)),training_rmse_Nm=float(np.sqrt(np.mean(residual**2))),
        uncertainty='conditional linear error floor; correlations and simulator error not calibrated',
        contributions_rms_Nm=np.sqrt(np.mean((X*p)**2,axis=0)).tolist(),samples=len(samples))


def predict_forward(data,ix,p):
    from scipy.integrate import solve_ivp
    I,b,c,gs,gc=p;t=data['t'][ix];truthq=data['q'][ix];truthv=data['qd'][ix];tau=data['torque'][ix]
    if I<=1e-8:return dict(status='unsupported inertia; no forward claim')
    # Coulomb stiction handled with a small integration-time proximal step;
    # no measured intermediate angle/velocity is injected after initial state.
    q=float(truthq[0]);v=float(truthv[0]);qq=[q];vv=[v]
    for j in range(1,len(t)):
        dt=float(t[j]-t[j-1]);n=max(1,int(np.ceil(dt/.001)));h=dt/n
        for _ in range(n):
            remaining=tau[j-1]-gs*np.sin(q)-gc*np.cos(q);momentum=I*v+h*remaining
            v=float(np.sign(momentum)*max(abs(momentum)-h*c,0)/(I+h*b));q+=h*v
        qq.append(q);vv.append(v)
    qq=np.array(qq);vv=np.array(vv)
    return dict(status='conditional_prediction_not_identifiability_proof',angle_RMSE_deg=float(np.rad2deg(np.sqrt(np.mean((qq-truthq)**2)))),
        final_angle_error_deg=float(np.rad2deg(qq[-1]-truthq[-1])),velocity_RMSE_rad_s=float(np.sqrt(np.mean((vv-truthv)**2))),
        intermediate_GT_state_injection=False)


def analyze(data,prior):
    samples=data['samples'];train=[s for s in samples if not s['segment'].startswith('VALIDATION')];test=[s for s in samples if s['segment'].startswith('VALIDATION')]
    axis=np.asarray(prior['axis_world']);vertical=abs(axis[2])/np.linalg.norm(axis)
    result=dict(model='constant I,b,Coulomb plus sinusoidal gravity first moments',training_bins=len(train),validation_bins=len(test),
        coverage=coverage(data),axis_vertical_fraction=float(vertical),source='compensated six-axis wrist interface; no GT joint torque',hidden_dynamics_read=False)
    if len(train)<8:return dict(result,status='insufficient independent fitting bins',identified_components=[])
    m=fit(train);result['fit']=m;X=features(train);both=any(s['qd']>0 for s in train) and any(s['qd']<0 for s in train)
    # Diagnostic observability only. These never block robot execution.
    scale=np.array(m['contributions_rms_Nm']);std=np.array(m['conditional_std']);p=np.array(m['parameters'])
    eligible=(scale>FLOOR)&(abs(p)>2*std)&(m['scaled_condition']<1e6)
    result['identifiability']=dict(both_directions=both,full_regressor_rank=m['algebraic_rank']==5,
        component_supported=[bool(eligible[0]),bool(both and eligible[1] and eligible[2]),bool(vertical<.95 and both and eligible[3:].any())],
        caveat='nonzero regularized values are not necessarily identified; narrow angle gravity/friction confounding retained')
    if test:
        predicted=features(test)@p;y=np.array([s['tau'] for s in test]);result['heldout_torque_RMSE_Nm']=float(np.sqrt(np.mean((predicted-y)**2)))
        result['heldout_forward']={}
        for label in dict.fromkeys(data['segments']):
            if str(label).startswith('VALIDATION'):
                ix=np.flatnonzero((data['segments']==label)&data['valid']);
                if len(ix)>1:result['heldout_forward'][str(label)]=predict_forward(data,ix,p)
    return result
