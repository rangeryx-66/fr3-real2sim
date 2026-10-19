"""Versioned angle resistance: measured-only V3.1 pipeline, no plant imports."""
import math
import numpy as np
from scipy.optimize import lsq_linear
from xarm_ft_v31_estimator import common_filter,regression_samples
from update_probing_v31_model import conditional_model

VERSION='V5-common-SG121-Bernstein'
CONFIG=dict(domain_deg=[0.,20.],degrees=[2,3],smoothness=[.0001,.001,.01],
    ridge=.001,minimum_cv_improvement_fraction=.10,minimum_FIT_trajectories=3,
    selection='complete FIT-trajectory cross-validation only; validation never selects model',
    uncertainty='conditional Bartlett-HAC with iid floor; systematic uncertainty uncalibrated')


def basis(q,degree):
    x=np.asarray(q)/np.deg2rad(CONFIG['domain_deg'][1])
    # No clipping of unseen angles to fake observed support.
    return np.column_stack([math.comb(degree,j)*x**j*(1-x)**(degree-j) for j in range(degree+1)])


def samples_from_piece(d):
    result=regression_samples(d);groups=np.floor(d['t']/.1).astype(np.int64)
    filtered={k:common_filter(basis(d['q'],k)*np.sign(d['native_qd'])[:,None],d['processing']['window']) for k in CONFIG['degrees']}
    for s in result:
        group=int(np.floor(s['t']/.1));ix=np.flatnonzero(d['valid']&(groups==group))
        if not len(ix):raise ValueError('regression bin mismatch')
        s['correlation_group']=d['block_id']
        s['angle_features']={str(k):v[ix].mean(axis=0).tolist() for k,v in filtered.items()}
    d['angle_features']=filtered
    return result


def matrix(samples,degree):
    return np.array([[s['qdd'],s['qd']]+(s['angle_features'][str(degree)] if degree else [s['direction']]) for s in samples])


def fit(samples,degree,smoothness):
    if degree==0:
        m=conditional_model(samples);m.update(degree=0,smoothness=0.,kind='constant');return m
    X=matrix(samples,degree);y=np.array([s['tau'] for s in samples]);scale=np.array([1.,.25]+[.045]*(degree+1));A=X*scale
    D=np.zeros((degree-1,degree+3))
    for j in range(degree-1):D[j,2+j:5+j]=[1.,-2.,1.]
    ridge=np.vstack([CONFIG['ridge']*np.eye(len(scale)),smoothness*D]);P=(ridge/scale).T@(ridge/scale)
    beta=lsq_linear(np.vstack([A,ridge]),np.r_[y,np.zeros(len(ridge))],bounds=(0,np.inf),tol=1e-12).x*scale
    error=y-X@beta;n=len(y);rank=np.linalg.matrix_rank(A);bread=np.linalg.pinv(X.T@X+P);Z=X*error[:,None];meat=Z.T@Z
    labels=np.array([s.get('correlation_group',s['segment']) for s in samples]);t=np.array([s['t'] for s in samples]);lags=5
    for lag in range(1,min(lags+1,n)):
        good=(labels[lag:]==labels[:-lag])&(abs(t[lag:]-t[:-lag]-.1*lag)<.06)
        cross=Z[lag:][good].T@Z[:-lag][good];meat+=(1-lag/(lags+1))*(cross+cross.T)
    hc=bread@meat@bread;var=max(float(error@error/max(n-rank,1)),1e-12);iid=var*bread@(X.T@X)@bread
    cov=(hc+hc.T)/2+np.diag(np.maximum(np.diag(iid)-np.diag(hc),0));val,vec=np.linalg.eigh(cov);cov=(vec*np.maximum(val,0))@vec.T
    sv=np.linalg.svd(A,compute_uv=False)
    return dict(kind='angle_coulomb',degree=degree,smoothness=smoothness,parameters=beta.tolist(),conditional_std=np.sqrt(np.diag(cov)).tolist(),conditional_covariance=cov.tolist(),algebraic_rank=int(rank),scaled_condition=float(sv[0]/max(sv[-1],1e-30)),scaled_regressor_singular_values=sv.tolist(),training_rmse_nm=float(np.sqrt(np.mean(error**2))),training_angle_range_deg=np.rad2deg([min(s['q'] for s in samples),max(s['q'] for s in samples)]).tolist(),supported=[bool(rank==len(beta) and b>3*s) for b,s in zip(beta,np.sqrt(np.diag(cov)))],uncertainty=CONFIG['uncertainty'],validation_used_for_parameters_or_uncertainty=False)


def curve(model):
    lo,hi=model['training_angle_range_deg'];q=np.linspace(0,20,81);degree=model['degree'];B=basis(np.deg2rad(q),degree) if degree else np.ones((len(q),1))
    beta=np.array(model['parameters']);cov=np.array(model['conditional_covariance'])[2:,2:];mean=B@beta[2:];std=np.sqrt(np.maximum(np.einsum('ij,jk,ik->i',B,cov,B),0));inside=(q>=lo)&(q<=hi)
    return dict(angle_deg=q.tolist(),resistance_nm=[float(v) if ok else None for v,ok in zip(mean,inside)],conditional_std_nm=[float(v) if ok else None for v,ok in zip(std,inside)],support_deg=[lo,hi],unvisited='unknown; no extrapolated precision',observability='conditional on nuisance I,b and measured excitation; systematic/contact error excluded')


def analyze(train):
    if len(train)<8:return dict(status='insufficient FIT')
    constant=fit(train,0,0);candidates=[];labels=list(dict.fromkeys(s['segment'] for s in train))
    for degree,smoothness in [(0,0)]+[(k,l) for k in CONFIG['degrees'] for l in CONFIG['smoothness']]:
        m=constant if degree==0 else fit(train,degree,smoothness);cv=[]
        for label in labels:
            tr=[s for s in train if s['segment']!=label];te=[s for s in train if s['segment']==label]
            if len(tr)<max(8,degree+4):continue
            cm=fit(tr,degree,smoothness);err=matrix(te,degree)@cm['parameters']-np.array([s['tau'] for s in te])
            cv.append(dict(segment=label,rmse_nm=float(np.sqrt(np.mean(err**2)))))
        m['FIT_only_leave_trajectory_out']=cv;m['cv_rmse_nm']=float(np.mean([v['rmse_nm'] for v in cv])) if len(cv)==len(labels) and len(labels)>=2 else None;candidates.append(m)
    eligible=[m for m in candidates[1:] if m['cv_rmse_nm'] is not None and m['algebraic_rank']==len(m['parameters'])]
    best=min(eligible,key=lambda m:m['cv_rmse_nm']) if eligible else None
    selected=constant
    if len(labels)>=CONFIG['minimum_FIT_trajectories'] and best and constant['cv_rmse_nm'] is not None and best['cv_rmse_nm']<(1-CONFIG['minimum_cv_improvement_fraction'])*constant['cv_rmse_nm']:selected=best
    for m in candidates:m['resistance_curve']=curve(m)
    return dict(status='updated',constant_model=constant,extended_model=best,selected_model='constant' if selected['degree']==0 else 'angle_coulomb',selected=selected,candidates=candidates,selection_uses='FIT-only trajectory CV, fixed >=10% improvement; no truth or VALIDATION',FIT_trajectories=len(labels),coverage_deg=constant['training_angle_range_deg'])


def prediction(d,model):
    edge=d['processing']['boundary_exclusion_samples'];ix=np.arange(edge,len(d['t'])-edge);t=d['t'][ix];q=d['q'][ix];v=d['native_qd'][ix];tau=d['torque'][ix]
    I,b=model['parameters'][:2];degree=model['degree'];qp=[float(q[0])];vp=[float(v[0])]
    if I<=1e-8:return dict(status='unknown inertia; no forward prediction')
    lo,hi=np.deg2rad(model['training_angle_range_deg']);outside=False
    for j in range(1,len(t)):
        z=qp[-1];outside=outside or not lo<=z<=hi
        # Predictions beyond observed support are flagged, never corrected to data.
        c=float(basis(np.array([z]),degree)[0]@model['parameters'][2:]) if degree else model['parameters'][2]
        dt=t[j]-t[j-1];mom=I*vp[-1]+dt*tau[j-1];vel=np.sign(mom)*max(abs(mom)-dt*max(c,0.),0)/(I+dt*b)
        vp.append(float(vel));qp.append(qp[-1]+dt*vel)
    qp=np.array(qp);vp=np.array(vp);X=np.c_[d['qdd'][ix],d['qd'][ix],d['angle_features'][degree][ix]] if degree else np.c_[d['qdd'][ix],d['qd'][ix],d['direction'][ix]]
    return dict(status='predicted',samples=len(ix),duration_s=float(t[-1]-t[0]),torque_rmse_nm=float(np.sqrt(np.mean((tau-X@model['parameters'])**2))),angle_rmse_deg=float(np.rad2deg(np.sqrt(np.mean((qp-q)**2)))),velocity_rmse_rad_s=float(np.sqrt(np.mean((vp-v)**2))),final_error_deg=float(np.rad2deg(qp[-1]-q[-1])),prediction_outside_training_support=bool(outside),input='same once-filtered held-out measured wrist F/T; one initial native q/qd; no state correction',filter_lookahead_s=d['processing']['filter_lookahead_s'])
