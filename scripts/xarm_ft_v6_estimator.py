"""Local nonnegative resistance on -5..50deg; V3.1 measurements unchanged."""
import numpy as np
from scipy.interpolate import BSpline
from xarm_ft_v31_estimator import common_filter,regression_samples
from xarm_ft_v5_estimator import fit as ridge_fit

VERSION='V6-common-SG121-local-BSpline'
CONFIG=dict(domain_deg=[-5.,50.],reported_domain_deg=[0.,45.],spline_degree=3,resolutions_deg=[5.,10.,15.],smoothness=[.0001,.001,.01],ridge=.001,minimum_FIT_trajectories=3,minimum_cv_improvement_fraction=.10,selection='complete FIT-trajectory CV only; no VALIDATION/truth',uncertainty='conditional Bartlett-HAC/iid floor; excludes model selection and systematic error')


def knots(spacing):
    lo,hi=CONFIG['domain_deg'];return np.deg2rad([lo]*4+list(np.arange(lo+spacing,hi-1e-8,spacing))+[hi]*4)


def basis(q,spacing):
    t=knots(spacing);return BSpline(t,np.eye(len(t)-4),3,extrapolate=False)(np.asarray(q))


def samples_from_piece(d):
    result=regression_samples(d);groups=np.floor(d['t']/.1).astype(int);filtered={s:common_filter(basis(d['q'],s)*np.sign(d['native_qd'])[:,None],d['processing']['window']) for s in CONFIG['resolutions_deg']}
    for row in result:
        ix=np.flatnonzero(d['valid']&(groups==int(np.floor(row['t']/.1))));row['correlation_group']=d['block_id'];row['angle_features']={str(s):v[ix].mean(0).tolist() for s,v in filtered.items()}
    d['angle_features']=filtered;return result


def matrix(samples,model):
    if model==0 or (isinstance(model,dict) and model['resolution_deg']==0):return np.array([[s['qdd'],s['qd'],s['direction']] for s in samples])
    return np.array([[s['qdd'],s['qd']]+[s['angle_features'][str(model['resolution_deg'])][i] for i in model['active_controls']] for s in samples])


def fit(samples,resolution,smoothness):
    if resolution==0:
        m=ridge_fit(samples,0,0);m.update(resolution_deg=0,active_controls=[0],basis='constant');return m
    features=np.array([s['angle_features'][str(resolution)] for s in samples]);active=np.flatnonzero(np.max(abs(features),axis=0)>1e-12)
    # Reuse frozen nonnegative ridge/HAC implementation on observed controls.
    tag=len(active)-1;adapted=[dict(s,angle_features={str(tag):features[j,active].tolist()}) for j,s in enumerate(samples)]
    m=ridge_fit(adapted,tag,smoothness);m.update(degree=3,resolution_deg=resolution,active_controls=active.tolist(),full_control_count=features.shape[1],basis='local nonnegative clamped cubic B-spline',knots_rad=knots(resolution).tolist())
    return m


def curve(model):
    q=np.linspace(0,45,181);lo,hi=model['training_angle_range_deg'];B=basis(np.deg2rad(q),model['resolution_deg'])[:,model['active_controls']] if model['resolution_deg'] else np.ones((len(q),1));mean=B@model['parameters'][2:];cov=np.array(model['conditional_covariance'])[2:,2:];sd=np.sqrt(np.maximum(np.einsum('ij,jk,ik->i',B,cov,B),0));inside=(q>=lo)&(q<=hi)
    return dict(angle_deg=q.tolist(),resistance_nm=[float(v) if k else None for v,k in zip(mean,inside)],conditional_std_nm=[float(v) if k else None for v,k in zip(sd,inside)],support_deg=[lo,hi],outside_qualified_FIT_support='unknown; not zero or extrapolated precision')


def analyze(train):
    if len(train)<8:return dict(status='insufficient FIT')
    labels=list(dict.fromkeys(s['segment'] for s in train));candidates=[]
    for resolution,smoothness in [(0,0)]+[(s,l) for s in CONFIG['resolutions_deg'] for l in CONFIG['smoothness']]:
        m=fit(train,resolution,smoothness);cv=[]
        for label in labels:
            tr=[s for s in train if s['segment']!=label];te=[s for s in train if s['segment']==label]
            if len(tr)<8:continue
            cm=fit(tr,resolution,smoothness);X=matrix(te,cm);y=np.array([s['tau'] for s in te]);cv.append(dict(segment=label,rmse_nm=float(np.sqrt(np.mean((y-X@cm['parameters'])**2)))))
        m['FIT_only_leave_trajectory_out']=cv;m['cv_rmse_nm']=float(np.mean([v['rmse_nm'] for v in cv])) if len(labels)>=2 and len(cv)==len(labels) else None;m['resistance_curve']=curve(m);candidates.append(m)
    constant=candidates[0];eligible=[m for m in candidates[1:] if m['cv_rmse_nm'] is not None and m['algebraic_rank']==len(m['parameters'])];best=min(eligible,key=lambda m:m['cv_rmse_nm']) if eligible else None;selected=constant
    if len(labels)>=3 and best and constant['cv_rmse_nm'] is not None and best['cv_rmse_nm']<.9*constant['cv_rmse_nm']:selected=best
    return dict(status='updated',constant_model=constant,extended_model=best,selected_model='constant' if selected['resolution_deg']==0 else 'local_angle_coulomb',selected=selected,candidates=candidates,FIT_trajectories=len(labels),selection_uses=CONFIG['selection'],coverage_deg=constant['training_angle_range_deg'])


def prediction(d,model):
    edge=d['processing']['boundary_exclusion_samples'];ix=np.arange(edge,len(d['t'])-edge);t=d['t'][ix];q=d['q'][ix];v=d['native_qd'][ix];tau=d['torque'][ix];I,b=model['parameters'][:2]
    if I<=1e-8:return dict(status='unknown inertia')
    qp=[float(q[0])];vp=[float(v[0])];lo,hi=np.deg2rad(model['training_angle_range_deg']);outside=False
    for j in range(1,len(t)):
        z=qp[-1];outside=outside or not lo<=z<=hi
        if not np.deg2rad(-5)<=z<=np.deg2rad(50):return dict(status='outside model domain; no clamp to measured state',prediction_outside_training_support=True)
        C=float(basis([z],model['resolution_deg'])[0,model['active_controls']]@model['parameters'][2:]) if model['resolution_deg'] else model['parameters'][2];dt=t[j]-t[j-1];mom=I*vp[-1]+dt*tau[j-1];vel=float(np.sign(mom)*max(abs(mom)-dt*C,0)/(I+dt*b));vp.append(vel);qp.append(qp[-1]+dt*vel)
    qp=np.array(qp);vp=np.array(vp);X=np.c_[d['qdd'][ix],d['qd'][ix],d['angle_features'][model['resolution_deg']][ix][:,model['active_controls']]] if model['resolution_deg'] else np.c_[d['qdd'][ix],d['qd'][ix],d['direction'][ix]]
    return dict(status='predicted',samples=len(ix),duration_s=float(t[-1]-t[0]),torque_rmse_nm=float(np.sqrt(np.mean((tau-X@model['parameters'])**2))),angle_rmse_deg=float(np.rad2deg(np.sqrt(np.mean((qp-q)**2)))),velocity_rmse_rad_s=float(np.sqrt(np.mean((vp-v)**2))),final_error_deg=float(np.rad2deg(qp[-1]-q[-1])),prediction_outside_training_support=bool(outside),input='held-out measured compensated wrist effort; one native initial q/qd, no state correction',filter_lookahead_s=d['processing']['filter_lookahead_s'])
