"""Unit-explicit revolute/prismatic identification; FIT-only model selection.

Uses the production V3.1 compensated measurement chain with an explicit native
coordinate. No degree conversion of rail motion and no torque/mass rescaling.
"""
import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import lsq_linear
from xarm_ft_v31_estimator import process, common_filter, regression_samples

VERSION = 'V7-SI-common-SG121'


def basis(q, domain, intervals):
    lo, hi = domain
    if not lo < hi or intervals < 1:
        raise ValueError('Ordered native-coordinate domain and positive intervals required')
    knots = np.r_[[lo]*4, np.linspace(lo, hi, intervals+1)[1:-1], [hi]*4]
    return BSpline(knots, np.eye(len(knots)-4), 3, extrapolate=False)(np.asarray(q))


def samples(piece, domain, intervals=(2, 4, 8)):
    result = regression_samples(piece)
    groups = np.floor(piece['t']/.1).astype(int)
    features = {n:common_filter(basis(piece['q'], domain, n)*np.sign(piece['native_qd'])[:, None], piece['processing']['window']) for n in intervals}
    for r in result:
        ix = np.flatnonzero(piece['valid'] & (groups == int(np.floor(r['t']/.1))))
        r['features'] = {str(n):v[ix].mean(0).tolist() for n,v in features.items()}
        r['correlation_group'] = piece['block_id']
    piece['features'] = features
    return result


def matrix(rows, model):
    n = model['intervals']
    if n == 0:
        return np.array([[r['qdd'],r['qd'],r['direction']] for r in rows])
    return np.array([[r['qdd'],r['qd']]+[r['features'][str(n)][i] for i in model['active_controls']] for r in rows])


def fit(rows, config, intervals=0, smoothness=0):
    if len(rows) < 8:
        raise ValueError('Insufficient FIT bins')
    active = [0] if intervals == 0 else np.flatnonzero(np.max(abs(np.array([r['features'][str(intervals)] for r in rows])),axis=0)>1e-12).tolist()
    model = dict(intervals=intervals,active_controls=active,domain_native=config['domain_native'],joint_type=config['joint_type'],units=config['units'])
    X = matrix(rows,model); y = np.array([r['tau'] for r in rows])
    scale = np.r_[config['parameter_scales'][:2],np.repeat(config['parameter_scales'][2],len(active))]
    ridge = config.get('ridge',.001)*np.diag(1/scale)
    D = np.diff(np.eye(intervals+3),n=2,axis=0)[:,active] if intervals else np.empty((0,1))
    P = np.c_[np.zeros((len(D),2)),D]*smoothness/config['parameter_scales'][2]
    beta = lsq_linear(np.vstack([X,ridge,P]),np.r_[y,np.zeros(len(ridge)+len(P))],bounds=(0,np.inf),tol=1e-12).x
    residual = y-X@beta; bread = np.linalg.pinv(X.T@X+ridge.T@ridge+P.T@P)
    Z = X*residual[:,None]; meat = Z.T@Z
    label = np.array([r['correlation_group'] for r in rows]); ts = np.array([r['t'] for r in rows]); lags=5
    for lag in range(1,min(lags+1,len(rows))):
        valid=(label[lag:]==label[:-lag]) & (abs(ts[lag:]-ts[:-lag]-.1*lag)<.06)
        cross=Z[lag:][valid].T@Z[:-lag][valid];meat+=(1-lag/(lags+1))*(cross+cross.T)
    rank=int(np.linalg.matrix_rank(X));sigma2=float(residual@residual/max(len(y)-rank,1))
    hc=bread@meat@bread;iid=sigma2*bread@(X.T@X)@bread
    cov=(hc+hc.T)/2+np.diag(np.maximum(np.diag(iid)-np.diag(hc),0))
    vals,V=np.linalg.eigh(cov);cov=(V*np.maximum(vals,0))@V.T
    singular=np.linalg.svd(X*scale,compute_uv=False)
    support=[]
    for group in dict.fromkeys(label):
        g=sorted([r for r in rows if r['correlation_group']==group],key=lambda r:r['t']);start=0
        for end in list(np.flatnonzero(np.diff([r['t'] for r in g])>.2)+1)+[len(g)]:
            q=[r['q'] for r in g[start:end]];support.append([min(q),max(q)]);start=end
    merged=[]
    for lo,hi in sorted(support):
        if merged and lo<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],hi)
        else:merged.append([lo,hi])
    support=merged
    model.update(parameters=beta.tolist(),conditional_covariance=cov.tolist(),conditional_std=np.sqrt(np.diag(cov)).tolist(),
        algebraic_rank=rank,scaled_singular_values=singular.tolist(),scaled_condition=float(singular[0]/singular[-1]) if singular[-1]>1e-15 else None,
        support_intervals_native=support,FIT_rmse=float(np.sqrt(np.mean(residual**2))),smoothness=smoothness,
        validation_used=False,uncertainty='conditional Bartlett-HAC with iid diagonal floor; excludes model selection, compensation and contact/systematic error',
        parameter_names=['I_eff_kg_m2','b_Nm_s_rad'] if config['joint_type']=='revolute' else ['m_eff_kg','b_N_s_m'])
    model['conditionally_supported']=[bool(rank==len(beta) and value>3*sd) for value,sd in zip(beta,np.sqrt(np.diag(cov)))]
    model['inertial_sensitivity_after_velocity_and_friction_projection']=float(np.linalg.norm(X[:,0]-X[:,1:]@np.linalg.lstsq(X[:,1:],X[:,0],rcond=None)[0]))
    model['uncalibrated_total_physical_accuracy']=True
    model['parameter_names'] += (['C_Nm'] if config['joint_type']=='revolute' else ['F_c_N']) if intervals==0 else [f'nonnegative_resistance_control_{i}' for i in active]
    return model


def analyze(train, config):
    if len(train)<8:return dict(status='insufficient informative FIT',training_bins=len(train))
    labels=list(dict.fromkeys(r['segment'] for r in train));candidates=[]
    for n,s in [(0,0)]+[(n,s) for n in config.get('intervals',[2,4,8]) for s in config.get('smoothness',[.0001,.001,.01])]:
        m=fit(train,config,n,s);scores=[]
        for held in labels:
            tr=[r for r in train if r['segment']!=held];te=[r for r in train if r['segment']==held]
            if len(tr)<8:continue
            other=fit(tr,config,n,s)
            # A held FIT region without trained local controls must not look
            # spuriously good. Missing basis weights contribute zero to the CV
            # prediction, not hidden/extrapolated fitted coefficients.
            err=np.array([r['tau'] for r in te])-matrix(te,other)@other['parameters']
            scores.append(dict(segment=held,rmse=float(np.sqrt(np.mean(err**2)))))
        m['FIT_only_trajectory_CV']=scores;m['cv_rmse']=float(np.mean([r['rmse'] for r in scores])) if len(scores)==len(labels) and len(labels)>=2 else None
        candidates.append(m)
    constant=candidates[0];extended=[m for m in candidates[1:] if m['algebraic_rank']==len(m['parameters']) and m['cv_rmse'] is not None]
    best=min(extended,key=lambda m:m['cv_rmse']) if extended else None
    selected=best if len(labels)>=3 and best and constant['cv_rmse'] is not None and best['cv_rmse']<.9*constant['cv_rmse'] else constant
    return dict(status='updated',constant_model=constant,extended_model=best,selected=selected,candidates=candidates,training_bins=len(train),selection='FIT-only leave-complete-trajectory-out; >=3 FIT trajectories and >=10% CV gain for extension')


def prediction(piece, model):
    edge=piece['processing']['boundary_exclusion_samples'];ix=np.arange(edge,len(piece['t'])-edge)
    if len(ix)<2:return dict(status='insufficient reliable interior')
    t=piece['t'][ix];q=piece['q'][ix];v=piece['native_qd'][ix];effort=piece['torque'][ix];mass,b=model['parameters'][:2]
    if mass<=1e-8:return dict(status='unsupported inertial term')
    qp=[float(q[0])];vp=[float(v[0])];outside=False
    for j in range(1,len(t)):
        z=qp[-1];outside |= not any(lo<=z<=hi for lo,hi in model['support_intervals_native'])
        if not model['domain_native'][0]<=z<=model['domain_native'][1]:return dict(status='prediction left declared domain; no state correction',outside_FIT_support=True)
        C=float(basis([z],model['domain_native'],model['intervals'])[0,model['active_controls']]@model['parameters'][2:]) if model['intervals'] else model['parameters'][2]
        dt=t[j]-t[j-1];p=mass*vp[-1]+dt*effort[j-1];vel=float(np.sign(p)*max(abs(p)-dt*C,0)/(mass+dt*b))
        vp.append(vel);qp.append(z+dt*vel)
    n=model['intervals'];X=np.c_[piece['qdd'][ix],piece['qd'][ix],piece['features'][n][ix][:,model['active_controls']]] if n else np.c_[piece['qdd'][ix],piece['qd'][ix],piece['direction'][ix]]
    return dict(status='predicted',effort_rmse=float(np.sqrt(np.mean((effort-X@model['parameters'])**2))),
        coordinate_rmse=float(np.sqrt(np.mean((np.array(qp)-q)**2))),velocity_rmse=float(np.sqrt(np.mean((np.array(vp)-v)**2))),
        final_coordinate_error=float(qp[-1]-q[-1]),outside_FIT_support=bool(outside),samples=len(ix),duration_s=float(t[-1]-t[0]),units=model['units'],
        input='held-out compensated wrist effort, one measured initial q/qd; no subsequent state correction')
