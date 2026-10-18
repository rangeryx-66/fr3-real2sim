"""Read-only V3 diagnostics. No simulator imports, plant truth, or archive writes.

All filters operate on one complete segment at a time. The variant menu is
declared before data are read; FIT-only leave-segment-out error chooses among
variants, never authored inertia or the reserved validation trajectory.
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter as sg
from scipy.spatial.transform import Rotation
from scipy.optimize import lsq_linear

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
from diagnose_probing_v3 import columns, rows_at
from xarm_ft_id_estimator import process
from xarm_ft_v1_estimator import regression_samples, fit, forward

PROTOCOL = dict(
    windows=[61, 121, 181], shifts_samples=[-8, -4, -2, -1, 0, 1, 2, 4, 8],
    common_edge_samples=211, original_edge_samples=121,
    differentiation=['position_SG2', 'native_velocity_SG1', 'position_gradient_SG0'],
    torque_filters=['none', 'SG0', 'SG2_equivalent'],
    selection='minimum sample-weighted FIT-only leave-complete-segment-out torque RMSE; no validation/truth selection',
    synthetic_parameters=[0.3, 0.4, 0.02], synthetic_frequencies_hz=[0.1, 0.31831, 0.5, 1., 1.5, 2.],
    weak_windows_s=[0.5, 1.0], nuisance_models=['bias', 'direction_offset', 'slip_rate', 'aperture_rate'],
    comparison_target='common original unfiltered compensated torque for CV and forward prediction; own-filter errors reported separately',
    matched_compensation='filter raw+bias+gravity once; subtract already smoothed tool inertia once (no second smoothing)',
    truth_inputs=False, diagnostic_only=True)
EDGE=PROTOCOL['common_edge_samples'] # 120 original angular-filter support + 90 additional filter support + 1.


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x)**2)))


def corr(x, y):
    return float(np.corrcoef(x, y)[0, 1]) if np.std(x)>1e-12 and np.std(y)>1e-12 else None


def project(w, sensor, prior):
    axis=np.asarray(prior['axis_world']); origin=np.asarray(prior['origin_world_m'])
    return (w[:,3:] + np.cross(sensor[:,:3,3]-origin, w[:,:3])) @ axis


def sg2_kernel(win, dt):
    """Equivalent acceleration smoother for SG2(position), discrete gradient2.

    For the separate analytic synthetic tests, integrate the SG2 convolution
    kernel twice in continuous sample coordinates: K(u)=sum c_j (j-u)_+.
    This has unit DC gain; its convolution with true qdd equals SG2(q) up to
    the trapezoid/discrete integration convention. No inverse filtering.
    """
    from scipy.signal import savgol_coeffs
    c=savgol_coeffs(win,3,deriv=2,delta=dt,use='conv')
    # The discrete acceleration associated with central second differences.
    # q[j+1]-2q[j]+q[j-1] = dt²*a[j]; two cumulative sums solve this identity.
    k=np.cumsum(np.cumsum(c))*dt**2
    k=np.roll(k,1); k[0]=0
    return k/k.sum()


def smooth_equivalent(x, win, dt):
    k=sg2_kernel(win,dt)
    return np.apply_along_axis(lambda v:np.convolve(np.pad(v,win//2,mode='edge'),k,mode='valid'),0,x)


def tool_terms(rows, prior, win=121, angular='cascade'):
    dt=float(np.median(np.diff([r['t'] for r in rows])))
    sensor=np.array([r['wrist_ft']['T_world_sensor'] for r in rows]);n=len(rows)
    parts={k:np.zeros((n,6)) for k in ['translation','rotation','gyro']}
    momentum=np.zeros(n); masses=[]
    for j in range(len(rows[0]['wrist_ft']['body_states'])):
        bodies=[r['wrist_ft']['body_states'][j] for r in rows]
        mass=bodies[0]['mass_kg'];masses.append(mass)
        pos=np.array([b['com_world_m'] for b in bodies]);R=np.array([b['T_world_body'] for b in bodies])[:,:3,:3]
        I=np.array([b['inertia_world_kg_m2'] for b in bodies])
        acc=sg(pos,win,3,deriv=2,delta=dt,axis=0)
        # Archived implementation uses backward angular increments, then SG0.
        increments=Rotation.from_matrix(R[1:]@R[:-1].transpose(0,2,1)).as_rotvec()/dt
        omega=np.vstack([np.zeros(3),increments]); omega=sg(omega,win,3,axis=0)
        if angular in ('centered','raw_momentum'):
            omega=np.vstack([increments[0],(increments[1:]+increments[:-1])/2,increments[-1]])
            if angular!='raw_momentum':omega=sg(omega,win,3,axis=0)
        alpha=sg(omega,win,3,deriv=1,delta=dt,axis=0)
        force=mass*acc
        parts['translation']+=np.c_[force,np.cross(pos-sensor[:,:3,3],force)]
        parts['rotation'][:,3:]+=np.einsum('nij,nj->ni',I,alpha)
        parts['gyro'][:,3:]+=np.cross(omega,np.einsum('nij,nj->ni',I,omega))
        velocity=np.gradient(pos,dt,axis=0) if angular=='raw_momentum' else sg(pos,win,3,deriv=1,delta=dt,axis=0)
        H=np.einsum('nij,nj->ni',I,omega)+np.cross(pos-np.asarray(prior['origin_world_m']),mass*velocity)
        momentum+=H@np.asarray(prior['axis_world'])
    return {k:project(w,sensor,prior) for k,w in parts.items()}, momentum, sum(masses)


def samples(pieces):
    result=[]
    for d in pieces:
        ss=regression_samples(d)
        if 'direction' in d:
            for s in ss:
                group=int(np.floor(s['t']/.1))
                ix=d['valid']&(np.floor(d['t']/.1).astype(int)==group)
                s['direction']=float(np.mean(d['direction'][ix]))
        result.extend(ss)
    return result


def xy(s):
    return np.array([[r['qdd'],r['qd'],r['direction']] for r in s]),np.array([r['tau'] for r in s])


def fit_score(pieces, common=None):
    s=samples(pieces);train=[r for r in s if r['segment'].startswith('FIT')];test=[r for r in s if r['segment'].startswith('VALIDATION')]
    X,y=xy(train); m=fit(train,'constant');beta=np.asarray(m['parameters']);res=y-X@beta
    errors=[];coefs=[];common_errors=[]
    common_train=[r for r in samples(common) if r['segment'].startswith('FIT')] if common else train
    common_X,common_y=xy(common_train)
    for label in dict.fromkeys(r['segment'] for r in train):
        a=[r for r in train if r['segment']!=label];b=[r for r in train if r['segment']==label];cm=fit(a,'constant');xx,yy=xy(b)
        errors.extend((yy-xx@cm['parameters']).tolist());coefs.append(dict(excluded=label,parameters=cm['parameters'],rmse_nm=rms(yy-xx@cm['parameters'])))
        other=[r for r in common_train if r['segment']==label];cx,cy=xy(other);common_errors.extend((cy-cx@cm['parameters']).tolist())
    out=dict(parameters=beta.tolist(),training_bins=len(train),training_rmse_nm=rms(res),FIT_LOSO_rmse_nm=rms(errors),FIT_LOSO=coefs,
             FIT_LOSO_residual_acceleration_correlation=corr(errors,X[:,0]),
             residual_acceleration_correlation=corr(res,X[:,0]),acceleration_rms=rms(X[:,0]),heldout={},
             common_unfiltered_training_rmse_nm=rms(common_y-common_X@beta),common_unfiltered_FIT_LOSO_rmse_nm=rms(common_errors),
             common_unfiltered_residual_acceleration_correlation=corr(common_y-common_X@beta,common_X[:,0]),common_unfiltered_heldout={})
    if test:
        xx,yy=xy(test);out['heldout_torque_rmse_nm']=rms(yy-xx@beta)
    for d in pieces:
        label=str(d['segments'][0])
        if label.startswith('VALIDATION'):
            ids=np.flatnonzero(d['valid']);lo,hi=ids[0],ids[-1];ids=np.arange(lo,hi+1)
            clone=dict(d);clone['qd']=d['qd'].copy();clone['qd'][lo]=d['native_qd'][lo]
            out['heldout'][label]=forward(clone,ids,m)
            original=next((v for v in common if str(v['segments'][0])==label),d) if common else d
            clone=dict(original);clone['qd']=original['qd'].copy();clone['qd'][lo]=original['native_qd'][lo]
            out['common_unfiltered_heldout'][label]=forward(clone,ids,m)
            # Full reserved segment including explicitly marked derivative edges.
            clone['qd'][0]=original['native_qd'][0]
            out.setdefault('full_reserved_heldout_including_derivative_edges',{})[label]=forward(clone,np.arange(len(clone['t'])),m)
    return out


def make_variant(base, name, window=121, derivative='position_SG2', torque_filter='none', shift=0, edge=EDGE, tool_window=None, centered=False, single_pass=False):
    pieces=[]
    for b in base:
        d={k:v.copy() if isinstance(v,np.ndarray) else v for k,v in b['d'].items()};dt=b['dt'];q=d['q']
        d['qd']=sg(q,window,3,deriv=1,delta=dt)
        d['qdd']=sg(q,window,3,deriv=2,delta=dt)
        if derivative=='native_velocity_SG1':
            d['qd']=sg(d['native_qd'],window,3);d['qdd']=sg(d['native_qd'],window,3,deriv=1,delta=dt)
        if derivative=='position_gradient_SG0':
            d['qd']=sg(np.gradient(q,dt),window,3);d['qdd']=sg(np.gradient(np.gradient(q,dt),dt),window,3)
        if tool_window is not None or centered:
            terms,_,_=tool_terms(b['rows'],b['prior'],tool_window or 121,'centered' if centered else 'cascade')
            d['torque']=b['before_tool']-sum(terms.values())
        if torque_filter=='SG0':d['torque']=sg(d['torque'],window,3)
        if torque_filter=='SG2_equivalent':d['torque']=smooth_equivalent(d['torque'],window,dt)
        if derivative=='matched_all':
            # Deliberately same linear filter for torque, native acceleration,
            # velocity AND Coulomb regressor, not sign(filtered velocity).
            d['qd']=smooth_equivalent(d['native_qd'],window,dt)
            d['qdd']=smooth_equivalent(np.gradient(d['native_qd'],dt),window,dt)
            d['direction']=smooth_equivalent(np.sign(d['native_qd']),window,dt)
        if single_pass:
            terms,_,_=tool_terms(b['rows'],b['prior'],window)
            d['torque']=smooth_equivalent(b['before_tool'],window,dt)-sum(terms.values())
        # positive shift: later F/T paired with current kinematics; no cross-segment roll admitted.
        if shift:d['torque']=np.roll(d['torque'],-shift)
        d['valid'][:edge]=False;d['valid'][-edge:]=False
        pieces.append(d)
    return pieces


def bins_like(d,values):
    clone=dict(d);clone['torque']=values
    return np.array([r['tau'] for r in regression_samples(clone)])


def synthetic():
    dt=1/240;t=np.arange(0,16,dt);I,b,C=PROTOCOL['synthetic_parameters'];out=[]
    for f in PROTOCOL['synthetic_frequencies_hz']:
        w=2*np.pi*f;q=.02*t+.004*np.sin(w*t);v=.02+.004*w*np.cos(w*t);a=-.004*w*w*np.sin(w*t);tau=I*a+b*v+C*np.sign(v)
        for win in PROTOCOL['windows']:
            V=sg(q,win,3,deriv=1,delta=dt);A=sg(q,win,3,deriv=2,delta=dt);g=np.arange(181,len(t)-181)
            for filt,T in [('none',tau),('SG0',sg(tau,win,3)),('SG2_equivalent',smooth_equivalent(tau,win,dt))]:
                D=np.sign(V) if filt!='SG2_equivalent' else smooth_equivalent(np.sign(v),win,dt)
                vv=V if filt!='SG2_equivalent' else smooth_equivalent(v,win,dt)
                X=np.c_[A,vv,D];beta=np.linalg.lstsq(X[g],T[g],rcond=None)[0]
                out.append(dict(frequency_hz=f,window=win,torque_filter=filt,known_parameters=[I,b,C],recovered=beta.tolist(),relative_I_error=float(beta[0]/I-1)))
    return out


def weak_samples(base,width,compensation='momentum',native_acceleration=False):
    """Integration by parts, fixed world hinge; no acceleration differentiation.

    Integral(phi*tau_contact) = Integral(phi*(raw+gravity))
    + Integral(phi'*H_tool). Boundary terms vanish. H is measured tool angular
    momentum about the fixed hinge. No authored object mass/inertia input.
    """
    result=[];dt=base[0]['dt'];half=round(width/(2*dt));u=np.linspace(-1,1,2*half+1)
    w=(1-u*u)**3;w/=w.sum();w=np.pad(w,(2,2));half+=2
    first=(np.roll(w,-1)-np.roll(w,1))/(2*dt)
    second=(np.roll(w,-1)-2*w+np.roll(w,1))/dt**2
    for b in base:
        d=b['d'];_,H,_=tool_terms(b['rows'],b['prior'],angular='raw_momentum')
        for center in range(EDGE,len(d['t'])-EDGE,24):
            sl=slice(center-half,center+half+1)
            if not d['valid'][sl].all():continue
            tau=w@b['before_tool'][sl]+first@H[sl]
            if compensation=='original':tau=w@d['torque'][sl]
            if compensation=='none':tau=w@b['before_tool'][sl]
            result.append(dict(segment=str(d['segments'][0]),t=float(d['t'][center]),q=float(w@d['q'][sl]),
                qdd=float(-first@d['native_qd'][sl] if native_acceleration else second@d['q'][sl]),qd=float(-first@d['q'][sl]),direction=float(w@np.sign(d['native_qd'][sl])),tau=float(tau)))
    return result


def weak_score(base,width,common,compensation='momentum',native_acceleration=False):
    ss=weak_samples(base,width,compensation,native_acceleration);train=[r for r in ss if r['segment'].startswith('FIT')];test=[r for r in ss if r['segment'].startswith('VALIDATION')]
    X,y=xy(train);m=fit(train,'constant');beta=np.array(m['parameters']);cv=[];cvcommon=[]
    for label in dict.fromkeys(r['segment'] for r in train):
        tr=[r for r in train if r['segment']!=label];te=[r for r in train if r['segment']==label];cm=fit(tr,'constant');xx,yy=xy(te);cv.extend(yy-xx@cm['parameters'])
        cs=[r for r in samples(common) if r['segment']==label];xx,yy=xy(cs);cvcommon.extend(yy-xx@cm['parameters'])
    xx,yy=xy(test);result=dict(parameters=beta.tolist(),training_bins=len(train),training_rmse_nm=rms(y-X@beta),FIT_LOSO_rmse_nm=rms(cv),
        common_unfiltered_FIT_LOSO_rmse_nm=rms(cvcommon),heldout_torque_rmse_nm=rms(yy-xx@beta),common_unfiltered_heldout={})
    errors=y-X@beta;Z=X*errors[:,None];bread=np.linalg.pinv(X.T@X);meat=Z.T@Z
    labels=np.array([s['segment'] for s in train]);nlags=round(width/.1)
    for lag in range(1,nlags+1):
        good=labels[lag:]==labels[:-lag];c=Z[lag:][good].T@Z[:-lag][good];meat+=(1-lag/(nlags+1))*(c+c.T)
    result['FIT_only_conditional_HAC_std']=np.sqrt(np.maximum(np.diag(bread@meat@bread),0)).tolist()
    result['full_residual_acceleration_correlation']=corr(errors,X[:,0])
    result['residual_without_I_acceleration_correlation']=corr(y-X[:,1:]@beta[1:],X[:,0])
    for d in common:
        label=str(d['segments'][0])
        if label.startswith('VALIDATION'):
            ids=np.arange(EDGE,len(d['t'])-EDGE);clone=dict(d);clone['qd']=d['qd'].copy();clone['qd'][ids[0]]=d['native_qd'][ids[0]]
            result['common_unfiltered_heldout'][label]=forward(clone,ids,m)
            clone['qd'][0]=d['native_qd'][0]
            result.setdefault('full_reserved_heldout_including_derivative_edges',{})[label]=forward(clone,np.arange(len(d['t'])),m)
    return result


def motion_synthetic(base):
    """Recorded motion spectra, synthetic torque coefficients unrelated to truth."""
    cases={k:[] for k in ['unmatched_object_only','matched_object_only','unmatched_object_plus_tool','single_pass_object_plus_tool']}
    beta=np.array(PROTOCOL['synthetic_parameters'])
    for b in base:
        d=b['d'];dt=b['dt'];q=sg(d['q'],31,3);v=np.gradient(q,dt);acc=np.gradient(v,dt)
        true_tau=np.c_[acc,v,np.sign(v)]@beta
        fast,_,_=tool_terms(b['rows'],b['prior'],31);tool=sum(fast.values())
        for name in cases:
            clone=dict(d);clone['qd']=sg(q,121,3,deriv=1,delta=dt);clone['qdd']=sg(q,121,3,deriv=2,delta=dt);clone['valid']=d['valid'].copy();clone['valid'][:EDGE]=False;clone['valid'][-EDGE:]=False
            if name=='unmatched_object_only':clone['torque']=true_tau
            if name=='matched_object_only':
                clone['torque']=smooth_equivalent(true_tau,121,dt);clone['qd']=smooth_equivalent(v,121,dt);clone['direction']=smooth_equivalent(np.sign(v),121,dt)
            if name=='unmatched_object_plus_tool':clone['torque']=true_tau+tool-b['tool']
            if name=='single_pass_object_plus_tool':
                clone['torque']=smooth_equivalent(true_tau+tool,121,dt)-b['tool'];clone['qd']=smooth_equivalent(v,121,dt);clone['direction']=smooth_equivalent(np.sign(v),121,dt)
            cases[name].append(clone)
    return {name:dict(known_parameters=beta.tolist(),**fit_score(pieces)) for name,pieces in cases.items()}


def contact_and_nuisance(base,a,variants):
    """Diagnostic covariates, never online gates or claimed friction measurements."""
    pieces=variants;records=[];by_segment=[]
    for b,d in zip(base,pieces):
        ix=b['ix'];dt=b['dt'];pos=a['tcp_pose'][ix,:3];axis=np.asarray(b['prior']['axis_world']);origin=np.asarray(b['prior']['origin_world_m']);r=pos-origin;r-=np.outer(r@axis,axis)
        x=r[0]/np.linalg.norm(r[0]);y=np.cross(axis,x);tcp_angle=np.unwrap(np.arctan2(r@y,r@x));delta=tcp_angle-d['q'];delta-=delta[0]
        proxies=dict(slip_rate=sg(a['relative_slip_m'][ix],61,3,deriv=1,delta=dt),aperture_rate=sg(a['aperture'][ix],61,3,deriv=1,delta=dt),
                     relative_angular_acceleration=sg(delta,61,3,deriv=2,delta=dt),pad_imbalance=a['pad_load'][ix,0]-a['pad_load'][ix,1])
        ss=samples([d]);bb={k:bins_like(d,v) for k,v in proxies.items()}
        for j,s in enumerate(ss):records.append(dict(**s,**{k:float(v[j]) for k,v in bb.items()}))
        by_segment.append(dict(segment=str(d['segments'][0]),relative_angular_extent_deg=np.rad2deg([delta.min(),delta.max()]).tolist(),
            relative_translation_extent_mm=(1000*np.array([a['relative_slip_m'][ix].min(),a['relative_slip_m'][ix].max()])).tolist()))
    train=[r for r in records if r['segment'].startswith('FIT')];test=[r for r in records if r['segment'].startswith('VALIDATION')]
    def design(rr,key):
        X,Y=xy(rr)
        extra=np.ones(len(rr)) if key=='bias' else np.array([r['q']*r['direction'] if key=='angle_direction' else r[key] for r in rr])
        return np.c_[X,extra],Y
    report={}
    for key in ['bias','angle_direction','slip_rate','aperture_rate','relative_angular_acceleration','pad_imbalance']:
        X,y=design(train,key);scale=np.linalg.norm(X,axis=0);beta=np.linalg.lstsq(X/scale,y,rcond=None)[0]/scale;cv=[]
        for label in dict.fromkeys(r['segment'] for r in train):
            tr=[r for r in train if r['segment']!=label];te=[r for r in train if r['segment']==label];xx,yy=design(tr,key);s=np.linalg.norm(xx,axis=0);b=np.linalg.lstsq(xx/s,yy,rcond=None)[0]/s;xx,yy=design(te,key);cv.extend(yy-xx@b)
        xx,yy=design(test,key);plain=np.asarray(fit(train,'constant')['parameters']);baseX,basey=xy(train)
        report[key]=dict(parameters=beta.tolist(),training_rmse_nm=rms(y-X@beta),FIT_LOSO_rmse_nm=rms(cv),heldout_torque_rmse_nm=rms(yy-xx@beta),
            extra_residual_correlation=corr(basey-baseX@plain,X[:,-1]),residual_acceleration_correlation=corr(y-X@beta,X[:,0]),
            forward_prediction='not supplied: measured nuisance trajectory would be conditional input, not independent object dynamics')
    return dict(per_segment=by_segment,unconstrained_diagnostic_nuisance_regressions=report,not_causal_identification=True)


def run(episode,out):
    began=time.perf_counter();out.mkdir(parents=True,exist_ok=True)
    (out/'PROTOCOL.json').write_text(json.dumps(PROTOCOL,indent=2))
    a,index=columns(episode);settings=json.loads((episode/'identification_input.json').read_text());meta=index['metadata'];prior=settings['prior'];bias=settings['bias']
    phase=np.array(meta['phases'])[a['phase']];base=[];audit=[]
    for s in index['segments']:
        if not s.get('completed') or not s.get('result',{}).get('ok'):continue
        ix=np.flatnonzero((a['segment']==s['id'])&np.isin(phase,['OPEN_CONTINUE','PANEL_PUSH']))
        if len(ix)<365:continue
        assert np.all(np.diff(a['step'][ix])==1) and np.all(a['ft_valid'][ix])
        rows=rows_at(a,ix,meta,s['role']+'_%02d'%s['id']);d=process(rows,prior,bias);d['native_qd']=a['door_dq'][ix]
        d['valid'][:121]=False;d['valid'][-121:]=False;dt=float(np.median(np.diff(d['t'])))
        sensor=np.array([r['wrist_ft']['T_world_sensor'] for r in rows]);R=sensor[:,:3,:3]
        def world(x):return np.c_[np.einsum('nij,nj->ni',R,x[:,:3]),np.einsum('nij,nj->ni',R,x[:,3:])]
        raw=project(world(d['raw']),sensor,prior);bias_t=project(world(np.broadcast_to(bias,d['raw'].shape)),sensor,prior)
        gravity=project(d['gravity_world'],sensor,prior);tool=project(d['inertial_world'],sensor,prior)
        terms,H,mass=tool_terms(rows,prior)
        assert np.max(abs(sum(terms.values())-tool))<1e-12
        good=d['valid'];audit.append(dict(segment=str(d['segments'][0]),n=len(ix),dt=dt,step_time_max_error=float(np.max(abs(np.diff(d['t'])-1/240))),
            raw_hinge_rms_nm=rms(raw[good]),bias_hinge_rms_nm=rms(bias_t[good]),gravity_hinge_rms_nm=rms(gravity[good]),
            tool_hinge_rms_nm=rms(tool[good]),compensated_hinge_rms_nm=rms(d['torque'][good]),total_tool_mass_kg=mass,
            tool_parts_rms_nm={k:rms(v[good]) for k,v in terms.items()},native_vs_SG_velocity_rms=rms(d['native_qd'][good]-d['qd'][good])))
        base.append(dict(d=d,rows=rows,dt=dt,prior=prior,ix=ix,raw=raw,bias=bias_t,gravity=gravity,tool=tool,parts=terms,H=H,before_tool=raw-bias_t+gravity))
    original=[b['d'] for b in base];original_score=fit_score(original)
    archived=json.loads((episode/'model/estimate.json').read_text())['constant_model']['parameters']
    assert np.max(abs(np.array(original_score['parameters'])-archived))<1e-10
    variants={};data={}
    common=make_variant(base,'common')
    def add(name,**kw):
        pieces=make_variant(base,name,**kw);data[name]=pieces;variants[name]=fit_score(pieces,common)
    add('baseline_common_edges')
    for win in PROTOCOL['windows']:
        for deriv in PROTOCOL['differentiation']:
            add('derivative_%s_%d'%(deriv,win),window=win,derivative=deriv)
        for filt in ['SG0','SG2_equivalent']:
            add('torque_%s_%d'%(filt,win),window=win,torque_filter=filt)
        add('joint_tool_window_%d'%win,window=win,tool_window=win)
        add('matched_all_%d'%win,window=win,derivative='matched_all',torque_filter='SG2_equivalent')
        add('single_pass_%d'%win,window=win,derivative='matched_all',single_pass=True)
    for shift in PROTOCOL['shifts_samples']:
        add('FT_shift_%+d'%shift,shift=shift)
    add('centered_tool_angular_velocity',centered=True)
    chosen=min(variants,key=lambda k:variants[k]['common_unfiltered_FIT_LOSO_rmse_nm'])
    weak={str(w):weak_score(base,w,common) for w in PROTOCOL['weak_windows_s']}
    weak_ablation={k:weak_score(base,.5,common,**kw) for k,kw in dict(original_compensation=dict(compensation='original'),no_tool_compensation=dict(compensation='none'),native_velocity=dict(native_acceleration=True)).items()}
    train=[r for r in samples(original) if r['segment'].startswith('FIT')];X,y=xy(train);z=X[:,0]-X[:,1:]@np.linalg.lstsq(X[:,1:],X[:,0],rcond=None)[0]
    decomposition={};component_arrays={}
    for k in ['raw','bias','gravity','tool']+['translation','rotation','gyro']:
        vv=[]
        for b in base:
            if not str(b['d']['segments'][0]).startswith('FIT'):continue
            vals=b['parts'][k] if k in b['parts'] else b[k];vv.extend(bins_like(b['d'],vals))
        v=np.array(vv);component_arrays[k]=v;decomposition[k]=dict(rms_nm=rms(v),acceleration_partial_coefficient_kg_m2=float(z@v/(z@z)),correlation_qdd=corr(v,X[:,0]))
    torque_changes={}
    for name in ['single_pass_61','single_pass_121','single_pass_181','centered_tool_angular_velocity']:
        differences=[]
        for b,variant in zip(base,data[name]):
            if not str(b['d']['segments'][0]).startswith('FIT'):continue
            # Same original 121-edge bins and same X to isolate torque stages.
            differences.extend(bins_like(b['d'],variant['torque']-b['d']['torque']))
        v=np.array(differences);torque_changes[name]=dict(delta_torque_rms_nm=rms(v),delta_I_FWL_kg_m2=float(z@v/(z@z)),correlation_with_acceleration=corr(v,X[:,0]))
    correlations=[]
    beta=np.array(original_score['parameters'])
    for b in base:
        d=b['d'];ix=b['ix'];g=d['valid'];res=d['torque']-np.c_[d['qdd'],d['qd'],np.sign(d['qd'])]@beta
        deriv=lambda v:sg(v,61,3,deriv=1,delta=b['dt'])
        proxies=dict(slip=a['relative_slip_m'][ix],slip_rate=deriv(a['relative_slip_m'][ix]),aperture=a['aperture'][ix],aperture_rate=deriv(a['aperture'][ix]),
            pad_imbalance=a['pad_load'][ix,0]-a['pad_load'][ix,1],load_sum=a['pad_load'][ix].sum(axis=1),reference_lag=a['reference_q'][ix]-a['door_q'][ix],native_velocity_delta=d['native_qd']-d['qd'])
        correlations.append(dict(segment=str(d['segments'][0]),residual_rms_nm=rms(res[g]),correlations={k:corr(res[g],v[g]) for k,v in proxies.items()},
            positive_normal_absence_fraction=(a['pad_load'][ix][g]<=.05).mean(axis=0).tolist(),qdd_correlation={k:corr(d['qdd'][g],v[g]) for k,v in proxies.items()},
            aperture_extent_mm=(np.array([a['aperture'][ix][g].min(),a['aperture'][ix][g].max()])*1000).tolist()))
    # Regressor FWL contributions are decompositions, not proposed calibration corrections.
    # Saved PhysX robot body tensors, matched only against known tool metadata.
    native=json.loads((episode/'NATIVE_MASS_TENSOR_READBACK.json').read_text());masses=np.array(native['get_masses'][0]);coms=np.array(native['get_coms'][0]);inertias=np.array(native['get_inertias'][0]).reshape(-1,3,3)
    tool_audit=[]
    for prop in meta['body_properties']:
        distances=abs(masses-prop['mass_kg'])+np.linalg.norm(coms[:,:3]-prop['com_body_m'],axis=1);j=int(np.argmin(distances))
        tool_audit.append(dict(name=prop['name'],native_row=j,mass_error_kg=float(masses[j]-prop['mass_kg']),COM_error_m=float(np.linalg.norm(coms[j,:3]-prop['com_body_m'])),tensor_max_abs_error_kg_m2=float(np.max(abs(inertias[j]-prop['inertia_body_kg_m2'])))))
    unloaded=[]
    candidates=np.flatnonzero(np.isin(phase,['APPROACH','RELEASE'])&(a['positive_contacts']==0)&(a['ft_valid']>0))
    runs=np.split(candidates,np.flatnonzero((np.diff(a['step'][candidates])!=1)|(np.diff(a['phase'][candidates])!=0))+1)
    for ix in runs:
        if len(ix)<2*EDGE+50:continue
        rows=rows_at(a,ix,meta,'UNLOADED_DIAGNOSTIC');d=process(rows,prior,bias);T=np.array([r['wrist_ft']['T_world_sensor'] for r in rows]);tool=project(d['inertial_world'],T,prior)
        torque=smooth_equivalent(d['torque']+tool,121,float(np.median(np.diff(d['t']))))-tool;good=np.zeros(len(ix),bool);good[EDGE:-EDGE]=True;d['valid']=good
        original_bins=bins_like(d,d['torque']);new_bins=bins_like(d,torque)
        unloaded.append(dict(phase=str(phase[ix[0]]),records=len(ix),interior_records=int(good.sum()),angle_deg=float(np.rad2deg(np.median(d['q']))),
            original_rms_nm=rms(d['torque'][good]),single_pass_rms_nm=rms(torque[good]),original_bin_rms_nm=rms(original_bins),single_pass_bin_rms_nm=rms(new_bins),
            original_residual_tool_inertia_correlation=corr(d['torque'][good],tool[good]),single_pass_residual_tool_inertia_correlation=corr(torque[good],tool[good]),
            not_a_calibrated_loaded_noise_floor=True))
    result=dict(reference='b75d5b8',protocol=PROTOCOL,archived_parameters=archived,original=original_score,variants=variants,FIT_only_selected_variant=chosen,weak_momentum=weak,weak_ablation=weak_ablation,
        motion_synthetic=motion_synthetic(base),tool_mass_tensor_audit=tool_audit,torque_processing_changes_FWL=torque_changes,
        contact_and_model_nuisance=contact_and_nuisance(base,a,data['single_pass_121']),
        unloaded_processing_comparison=unloaded,
        stage_audit=audit,stage_FWL=decomposition,contact_proxy_correlations=correlations,synthetic=synthetic(),
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),ROOT/'scripts/xarm_ft_id_estimator.py',ROOT/'scripts/xarm_wrist_ft.py',ROOT/'scripts/xarm_ft_v1_estimator.py',ROOT/'scripts/update_probing_v3_model.py']},
        input_index_sha256=hashlib.sha256((episode/'telemetry/index.json').read_bytes()).hexdigest(),original_estimate_sha256=hashlib.sha256((episode/'model/estimate.json').read_bytes()).hexdigest(),
        raw_copied=False,simulator_started=False,wall_s=time.perf_counter()-began)
    (out/'DIAGNOSTICS.json').write_text(json.dumps(result,indent=2))
    # Compact, binned residual-versus-acceleration comparison. Residual removes
    # damping/Coulomb only, so the inertia slope remains visible (a full LS
    # residual would be orthogonal to fitted acceleration by construction).
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(10,3.8),sharey=True)
    diagnostic='single_pass_121' # Same 121 window; fixed physical-consistency test, not best truth/CV.
    for ax,name,pieces,score in [(axes[0],'Original SG121',original,original_score),(axes[1],'Consistent single-pass SG121',data[diagnostic],variants[diagnostic])]:
        ss=samples(pieces);b=np.array(score['parameters'])
        for role,color in [('FIT','#287ab0'),('VALIDATION','#d26a25')]:
            selected=[r for r in ss if r['segment'].startswith(role)];xx,yy=xy(selected);rr=yy-xx[:,1:]@b[1:]
            ax.scatter(xx[:,0],rr*1000,s=9,alpha=.45,label=role,color=color)
        aa=np.linspace(-.08,.08,100);ax.plot(aa,1000*b[0]*aa,color='black',lw=1,label='fitted inertia slope')
        ax.set_title(name+'\nI=%.5f kg m²'%b[0],fontsize=9);ax.set_xlabel('measured angular acceleration (rad/s²)');ax.grid(alpha=.2);ax.legend(fontsize=7)
    axes[0].set_ylabel('torque minus fitted damping / Coulomb (mNm)');fig.tight_layout();fig.savefig(out/'RESIDUAL_ACCELERATION.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(original=original_score,selected=chosen,selected_result=variants[chosen],stage_FWL=decomposition,wall_s=result['wall_s']),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',required=True,type=Path);args=p.parse_args();run(args.episode,args.output)
