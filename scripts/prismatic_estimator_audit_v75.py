"""Read-only prismatic generality diagnostics, starting at raw sensor inputs.

No scene truth, controller or production-result writes. Synthetic/scenario
results are explicitly not native-contact or real-hardware validation.
"""
import copy, hashlib
import numpy as np
from scipy.optimize import lsq_linear
from xarm_ft_v31_estimator import process
from xarm_ft_v7_estimator import basis, samples, fit, matrix, prediction
from update_prismatic_v73_model import qualify
from xarm_wrist_ft import tool_gravity_world


def matched_mask(old, qualified, t, stratum_s=1.):
    """Equal counts in fixed time strata, uniform by index, never by force/GT."""
    old=np.asarray(old,bool);qualified=np.asarray(qualified,bool);t=np.asarray(t)
    if np.any(qualified & ~old):raise ValueError('Qualification must be a subset')
    out=np.zeros(len(t),bool);strata=np.floor((t-t[0])/stratum_s).astype(int)
    audit=[]
    for group in np.unique(strata):
        pool=np.flatnonzero(old & (strata==group));count=int(np.sum(qualified & (strata==group)))
        if count:
            selected=pool[np.linspace(0,len(pool)-1,count).round().astype(int)]
            out[selected]=True
        audit.append(dict(stratum=int(group),original=len(pool),retained=count))
    return out,audit


def perturb_raw(rows, q, velocity, scenario, seed=7501):
    """Apply one declared measurement scenario BEFORE common processing.

    Resolution is deterministic uniform quantization. Gaussian SDs and timing /
    calibration errors are scenario assumptions, not claimed manufacturer noise.
    Delay clamps to the same segment's first sample; no other block is accessed.
    """
    r=copy.deepcopy(rows);q=np.array(q,float,copy=True);v=np.array(velocity,float,copy=True)
    if len(r)<3 or q.shape!=(len(r),) or v.shape!=q.shape:raise ValueError('Aligned raw segment required')
    # Independent deterministic measurement errors per block, including held-out
    # blocks. Reusing one noise vector in FIT and VALIDATION biases evaluation.
    label=str(r[0]['identification_segment'])+'/'+repr(r[0]['t'])
    stream=int.from_bytes(hashlib.sha256(label.encode()).digest()[:8],'little')
    rng=np.random.default_rng(np.random.SeedSequence([seed,stream]))
    for name in ('bias_sensor','noise_sd_sensor','quantum_sensor'):
        if name in scenario:
            value=np.asarray(scenario[name],float)
            if value.shape!=(6,)or not np.isfinite(value).all():raise ValueError('Finite six-axis scenario required: '+name)
            if name!='bias_sensor'and np.any(value<0):raise ValueError('Nonnegative noise/quantum required')
    raw=np.array([x['wrist_ft']['raw_wrench_sensor']for x in r],float)
    raw+=np.asarray(scenario.get('bias_sensor',[0.]*6))
    raw+=rng.normal(size=raw.shape)*np.asarray(scenario.get('noise_sd_sensor',[0.]*6))
    quantum=np.asarray(scenario.get('quantum_sensor',[0.]*6),float)
    if np.any(quantum<0):raise ValueError('Nonnegative quantization required')
    active=quantum>0;raw[:,active]=np.round(raw[:,active]/quantum[active])*quantum[active]
    delay=scenario.get('delay_samples',0)
    if type(delay)is not int:raise ValueError('Integer native sample delay required')
    if delay<0 or delay>=len(r):raise ValueError('Bounded same-segment causal delay required')
    raw=raw[np.maximum(0,np.arange(len(raw))-delay)]
    q+=rng.normal(0,float(scenario.get('q_noise_sd_m',0)),len(q))
    step=float(scenario.get('q_quantum_m',0))
    if step>0:q=np.round(q/step)*step
    if scenario.get('velocity_from_position'):
        v=np.gradient(q,np.array([x['t']for x in r]))
    mass_scale=float(scenario.get('tool_mass_scale',1.))
    com_offset=np.asarray(scenario.get('tool_com_offset_body_m',[0.,0.,0.]))
    if mass_scale<=0:raise ValueError('Positive tool calibration mass required')
    for i,row in enumerate(r):
        ft=row['wrist_ft'];ft['raw_wrench_sensor']=raw[i]
        if mass_scale!=1. or np.any(com_offset):
            for body in ft['body_states']:
                R=np.asarray(body['T_world_body'])[:3,:3]
                body['mass_kg']*=mass_scale
                body['com_world_m']=np.asarray(body['com_world_m'])+R@com_offset
                body['inertia_world_kg_m2']=np.asarray(body['inertia_world_kg_m2'])*mass_scale
            ft['tool_gravity_wrench_world']=tool_gravity_world(ft['body_states'],np.asarray(ft['T_world_sensor'])[:3,3])
    return r,q,v


def momentum_fit(pieces,config,template,window_s=.5):
    """Independent integral balance on complete resolved-sliding windows.

    Int(F dt) = m*Delta(v) + b*Delta(x) + Int(C(x)*sign(v)dt).
    The effort is unfiltered but already compensated once. No acceleration
    differentiation or filtered-regression coefficient enters this formulation.
    """
    X=[];Y=[];windows=[];n=template['intervals'];controls=template['active_controls']
    for d in pieces:
        if not d['segments'][0].startswith('FIT'):continue
        valid=d['valid'];t=d['t'];count=max(2,int(round(window_s/d['processing']['dt_s'])))
        for first in range(0,len(t)-count,count):
            last=first+count;ix=np.arange(first,last+1)
            if not valid[ix].all():continue
            h=t[last]-t[first]
            C=basis(d['q'][ix],config['domain_native'],n)[:,controls] if n else np.ones((len(ix),1))
            integral=np.trapezoid(C*np.sign(d['native_qd'][ix])[:,None],t[ix],axis=0)
            X.append(np.r_[d['native_qd'][last]-d['native_qd'][first],d['q'][last]-d['q'][first],integral]/h)
            Y.append(np.trapezoid(d['unfiltered_torque'][ix],t[ix])/h)
            windows.append(dict(block=d['block_id'],interval_s=[float(t[first]),float(t[last])]))
    if len(X)<8:return dict(status='unknown: insufficient independent sliding integral windows',windows=windows)
    X=np.asarray(X);Y=np.asarray(Y);scale=np.r_[config['parameter_scales'][:2],np.repeat(config['parameter_scales'][2],len(controls))]
    ridge=config.get('ridge',.001)*np.diag(1/scale)
    D=np.diff(np.eye(n+3),n=2,axis=0)[:,controls]if n else np.empty((0,1))
    P=np.c_[np.zeros((len(D),2)),D]*template.get('smoothness',0)/config['parameter_scales'][2]
    beta=lsq_linear(np.vstack([X,ridge,P]),np.r_[Y,np.zeros(len(ridge)+len(P))],bounds=(0,np.inf),tol=1e-12).x
    s=np.linalg.svd(X*scale,compute_uv=False);rank=int(np.linalg.matrix_rank(X))
    return dict(status='estimated'if rank==len(beta)else'unknown: rank deficient integral design',
        parameters=beta.tolist(),rank=rank,columns=len(beta),scaled_condition=float(s[0]/s[-1])if s[-1]>1e-15 else None,
        force_balance_RMSE_N=float(np.sqrt(np.mean((Y-X@beta)**2))),windows=windows,
        same_model_family=True,used_GT=False,not_a_production_replacement=True)


def stability(rows,config,template,replicates=32,block_s=1.,seed=7502):
    """Same selected family in LOFO and within-trajectory contiguous bootstrap.

    A per-trajectory constant fit is NOT substituted for a selected C(x) law.
    Empirical variation is reported separately from conditional covariance.
    """
    labels=list(dict.fromkeys(r['segment']for r in rows));n=template['intervals'];smooth=template.get('smoothness',0)
    leave=[]
    for label in labels:
        kept=[r for r in rows if r['segment']!=label]
        if len(kept)<8:leave.append(dict(omitted=label,status='unknown: insufficient bins'));continue
        m=fit(kept,config,n,smooth)
        leave.append(dict(omitted=label,status='estimated',model=m,
                          active_control_change=m['active_controls']!=template['active_controls']))
    blocks={}
    for label in labels:
        rr=sorted([r for r in rows if r['segment']==label],key=lambda r:r['t']);t0=rr[0]['t'];group={}
        for r in rr:group.setdefault(int((r['t']-t0)//block_s),[]).append(r)
        blocks[label]=list(group.values())
    rng=np.random.default_rng(seed);boot=[]
    for k in range(replicates):
        rr=[]
        for label,groups in blocks.items():
            for j,i in enumerate(rng.integers(0,len(groups),len(groups))):
                rr.extend(dict(r,correlation_group=f'{label}/bootstrap{k}/block{j}')for r in groups[int(i)])
        if len(rr)<8:continue
        m=fit(rr,config,n,smooth)
        boot.append(dict(parameters=m['parameters'],active_controls=m['active_controls'],rank=m['algebraic_rank']))
    return dict(leave_one_FIT_out=leave,bootstrap=boot,replicates=replicates,block_s=block_s,
                family_fixed_from_FIT_only_selection=True,validation_used=False,
                empirical_spread_is_not_calibrated_confidence=True)


def compare_qualification(pieces,config,template):
    """Identical compensation/model/validation, eligibility is the only factor."""
    variants={k:[]for k in ('original','full_support','matched_count_time_control')};retention=[]
    for original in pieces:
        if not original['segments'][0].startswith('FIT'):continue
        strict=copy.deepcopy(original);packet=qualify(strict)
        control=copy.deepcopy(original);control['valid'],strata=matched_mask(original['valid'],strict['valid'],original['t'])
        retention.append(dict(packet,block=original['block_id'],time_strata=strata,
            old_acceleration_energy=float(np.sum(original['qdd'][original['valid']]**2)),
            retained_acceleration_energy=float(np.sum(strict['qdd'][strict['valid']]**2)),
            matched_acceleration_energy=float(np.sum(control['qdd'][control['valid']]**2))))
        for name,d in [('original',copy.deepcopy(original)),('full_support',strict),('matched_count_time_control',control)]:
            variants[name].append(d)
    result={};held=[copy.deepcopy(d)for d in pieces if d['segments'][0].startswith('VALIDATION')]
    for d in held:samples(d,config['domain_native'],config['intervals'])
    for name,pp in variants.items():
        rr=[]
        for d in pp:rr.extend(samples(d,config['domain_native'],config['intervals']))
        row=dict(bins=len(rr),eligible_samples=sum(int(d['valid'].sum())for d in pp))
        if len(rr)>=8:
            m=fit(rr,config,template['intervals'],template.get('smoothness',0));row.update(model=m,
                heldout={d['block_id']:prediction(d,m)for d in held},
                integral=momentum_fit(pp,config,m))
        else:row['status']='unknown: insufficient eligible FIT'
        result[name]=row
    return dict(variants=result,retention=retention,model_family_fixed=True,
                validation_used_for_selection=False,production_results_modified=False)


def applicability(rows,model,replicates=32,seed=7503):
    """FIT-only lack-of-fit warning, never a manipulation gate or calibrated CI.

    Use the SAME fitted C(x) law before stratifying residuals. Grouped means are
    block-resampled in one-second units; no per-direction refit invents masses.
    Numerical zero and the measurable regularization sensitivity are reported.
    """
    if not rows:return dict(status='unknown: no qualified FIT')
    X=matrix(rows,model);y=np.array([r['tau']for r in rows]);res=y-X@model['parameters']
    if np.linalg.matrix_rank(X)<X.shape[1]:return dict(status='unknown: rank deficient',execution_gate=False)
    numerical=64*np.finfo(float).eps*max(1.,float(np.max(abs(y))))
    rng=np.random.default_rng(seed);groups=[]
    velocity=np.array([r['qd']for r in rows]);acc=np.array([r['qdd']for r in rows])
    for label,mask in [('opening',velocity>0),('closing',velocity<0),
                       ('slower',abs(velocity)<=np.median(abs(velocity))),
                       ('faster',abs(velocity)>np.median(abs(velocity))),
                       ('accelerating',acc>=0),('decelerating',acc<0)]:
        blocks={}
        for i in np.flatnonzero(mask):
            r=rows[i];blocks.setdefault((r['segment'],int(r['t'])),[]).append(float(res[i]))
        means=np.array([np.mean(v)for v in blocks.values()])
        if len(means)<3:
            groups.append(dict(label=label,status='unknown: fewer than three time blocks'));continue
        boot=[float(rng.choice(means,len(means),replace=True).mean())for _ in range(replicates)]
        low,high=np.quantile(boot,[.025,.975]);nonzero=bool(low>numerical or high< -numerical)
        groups.append(dict(label=label,time_blocks=len(means),mean_residual_N=float(means.mean()),
            empirical_mean_interval_N=[float(low),float(high)],repeatable_nonzero_residual=nonzero))
    mismatch=any(g.get('repeatable_nonzero_residual')for g in groups)
    unregularized=np.linalg.lstsq(X,y,rcond=None)[0]
    return dict(status='untrusted: structured FIT residual; single-law parameters need qualification'if mismatch else'conditional_only: no structured residual detected in tested FIT',
        groups=groups,regularization_and_constraint_effect_RMS_N=float(np.sqrt(np.mean((X@(np.asarray(model['parameters'])-unregularized))**2))),
        empirical_intervals_are_not_calibrated_sensor_confidence=True,validation_used=False,
        hidden_truth_used=False,execution_gate=False,coefficients_modified=False)
