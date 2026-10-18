"""Production V3.1 numeric worker. Completed-segment input, atomic compact output.

No simulator imports, model-truth reads, control changes, or momentum-fit
corrections. V3 reproduction remains in update_probing_v3_model.py.
"""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v31_estimator import process,regression_samples,VERSION
from xarm_ft_v1_estimator import fit,forward
from interaction_runtime.probing_profiles_v3 import information


def XY(samples):
    return np.array([[s['qdd'],s['qd'],s['direction']] for s in samples]),np.array([s['tau'] for s in samples])


def motion_prediction(data,indices,model):
    value=forward(data,indices,model)
    if 'prediction_outside_observed_domain' in value:
        value['prediction_outside_configured_0_to_20_domain']=value.pop('prediction_outside_observed_domain')
    limits=model['training_angle_range_deg'];actual=np.rad2deg(data['q'][indices])
    value.update(training_angle_range_deg=limits,measured_validation_outside_training_range_fraction=float(np.mean((actual<limits[0])|(actual>limits[1]))),
        prediction_input='measured wrist wrench; one initial state only, not a predicted contact force',actual_training_domain_predicted_trajectory_check='not supplied by frozen forward integrator; configured-domain flag is not a training-coverage guarantee')
    return value


def conditional_model(samples,window=121):
    m=fit(samples,'constant');X,y=XY(samples);beta=np.array(m['parameters']);e=y-X@beta;n=len(y)
    ridge=np.diag((.001/np.array([1.,.25,.045]))**2);bread=np.linalg.pinv(X.T@X+ridge)
    Z=X*e[:,None];meat=Z.T@Z;labels=np.array([s.get('correlation_group',s['segment']) for s in samples]);t=np.array([s['t'] for s in samples])
    lags=max(5,int(np.ceil((window-1)/240/.1)))
    for lag in range(1,min(lags+1,n)):
        valid=(labels[lag:]==labels[:-lag])&(abs(t[lag:]-t[:-lag]-.1*lag)<.06)
        cross=Z[lag:][valid].T@Z[:-lag][valid];meat+=(1-lag/(lags+1))*(cross+cross.T)
    hc=bread@meat@bread;sigma2=max(float(e@e/max(n-m['algebraic_rank'],1)),1e-12)
    iid=sigma2*bread@(X.T@X)@bread
    covariance=(hc+hc.T)/2+np.diag(np.maximum(np.diag(iid)-np.diag(hc),0))
    values,vectors=np.linalg.eigh(covariance);covariance=(vectors*np.maximum(values,0))@vectors.T;std=np.sqrt(np.diag(covariance))
    m.pop('scaled_fisher_information',None)
    m.update(conditional_std=std.tolist(),conditional_covariance=covariance.tolist(),residual_variance_nm2=sigma2,HAC_lags=lags,
        parameter_names=['I_eff_kg_m2','b_Nm_s_rad','C_Nm'],
        supported=[bool(m['algebraic_rank']==3 and b>3*s) for b,s in zip(beta,std)],
        support_semantics='conditional regressor/residual support only; not independently validated physical accuracy',
        uncertainty='FIT-only Bartlett HAC with iid diagonal floor; correlated-bin and ridge-aware; covariance diagonal equals std squared; no systematic-error coverage',
        validation_used_for_parameters_or_uncertainty=False,physical_accuracy_validated=False,
        information=information(X),training_angle_range_deg=np.rad2deg([min(s['q'] for s in samples),max(s['q'] for s in samples)]).tolist())
    return m


def load_segments(folder,window=121,allow_interrupted_fit=False):
    a,idx=columns(folder);settings=json.loads((folder/'identification_input.json').read_text());meta=idx['metadata'];phase=np.array(meta['phases'])[a['phase']]
    pieces=[];provenance=[];train=[];test=[]
    for s in idx['segments']:
        ix=np.flatnonzero((a['segment']==s['id'])&np.isin(phase,['OPEN_CONTINUE','PANEL_PUSH']))
        record=dict(id=s['id'],role=s['role'],records=len(ix),blocks=[]);provenance.append(record)
        from interaction_runtime.segment_quality import selection
        ix,quality=selection(s,a,ix,allow_interrupted_fit);record['quality']=quality
        if not len(ix):
            record['status']='not eligible for fit/validation; original action status retained';continue
        if not s['role'].startswith(('FIT','VALIDATION')):
            record['status']='undeclared role, not fit';continue
        finite=np.isfinite(a['door_dq'][ix])&np.isfinite(a['door_q'][ix])&np.isfinite(a['raw_ft'][ix]).all(axis=1)&np.isfinite(a['sensor_pose'][ix]).all(axis=1)&np.isfinite(a['tool_poses'][ix]).all(axis=(1,2))&(a['ft_valid'][ix]>0)
        valid_ix=ix[finite];cuts=np.flatnonzero((np.diff(a['step'][valid_ix])!=1)|(abs(np.diff(a['t'][valid_ix])-1/240)>1e-7))+1
        for block,run in enumerate(np.split(valid_ix,cuts)):
            label=s['role']+'_%02d'%s['id'];rows=rows_at(a,run,meta,label)
            d=process(rows,settings['prior'],settings['bias'],a['door_dq'][run],window=window) if len(run)>0 else None
            if d is None:
                record['blocks'].append(dict(records=len(run),status='insufficient local support; no interpolation across gaps'));continue
            d['block_id']=label+'#%d'%block;ss=regression_samples(d)
            for sample in ss:sample['correlation_group']=d['block_id']
            (train if label.startswith('FIT') else test).extend(ss);pieces.append(d)
            record['blocks'].append(dict(records=len(run),valid=int(d['valid'].sum()),bins=len(ss),first_step=int(a['step'][run[0]]),last_step=int(a['step'][run[-1]]),processing=d['processing']))
    return pieces,train,test,provenance


def update(folder,output=None,window=121,plot=True,allow_interrupted_fit=False):
    began=time.perf_counter();folder=Path(folder);out=Path(output) if output is not None else folder/'model';out.mkdir(parents=True,exist_ok=True)
    if (out/'estimate.json').exists() and json.loads((out/'estimate.json').read_text()).get('estimator_version')!=VERSION:
        raise ValueError('Preserve legacy result: specify a NEW --output directory')
    pieces,train,test,provenance=load_segments(folder,window,allow_interrupted_fit)
    sources=[Path(__file__),ROOT/'scripts/xarm_ft_v31_estimator.py',ROOT/'scripts/xarm_ft_v1_estimator.py',ROOT/'interaction_runtime/segment_quality.py']
    result=dict(estimator_version=VERSION,status='insufficient informative data',model='tau=I_eff*qdd+b*qd+C*sign(qd)',window=window,
        training_bins=len(train),validation_bins=len(test),segments=provenance,hidden_dynamics_used=False,segment_local_processing=True,
        complete_validation_excluded_from_fit_uncertainty_and_selection=True,tool_compensation_count=1,interrupted_fit_selection_enabled=allow_interrupted_fit,
        source_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},
        input_index_sha256=hashlib.sha256((folder/'telemetry/index.json').read_bytes()).hexdigest(),
        limitations=['ideal virtual load cell and approximate actuators','conditional uncertainty excludes compensation/solver/model bias','authored inertia is not an independent effective PhysX measurement','finite-difference and centered-filter latency remain'])
    if len(train)>=8:
        model=conditional_model(train,window);result.update(status='updated',constant_model=model,identified_parameters={n:float(b) if s else None for n,b,s in zip(['I_eff_kg_m2','b_Nm_s_rad','C_Nm'],model['parameters'],model['supported'])})
        cv=[]
        for label in dict.fromkeys(s['segment'] for s in train):
            left=[s for s in train if s['segment']!=label];held=[s for s in train if s['segment']==label]
            if len(left)<8:continue
            cm=conditional_model(left,window);X,y=XY(held);cv.append(dict(held_segment=label,parameters=cm['parameters'],rmse_nm=float(np.sqrt(np.mean((y-X@cm['parameters'])**2)))))
        model['FIT_only_leave_segment_out']=cv
        if test:
            X,y=XY(test);model['heldout_torque_rmse_nm']=float(np.sqrt(np.mean((y-X@model['parameters'])**2)))
        model['heldout_motion']={};model['heldout_motion_unfiltered_input']={};model['full_segment_boundary_sensitivity']={}
        for d in pieces:
            if not str(d['segments'][0]).startswith('VALIDATION'):continue
            edge=d['processing']['boundary_exclusion_samples'];indices=np.arange(edge,len(d['t'])-edge);clone=dict(d);clone['qd']=d['native_qd'].copy()
            value=motion_prediction(clone,indices,model);value.update(initial_velocity='native measured, one initial condition only',velocity_reference='native measured',input='once-filtered compensated wrist torque; no state correction',filter_lookahead_s=d['processing']['filter_lookahead_s'])
            model['heldout_motion'][d['block_id']]=value
            clone['torque']=d['unfiltered_torque'];model['heldout_motion_unfiltered_input'][d['block_id']]=motion_prediction(clone,indices,model)
            clone['torque']=d['torque'];model['full_segment_boundary_sensitivity'][d['block_id']]=dict(**motion_prediction(clone,np.arange(len(d['t'])),model),includes_unreliable_filter_and_derivative_edges=True)
    if plot and pieces:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,1,figsize=(8,4),sharex=True)
        for d in pieces:
            good=d['valid'];axes[0].plot(d['t'],np.rad2deg(d['q']),label=str(d['segments'][0]));axes[1].plot(d['t'][good],d['torque'][good],lw=.6)
            if 'constant_model' in result:axes[1].plot(d['t'][good],np.c_[d['qdd'][good],d['qd'][good],d['direction'][good]]@result['constant_model']['parameters'],lw=.6)
        axes[0].legend(fontsize=6);axes[0].set_ylabel('actual degrees');axes[1].set_ylabel('filtered hinge Nm');axes[1].set_xlabel('simulation seconds');fig.tight_layout();fig.savefig(out/'feedback.png',dpi=90);plt.close(fig)
    result['worker_wall_s']=time.perf_counter()-began
    atomic_json(out/'estimate.json',result)
    h=out/'updates.json';history=json.loads(h.read_text()) if h.exists() else []
    history.append(dict(estimator_version=VERSION,training_bins=len(train),validation_bins=len(test),parameters=result.get('constant_model',{}).get('parameters'),conditional_std=result.get('constant_model',{}).get('conditional_std'),worker_wall_s=result['worker_wall_s']))
    atomic_json(h,history)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);parser.add_argument('--output',type=Path);parser.add_argument('--window',type=int,default=121);parser.add_argument('--no-plot',action='store_true');parser.add_argument('--allow-interrupted-fit',action='store_true');args=parser.parse_args()
    r=update(args.folder,args.output,args.window,not args.no_plot,args.allow_interrupted_fit);print(json.dumps(dict(estimator_version=r['estimator_version'],status=r['status'],worker_wall_s=r['worker_wall_s'])))
