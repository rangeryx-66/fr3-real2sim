"""Cached 240 Hz, segment-local identification. No hidden plant parameter input."""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_id_estimator import process
from xarm_ft_v1_estimator import regression_samples,fit,forward
from interaction_runtime.probing_profiles_v3 import information

def XY(samples):
    return np.array([[s['qdd'],s['qd'],s['direction']] for s in samples]),np.array([s['tau'] for s in samples])

def conditional_model(samples):
    m=fit(samples,'constant');X,y=XY(samples);error=X@np.array(m['parameters'])-y;n=len(y);bread=np.linalg.pinv(X.T@X)
    # Correlated 100 ms bins: report a lag-5 sandwich, never iid-240-Hz certainty.
    Z=X*error[:,None];meat=Z.T@Z
    labels=[s['segment'] for s in samples]
    for lag in range(1,min(6,n)):
        valid=np.array([labels[i]==labels[i-lag] for i in range(lag,n)])
        cross=Z[lag:][valid].T@Z[:-lag][valid];meat+=(1-lag/6)*(cross+cross.T)
    hc=bread@meat@bread;std=np.sqrt(np.maximum(np.diag(hc),0))
    iid=np.sqrt(np.maximum(np.diag(max(float(np.mean(error**2)),1e-12)*bread),0));std=np.maximum(std,iid)
    m.pop('scaled_fisher_information',None)
    m.update(conditional_std=std.tolist(),conditional_covariance=hc.tolist(),scaled_information_eigenvalues=(np.array(information(X)['singular_values'])**2).tolist(),
        supported=[bool(b>3*u and m['algebraic_rank']==3) for b,u in zip(m['parameters'],std)],
        historical_worst_case_residual_not_admission_floor=True,uncertainty='FIT-only residual lag-5 HAC and iid maximum; three-standard-error diagnostic support only, not calibrated confidence or execution gate')
    return m

def update(folder):
    began=time.perf_counter();a,idx=columns(folder);settings=json.loads((folder/'identification_input.json').read_text());meta=idx['metadata'];phases=np.array(meta['phases'])[a['phase']]
    pieces=[];provenance=[];train=[];test=[];unloaded=[]
    for segment in idx['segments']:
        sid=segment['id'];ix=np.flatnonzero((a['segment']==sid)&np.isin(phases,['OPEN_CONTINUE','PANEL_PUSH']))
        item=dict(id=sid,role=segment['role'],records=len(ix),result=segment.get('result',{}));provenance.append(item)
        if not segment.get('completed') or not segment.get('result',{}).get('ok'):
            item['fit_status']='incomplete/failed segment retained, not used for parameter fit';continue
        if len(ix)<247 or np.any(np.diff(a['step'][ix])!=1):item['fit_status']='insufficient contiguous derivative support';continue
        label=segment['role']+'_%02d'%sid;d=process(rows_at(a,ix,meta,label),settings['prior'],settings['bias']);d['valid'][:121]=False;d['valid'][-121:]=False
        d['native_qd']=a['door_dq'][ix];pieces.append(d);samples=regression_samples(d);(train if label.startswith('FIT') else test).extend(samples)
        good=d['valid'];X=np.c_[d['qdd'][good],d['qd'][good],np.sign(d['qd'][good])]
        item.update(fit_status='segment-local processed',valid=int(good.sum()),duration_s=float(d['t'][-1]-d['t'][0]),angle_range_deg=np.rad2deg([d['q'].min(),d['q'].max()]).tolist(),
            velocity_quantiles_rad_s=np.quantile(d['qd'][good],[.05,.5,.95]).tolist(),acceleration_rms_rad_s2=float(np.sqrt(np.mean(d['qdd'][good]**2))),information=information(X),
            slip_range_mm=(np.quantile(a['relative_slip_m'][ix],[0,.5,1])*1000).tolist(),native_vs_position_velocity_rms_rad_s=float(np.sqrt(np.mean((a['door_dq'][ix][good]-d['qd'][good])**2))))
    # Phase/pose-conditioned residual references; not calibrated loaded noise bounds.
    candidates=np.flatnonzero(np.isin(phases,['APPROACH','RELEASE'])&(a['positive_contacts']==0)&(a['ft_valid']>0))
    runs=np.split(candidates,np.flatnonzero((np.diff(a['step'][candidates])!=1)|(np.diff(a['phase'][candidates])!=0))+1)
    for ix in runs:
        if len(ix)<500:continue
        d=process(rows_at(a,ix,meta,'UNLOADED_CALIBRATION'),settings['prior'],settings['bias']);inside=np.arange(121,len(ix)-121);tau=d['torque'][inside];bins=np.floor(d['t'][inside]/.1).astype(int);bt=np.array([tau[bins==k].mean() for k in np.unique(bins) if (bins==k).sum()>=12]);pose=a['tcp_pose'][ix,:3];speed=np.linalg.norm(np.diff(pose,axis=0),axis=1)*240
        unloaded.append(dict(phase=str(phases[ix[0]]),records=len(ix),angle_deg=float(np.rad2deg(np.median(d['q']))),rms_nm=float(np.sqrt(np.mean(tau**2))),bin_rms_nm=float(np.sqrt(np.mean(bt**2))),
            bin_P95_abs_nm=float(np.quantile(abs(bt),.95)),TCP_speed_P50_P95_mm_s=(np.quantile(speed,[.5,.95])*1000).tolist(),not_a_loaded_noise_bound=True))
    result=dict(status='insufficient informative data',model='tau=I_eff*qdd+b*qd+C*sign(qd)',hidden_dynamics_used=False,segments=provenance,unloaded_residuals=unloaded,
        training_bins=len(train),validation_bins=len(test),segment_local_derivatives=True,validation_entirely_excluded_from_fit_and_uncertainty=True,worker_decode='NPZ columns materialized once',
        limitations=['ideal virtual load cell, no hardware accuracy claim','conditional uncertainty does not bound contact/compensation bias','mass/COM/full tensor and gravity not independently identified'])
    if len(train)>=8:
        m=conditional_model(train);X,y=XY(train);beta=np.array(m['parameters']);info=information(X);result.update(status='updated',constant_model=m,information=info)
        m['parameter_names']=['I_eff_kg_m2','b_Nm_s_rad','C_Nm'];cv=[]
        for label in dict.fromkeys(s['segment'] for s in train):
            left=[s for s in train if s['segment']!=label];held=[s for s in train if s['segment']==label]
            if len(left)<8:continue
            cm=conditional_model(left);xx,yy=XY(held);cv.append(dict(held_segment=label,parameters=cm['parameters'],rmse_nm=float(np.sqrt(np.mean((xx@cm['parameters']-yy)**2)))))
        m['FIT_only_leave_segment_out']=cv
        m['phase_residual_uncertainty_sensitivity']=[dict(phase=r['phase'],bin_residual_nm=r['bin_rms_nm'],conditional_std=(np.sqrt(np.maximum(np.diag(np.linalg.pinv(X.T@X)),0))*max(r['bin_rms_nm'],m['training_rmse_nm'])).tolist()) for r in unloaded]
        result['excitation']=dict(acceleration_rms_rad_s2=float(np.sqrt(np.mean(X[:,0]**2))),velocity_rms_rad_s=float(np.sqrt(np.mean(X[:,1]**2))),inertial_torque_rms_nm=float(beta[0]*np.sqrt(np.mean(X[:,0]**2))),damping_torque_rms_nm=float(beta[1]*np.sqrt(np.mean(X[:,1]**2))),
            projected_sensitivities=[float(np.linalg.norm(X[:,j]-np.delete(X,j,axis=1)@np.linalg.lstsq(np.delete(X,j,axis=1),X[:,j],rcond=None)[0])) for j in range(3)])
        result['identified_parameters']={k:(float(b) if support else None) for k,b,support in zip(m['parameter_names'],beta,m['supported'])}
        m['heldout_motion']={}
        if test:
            xx,yy=XY(test);m['heldout_torque_rmse_nm']=float(np.sqrt(np.mean((xx@beta-yy)**2)))
        for d in pieces:
            label=str(d['segments'][0])
            if not label.startswith('VALIDATION'):continue
            # One measured initial state; thereafter free prediction using measured
            # F/T input only. No trajectory-following correction.
            ix=np.arange(121,len(d['t'])-121);clone=dict(d);clone['qd']=d['qd'].copy();clone['qd'][ix[0]]=d['native_qd'][ix[0]]
            value=forward(clone,ix,m);value.update(initial_velocity='native measured at sole initial condition',evaluation='complete held-out trajectory reserved; 121-sample derivative edges excluded',training_angle_range_deg=np.rad2deg([min(s['q'] for s in train),max(s['q'] for s in train)]).tolist());m['heldout_motion'][label]=value
        result['parameter_stability_warning']=bool(cv and np.any(np.ptp(np.array([v['parameters'] for v in cv]),axis=0)>2*np.maximum(beta,1e-8)))
    out=folder/'model';out.mkdir(exist_ok=True);result['worker_wall_s']=time.perf_counter()-began;atomic_json(out/'estimate.json',result)
    h=out/'updates.json';history=json.loads(h.read_text()) if h.exists() else [];history.append(result);atomic_json(h,history)
    if pieces:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(4,1,figsize=(10,8),sharex=True)
        for d in pieces:
            good=d['valid'];label=str(d['segments'][0]);ax[0].plot(d['t'],np.rad2deg(d['q']),label=label);ax[1].plot(d['t'],d['qd'],lw=.7);ax[2].plot(d['t'],d['qdd'],lw=.7);ax[3].plot(d['t'][good],d['torque'][good],lw=.6)
            if result.get('constant_model'):ax[3].plot(d['t'][good],np.c_[d['qdd'][good],d['qd'][good],np.sign(d['qd'][good])]@result['constant_model']['parameters'],lw=.6)
        ax[0].legend(fontsize=7)
        for row,ylabel in zip(ax,['actual degrees','rad/s','rad/s²','F/T hinge Nm']):row.set_ylabel(ylabel);row.grid(alpha=.2)
        ax[-1].set_xlabel('simulation seconds');fig.tight_layout();fig.savefig(out/'feedback.png',dpi=110);plt.close(fig)
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();print(json.dumps(update(a.folder)))
