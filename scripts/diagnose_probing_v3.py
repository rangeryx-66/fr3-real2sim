"""Read-only archived excitation/residual audit. No hidden dynamics imports."""
import argparse,json,time
from pathlib import Path
import numpy as np
from update_probing_v2_model import decode,matrix,process,regression_samples
from xarm_wrist_ft import tool_gravity_world

def columns(p):
    index=json.loads((p/'telemetry/index.json').read_text());pieces={}
    for c in index['chunks']:
        with np.load(p/'telemetry'/c['file'],allow_pickle=False) as f:
            for k in f.files:pieces.setdefault(k,[]).append(f[k])
    return {k:np.concatenate(v) for k,v in pieces.items()},index

def rows_at(a,ix,meta,label):
    rows=[]
    for i in ix:
        bodies=[]
        for prop,pose in zip(meta['body_properties'],a['tool_poses'][i]):
            T=matrix(pose);R=T[:3,:3];bodies.append(dict(name=prop['name'],mass_kg=prop['mass_kg'],T_world_body=T,com_world_m=R@prop['com_body_m']+T[:3,3],inertia_world_kg_m2=R@np.array(prop['inertia_body_kg_m2'])@R.T))
        T=matrix(a['sensor_pose'][i]);rows.append(dict(t=float(a['t'][i]),door_angle_deg=float(np.rad2deg(a['door_q'][i])),phase=meta['phases'][a['phase'][i]],identification_segment=label,
            relative_translation_slip_m=float(a['relative_slip_m'][i]),forces_n=dict(L=float(a['pad_load'][i,0]),R=float(a['pad_load'][i,1])),force_event=dict(status='HARD_STOP' if a['safety_stop'][i] else 'NORMAL'),
            wrist_ft=dict(valid=bool(a['ft_valid'][i]),raw_wrench_sensor=a['raw_ft'][i],T_world_sensor=T,body_states=bodies,tool_gravity_wrench_world=tool_gravity_world(bodies,T[:3,3]))))
    return rows

def corr(a,b):
    return float(np.corrcoef(a,b)[0,1]) if np.std(a)>1e-12 and np.std(b)>1e-12 else None

def diagnose(p,out):
    start=time.perf_counter();groups,settings,segments=decode(p);a,index=columns(p);report=dict(hidden_truth_used=False,segments=[],unloaded_residuals=[]);train=[]
    beta=np.array(json.loads((p/'model/estimate.json').read_text())['constant_model']['parameters'])
    for sid,rows in groups.items():
        d=process(rows,settings['prior'],settings['bias']);mask=d['valid'].copy();edge=np.zeros(len(mask),bool);edge[121:-121]=True;mask &= edge
        if not mask.any():continue
        X=np.c_[d['qdd'],d['qd'],np.sign(d['qd'])];res=d['torque']-X@beta;ix=a['segment']==sid;slip=a['relative_slip_m'][ix];ap=a['aperture'][ix];lag=a['reference_q'][ix]-a['door_q'][ix]
        row=dict(id=sid,role=segments[sid]['role'],records=len(rows),duration_s=float(d['t'][-1]-d['t'][0]),velocity_quantiles_rad_s=np.quantile(d['qd'][mask],[0,.1,.5,.9,1]).tolist(),
            acceleration_rms_rad_s2=float(np.sqrt(np.mean(d['qdd'][mask]**2))),acceleration_P95_rad_s2=float(np.quantile(abs(d['qdd'][mask]),.95)),
            acceleration_energy_kept_by_121_edge_fraction=float(np.sum(d['qdd'][mask]**2)/max(np.sum(d['qdd'][60:-60]**2),1e-30)),
            slip_change_mm=float((slip[-1]-slip[0])*1000),aperture_change_mm=float((ap[-1]-ap[0])*1000),reference_lead_deg=np.rad2deg(np.quantile(lag,[.1,.5,.9])).tolist(),
            residual_RMS_nm=float(np.sqrt(np.mean(res[mask]**2))),residual_slip_rate_correlation=corr(res[mask],np.gradient(slip,d['t'])[mask]),residual_angle_correlation=corr(res[mask],d['q'][mask]),
            raw_native_velocity_vs_SG_RMS_rad_s=float(np.sqrt(np.mean((a['door_dq'][ix][mask]-d['qd'][mask])**2))))
        report['segments'].append(row)
        d['valid']=mask
        if segments[sid]['role'].startswith('FIT') and segments[sid]['result'].get('ok'):train += regression_samples(d)
    X=np.array([[s['qdd'],s['qd'],s['direction']] for s in train]);A=X*np.array([1.,.25,.045]);s=np.linalg.svd(A,compute_uv=False)
    report['successful_train']=dict(bins=len(train),scaled_condition=float(s[0]/s[-1]),scaled_singular_values=s.tolist(),regressor_correlation=np.corrcoef(X.T).tolist(),
        measured_qdd_rms=float(np.sqrt(np.mean(X[:,0]**2))),measured_qd_rms=float(np.sqrt(np.mean(X[:,1]**2))),inertia_sensitivity_after_projecting_velocity_and_sign=float(np.linalg.norm(X[:,0]-X[:,1:]@np.linalg.lstsq(X[:,1:],X[:,0],rcond=None)[0])))
    phases=np.array(index['metadata']['phases'])[a['phase']];ids=np.flatnonzero((phases=='RELEASE')&(a['positive_contacts']==0)&(a['ft_valid']>0))
    runs=np.split(ids,np.flatnonzero(np.diff(a['step'][ids])!=1)+1)
    for ix in runs:
        if len(ix)<500:continue
        rows=rows_at(a,ix,index['metadata'],'UNLOADED_RELEASE');d=process(rows,settings['prior'],settings['bias']);inside=np.arange(121,len(ix)-121);tau=d['torque'][inside];bin_ids=np.floor(d['t'][inside]/.1).astype(int);binned=np.array([tau[bin_ids==j].mean() for j in np.unique(bin_ids) if np.sum(bin_ids==j)>=12])
        report['unloaded_residuals'].append(dict(phase='jaw opening after final native positive-contact report; wrist near preceding grasp orientation',records=len(ix),angle_deg=float(np.rad2deg(np.median(d['q']))),
            duration_s=float(d['t'][-1]-d['t'][0]),compensated_torque_mean_nm=float(tau.mean()),compensated_torque_rms_nm=float(np.sqrt(np.mean(tau**2))),
            robust_sigma_nm=float(1.4826*np.median(abs(tau-np.median(tau)))),P95_abs_nm=float(np.quantile(abs(tau),.95)),
            bins_100ms=len(binned),bin_RMS_nm=float(np.sqrt(np.mean(binned**2))),bin_P95_abs_nm=float(np.quantile(abs(binned),.95)),
            tool_inertia_wrench_RMS=np.sqrt(np.mean(d['inertial_world'][inside]**2,axis=0)).tolist(),not_calibrated_loaded_dynamic_noise_bound=True))
    report.update(wall_s=time.perf_counter()-start,limitations=['unloaded free-space data mostly 10 Hz, not phase-matched 240 Hz loaded-speed calibration','release residual includes moving jaws; sensor bias and compensation/model errors are not separable from this record alone','loaded model residual is not a sensor-only noise measurement'])
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2));return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(diagnose(a.episode,a.output),indent=2))
