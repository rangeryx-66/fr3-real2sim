"""Archived prismatic measurement/identifiability audit; no hidden-truth input.

All alternatives are diagnostic. Frozen production outputs are never replaced.
Complete VALIDATION is used only for reporting, never fitting or variant choice.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from update_probing_v7_model import load_segments
from xarm_ft_v7_estimator import samples,fit,matrix,prediction
from xarm_ft_v31_estimator import common_filter
from diagnose_probing_v3 import columns


def rms(a):return float(np.sqrt(np.mean(np.asarray(a)**2)))
def correlation(a,b):
    return float(np.corrcoef(a,b)[0,1]) if min(np.std(a),np.std(b))>1e-12 else None
def safe(d):
    if isinstance(d,np.ndarray):return safe(d.tolist())
    if isinstance(d,(np.floating,np.integer,np.bool_)):return safe(d.item())
    if isinstance(d,float) and not np.isfinite(d):return None
    if isinstance(d,dict):return {str(k):safe(v) for k,v in d.items()}
    if isinstance(d,(tuple,list)):return [safe(v) for v in d]
    return d


def diagnose(folder):
    folder=Path(folder);pieces,settings,provenance=load_segments(folder,True)
    config=settings['estimation'];a,index=columns(folder);axis=np.array(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    train=[];held=[];segment_rows=[];stage_rows={k:[] for k in ['sensor_world','tool_gravity','tool_inertia']}
    for d in pieces:
        rows=samples(d,config['domain_native'],config['intervals'])
        is_fit=d['segments'][0].startswith('FIT')
        (train if is_fit else held).extend(rows)
        ix=np.searchsorted(a['t'],d['t']);ix=np.minimum(ix,len(a['t'])-1);v=d['valid'];k=np.flatnonzero(v)
        if not len(k):continue
        Y=np.c_[d['qdd'][v],d['qd'][v],d['direction'][v]]
        normal=-a['normal_force_on_fingers_world_N'][ix].sum(1)@axis
        tangential=-a['friction_force_on_fingers_world_N'][ix].sum(1)@axis
        traction=normal+tangential
        dr=np.array([d['q'][-1]-d['q'][0]])
        tcp=a['tcp_pose'][ix,:3]@axis;command=a['command_TCP_pose'][ix,:3]@axis
        point=dict(block=d['block_id'],FIT=is_fit,bins=len(rows),valid_samples=int(v.sum()),duration_s=float(d['t'][-1]-d['t'][0]),
            q_range_m=[float(d['q'].min()),float(d['q'].max())],net_displacement_m=float(dr[0]),
            velocity_rms_m_s=rms(d['qd'][v]),acceleration_rms_m_s2=rms(d['qdd'][v]),
            regressor_correlation=np.corrcoef(Y.T),
            command_measured_tracking_delta_m=float((command[-1]-command[0])-(tcp[-1]-tcp[0])),
            measured_TCP_drawer_relative_delta_m=float((tcp[-1]-tcp[0])-dr[0]),
            aperture_change_m=float(a['aperture'][ix[-1]]-a['aperture'][ix[0]]),
            pad_load_mean_N=a['pad_load'][ix][v].mean(0),pad_load_max_N=a['pad_load'][ix][v].max(0),
            wrist_force_mean_N=float(d['torque'][v].mean()),normal_rail_mean_N=float(normal[v].mean()),
            friction_rail_mean_N=float(tangential[v].mean()),friction_API_available_fraction=a['friction_api_valid'][ix][v].mean(0),
            wrist_minus_native_traction_rms_N=rms(d['torque'][v]-common_filter(traction)[v]),
            wrist_minus_native_traction_acceleration_correlation=correlation((d['torque']-common_filter(traction))[v],d['qdd'][v]))
        if len(rows)>=8:
            fitted=fit(rows,config);point['own_diagnostic_parameters']=fitted['parameters'];point['scaled_condition']=fitted['scaled_condition']
        # Quantify the linear contribution of each measurement stage without
        # selecting compensation coefficients against authored parameters.
        for key,values,sign in [('sensor_world',d['raw_world_hinge'][:,:3]@axis,1),('tool_gravity',d['gravity_world_hinge'][:,:3]@axis,1),('tool_inertia',d['inertial_world_hinge'][:,:3]@axis,-1)]:
            q=dict(d,torque=sign*common_filter(values,d['processing']['window']))
            rr=samples(q,config['domain_native'],config['intervals'])
            if is_fit:stage_rows[key].extend(rr)
        segment_rows.append(point)
    result=dict(hidden_truth_used=False,production_outputs_modified=False,quality=provenance,segments=segment_rows,FIT_bins=len(train),VALIDATION_bins=len(held))
    if len(train)<8:return result
    m=fit(train,config);X=matrix(train,m);y=np.array([r['tau'] for r in train]);beta=np.asarray(m['parameters']);residual=y-X@beta
    result['production_constant_replay']=m
    result['regressor_correlation']=np.corrcoef(X.T)
    result['coefficient_correlation']=np.asarray(m['conditional_covariance'])/np.maximum(np.outer(m['conditional_std'],m['conditional_std']),1e-20)
    result['residual_acceleration_correlation']=correlation(residual,X[:,0])
    result['measurement_stage_unconstrained_coefficients']={key:np.linalg.lstsq(X,np.array([r['tau'] for r in rr]),rcond=None)[0] for key,rr in stage_rows.items()}
    result['leave_one_FIT_trajectory_out']=[]
    labels=list(dict.fromkeys(r['segment'] for r in train))
    for label in labels:
        keep=[r for r in train if r['segment']!=label]
        if len(keep)<8:continue
        f=fit(keep,config)
        result['leave_one_FIT_trajectory_out'].append(dict(removed=label,parameters=f['parameters'],condition=f['scaled_condition'],heldout={d['block_id']:prediction(d,f) for d in pieces if d['segments'][0].startswith('VALIDATION')}))
    # Intercept-per-FIT-trajectory is a confounding diagnostic, not an accepted
    # new object law. It can expose changed grasp/contact offsets across FIT.
    Z=np.array([[float(r['segment']==label) for label in labels] for r in train]);XZ=np.c_[X,Z]
    nuisance=np.linalg.lstsq(XZ,y,rcond=None)[0]
    result['diagnostic_per_trajectory_offset']=dict(parameters=nuisance[:3],offsets=dict(zip(labels,nuisance[3:])),rank=int(np.linalg.matrix_rank(XZ)),columns=XZ.shape[1],FIT_rmse_N=rms(y-XZ@nuisance),not_a_production_model=True)
    result['complete_validation_original']={d['block_id']:prediction(d,m) for d in pieces if d['segments'][0].startswith('VALIDATION')}
    result['force_offset_sensitivity_1mN']=np.linalg.lstsq(X,np.ones(len(X))*.001,rcond=None)[0]
    return safe(result)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episodes',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result={str(f):diagnose(f) for f in args.episodes};args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(safe(result),indent=2))
    print(json.dumps({k:{'FIT_bins':v['FIT_bins'],'VALIDATION_bins':v['VALIDATION_bins'],'parameters':v.get('production_constant_replay',{}).get('parameters'),'condition':v.get('production_constant_replay',{}).get('scaled_condition'),'segments':[{'block':s['block'],'acc_rms':s['acceleration_rms_m_s2'],'velocity_rms':s['velocity_rms_m_s'],'own_parameters':s.get('own_diagnostic_parameters'),'traction_residual':s['wrist_minus_native_traction_rms_N']} for s in v['segments']]} for k,v in result.items()},indent=2))
