"""Robot compensation and virtual-work diagnostic, using allowed channels only.

Native object q is deliberately not loaded here. No object coefficients fitted.
No Isaac app/import, wrist wrench, hidden object values or controller commands.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import pinocchio as pin
from xarm_ft_v31_estimator import common_filter
from .effort import load_allowed,contiguous,virtual_work,FIELDS


def summary(x):
    x=np.asarray(x);return dict(rmse=np.sqrt(np.mean(x*x,axis=0)).tolist(),
        mean=np.mean(x,axis=0).tolist(),mae=np.mean(abs(x),axis=0).tolist())


def geometry(episode):
    # Only the declared allowed kinematic prior is exported, never bias or wrench.
    d=json.loads((episode/'identification_input.json').read_text())['prior']
    return {k:d[k]for k in ['joint_type','axis_world','origin_world_m']}


def prepare(episode):
    a,idx=load_allowed(episode);metadata=idx['metadata'];load=metadata['v83_load_telemetry']
    names=load['native_dof_order'];arm=np.asarray(load['arm_indices']);phases=np.array(metadata['phases'])[a['phase']]
    io=episode/'robot_io';urdf=io/'dynamics_urdf.urdf';model=pin.buildModelFromUrdf(str(urdf));data=model.createData()
    jointnames=list(model.names)[1:];pmap=[names.index(n)for n in jointnames]
    p_arm=[model.getJointId('joint%d'%j)-1 for j in range(1,8)]
    config=json.loads((io/'NATIVE_CONFIG.json').read_text());frame=model.getFrameId(config['flange_frame'])
    assert frame<len(model.frames)
    base=np.array(json.loads((episode/'V81_MOBILE_MOUNT.json').read_text())['initial_T_world_base'])
    R=base[:3,:3];axis=np.asarray(geometry(episode)['axis_world'],float);axis/=np.linalg.norm(axis)
    origin=np.asarray(geometry(episode)['origin_world_m']);kind=geometry(episode)['joint_type']
    n=len(a['t']);h=np.full((n,7),np.nan);g=h.copy();raw=a['projected_joint_efforts_native'][:,arm]
    valid=(a['projected_joint_efforts_native_valid']>0)&np.isfinite(raw).all(1)&(a['safety_stop']==0)
    dense=np.zeros(n,bool);qdd=np.zeros((n,model.nv))
    for run in contiguous(a):
        if len(run)<247:continue
        qdd[run]=np.gradient(a['dq'][run][:,pmap],a['t'][run],axis=0)
        dense[run[121:-121]]=True
    # Compute full-robot RNEA including moving native G1 links, not a tool-only
    # gravity subtraction. Underexcited friction is not fitted as object friction.
    for i in np.flatnonzero(dense & valid):
        q=a['q'][i,pmap];v=a['dq'][i,pmap]
        h[i]=pin.rnea(model,data,q,v,qdd[i])[p_arm]
        g[i]=pin.computeGeneralizedGravity(model,data,q)[p_arm]
    allowed_no_contact=np.isin(phases,['PREGRASP','APPROACH','SAFE_PARK_MOTION'])
    no_contact=dense&valid&allowed_no_contact&(a['positive_contacts']==0)&(np.max(abs(a['dq'][:,:2]),axis=1)<1e-4)
    return dict(episode_id=episode.name,a=a,idx=idx,names=names,arm=arm,phases=phases,model=model,data=data,pmap=pmap,p_arm=p_arm,
        frame=frame,R=R,base=base,axis=axis,origin=origin,kind=kind,h=h,g=g,raw=raw,
        valid=valid,dense=dense,no_contact=no_contact,
        robot_urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest())


def calibrate(p):
    runs=[r for r in contiguous(p['a'],p['no_contact'])if len(r)>=30]
    if len(runs)<2:raise ValueError('INSUFFICIENT_SEPARATE_NO_CONTACT_BLOCKS')
    # Whole-run chronological split, not random adjacent samples.
    split=max(1,int(len(runs)*2/3));train=np.concatenate(runs[:split]);held=np.concatenate(runs[split:])
    raw=p['raw'];h=p['h']
    sign=min([1,-1],key=lambda s:np.mean((s*raw[train]-h[train])**2))
    bias=np.median(sign*raw[train]-h[train],axis=0)
    residual=sign*raw[held]-h[held]-bias
    v=p['a']['dq'][train][:,p['arm']];design=[]
    for j in range(7):
        X=np.c_[np.ones(len(v)),v[:,j],np.sign(v[:,j])];norm=np.linalg.norm(X,axis=0)
        sv=np.linalg.svd(X/np.where(norm>0,norm,1),compute_uv=False)
        design.append(dict(rank=int(np.linalg.matrix_rank(X)),scaled_singular_values=sv.tolist(),
            condition=float(sv[0]/sv[-1]) if sv[-1]>0 else None))
    return dict(schema='v85-no-FT-robot-compensation-v1',sign=sign,bias_nm=bias.tolist(),
        input_fields=list(FIELDS),F_T_used=False,object_state_used=False,
        calibration_blocks=[dict(first_step=int(p['a']['step'][r[0]]),last_step=int(p['a']['step'][r[-1]]),samples=len(r))for r in runs],
        calibration_train_blocks=split,calibration_heldout_blocks=len(runs)-split,
        train_samples=len(train),heldout_samples=len(held),
        heldout_gravity_only=summary(sign*raw[held]-p['g'][held]),
        heldout_RNEA=summary(sign*raw[held]-h[held]),heldout_RNEA_plus_bias=summary(residual),
        no_contact_RMS_matrix=(residual.T@residual/len(residual)).tolist(),
        friction_calibration_design=design,robot_urdf_sha256=p['robot_urdf_sha256'],
        limitations=['Bias-only calibration; no claim that robot friction/inertia errors have been uniquely identified.',
        'Original no-contact paths are incidental, not a rich independent robot calibration experiment.',
        'Propagated no-contact RMS is a diagnostic proxy, not a calibrated confidence interval.'])


def evaluate(p,calibration,folder):
    a=p['a'];h=p['h'];arm=p['arm'];raw=p['raw'];sign=calibration['sign'];bias=np.asarray(calibration['bias_nm'])
    residual=sign*raw-h-bias;cov=np.asarray(calibration['no_contact_RMS_matrix']);n=len(a['t'])
    effort=np.full(n,np.nan);condition=effort.copy();uncertainty=effort.copy();fit_error=effort.copy()
    # Segments/phase/gaps remain separate. Filters use only complete support.
    for run in contiguous(a):
        if len(run)<247:continue
        interior=run[121:-121]
        valid=interior[p['valid'][interior]&p['dense'][interior]]
        # RNEA is undefined at unprocessed edges; filter only a reliable interior,
        # then exclude its own 60-sample support before retaining predictions.
        if len(valid)<125:continue
        if np.any(np.diff(a['step'][valid])!=1):
            raise ValueError('INVALID_EFFORT_GAP_MUST_NOT_BE_FILTERED_ACROSS')
        filtered=common_filter(residual[valid]);retained=valid[62:-62]
        for ii,i in enumerate(retained,start=62):
            q=a['q'][i,p['pmap']]
            pin.forwardKinematics(p['model'],p['data'],q);pin.updateFramePlacements(p['model'],p['data'])
            pose=p['data'].oMf[p['frame']];J=pin.computeFrameJacobian(p['model'],p['data'],q,p['frame'],pin.LOCAL_WORLD_ALIGNED)[:,p['p_arm']]
            offset=a['tcp_pose'][i,:3]-(p['base'][:3,3]+np.r_[a['q'][i,:2],0]+p['R']@pose.translation)
            Jw=np.vstack((p['R']@J[:3],p['R']@J[3:]))
            Jw[:3]+=np.cross(Jw[3:].T,offset).T
            S=np.r_[np.cross(p['axis'],a['tcp_pose'][i,:3]-p['origin']),p['axis']] if p['kind']=='revolute' else np.r_[p['axis'],np.zeros(3)]
            try:r=virtual_work(Jw,S,filtered[ii])
            except ValueError:continue
            effort[i]=r['generalized_effort'];condition[i]=r['condition'];fit_error[i]=r['residual_not_explained_by_single_wrench']
            v=r['virtual_joint_velocity'];uncertainty[i]=np.sqrt(max(0,float(v@cov@v)))
    folder.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(folder/'effort_only.npz',step=a['step'],t=a['t'],generalized_effort=effort,
        jacobian_condition=condition,no_contact_RMS_proxy=uncertainty,single_wrench_unexplained_joint_residual=fit_error)
    no=p['no_contact'];out=dict(episode_id=p['episode_id'],F_T_input_used=False,
        object_model_fitted=False,source_kind='native_solver_projected_joint_effort_proxy',
        robot_urdf_sha256=p['robot_urdf_sha256'],no_contact_all_replay=summary(residual[no])if np.any(no)else None,
        no_contact_scope='All episode no-contact; includes calibration train for R02, drawer is cross-episode holdout.',
        reconstructed_samples=int(np.isfinite(effort).sum()),
        jacobian_condition_quantiles=np.nanquantile(condition,[0,.5,.95,1]).tolist(),
        propagated_no_contact_RMS_proxy_quantiles=np.nanquantile(uncertainty,[0,.5,.95,1]).tolist(),
        explicit_arm_command_max_abs_Nm=float(np.max(abs(a['explicit_applied_joint_efforts_native'][:,arm]))),
        raw_step_sha256=hashlib.sha256(a['step'].tobytes()).hexdigest(),
        freeze_note='Compensation fixed before F/T-reference comparison; no F/T channel loaded. Native q not admitted as visual object motion.')
    (folder/'SUMMARY.json').write_text(json.dumps(out,indent=2,allow_nan=False))
    return out


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--r02',type=Path,required=True);parser.add_argument('--drawer',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    p=prepare(args.r02);cal=calibrate(p)
    path=args.output/'ROBOT_COMPENSATION_FROZEN.json';path.write_text(json.dumps(cal,indent=2,allow_nan=False))
    print('FROZEN',hashlib.sha256(path.read_bytes()).hexdigest(),flush=True)
    for name,source,data in [('r02',args.r02,p),('drawer',args.drawer,None)]:
        d=data or prepare(source);print(name,json.dumps(evaluate(d,cal,args.output/name)),flush=True)


if __name__=='__main__':main()
