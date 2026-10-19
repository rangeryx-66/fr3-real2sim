"""EE-only response evidence; encoder q is a consistency diagnostic, not a vote."""
import numpy as np
from scipy.spatial.transform import Rotation
from interactive_twin.sysid import _aligned, _interp


def active_mask(log, t=None):
    t=np.asarray(log['time_s']) if t is None else np.asarray(t)
    c=log['commands'];f=c['fields']
    if 'drive_active' not in f:return np.ones(len(t),bool)
    # Applied commands are piecewise constant, never interpolate an active flag.
    ix=np.clip(np.searchsorted(c['time_s'],t,side='right')-1,0,len(c['time_s'])-1)
    return np.asarray(c['values'])[ix,f.index('drive_active')]>.5


def windows(mask):
    edge=np.diff(np.r_[False,mask,False].astype(int))
    return list(zip(np.flatnonzero(edge==1),np.flatnonzero(edge==-1)))


def onset(t,p,active,threshold,hold_s):
    starts=windows(active)
    if not starts:return None
    lo,hi=starts[0];before=np.flatnonzero((t>=t[lo]-.05)&(t<t[lo]))
    origin=p[before].mean(0) if len(before) else p[lo]
    distance=np.linalg.norm(p-origin,axis=1)
    for i in range(lo,hi):
        j=np.searchsorted(t,t[i]+hold_s)
        if j<hi and np.all(distance[i:j+1]>=threshold):return float(t[i]-t[lo])
    return None


def quadratic(x,cov):
    x=np.asarray(x);cov=np.asarray(cov)
    if not len(x):return None
    inverse=np.linalg.inv(cov)
    return float(np.mean(np.einsum('ni,ij,nj->n',x,inverse,x))/x.shape[1])


def score(reference,predicted,calibration,*,evaluation=False):
    if not evaluation and ('heldout' in (reference['split'],predicted['split'])):
        raise ValueError('HELDOUT_ACCESS_DENIED')
    t,r,p=_aligned(reference,predicted)
    R=r['ee_T_world_tcp'][:,:3,:3];rp=r['ee_T_world_tcp'][:,:3,3];pp=p['ee_T_world_tcp'][:,:3,3]
    rot=Rotation.from_matrix(p['ee_T_world_tcp'][:,:3,:3]@R.transpose(0,2,1)).as_rotvec()
    velocity=np.gradient(pp,t,axis=0)-np.gradient(rp,t,axis=0)
    local=lambda x:np.einsum('nji,nj->ni',R,x)
    residual=np.c_[local(pp-rp),local(rot),local(velocity)]
    active=active_mask(reference,t);cov=np.asarray(calibration['response_covariance'])
    terms={'active_ee_pose_velocity':quadratic(residual[active],cov)}
    threshold=calibration['onset_threshold_m'];hold=calibration['onset_hold_s']
    rs=onset(t,rp,active,threshold,hold);ps=onset(t,pp,active,threshold,hold)
    first=windows(active);duration=(t[first[0][1]-1]-t[first[0][0]]) if first else float(t[-1]-t[0])
    delay_error=abs((duration if rs is None else rs)-(duration if ps is None else ps))
    terms['start_delay']=(delay_error/calibration['start_time_scale_s'])**2 if first else None
    dwell=[];dwell_position=[];last_drift=None
    for lo,hi in windows(~active):
        if lo==0:continue  # pre-drive settling is not a stop-response observation
        anchor=lo-1
        dr=(pp[lo:hi]-pp[anchor])-(rp[lo:hi]-rp[anchor])
        # Incremental orientation at stop, avoiding duplicate absolute offset.
        rpR=R[lo:hi]@R[anchor].T
        ppR=p['ee_T_world_tcp'][lo:hi,:3,:3]@p['ee_T_world_tcp'][anchor,:3,:3].T
        dR=Rotation.from_matrix(ppR@rpR.transpose(0,2,1)).as_rotvec()
        mat=R[anchor].T
        dwell.append(np.c_[dr@mat.T,dR@mat.T]);dwell_position.append(dr)
        last_drift={'reference_net_m':float(np.linalg.norm(rp[hi-1]-rp[anchor])),
                    'predicted_net_m':float(np.linalg.norm(pp[hi-1]-pp[anchor])),
                    'vector_error_m':float(np.linalg.norm(dr[-1])),
                    'duration_s':float(t[hi-1]-t[anchor])}
    # Difference of two noisy poses: twice the independently calibrated covariance.
    terms['stop_incremental_pose']=quadratic(np.concatenate(dwell),2*cov[:6,:6]) if dwell else None
    dimensions={'active_ee_pose_velocity':9,'start_delay':1,'stop_incremental_pose':6}
    valid=[k for k,v in terms.items() if v is not None]
    if not valid:raise ValueError('NO_OBSERVABLE_RESPONSE_BLOCK')
    loss=sum(dimensions[k]*terms[k] for k in valid)/sum(dimensions[k] for k in valid)
    m={'ee_position_rmse_m':float(np.sqrt(np.mean((pp-rp)**2))),
       'ee_rotation_rmse_rad':float(np.sqrt(np.mean(np.sum(rot**2,axis=1)))),
       'ee_velocity_rmse_m_s':float(np.sqrt(np.mean(velocity**2))),
       'start_time_error_s':float(delay_error),
       'dwell_incremental_position_rmse_m':float(np.sqrt(np.mean(np.concatenate(dwell_position)**2))) if dwell else None,
       'final_displacement_error_m':float(np.linalg.norm((pp[-1]-pp[0])-(rp[-1]-rp[0])))}
    return {'loss':float(loss),'whitened_rms':float(np.sqrt(loss)),
            'blocks':terms,'block_dimensions':dimensions,'metrics':m,'start_delay_reference_s':rs,'start_delay_prediction_s':ps,
            'onset_censored':rs is None or ps is None,'post_drive_drift':last_drift,
            'q_diagnostic_rmse_rad':float(np.sqrt(np.mean((p['q_rad']-r['q_rad'])**2))),
            'qdot_diagnostic_rmse_rad_s':float(np.sqrt(np.mean((p['qdot_rad_s']-r['qdot_rad_s'])**2))),
            'q_used_in_physics_loss':False,'time_warping':False,
            'noise_calibration_sha256':calibration['calibration_sha256'],
            'sample_count':len(t),'evidence_units':'complete action blocks; frames are not independent likelihood trials'}
