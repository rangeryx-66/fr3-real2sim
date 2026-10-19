"""Frozen-axis, start-anchored geometric prediction; no held-out reference fit."""
import numpy as np
from scipy.spatial.transform import Rotation


def prediction(T, fit):
    T=np.asarray(T,float);a=np.asarray(fit['revolute']['axis']);a=a/np.linalg.norm(a)
    c=np.asarray(fit['revolute']['point_on_axis']);R0=T[0,:3,:3];p0=T[0,:3,3]
    # Encoder orientation provides phase, not a GT articulation coordinate.
    theta=Rotation.from_matrix(T[:,:3,:3]@R0.T).as_rotvec()@a
    R=Rotation.from_rotvec(theta[:,None]*a).as_matrix()
    p=c+np.einsum('nij,j->ni',R,p0-c);rot=R@R0
    ep=np.linalg.norm(p-T[:,:3,3],axis=1)
    er=Rotation.from_matrix(rot@T[:,:3,:3].transpose(0,2,1)).magnitude()
    dp=np.diff(p,axis=0);dy=np.diff(T[:,:3,3],axis=0)
    usable=(np.linalg.norm(dp,axis=1)>1e-6)&(np.linalg.norm(dy,axis=1)>1e-6)
    raw_angles=np.arccos(np.clip(np.sum(dp[usable]*dy[usable],axis=1)/(np.linalg.norm(dp[usable],axis=1)*np.linalg.norm(dy[usable],axis=1)),-1,1))
    # A per-frame displacement can be smaller than encoder/pose noise. Compare
    # chord tangents only over the unchanged 1 mm observable motion scale.
    angles=[];anchor=0
    for i in range(1,len(T)):
        observed=T[i,:3,3]-T[anchor,:3,3]
        if np.linalg.norm(observed)<.001:continue
        predicted=p[i]-p[anchor]
        angles.append(np.arccos(np.clip(predicted@observed/(np.linalg.norm(predicted)*np.linalg.norm(observed)),-1,1)));anchor=i
    return {'position_rmse_m':float(np.sqrt(np.mean(ep**2))), 'rotation_rmse_rad':float(np.sqrt(np.mean(er**2))),
            'endpoint_error_m':float(ep[-1]),'tangent_error_deg':float(np.rad2deg(np.mean(angles))) if len(angles) else None,
            'raw_frame_tangent_error_deg':float(np.rad2deg(np.mean(raw_angles))) if len(raw_angles) else None,
            'tangent_motion_scale_m':.001,
            'maximum_position_error_m':float(ep.max()),'observed_travel_m':float(np.linalg.norm(T[-1,:3,3]-p0)),
            'predicted_positions':p.tolist(),'position_errors_m':ep.tolist(),'rotation_errors_rad':er.tolist(),
            'reference_fit_on_heldout':False,'phase_source':'measured EE orientation; conditional geometric prediction, not dynamic prediction'}
