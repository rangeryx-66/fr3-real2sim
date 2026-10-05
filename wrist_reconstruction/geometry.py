"""Camera calibration and robot-realizable view proposals; no object joints."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def calibration(path):
    d=json.loads(Path(path).read_text());K=np.asarray(d['K'],float);X=np.asarray(d['T_tcp_camera_optical'],float)
    if K.shape!=(3,3) or X.shape!=(4,4) or not np.all(np.isfinite(K)) or not np.all(np.isfinite(X)):raise ValueError('INVALID_CAMERA_CALIBRATION')
    if not np.allclose(X[3],[0,0,0,1]) or not np.allclose(X[:3,:3].T@X[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(X[:3,:3])<.999:raise ValueError('INVALID_CAMERA_EXTRINSIC')
    if min(K[0,0],K[1,1])<=0:raise ValueError('INVALID_CAMERA_FOCAL_LENGTH')
    d['K']=K;d['X']=X;return d


def look_at(eye,center):
    forward=np.asarray(center)-eye;forward/=np.linalg.norm(forward)
    right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right)
    T=np.eye(4);T[:3,:3]=np.column_stack((right,np.cross(forward,right),forward));T[:3,3]=eye;return T


def proposed_views(center,normal,extent,policy):
    center=np.asarray(center,float);normal=np.asarray(normal,float);normal[2]=0;normal/=np.linalg.norm(normal)
    radius=max(policy['minimum_standoff_m'],float(np.linalg.norm(extent))*policy['extent_standoff_factor'])
    result=[]
    for index,(azimuth,elevation,roll) in enumerate(policy['view_angles_deg']):
        d=Rotation.from_euler('z',azimuth,degrees=True).apply(normal)
        eye=center+radius*(np.cos(np.deg2rad(elevation))*d+np.array([0,0,np.sin(np.deg2rad(elevation))]))
        T=look_at(eye,center);T[:3,:3]=T[:3,:3]@Rotation.from_euler('z',roll,degrees=True).as_matrix()
        result.append({'view_id':index,'T_camera':T,'T_tcp':None,'role':'frontal/oblique/side/top/interior proposal; visibility verified from pixels'})
    return result


def optical_to_tcp(T_camera,X):return np.asarray(T_camera)@np.linalg.inv(X)


def visibility(P,T,K,resolution):
    Q=(P-T[:3,3])@T[:3,:3];valid=Q[:,2]>.03
    uv=Q[:,:2]/np.maximum(Q[:,2:],1e-8)*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]
    w,h=resolution;valid &= (uv[:,0]>8)&(uv[:,0]<w-8)&(uv[:,1]>8)&(uv[:,1]<h-8)
    return float(valid.mean())


def observed_volume(P):
    """Camera framing bounds, not a collision or reconstructed object model.

    A visible-surface median depends on sampling/viewpoint and is not the
    object's framing center. Sparse edge samples still matter for cropping.
    """
    P=np.asarray(P,float);P=P[np.all(np.isfinite(P),axis=1)]
    if len(P)<8:raise ValueError('INSUFFICIENT_SENSOR_VOLUME_POINTS')
    lo,hi=np.quantile(P,[.001,.999],axis=0)
    corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
    return (lo+hi)/2,hi-lo,corners


def coverage_views(P,center,normal,cal,policy):
    """Finite observed-cloud/FOV-driven pool; no joint/asset identifiers."""
    center,_,corners=observed_volume(P)
    normal=np.asarray(normal,float).copy();normal[2]=0;normal/=np.linalg.norm(normal)
    result=[]
    for elevation in policy['pool_elevations_deg']:
        for azimuth in policy['pool_azimuths_deg']:
            horizontal=Rotation.from_euler('z',azimuth,degrees=True).apply(normal)
            ray=np.cos(np.deg2rad(elevation))*horizontal+np.array([0,0,np.sin(np.deg2rad(elevation))])
            # Fit the observed volume, rather than permitting 3% of a dense
            # front surface to hide the sparse edges that image QA rejects.
            lo=policy['minimum_standoff_m'];hi=policy['maximum_standoff_m']
            for _ in range(12):
                middle=(lo+hi)/2;T=look_at(center+middle*ray,center)
                if visibility(corners,T,cal['K'],cal['resolution_wh'])==1.:hi=middle
                else:lo=middle
            for factor in policy['pool_distance_factors']:
                T=look_at(center+hi*factor*ray,center)
                result.append({'view_id':len(result),'T_camera':T,'azimuth_deg':azimuth,'elevation_deg':elevation,'lens_distance_m':hi*factor,'observed_volume_in_frame':visibility(corners,T,cal['K'],cal['resolution_wh']),'role':'sensor/FOV driven front/oblique/side/top/interior candidate'})
    # Frontal and modest side baselines first; coverage continues to side/top.
    result.sort(key=lambda v:(abs(v['azimuth_deg'])+abs(v['elevation_deg']-20),v['lens_distance_m']))
    return result


def distinct_view(T,selected,minimum_baseline_m,minimum_angle_deg):
    if not selected:return True
    for previous in selected:
        distance=np.linalg.norm(np.asarray(T)[:3,3]-np.asarray(previous)[:3,3])
        angle=np.rad2deg(Rotation.from_matrix(np.asarray(T)[:3,:3]@np.asarray(previous)[:3,:3].T).magnitude())
        if distance<minimum_baseline_m or angle<minimum_angle_deg:return False
    return True
