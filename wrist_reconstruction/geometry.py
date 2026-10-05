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
