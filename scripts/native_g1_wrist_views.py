"""Bounded wrist view IK/routes. No camera teleport or object-state command.

The coarse cabinet envelope seeds a generic survey, never a handle grasp.
Subsequent look targets come only from observed metric points/panel normal.
All routes reuse the original .05 margin and collision checker.
"""
import json, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def calibration(path=None):
    c=json.loads(Path(path or ROOT/'configs/xarm7_wrist_rgbd_sim.json').read_text())
    X=np.eye(4);X[:3,:3]=Rotation.from_euler('xyz',c['T_tcp_camera_rotation_xyz_deg'],degrees=True).as_matrix()
    X[:3,3]=c['T_tcp_camera_translation_m'];return c,X


def look_at(eye,target):
    z=np.asarray(target)-eye;z/=np.linalg.norm(z)
    x=np.cross(z,[0.,0.,1.]);x/=np.linalg.norm(x)
    C=np.eye(4);C[:3,:3]=np.column_stack([x,np.cross(z,x),z]);C[:3,3]=eye
    return C


def survey_candidates(model,c):
    # Coarse cabinet envelope only; no interaction_geometry/handle selection.
    bounds=np.asarray(model.manifest['static_source_bounds'])*model.manifest['scale_source_to_meters']
    lo,hi=bounds;corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
    points=corners@model.asset_T[:3,:3].T+model.asset_T[:3,3]
    low,high=points.min(0),points.max(0)
    normal=model.asset_T[:3,2].copy();normal[2]=0.;normal/=np.linalg.norm(normal)
    center=(low+high)/2
    # Front face of the broad cabinet box, NOT the handle surface.
    center+=normal*(np.max(points@normal)-center@normal)
    for f in c['survey_height_fractions']:
        target=center.copy();target[2]=low[2]+f*(high[2]-low[2])
        for standoff in c['survey_standoffs_m']:
            eye=target+standoff*normal
            yield dict(kind='coarse_cabinet_survey',height_fraction=f,standoff_m=standoff,T_camera=look_at(eye,target))


def observed_candidates(points,normal,c):
    target=np.median(points,axis=0);n=np.asarray(normal).copy();n/=np.linalg.norm(n)
    side=np.cross([0.,0.,1.],n);side/=np.linalg.norm(side)
    for a in c['view_azimuths_deg']:
        for distance in c['view_standoffs_m']:
            r=np.deg2rad(a);eye=target+distance*(np.cos(r)*n+np.sin(r)*side)+[0,0,.06]
            yield dict(kind='current_wrist_observation',azimuth_deg=a,standoff_m=distance,T_camera=look_at(eye,target))


def in_frame(points,C,c):
    Q=(points-C[:3,3])@C[:3,:3];uv=Q@np.asarray(c['K']).T
    uv=uv[:,:2]/np.maximum(Q[:,2,None],1e-9);w,h=c['resolution_wh']
    return (Q[:,2]>c['clipping_range_m'][0])&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)


def path_to(model,solver,scene,base,start,C,c,X):
    target=C@np.linalg.inv(X)
    q=solver.solve(target,base,start,starts=c['ik_starts'])
    if q is None:return None,'NO_IK'
    def check(x):
        if model.margin(x)<=.05:return False,'LOW_JOINT_MARGIN'
        return scene.check(model.poses(x,base,width=.084),scene.moving_reference,False)
    ok,reason=check(q)
    if not ok:return None,reason
    def edge(a,b):
        return all(check(x)[0] for x in np.linspace(a,b,max(2,int(np.ceil(np.max(abs(a-b))/.025))+1)))
    start=np.asarray(start);q=np.asarray(q)
    if edge(start,q):return [start.tolist(),q.tolist()],'DIRECT'
    nodes=[start];parents=[-1];rng=np.random.default_rng(610)
    for _ in range(c['route_iterations']):
        sample=q if rng.random()<.2 else rng.uniform(model.limits[:,0]+.050001,model.limits[:,1]-.050001)
        ix=int(np.argmin([np.linalg.norm(x-sample) for x in nodes]));delta=sample-nodes[ix]
        end=nodes[ix]+delta*min(1.,.18/max(np.linalg.norm(delta),1e-12))
        if not edge(nodes[ix],end):continue
        nodes.append(end);parents.append(ix)
        if np.linalg.norm(end-q)<.35 and edge(end,q):
            path=[q,end];ix=len(nodes)-1
            while parents[ix]>=0:ix=parents[ix];path.append(nodes[ix])
            return [x.tolist() for x in reversed(path)],'RRT'
    return None,'NO_ARM_PATH'


def select_route(model,solver,scene,base,start,proposals,c,X,observed=None):
    logs=[];began=time.perf_counter()
    for p in proposals:
        record={k:v for k,v in p.items() if k!='T_camera'}
        record['requested_camera']=p['T_camera'].tolist()
        if observed is not None:record['observed_points_in_frame_fraction']=float(in_frame(observed,p['T_camera'],c).mean())
        path,reason=path_to(model,solver,scene,base,start,p['T_camera'],c,X)
        record.update(reason=reason,feasible=path is not None);logs.append(record)
        if path is not None:
            record.update(q_path=path,minimum_joint_margin_rad=min(model.margin(np.asarray(x)) for x in path),planning_wall_s=time.perf_counter()-began)
            return record,logs
    return None,logs
