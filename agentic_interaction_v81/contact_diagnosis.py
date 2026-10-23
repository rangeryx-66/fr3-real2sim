"""Archived measured contact mechanics; no plant configuration or simulator.

The finger/object material-point velocities use measured rigid-body poses,
including angular motion. Normal-report disappearance is never separation.
Centroids describe reported contact migration, not pressure distributions.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def transform(pose):
    p=np.asarray(pose);T=np.repeat(np.eye(4)[None],len(p),axis=0)
    T[:,:3,3]=p[:,:3];T[:,:3,:3]=Rotation.from_quat(p[:,3:]).as_matrix()
    return T


def point_velocity(previous,current,point,dt):
    """Rigid material velocity at a world point; no articulation-axis input."""
    w=Rotation.from_matrix(current[:3,:3]@previous[:3,:3].T).as_rotvec()/dt
    return (current[:3,3]-previous[:3,3])/dt+np.cross(w,point-current[:3,3])


def stats(x):
    x=np.asarray(x);x=x[np.isfinite(x)]
    return None if not len(x) else dict(n=len(x),median=float(np.median(x)),
        p90=float(np.percentile(x,90)),maximum=float(np.max(x)))


def analyze(folder):
    from diagnose_probing_v3 import columns
    folder=Path(folder);a,index=columns(folder)
    bodies=[b['name'] for b in index['metadata']['body_properties']]
    body=transform(a['moving_link_pose'])
    fingers=[transform(a['tool_poses'][:,bodies.index(n)]) for n in ('left_finger','right_finger')]
    ci=folder/'contact_evidence/index.json'
    points={};anchors={}
    if ci.exists():
        contact_index=json.loads(ci.read_text())
        for f in contact_index['files']:
            with np.load(ci.parent/f['file'],allow_pickle=False) as z:
                anchor=z['kind']==1
                for k in ('step','finger','point_pad'):
                    anchors.setdefault(k,[]).append(z[k][anchor])
                keep=z['kind']==0
                for k in ('step','finger','point_world','point_pad','normal_world',
                          'longitudinal_margin_m','lateral_margin_m'):
                    points.setdefault(k,[]).append(z[k][keep])
        points={k:np.concatenate(v) for k,v in points.items()}
        anchors={k:np.concatenate(v) for k,v in anchors.items()}
    else:contact_index={}
    lookup={int(s):i for i,s in enumerate(a['step'])}
    relative=np.full(len(points.get('step',[])),np.nan)
    moving_local=np.full((len(relative),3),np.nan)
    for j,step in enumerate(points.get('step',[])):
        i=lookup.get(int(step))
        # Reports refer to the pre-integration geometry. Reconstruct those
        # measured frames only where two earlier native samples actually exist.
        if i is None or i<2 or not np.all(np.diff(a['step'][i-2:i+1])==1):continue
        dt=float(a['t'][i-1]-a['t'][i-2])
        if abs(dt-1/240)>1e-7:continue
        point=points['point_world'][j];normal=points['normal_world'][j]
        if not np.isfinite(normal).all() or np.linalg.norm(normal)<1e-9:continue
        f=int(points['finger'][j]);normal=normal/np.linalg.norm(normal)
        velocity=(point_velocity(fingers[f][i-2],fingers[f][i-1],point,dt)
                  -point_velocity(body[i-2],body[i-1],point,dt))
        relative[j]=np.linalg.norm(velocity-normal*(velocity@normal))
        moving_local[j]=body[i-1,:3,:3].T@(point-body[i-1,:3,3])
    result=[]
    for segment in index['segments']:
        ix=np.flatnonzero(a['segment']==segment['id'])
        if not len(ix):continue
        t=a['t'][ix];first,last=ix[0],ix[-1]
        pd=[]
        for side in (0,1):
            mask=(points.get('step',np.array([]))>=a['step'][first])&(points.get('step',np.array([]))<=a['step'][last])&(points.get('finger',np.array([]))==side)
            p=np.flatnonzero(mask)
            anchor=(anchors.get('step',np.array([]))>=a['step'][first])&(anchors.get('step',np.array([]))<=a['step'][last])&(anchors.get('finger',np.array([]))==side)
            anchor_points=anchors['point_pad'][anchor] if len(anchors) else np.empty((0,3))
            pd.append(dict(finger=side,normal_point_records=len(p),
                friction_anchor_records=len(anchor_points),
                friction_anchor_pad_bounds_m=np.percentile(anchor_points,[5,95],axis=0).tolist() if len(anchor_points) else None,
                lateral_margin_m=stats(points['lateral_margin_m'][p]) if len(p) else None,
                longitudinal_margin_m=stats(points['longitudinal_margin_m'][p]) if len(p) else None,
                derived_material_tangent_speed_m_s=stats(relative[p]),
                contact_point_pad_bounds_m=np.percentile(points['point_pad'][p],[5,95],axis=0).tolist() if len(p) else None,
                contact_point_object_bounds_m=np.nanpercentile(moving_local[p],[5,95],axis=0).tolist() if len(p) and np.isfinite(moving_local[p]).any() else None))
        motion=dict(start_native=float(a['joint_q'][first]),end_native=float(a['joint_q'][last]))
        # Kinematic relative transform, independently of any hinge/rail model.
        tcp=transform(a['tcp_pose'][ix]);rel=np.einsum('nji,njk->nik',body[ix,:3,:3],tcp[:,:3,:3])
        position=np.einsum('nji,nj->ni',body[ix,:3,:3],tcp[:,:3,3]-body[ix,:3,3])
        drift=np.linalg.norm(position-position[0],axis=1)
        rotation=Rotation.from_matrix(rel@rel[0].T).magnitude()
        normal=a['normal_force_on_fingers_world_N'][ix]
        tangential=a['friction_force_on_fingers_world_N'][ix]
        reported_ratio=np.linalg.norm(tangential,axis=2)/np.maximum(np.linalg.norm(normal,axis=2),1e-9)
        def window(indices):
            return dict(pad_mean_n=a['pad_load'][indices].mean(0).tolist(),
                normal_vector_mean_on_finger_N=a['normal_force_on_fingers_world_N'][indices].mean(0).tolist(),
                friction_vector_mean_on_finger_N=a['friction_force_on_fingers_world_N'][indices].mean(0).tolist(),
                master_aperture_mean_m=float(a['aperture'][indices].mean()),
                native_coupled_joint_range_rad=np.ptp(a['q'][indices,-6:],axis=0).tolist())
        result.append(dict(id=segment['id'],role=segment['role'],status=segment.get('result',{}).get('status'),
            reason=segment.get('result',{}).get('reason'),**motion,duration_s=float(t[-1]-t[0]),
            master_aperture_range_m=[float(a['aperture'][ix].min()),float(a['aperture'][ix].max())],
            master_aperture_drift_m=float(a['aperture'][last]-a['aperture'][first]),
            relative_translation_drift_m=stats(drift),relative_rotation_drift_rad=stats(rotation),
            start_window=window(ix[t<=t[0]+.5]),end_window=window(ix[t>=t[-1]-.5]),
            reported_tangent_to_normal_ratio=[stats(reported_ratio[:,j][np.linalg.norm(normal[:,j],axis=1)>.05])for j in (0,1)],
            points=pd))
    return dict(episode=folder.name,segments=result,
        coordinate_unit='m' if index['metadata']['articulation']['joint_type']=='prismatic' else 'rad',
        native_rate_Hz=240,hidden_truth_used=False,normal_absence_is_not_separation=True,
        contact_centroids_are_not_material_point_tracks=True,
        tangent_velocity_is_pose_derived_not_measured_friction=True,
        reported_force_ratio_is_not_a_material_coefficient=True,
        preintegration_alignment='two previous contiguous native body poses; gaps excluded',
        revolute_velocity_uses_full_measured_rigid_motion=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path);parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();a.output.write_text(json.dumps(analyze(a.episode),indent=2)+'\n')
