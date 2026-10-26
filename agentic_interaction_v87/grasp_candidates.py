"""Observation-frame GraspGenX proposals in the native G1 TCP convention.

Network confidence never replaces the existing observed-contact score or any
native safety predicate. Family fields describe the observed pose; the SE(3)
fingerprint is its independent search identity.
"""
import hashlib,inspect,json,types
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def pose_id(T):
    T=np.asarray(T,dtype='<f8')
    return hashlib.sha256(np.round(T,7).tobytes()).hexdigest()


def transforms(payload,camera):
    if payload.get('gripper')!='XARM7_NATIVE_G1' or payload.get('frame')!='camera_optical':
        raise ValueError('NATIVE_G1_CAMERA_OPTICAL_CANDIDATES_REQUIRED')
    W=np.asarray(camera['T_world_camera_optical'],float)
    if not np.allclose(payload['T_B_C'],W,atol=1e-10,rtol=0):
        raise ValueError('INFERENCE_OBSERVATION_FRAME_MISMATCH')
    found=set();result=[]
    for item in payload['grasps']:
        C=np.asarray(item['T_C_G'],float);G=np.asarray(item['T_G_TCP'],float)
        if C.shape!=(4,4) or G.shape!=(4,4) or not np.isfinite(np.r_[C.ravel(),G.ravel()]).all():
            raise ValueError('INVALID_GRASP_TRANSFORM')
        T=W@C@G;R=T[:3,:3]
        if not (np.allclose(T[3],[0,0,0,1],atol=1e-6) and
                np.allclose(R.T@R,np.eye(3),atol=1e-4) and abs(np.linalg.det(R)-1)<1e-4):
            raise ValueError('NON_RIGID_GRASP_TRANSFORM')
        key=pose_id(T)
        if key in found:continue
        found.add(key);result.append(dict(T=T,pose_id=key,provider='GraspGenX',
            generator_score=float(item['score']),generator_rank=int(item['rank'])))
    return result


def motion_tangent(joint_type,joint_axis,point,origin):
    axis=np.asarray(joint_axis,float)
    if joint_type=='prismatic':return axis
    if joint_type=='revolute':return np.cross(axis,np.asarray(point)-origin)
    raise ValueError('EXPLICIT_SINGLE_JOINT_TYPE_REQUIRED')


def geometry_candidates(payload,camera,data,model,surfaces,scene,base,origin,joint_axis,joint_type):
    import plan_shared_door_skill as shared
    import xarm7_native_planning as native
    from native_g1_multiview_input import fast_ray
    closing_ns=dict(shared.__dict__)
    closing_source=inspect.getsource(shared.closing_quality).replace(
        'np.linspace(.1,0.,41)','np.linspace(.084,.001641,41)')
    exec(closing_source,closing_ns)
    common_ns=dict(native.__dict__,ray_surface=fast_ray)
    exec(inspect.getsource(native.common_quality).replace(
        ' from handle_contact_quality import ray_surface\n',''),common_ns)
    center=np.asarray(data['legacy_estimate']['anchor_world_m'])
    axis=np.asarray(data['axis_world']);normal=np.asarray(data['normal_world'])
    frame=np.column_stack((axis,np.cross(-normal,axis),-normal))
    length=float(data['dimensions_m'][0]);joint_axis=np.asarray(joint_axis,float)
    output=[]
    for item in transforms(payload,camera):
        T=item['T'];quality=closing_ns['closing_quality'](model,scene,T,base,.084)
        tangent=motion_tangent(joint_type,joint_axis,T[:3,3],origin)
        if np.linalg.norm(tangent)<1e-10:continue
        quality['patch']=common_ns['common_quality'](model,surfaces,T,base,tangent)
        quality['physical_aperture_m']=float(np.clip(quality['patch']['native']['best_common_pose_aperture']['common_aperture_m'],.001641,.084))
        quality['support_score']=(quality['patch']['score']/max(1,1+quality['root_contact_count'])
            if quality['non_handle_closing_collision'] is None else 0.)
        quality['rank']+=quality['support_score']
        angles=Rotation.from_matrix(frame.T@T[:3,:3]).as_euler('zyx',degrees=True)
        item.update(section=float((T[:3,3]-center)@axis/max(length,1e-9)),
            insertion_m=float((T[:3,3]-center)@normal),roll=float(angles[0]),
            pitch=float(angles[2]),contact=quality)
        output.append(item)
    return output


def novel_shortlist(geometries,limit):
    """Finite pose diversity for continuous proposals, same contact-score order."""
    ranked=sorted(geometries,key=lambda g:g['contact']['rank'],reverse=True)
    selected=[]
    # Physical native pad dimensions give a reproducible redundancy scale.
    # This selects search work; it never authorizes a collision or grasp.
    for item in ranked:
        T=item['T']
        if any(np.linalg.norm(T[:3,3]-old['T'][:3,3])<.003 and
               np.linalg.norm(Rotation.from_matrix(T[:3,:3]@old['T'][:3,:3].T).as_rotvec())<np.deg2rad(5)
               for old in selected):continue
        selected.append(item)
        if len(selected)>=limit:break
    return selected
