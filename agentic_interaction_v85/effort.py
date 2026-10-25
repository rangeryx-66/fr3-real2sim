"""Effort-only primitives. No F/T import, feature, compensation or selection.

Native solver projected effort is a simulation proxy, not hardware current
calibration. Object generalized force uses virtual work, avoiding a claim that
all six wrench components are reliably identified.
"""
import hashlib,json
from pathlib import Path
import numpy as np

FIELDS=('step','t','phase','segment','q','dq','tcp_pose','positive_contacts',
        'projected_joint_efforts_native','projected_joint_efforts_native_valid',
        'explicit_applied_joint_efforts_native','explicit_applied_joint_efforts_native_valid','safety_stop')


def load_allowed(episode):
    index=json.loads((episode/'telemetry/index.json').read_text());parts={k:[] for k in FIELDS}
    for chunk in index['chunks']:
        path=episode/'telemetry'/chunk['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=chunk['sha256']:
            raise ValueError('TELEMETRY_HASH_MISMATCH')
        with np.load(path,allow_pickle=False) as f:
            for k in FIELDS:parts[k].append(f[k])
    return {k:np.concatenate(v)for k,v in parts.items()},index


def contiguous(a,mask=None):
    ids=np.flatnonzero(np.ones(len(a['t']),bool) if mask is None else mask)
    if not len(ids):return []
    cuts=np.flatnonzero((np.diff(a['step'][ids])!=1)
        |(abs(np.diff(a['t'][ids])-1/240)>1e-7)
        |(np.diff(a['phase'][ids])!=0)|(np.diff(a['segment'][ids])!=0))+1
    return [r for r in np.split(ids,cuts)if len(r)]


def virtual_work(jacobian,twist,residual,rotation_length_m=.3):
    J=np.asarray(jacobian,dtype=float);S=np.asarray(twist,dtype=float)
    if J.shape!=(6,7) or S.shape!=(6,) or not np.isfinite(J).all():
        raise ValueError('FINITE_6_BY_7_JACOBIAN_REQUIRED')
    # Dimensionally scaled rows for the SVD; no data-selected cutoff.
    scale=np.r_[np.full(3,1/rotation_length_m),np.ones(3)]
    A=scale[:,None]*J;b=scale*S
    velocity,_,rank,sv=np.linalg.lstsq(A,b,rcond=1e-6)
    if rank<6:raise ValueError('GENERALIZED_FORCE_NOT_CERTIFIED_AT_RANK_DEFICIENT_POSE')
    return dict(generalized_effort=float(velocity@residual),virtual_joint_velocity=velocity,
        condition=float(sv[0]/sv[-1]),compatibility_error=float(np.linalg.norm(A@velocity-b)),
        residual_not_explained_by_single_wrench=float(np.linalg.norm(residual-J.T@np.linalg.lstsq(J.T,residual,rcond=1e-6)[0])))


def kinematic_scale_ambiguity(parameters,scale,acceleration,velocity,effort=None):
    if effort is not None:raise ValueError('KINEMATICS_ONLY_MUST_NOT_RECEIVE_EFFORT')
    theta=np.asarray(parameters,float)
    if scale<=0 or theta.shape!=(3,):raise ValueError('POSITIVE_SCALE_AND_THREE_PARAMETERS_REQUIRED')
    # With unobserved input u, both parameter vectors reproduce identical q(t)
    # by scaling the unobserved input. Prescribed-motion data do not fix scale.
    X=np.c_[acceleration,velocity,np.sign(velocity)]
    return X@theta,X@(theta*scale)
