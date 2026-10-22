"""FIT-only excitation diagnostics and ideal-profile previews.

These rank information opportunities; they neither certify physical contact nor
estimate hidden dynamics. The production V3.1 operator is reused unchanged.
"""
import math
import numpy as np
from scipy.ndimage import minimum_filter1d
from interaction_runtime.probing_profiles_v3 import speed_fraction
from scripts.xarm_ft_v31_estimator import common_filter


def information(qdd,velocity,direction,scales):
    X=np.column_stack([qdd,velocity,direction]);scales=np.asarray(scales,float)
    if X.shape[0]<8 or scales.shape!=(3,) or not np.isfinite(X).all() or not np.isfinite(scales).all() or min(scales)<=0:
        raise ValueError('FINITE_ELIGIBLE_REGRESSORS_AND_DECLARED_SCALES_REQUIRED')
    singular=np.linalg.svd(X*scales,compute_uv=False)
    other=X[:,1:];orthogonal=X[:,0]-other@np.linalg.lstsq(other,X[:,0],rcond=None)[0]
    return dict(samples=len(X),algebraic_rank=int(np.linalg.matrix_rank(X)),
        scaled_singular_values=singular.tolist(),
        scaled_condition=float(singular[0]/singular[-1]) if singular[-1]>1e-15 else None,
        acceleration_rms_native=float(np.sqrt(np.mean(X[:,0]**2))),
        independent_acceleration_rms_native=float(np.sqrt(np.mean(orthogonal**2))),
        velocity_rms_native=float(np.sqrt(np.mean(X[:,1]**2))),
        velocity_p05_p95_native=np.quantile(X[:,1],[.05,.95]).tolist(),
        direction_counts=dict(positive=int(np.count_nonzero(X[:,2]>0)),negative=int(np.count_nonzero(X[:,2]<0))),
        estimated_parameters=None,hidden_dynamics_used=False,
        interpretation='Regressor opportunity only. Compare measured acceleration sensitivity with phase-matched compensated F/T residual; rank alone cannot establish physical parameter accuracy.')


def preview(action,start,target,*,joint_type,radius,motion,scales,dt=1/240,max_s=600):
    """Nominal tracking preview of an Agent-proposed bounded motion profile.

    No startup/traction assumption becomes a physical completion. For a rail,
    radius is ignored. For a hinge, supply the perpendicular measured radius.
    """
    from interaction_runtime.progress import ProgressClock
    from interaction_runtime.segments import validate_action
    validate_action(action)
    if joint_type not in ('revolute','prismatic'):raise ValueError('SUPPORTED_JOINT_TYPE_REQUIRED')
    linear=joint_type=='prismatic';radius=1. if linear else float(radius)
    if not all(math.isfinite(v) and v>0 for v in (radius,dt,max_s)) or max_s>3600:
        raise ValueError('FINITE_BOUNDED_PREVIEW_AND_RADIUS_REQUIRED')
    if not all(math.isfinite(v) for v in (start,target)) or start==target:
        raise ValueError('FINITE_NONZERO_PROPOSED_DISPLACEMENT_REQUIRED')
    lookahead=(motion['speed_m_s']**2/(2*motion['acceleration_m_s2'])+motion['speed_m_s']*dt) if linear else math.radians(.5)
    c=ProgressClock(start,target,lookahead=lookahead);q=float(start);positions=[];vel=[]
    for tick in range(math.ceil(max_s/dt)):
        speed=min(motion['speed_m_s'],action.get('reference_speed_m_s',motion['speed_m_s']))*speed_fraction(tick*dt,action)
        acc=min(motion['acceleration_m_s2'],action.get('reference_acceleration_m_s2',motion['acceleration_m_s2']))
        following,state=c.advance(q,speed,radius,dt,
            motion['speed_m_s'] if linear else motion['angular_speed_rad_s'],acc,
            motion['acceleration_m_s2'] if linear else motion['angular_acceleration_rad_s2'])
        positions.append(following);vel.append((following-q)/dt);q=following
        if c.direction*(target-q)<=0:break
    v=np.asarray(vel);n=len(v);valid=np.ones(n,bool);valid[:121]=False;valid[-121:]=False
    if n<247:return dict(nominal_duration_s=n*dt,nominal_endpoint_native=q,
        nominal_target_reached=c.direction*(target-q)<=0,eligible_sample_opportunity=0,
        physical_feasibility_verified=False,information=None,reason='insufficient nominal local SG support')
    if linear:
        positive=minimum_filter1d((v>1e-4).astype(np.uint8),size=125,mode='constant',cval=0).astype(bool)
        negative=minimum_filter1d((v<-1e-4).astype(np.uint8),size=125,mode='constant',cval=0).astype(bool)
        valid&=positive|negative
    filtered_v=common_filter(v);a=common_filter(np.gradient(v,dt));direction=common_filter(np.sign(v))
    valid&=abs(filtered_v)>=1e-4
    result=dict(nominal_duration_s=n*dt,nominal_endpoint_native=q,nominal_target_reached=c.direction*(target-q)<=0,
        eligible_sample_opportunity=int(valid.sum()),coordinate_unit='m' if linear else 'rad',
        physical_feasibility_verified=False,preview='ideal tracking, no object physics or contact model',
        policy_limits_changed=False,action=action)
    if valid.sum()>=8:result['information']=information(a[valid],filtered_v[valid],direction[valid],scales)
    else:result['information']=None
    return result
