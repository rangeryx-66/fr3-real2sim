"""Bounded reference-speed programs; no actuator, guard or object-state command."""
import math
import numpy as np

def speed_fraction(elapsed,action):
    program=action.get('velocity_program','constant')
    if program=='constant':return 1.
    low=float(action.get('low_speed_fraction',.125))
    if program=='sine':
        period=float(action.get('period_s',3.1));return low+(1-low)*(.5-.5*math.cos(2*math.pi*elapsed/period))
    if program=='plateaus':
        ramp=float(action.get('ramp_s',.45));hold=float(action.get('plateau_s',.55));t=elapsed%(2*(ramp+hold))
        if t<hold:return low
        if t<hold+ramp:return low+(1-low)*(.5-.5*math.cos(math.pi*(t-hold)/ramp))
        if t<2*hold+ramp:return 1.
        return low+(1-low)*(.5+.5*math.cos(math.pi*(t-2*hold-ramp)/ramp))
    raise ValueError('Unknown bounded velocity program')

def information(X,noise=.001):
    A=np.asarray(X)*np.array([1.,.25,.045]);s=np.linalg.svd(A,compute_uv=False)
    return dict(singular_values=s.tolist(),condition=float(s[0]/max(s[-1],1e-30)),
        logdet_regularized_fisher=float(np.linalg.slogdet(A.T@A/noise**2+1e-3*np.eye(3))[1]),
        velocity_sign_correlation=float(np.corrcoef(np.asarray(X)[:,1:].T)[0,1]) if np.std(np.asarray(X)[:,2]) else None)

def surrogate(T,axis,origin,action,start_deg=0.,lag_s=.05):
    """Existing progress clock + Cartesian limiter, approximate rigid contact.

    No GT dynamics. This is an excitation opportunity estimate, not physical
    feasibility certification. Actual execution remains authoritative.
    """
    from scipy.spatial.transform import Rotation
    from scipy.signal import savgol_filter
    from .motion import CartesianReference,progress_reference
    from .policy import Policy
    from pathlib import Path
    dt=1/240;T=np.asarray(T);axis=np.asarray(axis);origin=np.asarray(origin);theta=math.radians(start_deg);initial=theta;target=math.radians(action['target_deg']);direction=np.sign(target-theta)
    policy=Policy.load(Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json')
    r0=T[:3,3]-origin;r0-=axis*(axis@r0);drive=CartesianReference(T);reference=theta;values=[]
    for tick in range(240*60):
        m=dict(policy.motion('LOADED'));m['speed_m_s']=min(m['speed_m_s'],action['reference_speed_m_s']*speed_fraction(tick*dt,action));m['acceleration_m_s2']=min(m['acceleration_m_s2'],action['reference_acceleration_m_s2'])
        radius=np.linalg.norm(drive.pose[:3,3]-origin);reference=progress_reference(theta,reference,target,direction,m['speed_m_s'],radius,dt,math.radians(.5));R=Rotation.from_rotvec(axis*(reference-initial)).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin
        pose=drive.advance(D@T,dt,m);r=pose[:3,3]-origin;r-=axis*(axis@r);geometric=initial+math.atan2(axis@np.cross(r0,r),r0@r)
        theta=geometric if lag_s==0 else theta+(geometric-theta)*(1-math.exp(-dt/lag_s));values.append(theta)
        if direction*(target-theta)<=0:break
    q=np.asarray(values);v=savgol_filter(q,121,3,deriv=1,delta=dt);a=savgol_filter(q,121,3,deriv=2,delta=dt);ix=np.arange(121,len(q)-121)
    return dict(duration_s=len(q)*dt,actual_surrogate_angle_deg=math.degrees(theta),velocity_range_rad_s=np.quantile(v[ix],[.05,.5,.95]).tolist(),acceleration_rms_rad_s2=float(np.sqrt(np.mean(a[ix]**2))),
        X=np.c_[a[ix],v[ix],np.sign(v[ix])],approximation='rigid-contact kinematic lag surrogate, not Isaac contact success')
