"""Continuous bounded timing, independent translation/rotation, no tiny-radius speed surrogate."""
import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation


def timed_path(solver, base, waypoints, dt, motion, joint_speed, joint_acceleration,
               collision_check, minimum_duration=0.):
    """C2 path, validated as executed; fall back to original checked segments.

    Timing samples bound velocities AND acceleration. Full robot collision and
    joint predicates are supplied by the caller. No contact physics is sampled
    offline and no robot state is set here.
    """
    q=np.asarray(waypoints,float)
    keep=np.r_[True,np.max(abs(np.diff(q,axis=0)),axis=1)>1e-10];q=q[keep]
    if len(q)<2:return iter(())
    d=np.linalg.norm(np.diff(q,axis=0),axis=1);s=np.r_[0.,np.cumsum(d)];s/=s[-1]
    curve=CubicSpline(s,q,axis=0,bc_type='natural')
    u=np.linspace(0,1,max(129,16*len(q)));p=10*u**3-15*u**4+6*u**5
    v=30*u**2*(1-u)**2;a=60*u-180*u*u+120*u**3
    qs=curve(p);qv=curve(p,1)*v[:,None];qa=curve(p,2)*v[:,None]**2+curve(p,1)*a[:,None]
    feasible=True
    for x in qs:
        ok,why=collision_check(x)
        if not ok:feasible=False;break
    if not feasible:
        if len(q)==2:raise RuntimeError('MOTION_PATH_UNAVAILABLE:'+why)
        # Same checked geometric edges; no smoothing through an obstacle.
        def segmented():
            for x,y in zip(q,q[1:]):yield from timed_path(solver,base,[x,y],dt,motion,joint_speed,joint_acceleration,collision_check)
        return segmented()
    from interaction_reference_handover import seed_pose
    poses=np.asarray([seed_pose(solver,base,x) for x in qs]);h=u[1]-u[0]
    dp=np.gradient(poses[:,:3,3],h,axis=0,edge_order=2);ddp=np.gradient(dp,h,axis=0,edge_order=2)
    rv=np.asarray([Rotation.from_matrix(y[:3,:3]@x[:3,:3].T).as_rotvec() for x,y in zip(poses,poses[1:])])/h
    ra=np.diff(rv,axis=0)/h
    duration=max(float(minimum_duration),
        float(np.max(abs(qv)/np.asarray(joint_speed))),
        math.sqrt(float(np.max(abs(qa)/np.asarray(joint_acceleration)))),
        float(np.max(np.linalg.norm(dp,axis=1))/motion['speed_m_s']),
        math.sqrt(float(np.max(np.linalg.norm(ddp,axis=1))/motion['acceleration_m_s2'])),
        float(np.max(np.linalg.norm(rv,axis=1))/motion['angular_speed_rad_s']),
        math.sqrt(float(np.max(np.linalg.norm(ra,axis=1))/motion['angular_acceleration_rad_s2'])) if len(ra) else 0.)
    steps=max(2,int(math.ceil(duration/dt)));duration=steps*dt
    def samples():
        for i in range(1,steps+1):
            u=i/steps;p=10*u**3-15*u**4+6*u**5;v=30*u*u*(1-u)**2/duration
            yield curve(p),curve(p,1)*v
    return samples()


class CartesianReference:
    def __init__(self, pose, velocity=None):
        self.pose=np.asarray(pose).copy();self.velocity=np.zeros(6) if velocity is None else np.asarray(velocity).copy()
    def advance(self, goal, dt, motion):
        goal=np.asarray(goal);err=np.r_[goal[:3,3]-self.pose[:3,3],Rotation.from_matrix(goal[:3,:3]@self.pose[:3,:3].T).as_rotvec()]
        inc=np.zeros(6)
        for sl,speed,acc in [(slice(0,3),motion['speed_m_s'],motion['acceleration_m_s2']),
                              (slice(3,6),motion['angular_speed_rad_s'],motion['angular_acceleration_rad_s2'])]:
            e=err[sl];n=np.linalg.norm(e);wanted=e/max(n,1e-12)*min(speed,math.sqrt(2*acc*n))
            change=wanted-self.velocity[sl];self.velocity[sl]+=change*min(1.,acc*dt/max(np.linalg.norm(change),1e-12))
            inc[sl]=self.velocity[sl]*dt
            # Do not snap to a goal at the cost of an abrupt velocity target.
        self.pose[:3,3]+=inc[:3];self.pose[:3,:3]=Rotation.from_rotvec(inc[3:]).as_matrix()@self.pose[:3,:3]
        return self.pose.copy()


def progress_reference(theta, reference, target, direction, speed, radius, dt,
                       overshoot_rad=0., lead_rad=math.radians(.05)):
    """The validated measured-progress clock, for pull and panel-push directions."""
    lag=max(0.,direction*(reference-theta))
    rate=speed/max(radius,1e-9)/(1+lag/lead_rad)
    return direction*min(direction*target+overshoot_rad,
        max(direction*theta+lead_rad,direction*reference+rate*dt))
