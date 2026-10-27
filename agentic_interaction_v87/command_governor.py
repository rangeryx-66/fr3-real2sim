"""Bound the final IK joint command in TCP space, not just its virtual goal.

All limits come from the existing phase policy and declared motion. A failed
projection rejects this action; it never widens a limit or submits that command.
The nonlinear FK check is authoritative even when the optimizer reports success.
"""
import numpy as np
from scipy.optimize import minimize,minimize_scalar
from scipy.spatial.transform import Rotation


class CommandGovernor:
    def __init__(self, fk, dt, *, preserve_direction=False):
        if not np.isfinite(dt) or dt <= 0:raise ValueError('INVALID_CONTROL_PERIOD')
        self.fk=fk;self.dt=float(dt);self.previous_twist=np.zeros(6)
        self.latest={};self.calls=0;self.limited=0
        self.preserve_direction=bool(preserve_direction)

    def reset(self):
        self.previous_twist=np.zeros(6)

    def velocity(self, q, wanted, previous, joint_speeds, joint_acceleration, motion):
        q=np.asarray(q,float);wanted=np.asarray(wanted,float);previous=np.asarray(previous,float)
        speeds=np.broadcast_to(joint_speeds,q.shape).astype(float)
        acceleration=np.broadcast_to(joint_acceleration,q.shape).astype(float)
        if not np.isfinite(np.r_[q,wanted,previous,speeds,acceleration]).all():
            raise ValueError('NONFINITE_COMMAND_GOVERNOR_INPUT')
        if np.any(speeds<=0) or np.any(acceleration<=0):raise ValueError('INVALID_JOINT_LIMITS')
        limits=np.array([motion['speed_m_s'],motion['angular_speed_rad_s'],
            motion['acceleration_m_s2'],motion['angular_acceleration_rad_s2']],float)
        if not np.isfinite(limits).all() or np.any(limits<=0):raise ValueError('INVALID_TCP_LIMITS')
        declared_limits=limits.copy()
        # Small numerical headroom, not a change to physical safety thresholds.
        limits*=1-1e-6
        low=np.maximum(-speeds,previous-acceleration*self.dt)
        high=np.minimum(speeds,previous+acceleration*self.dt)
        if np.any(low>high):raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:JOINT_RATE_INFEASIBLE')
        origin=np.asarray(self.fk(q),float)
        if origin.shape!=(4,4) or not np.isfinite(origin).all():raise ValueError('INVALID_COMMAND_FK')
        cached={}
        def twist(v):
            key=np.asarray(v).tobytes()
            if key not in cached:
                pose=np.asarray(self.fk(q+v*self.dt),float)
                if pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError('INVALID_COMMAND_FK')
                cached[key]=np.r_[(pose[:3,3]-origin[:3,3])/self.dt,
                    Rotation.from_matrix(pose[:3,:3]@origin[:3,:3].T).as_rotvec()/self.dt]
            return cached[key]
        def constraints(v):
            t=twist(v);delta=(t-self.previous_twist)/self.dt
            return 1-(np.array([np.linalg.norm(t[:3]),np.linalg.norm(t[3:]),
                np.linalg.norm(delta[:3]),np.linalg.norm(delta[3:])])/limits)**2
        desired=np.clip(wanted,low,high);scalar_brake_repaired=False
        if not np.any(wanted) and np.any(previous):
            # Brake a coordinated allocation together. Independent slew clips
            # can remove a compensating joint before the base slows, raising
            # world TCP speed. The feasible-anchor projection then keeps the
            # old command forever. A common fraction respects every joint's
            # slew bound while retaining that cancellation; nonlinear TCP
            # speed/acceleration checks below still own acceptance.
            moving=previous!=0
            fraction=min(1.,float(np.min(
                acceleration[moving]*self.dt/np.abs(previous[moving]))))
            desired=np.clip(previous*(1.-fraction),low,high)
            if np.min(constraints(desired)) < -1e-8:
                # The maximal joint-slew brake can slightly exceed TCP slew.
                # At a speed boundary neither clipped endpoint need be a
                # feasible optimizer anchor. Search the ONE shared braking
                # fraction first: retain the coordinated direction instead of
                # asking a nine-variable solve to discover that interior.
                trial=lambda f:np.clip(previous*(1.-f),low,high)
                result=minimize_scalar(lambda f:-float(np.min(constraints(trial(f)))),
                    bounds=(0.,fraction),method='bounded',
                    options=dict(xatol=1e-14,maxiter=80))
                if np.min(constraints(trial(result.x)))>=-1e-9:
                    left=float(result.x);right=fraction
                    for _ in range(30):
                        middle=(left+right)/2
                        if np.min(constraints(trial(middle)))>=-1e-9:left=middle
                        else:right=middle
                    desired=trial(left);scalar_brake_repaired=True
        selected=desired.copy();limited=scalar_brake_repaired
        if self.preserve_direction and np.min(constraints(selected)) < -1e-8:
            # Keep an already jointly planned allocation. Acceleration-scaled
            # SLSQP can invent opposing XY/arm null motion because their units
            # and slew scales differ. Search from a VERIFIED feasible command;
            # if no anchor exists, the original nonlinear optimizer still owns
            # recovery. Every result retains the independent checks below.
            anchors=[np.clip(previous,low,high),np.clip(np.zeros_like(previous),low,high)]
            anchor=next((v for v in anchors if np.min(constraints(v))>=-1e-9),None)
            if anchor is not None:
                limited=True;left=0.;right=1.
                for _ in range(30):
                    middle=(left+right)/2;trial=anchor+middle*(desired-anchor)
                    if np.min(constraints(trial))>=-1e-9:left=middle
                    else:right=middle
                selected=anchor+left*(desired-anchor)
        if np.min(constraints(selected)) < -1e-8:
            limited=True
            initial=np.clip(previous,low,high)
            # Optimize an acceleration-sized increment. Raw joint velocities
            # have milliradian-wide bounds at 240 Hz; finite differences there
            # make an otherwise feasible braking step numerically ill-scaled.
            # Unit-sized variables preserve exactly the same physical bounds.
            scale=acceleration*self.dt
            zlow=(low-previous)/scale;zhigh=(high-previous)/scale
            zd=(desired-previous)/scale
            expand=lambda z:previous+np.asarray(z)*scale
            result=minimize(lambda z:float(np.sum((z-zd)**2)),
                (initial-previous)/scale,method='SLSQP',
                bounds=list(zip(zlow,zhigh)),
                constraints=[dict(type='ineq',fun=lambda z:constraints(expand(z)))],
                options=dict(maxiter=80,ftol=1e-10,eps=1e-5))
            selected=expand(result.x)
            # A numerical optimizer may stop just outside a constraint. Stay
            # inside the *unchanged* bound by searching from an independently
            # verified feasible velocity; never accept its success flag alone.
            if np.min(constraints(selected)) < 0:
                anchors=[initial,np.clip(np.zeros_like(previous),low,high)]
                # This tolerance is far smaller than the 1e-6 headroom above;
                # finite precision at a boundary must not discard a safe anchor.
                anchor=next((v for v in anchors if np.min(constraints(v))>=-1e-9),None)
                if anchor is not None:
                    left=0.;right=1.
                    for _ in range(30):
                        middle=(left+right)/2
                        trial=anchor+middle*(selected-anchor)
                        if np.min(constraints(trial))>=-1e-9:left=middle
                        else:right=middle
                    selected=anchor+left*(selected-anchor)
        # The optimizer aims inside a small numerical headroom. That target
        # is not another physical safety limit: a converged-or-not proposal
        # must be checked against the ORIGINAL declared limits independently.
        # The closed-record failure was inside every original limit, yet just
        # outside this artificial target after SLSQP exhausted its iterations.
        if (not np.isfinite(selected).all() or np.any(selected<low-1e-10)
                or np.any(selected>high+1e-10)):
            raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:TCP_PROJECTION_INFEASIBLE')
        current=twist(selected);delta=(current-self.previous_twist)/self.dt
        measured_norms=np.array([np.linalg.norm(current[:3]),np.linalg.norm(current[3:]),
            np.linalg.norm(delta[:3]),np.linalg.norm(delta[3:])])
        if not np.isfinite(measured_norms).all() or np.any(measured_norms>declared_limits*(1+1e-12)):
            raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:TCP_PROJECTION_INFEASIBLE')
        slack=float(np.min(constraints(selected)))
        self.previous_twist=current.copy();self.calls+=1;self.limited+=int(limited)
        self.latest=dict(limited=limited,linear_speed_m_s=float(np.linalg.norm(current[:3])),
            angular_speed_rad_s=float(np.linalg.norm(current[3:])),
            linear_acceleration_m_s2=float(np.linalg.norm(delta[:3])),
            angular_acceleration_rad_s2=float(np.linalg.norm(delta[3:])),
            minimum_normalized_slack=slack,
            minimum_declared_slack=float(np.min(1-(measured_norms/declared_limits)**2)))
        return selected
