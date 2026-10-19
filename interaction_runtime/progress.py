"""Measured-progress clock V2. Frozen legacy clock remains in motion.py.

Normal tracking has no lag divisor. A finite lookahead has a braking-distance
limit, so a blocked object cannot accumulate an unbounded motion reference.
All arm/contact limits remain owned by Runtime and its original supervisor.
"""
import math
import numpy as np

LOOKAHEAD_RAD=math.radians(.5)  # existing half-degree allowance, now explicit


def hinge_radius(position,origin,axis):
    axis=np.asarray(axis,float);axis=axis/np.linalg.norm(axis)
    return float(np.linalg.norm(np.cross(np.asarray(position)-origin,axis)))


class ProgressClock:
    def __init__(self,theta,target,lookahead=LOOKAHEAD_RAD):
        self.reference=float(theta);self.target=float(target);self.direction=1 if target>=theta else -1
        self.velocity=0.;self.lookahead=float(lookahead);self.limited_steps=0
    def advance(self,theta,speed,radius,dt,angular_speed,linear_acceleration,angular_acceleration):
        radius=max(radius,1e-9);acc=min(linear_acceleration/radius,angular_acceleration)
        rate=min(speed/radius,angular_speed);lag=self.direction*(self.reference-theta)
        # Target lookahead only overcomes measured tracking lag. Physical success
        # is separately tested against actual theta, never this reference.
        remaining=self.direction*(self.target-self.reference)+self.lookahead
        room=min(self.lookahead-lag,remaining)
        braking=max(0.,math.sqrt((acc*dt)**2+2*acc*max(0.,room))-acc*dt)
        wanted=self.direction*min(rate,braking)
        before=self.velocity;self.velocity+=float(np.clip(wanted-before,-acc*dt,acc*dt))
        self.reference+=.5*(before+self.velocity)*dt
        limited=braking<rate
        self.limited_steps+=int(limited)
        return self.reference,dict(lag_rad=lag,requested_rate_rad_s=self.direction*rate,reference_rate_rad_s=self.velocity,tracking_limited=limited)


def segment_plan(theta,target,action,motion,radius,dt=1/240):
    """Budget consistency before actuation, not a contact-success certificate.

    Optional truncation is declared before the segment. Requested and selected
    endpoints remain separate; a clipped target never means the original one
    was reached. Unknown contact lag may still produce a genuine timeout.
    """
    from .probing_profiles_v3 import speed_fraction
    budget=float(action.get('max_sim_s',45.));clock=ProgressClock(theta,target)
    reached=None;last=theta
    for i in range(int(budget/dt)):
        local=dict(motion);speed=min(local['speed_m_s'],action.get('reference_speed_m_s',local['speed_m_s']))*speed_fraction(i*dt,action)
        acc=min(local['acceleration_m_s2'],action.get('reference_acceleration_m_s2',local['acceleration_m_s2']))
        value,_=clock.advance(last,speed,radius,dt,local['angular_speed_rad_s'],acc,local['angular_acceleration_rad_s2'])
        # Ideal-contact timing estimate; no guessed object mass/friction.
        last=value
        stop_time=abs(clock.velocity)/min(acc/max(radius,1e-9),local['angular_acceleration_rad_s2'])
        if clock.direction*(last-target)>=0 and (i+1)*dt+stop_time<=budget:reached=(i+1)*dt;break
    possible=clock.direction*(last-theta)
    # A selected shorter endpoint reserves its own deceleration travel/time.
    available=max(0.,possible-abs(clock.velocity)*stop_time)
    selected=target if reached is not None else theta+clock.direction*available
    clipped=reached is None and bool(action.get('allow_target_truncation',False))
    return dict(valid=reached is not None or clipped,requested_target_rad=float(target),selected_target_rad=float(selected if clipped else target),target_truncated=clipped,
        nominal_duration_s=reached,max_sim_s=budget,stop_reserve_s=stop_time,timing_assumption='ideal tracking; measured lag/contact may still timeout',reason=None if reached is not None or clipped else 'TARGET_EXCEEDS_NOMINAL_BUDGET')


def validate_action(action):
    budget=action.get('max_sim_s',45.)
    if not np.isfinite(budget) or budget<1/240:raise ValueError('positive finite segment budget required')
    if 'allow_target_truncation' in action and type(action['allow_target_truncation']) is not bool:raise ValueError('boolean target truncation declaration required')
    windows=action.get('fit_intervals_s',[])
    if windows and not action['role'].startswith('FIT'):raise ValueError('FIT intervals cannot admit incomplete VALIDATION')
    previous=0.
    for interval in windows:
        if len(interval)!=2 or not np.isfinite(interval).all() or not previous<=interval[0]<interval[1]<=budget:raise ValueError('ordered non-overlapping FIT intervals inside segment budget required')
        previous=interval[1]
