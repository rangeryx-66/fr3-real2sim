"""Finite progress-aware action contract; ProgressClock and safety policy are unchanged.

Deadlines are episode resources, not estimates of contact health. A caller may
specify an explicit engineering ceiling, but there is no default segment timeout.
"""
import math
import time
from collections import deque
from datetime import datetime
import numpy as np
from .progress import ProgressClock, LOOKAHEAD_RAD
from .probing_profiles_v3 import speed_fraction

class ExecutionBudgetExhausted(RuntimeError):
    """Recoverable operation stop; existing protected exit remains executable."""

MIN_TRANSMISSION_RATIO=.1
LEAD_SATURATION_FRACTION=.9

def slip_stall_confirmed(windows,lag,lookahead=LOOKAHEAD_RAD):
    # Three slip windows alone are evidence of compliance/migration, not proof
    # that a slowly building load has exhausted the existing reference allowance.
    return bool(windows>=3 and lag>=LEAD_SATURATION_FRACTION*lookahead)

STATUSES=('TARGET_REACHED','SAFE_PARTIAL_PROGRESS','TRANSMISSION_FAILURE','SAFETY_STOP','BUDGET_EXHAUSTED')

def validate_action(action):
    cap=action.get('max_sim_s')
    if cap is not None and (not np.isfinite(cap) or cap<1/240):raise ValueError('positive finite explicit ceiling required')
    if 'allow_target_truncation' in action and type(action['allow_target_truncation']) is not bool:raise ValueError('boolean truncation declaration required')
    for key in ('reference_speed_m_s','reference_acceleration_m_s2'):
        if key in action and (not np.isfinite(action[key]) or action[key]<=0):raise ValueError('positive finite reference limits required')
    if action.get('velocity_program','constant') not in ('constant','sine','plateaus'):raise ValueError('unknown velocity program')
    if not 0<float(action.get('low_speed_fraction',.125))<=1:raise ValueError('positive bounded profile speed fraction required')
    for key in ('period_s','ramp_s','plateau_s'):
        if key in action and (not np.isfinite(action[key]) or action[key]<=0):raise ValueError('positive finite profile times required')
    previous=0.
    for interval in action.get('fit_intervals_s',[]):
        if not action['role'].startswith('FIT') or len(interval)!=2 or not np.isfinite(interval).all() or not previous<=interval[0]<interval[1] or (cap is not None and interval[1]>cap):raise ValueError('ordered predeclared FIT intervals required')
        previous=interval[1]
    if any(x!='SAFE_PARTIAL_PROGRESS' for x in action.get('continue_after',[])):raise ValueError('only safe partial progress permits an explicit conditional successor')

def profile_cycle(action):
    p=action.get('velocity_program','constant')
    if p=='sine':return max(1.5,float(action.get('period_s',3.1)))
    if p=='plateaus':return max(1.5,2*(float(action.get('ramp_s',.45))+float(action.get('plateau_s',.55))))
    return 1.5

class EpisodeBudget:
    """Reserve wall AND simulated time for the existing guarded release/park.

    Default resources are a 40-minute / 30-sim-minute episode with 3 wall
    minutes / 60 sim seconds reserved for exit. Launchers may declare resources;
    these are finite execution bounds, never physical safety thresholds.
    """
    def __init__(self,job,now=time.monotonic):
        self.now=now;self.started=now();self.wall=float(job.get('episode_wall_budget_s',2400.))
        if job.get('deadline_shanghai'):self.wall=min(self.wall,max(0.,datetime.fromisoformat(job['deadline_shanghai']).timestamp()-time.time()))
        self.sim=float(job.get('episode_sim_budget_s',1800.));self.reserve_wall=float(job.get('safe_exit_wall_reserve_s',180.));self.reserve_sim=float(job.get('safe_exit_sim_reserve_s',60.))
        if not all(math.isfinite(v) and v>0 for v in (self.wall,self.sim,self.reserve_wall,self.reserve_sim)):raise ValueError('finite positive episode resources required')
    def remaining(self,sim_s):
        return dict(wall_s=max(0.,self.wall-(self.now()-self.started)-self.reserve_wall),sim_s=max(0.,self.sim-sim_s-self.reserve_sim),exit_reserved=True)


def segment_plan(theta,target,action,motion,radius,dt,available):
    validate_action(action);clock=ProgressClock(theta,target);last=theta
    # Upper integration bound follows displacement and the minimum profile rate,
    # with room for acceleration and the existing lookahead. Not a loaded deadline.
    speed=min(motion['speed_m_s'],action.get('reference_speed_m_s',motion['speed_m_s']))
    acc=min(motion['acceleration_m_s2'],action.get('reference_acceleration_m_s2',motion['acceleration_m_s2']))
    rate=min(speed/max(radius,1e-9),motion['angular_speed_rad_s']);alpha=min(acc/max(radius,1e-9),motion['angular_acceleration_rad_s2'])
    low=1. if action.get('velocity_program','constant')=='constant' else float(action.get('low_speed_fraction',.125))
    upper=(abs(target-theta)+LOOKAHEAD_RAD)/max(rate*low,1e-9)+2*rate/alpha+profile_cycle(action)
    reached=None
    integration_bound=min(upper,available['sim_s'])
    for i in range(math.ceil(integration_bound/dt)):
        last,_=clock.advance(last,speed*speed_fraction(i*dt,action),radius,dt,motion['angular_speed_rad_s'],acc,motion['angular_acceleration_rad_s2'])
        if clock.direction*(last-target)>=0:reached=(i+1)*dt;break
    stop=rate/alpha+dt+.3  # existing safe-park stationary hold; Operations uses PARK_DEFINITION
    cap=min(available['sim_s'],float(action.get('max_sim_s') or math.inf))
    nominal=reached if reached is not None else upper
    return dict(valid=cap>stop and available['wall_s']>0,requested_target_rad=float(target),selected_target_rad=float(target),target_truncated=False,
        nominal_duration_s=nominal,nominal_is_conservative_bound=reached is None,max_sim_s=cap,stop_reserve_s=stop,profile_cycle_s=profile_cycle(action),episode_remaining=available,
        timing_assumption='nominal ideal tracking; live measured ETA and finite episode resources; no implicit segment ceiling',reason=None if cap>stop and available['wall_s']>0 else 'EXIT_RESERVE_REACHED')

class TransmissionMonitor:
    """Detect plateaued slip as well as the original increasing-slip stall.

    Two complete non-overlapping speed-program cycles must show <10% of
    requested physical travel while >=90% of the existing lead is consumed.
    This is a recoverable transmission diagnosis, NOT a new hard safety gate.
    No pad-load symmetry or positive-normal-report assumption is used.
    """
    def __init__(self,start,direction,action):
        self.start=float(start);self.direction=direction;self.window=profile_cycle(action);self.samples=deque();self.bad=0;self.last=None;self.failure=None;self.valid_until=0.
    def update(self,t,q,state,dt,remaining):
        q=float(q);remaining=float(remaining)
        self.samples.append((t,q,abs(state['requested_rate_rad_s'])*dt,state['lag_rad']))
        if self.samples[-1][0]-self.samples[0][0]<self.window:return None
        first=self.samples[0];duration=t-first[0];progress=self.direction*(q-first[1]);demand=sum(s[2] for s in self.samples)
        ratio=progress/max(demand,1e-12);lead=float(np.median([s[3] for s in self.samples]))
        bad=bool(ratio<MIN_TRANSMISSION_RATIO and lead>=LEAD_SATURATION_FRACTION*LOOKAHEAD_RAD and remaining>0)
        self.bad=self.bad+1 if bad else 0
        self.last=dict(t_s=t,window_s=duration,progress_rad=progress,requested_travel_rad=demand,transmission_ratio=ratio,lead_rad=lead,suspect=bad)
        if not bad:self.valid_until=t
        self.samples.clear();self.samples.append((t,float(q),0.,state['lag_rad']))
        if self.bad>=2:self.failure=dict(self.last,first_suspect_s=t-2*self.window);return self.failure
        return None
    def stable(self):return self.last is not None and not self.last['suspect'] and self.last['transmission_ratio']>=MIN_TRANSMISSION_RATIO


def continuation_allowed(result,next_action,latched,remaining):
    if latched is not None:return False,'SAFETY_LATCH'
    if remaining['wall_s']<=0 or remaining['sim_s']<=0:return False,'EXIT_RESERVE_REACHED'
    if result.get('status')=='TARGET_REACHED':return True,'COMPLETED_PREAPPROVED_SUCCESSOR'
    if result.get('status')!='SAFE_PARTIAL_PROGRESS' or 'SAFE_PARTIAL_PROGRESS' not in next_action.get('continue_after',[]):return False,'UNAUTHORIZED_OR_UNSAFE_PARTIAL_SUCCESSOR'
    if not result.get('safely_stopped') or not result.get('physical_transmission_valid'):return False,'CONTACT_NOT_STABLE'
    return True,'PREAPPROVED_MEASURED_STATE_SUCCESSOR'
