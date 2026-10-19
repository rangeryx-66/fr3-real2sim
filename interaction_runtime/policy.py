"""One policy/guard per scene. Warnings never throw; histories survive phase changes."""
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

PHASES = {
 'SETTLE':'FREE', 'NO_OPERATION':'FREE', 'PREGRASP':'FREE', 'PREFLIGHT':'FREE',
 'WRIST_OBSERVATION_MOTION':'FREE', 'WRIST_OBSERVATION_SETTLE':'FREE',
 'APPROACH':'APPROACH', 'SECOND_APPROACH':'APPROACH', 'PANEL_APPROACH':'APPROACH',
 'CLOSE':'ESTABLISH', 'SLOW_CLOSE':'ESTABLISH', 'CENTER':'ESTABLISH',
 'LOW_PRELOAD':'ESTABLISH', 'FORCE_HOLD':'ESTABLISH', 'LOCAL_CENTERING':'ESTABLISH',
 'VALIDATED_PREFIX_EXTRA_HOLD':'ESTABLISH', 'TRANSMISSION_PROBE':'LOADED',
 'OPEN_CONTINUE':'LOADED', 'FIXED_BASE_PINK_OPEN':'LOADED', 'PANEL_PUSH':'LOADED',
 'CONTACT_PRELOAD':'LOADED',
 'IDENTIFICATION_STATIC_PAUSE':'LOADED', 'FINAL_HOLD':'LOADED',
 'RELEASE':'RELEASE', 'CLEARANCE_RETREAT':'SEPARATE', 'PANEL_SEPARATION':'SEPARATE',
 'SAFE_PARK_MOTION':'PARK', 'PARK':'PARK'
}

class SafetyStop(RuntimeError):
    def __init__(self, reason, evidence=None):
        self.reason=reason; self.evidence=evidence or {}
        super().__init__(reason)

@dataclass(frozen=True)
class Policy:
    data: dict
    sha256: str
    @classmethod
    def load(cls, path):
        b=Path(path).read_bytes(); d=json.loads(b)
        if d['schema']!='isaac-interaction-runtime-v1' or d['physics_hz']!=240 or d['sensor_hz']!=240:
            raise ValueError('Unsupported runtime/physics configuration')
        for phase, p in d['phases'].items():
            if any(not math.isfinite(v) or v<=0 for v in p.values()):raise ValueError(phase)
            if p['measured_speed_stop_m_s']<=p['speed_m_s']:raise ValueError('Measured envelope must include command envelope')
        f=d['force']
        for prefix in ['pad','wrist','torque']:
            unit='nm' if prefix=='torque' else 'n'
            if not 0<f[prefix+'_warning_'+unit]<f[prefix+'_sustained_'+unit]<f[prefix+'_emergency_'+unit]:raise ValueError(prefix)
        return cls(d, hashlib.sha256(b).hexdigest())
    def phase(self, name):
        key=name if name in self.data['phases'] else PHASES.get(name)
        if key is None:raise ValueError('Unmapped runtime phase: '+str(name))
        return key
    def motion(self, name):return self.data['phases'][self.phase(name)]

class ForceMonitor:
    def __init__(self, policy):
        self.p=policy.data['force'];self.history=deque();self.last={};self.events=[];self.pending=False
    def update(self, pads, dt, t, wrench=None):
        pad=max(pads.values(), default=0.)
        if not math.isfinite(pad) or pad<0:raise SafetyStop('INVALID_CONTACT_MEASUREMENT')
        f=m=0.
        if wrench is not None:
            if len(wrench)!=6 or not all(math.isfinite(x) for x in wrench):raise SafetyStop('INVALID_WRIST_WRENCH')
            f=math.sqrt(sum(x*x for x in wrench[:3]));m=math.sqrt(sum(x*x for x in wrench[3:]))
        ratios=[pad/self.p['pad_sustained_n'],f/self.p['wrist_sustained_n'],m/self.p['torque_sustained_nm']]
        self.history.append((float(t),float(dt),pad,f,m))
        while self.history and self.history[0][0] < t-self.p['sustained_s']+dt*.5:self.history.popleft()
        duration=sum(x[1] for x in self.history)
        # A trailing sustained interval must be observed in full. No counters or
        # force histories reset at direction change, release or mode transition.
        means=[sum(x[1]*x[k] for x in self.history)/max(duration,dt) for k in [2,3,4]]
        hard=None
        if pad>=self.p['pad_emergency_n']:hard='EMERGENCY_PAD_OVERLOAD'
        elif f>=self.p['wrist_emergency_n']:hard='EMERGENCY_WRIST_FORCE'
        elif m>=self.p['torque_emergency_nm']:hard='EMERGENCY_WRIST_TORQUE'
        elif duration>=self.p['sustained_s']-dt*.5 and max(means[i]/self.p[k] for i,k in enumerate(['pad_sustained_n','wrist_sustained_n','torque_sustained_nm']))>1:
            hard='SUSTAINED_OVERLOAD'
        warning=pad>=self.p['pad_warning_n'] or f>=self.p['wrist_warning_n'] or m>=self.p['torque_warning_nm']
        self.last=dict(status=hard or ('INFORMATIONAL_LOAD_WARNING' if warning else 'NORMAL'),
            raw_peak_n=pad,wrist_force_n=f,wrist_torque_nm=m,window_mean_n=means[0],
            sustained_duration_s=duration,t=t,stop=hard is not None,normal_only_pad_measurement=True)
        if warning or hard:self.events.append(self.last.copy())
        return self.last.copy()
    def settled(self):
        # Unloaded decision state still uses the old physical pad criterion.
        return bool(self.last and self.last['raw_peak_n']<=self.p['unloaded_pad_n'] and not self.last['stop'])

class SafetySupervisor:
    def __init__(self, policy, joint_limits, speed_limits, effort_limits):
        self.policy=policy;self.limits=joint_limits;self.speeds=speed_limits;self.efforts=effort_limits
        self.force=ForceMonitor(policy);self.latched=None
    def check(self, *, phase, q, dq, tcp_speed, pads, dt, t, wrench=None,
              collision_ok=True, collision_reason='', actuator_efforts=None):
        if self.latched is not None:raise SafetyStop('LATCHED_'+self.latched['reason'],self.latched)
        event=self.force.update(pads,dt,t,wrench)
        reason=None;motion=self.policy.motion(phase)
        if not collision_ok:reason='COLLISION:'+collision_reason
        elif any(not math.isfinite(v) for v in list(q)+list(dq)+[tcp_speed]):reason='INVALID_ROBOT_STATE'
        elif min(min(x-lo,hi-x) for x,(lo,hi) in zip(q,self.limits))<=self.policy.data['joint_margin_rad']:reason='JOINT_MARGIN'
        elif any(abs(v)>s*self.policy.data['arm_velocity_fraction'] for v,s in zip(dq,self.speeds)):reason='JOINT_VELOCITY'
        elif tcp_speed>motion['measured_speed_stop_m_s']:reason='PHASE_TCP_VELOCITY'
        elif actuator_efforts is not None and any(abs(v)>c for v,c in zip(actuator_efforts,self.efforts)):reason='ACTUATOR_EFFORT'
        elif event['stop']:reason=event['status']
        if reason:
            self.latched=dict(reason=reason,phase=phase,t=t,force=event,tcp_speed_m_s=tcp_speed)
            raise SafetyStop(reason,self.latched)
        return event
