"""Sensor-only controllers. No URDF, object joint, link pose, or simulator imports."""
from dataclasses import dataclass
import numpy as np

@dataclass
class ContactClosure:
    preload_n: float
    effort_limit_n: float=10.
    state: str='CLOSE'
    stable_s: float=0.
    effort_n: float=0.
    opening_command_m: float|None=None

    def update(self,opening,pad_forces,dt):
        f=np.asarray(pad_forces,float)
        if self.state=='CLOSE':
            if f.min()>=.2:self.state='PRELOAD';self.effort_n=min(self.preload_n,self.effort_limit_n)
            elif f.max()>.01:self.state='SEEK_SECOND_PAD';self.effort_n=min(.5,self.preload_n,self.effort_limit_n)
            else:
                if self.opening_command_m is None:self.opening_command_m=float(opening)
                self.opening_command_m=max(0.,self.opening_command_m-.01*dt)
                return {'mode':'position','opening_m':self.opening_command_m,'state':self.state}
        if self.state=='SEEK_SECOND_PAD':
            if f.min()>=.2:self.state='PRELOAD';self.effort_n=min(self.preload_n,self.effort_limit_n)
            else:return {'mode':'effort','closing_effort_n':self.effort_n,'state':self.state}
        self.effort_n=float(np.clip(self.effort_n+2.*(self.preload_n-f.mean())*dt,0,self.effort_limit_n))
        self.stable_s=self.stable_s+dt if f.min()>=.8*self.preload_n else 0.
        if self.stable_s>=.2:self.state='FORCE_HOLD'
        return {'mode':'effort','closing_effort_n':self.effort_n,'state':self.state}

class JawCenteredClosure:
    """Geometric pad identity sets direction; no wrench polarity or object GT."""
    def __init__(self,T0,policy):
        self.origin=np.asarray(T0).copy();self.command=self.origin.copy();self.p=policy
        self.state='SLOW_CLOSE';self.opening=None;self.effort=0.;self.stable=0.
    def update(self,T,opening,forces,pad_centers,dt):
        f=np.asarray(forces,float);centers=np.asarray(pad_centers);axis=centers[0]-centers[1];axis/=np.linalg.norm(axis)
        if self.opening is None:self.opening=float(opening)
        unilateral=f.max()>=.05 and f.min()<.05
        if unilateral:
            self.state='CENTER';self.opening=float(opening) # pause further squeeze
        elif self.state=='CENTER':self.state='SLOW_CLOSE'
        if self.state in ('SLOW_CLOSE','CENTER'):
            if self.state=='SLOW_CLOSE':self.opening=max(0.,self.opening-self.p['slow_closure_m_s']*dt)
            if f.min()>=.05:self.state='LOW_PRELOAD';self.effort=min(self.p['preload_n'],self.p['finger_effort_limit_n'])
        if f.max()>.02:
            speed=float(np.clip(.003*(f[0]-f[1]),-self.p['max_centering_speed_m_s'],self.p['max_centering_speed_m_s']))
            delta=axis*np.clip(speed*dt,-self.p['max_centering_step_m'],self.p['max_centering_step_m']);self.command[:3,3]+=delta
            total=self.command[:3,3]-self.origin[:3,3];length=np.linalg.norm(total);bound=self.p['max_centering_displacement_m']
            if length>bound:self.command[:3,3]=self.origin[:3,3]+total*bound/length
        if self.state in ('LOW_PRELOAD','FORCE_HOLD'):
            self.effort=float(np.clip(self.effort+2*(self.p['preload_n']-f.mean())*dt,0,self.p['finger_effort_limit_n']))
            self.stable=self.stable+dt if f.min()>=.8*self.p['preload_n'] else 0.
            if self.stable>=.2:self.state='FORCE_HOLD'
            return {'mode':'effort','closing_effort_n':self.effort,'state':self.state,'T':self.command.copy()}
        return {'mode':'position','opening_m':self.opening,'state':self.state,'T':self.command.copy()}

class PadCompliantPull:
    """Bounded incremental pull with pad-balance compliance; no hinge or F/T.

    Orientation follows measured EE rather than a locked initial 6D pose. This
    is a small velocity probe, not a full six-axis force/admittance controller.
    """
    def __init__(self,T0,policy):
        self.command=np.asarray(T0).copy();self.direction=-self.command[:3,2];self.p=policy;self.center_offset=np.zeros(3)
    def update(self,T,forces,pad_centers,dt):
        centers=np.asarray(pad_centers);axis=centers[0]-centers[1];axis/=np.linalg.norm(axis);f=np.asarray(forces)
        speed=float(np.clip(.003*(f[0]-f[1]),-.001,.001))
        delta=axis*np.clip(speed*dt,-self.p['max_centering_step_m'],self.p['max_centering_step_m']);previous=self.center_offset.copy();self.center_offset+=delta;n=np.linalg.norm(self.center_offset)
        if n>self.p['max_centering_displacement_m']:self.center_offset*=self.p['max_centering_displacement_m']/n
        self.command[:3,3]+=.0005*self.direction*dt+self.center_offset-previous
        lag=self.command[:3,3]-np.asarray(T)[:3,3];n=np.linalg.norm(lag)
        if n>.0005:self.command[:3,3]=np.asarray(T)[:3,3]+lag*.0005/n
        self.command[:3,:3]=np.asarray(T)[:3,:3]
        return self.command.copy()
