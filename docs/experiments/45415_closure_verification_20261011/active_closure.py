import numpy as np

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
            self.state='CENTER';self.opening=min(self.opening,float(opening)) # retain closing command while centering
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
