"""Isolated contact-feedback closure; native contact loads are simulation-only.

CENTER alternates unloaded TCP correction and slow closure using the existing
load threshold, preload, sensor averaging period and centering bounds. No
handle pose, articulation state or asset identity enters this policy.
"""
import numpy as np

class JawCenteredClosure:
    def __init__(self,T0,policy):
        self.origin=np.asarray(T0).copy();self.command=self.origin.copy();self.p=policy
        self.state='SLOW_CLOSE';self.opening=None;self.effort=0.;self.stable=0.
        self.response_period_s=.08 # same existing filtered-load averaging window
        self.center_phase='UNLOAD';self.response_time=0.;self.previous_unilateral=False
        self.response_origin=self.origin[:3,3].copy();self.response_load=0.;self.response_direction=np.zeros(3)
        self.correction_count=0;self.cumulative_command_distance_m=0.;self.response_records=[];self.detail={}
    def update(self,T,opening,forces,pad_centers,dt):
        f=np.asarray(forces,float);centers=np.asarray(pad_centers);axis=centers[0]-centers[1];axis/=np.linalg.norm(axis)
        if self.opening is None:self.opening=float(opening)
        unilateral=f.max()>=.05 and f.min()<.05
        before=self.command[:3,3].copy();close=False
        if unilateral:
            self.state='CENTER';self.opening=min(self.opening,float(opening))
            direction=axis if f[0]>f[1] else -axis
            loaded=float(f.max())
            if not self.previous_unilateral:
                self.center_phase='UNLOAD';self.response_time=0.
                self.response_origin=np.asarray(T)[:3,3].copy();self.response_load=loaded;self.response_direction=direction.copy()
            self.response_time+=dt
            if self.response_time>=self.response_period_s:
                displacement=float((np.asarray(T)[:3,3]-self.response_origin)@self.response_direction)
                unloading=loaded<self.response_load
                self.response_records.append({'phase':self.center_phase,'loaded_force_before_n':self.response_load,'loaded_force_after_n':loaded,'actual_TCP_toward_loaded_m':displacement,'unloading_observed':bool(unloading),'geometry_direction_world':self.response_direction.tolist()})
                # Feedback changes reference progression, never safety limits.
                self.center_phase='CLOSE' if unloading and displacement>0 else 'UNLOAD'
                self.response_time=0.;self.response_origin=np.asarray(T)[:3,3].copy();self.response_load=loaded;self.response_direction=direction.copy()
            if loaded>=self.p['preload_n']:self.center_phase='UNLOAD'
            close=self.center_phase=='CLOSE'
        elif self.state=='CENTER':self.state='SLOW_CLOSE'
        self.previous_unilateral=bool(unilateral)
        if self.state in ('SLOW_CLOSE','CENTER'):
            if self.state=='SLOW_CLOSE' or close:self.opening=max(0.,self.opening-self.p['slow_closure_m_s']*dt)
            if f.min()>=.05:self.state='LOW_PRELOAD';self.effort=min(self.p['preload_n'],self.p['finger_effort_limit_n'])
        if f.max()>.02:
            speed=float(np.clip(.003*(f[0]-f[1]),-self.p['max_centering_speed_m_s'],self.p['max_centering_speed_m_s']))
            delta=axis*np.clip(speed*dt,-self.p['max_centering_step_m'],self.p['max_centering_step_m']);self.command[:3,3]+=delta
            total=self.command[:3,3]-self.origin[:3,3];length=np.linalg.norm(total);bound=self.p['max_centering_displacement_m']
            if length>bound:self.command[:3,3]=self.origin[:3,3]+total*bound/length
        movement=float(np.linalg.norm(self.command[:3,3]-before));self.correction_count+=int(movement>0);self.cumulative_command_distance_m+=movement
        self.detail={'center_phase':self.center_phase,'closing_advanced':bool(close or self.state=='SLOW_CLOSE'),'correction_count':self.correction_count,'cumulative_command_distance_m':self.cumulative_command_distance_m,'command_displacement_m':float(np.linalg.norm(self.command[:3,3]-self.origin[:3,3])),'closing_axis_world':axis.tolist(),'response_count':len(self.response_records),'latest_response':self.response_records[-1] if self.response_records else None}
        if self.state in ('LOW_PRELOAD','FORCE_HOLD'):
            self.effort=float(np.clip(self.effort+2*(self.p['preload_n']-f.mean())*dt,0,self.p['finger_effort_limit_n']))
            self.stable=self.stable+dt if f.min()>=.8*self.p['preload_n'] else 0.
            if self.stable>=.2:self.state='FORCE_HOLD'
            return {'mode':'effort','closing_effort_n':self.effort,'state':self.state,'T':self.command.copy()}
        return {'mode':'position','opening_m':self.opening,'state':self.state,'T':self.command.copy()}
