"""Temporal contact-load warning. Emergency reaction bound is SIMULATION ONLY.

10 N comes from the existing simulation gripper actuator effort ceiling; it is
NOT a certified hardware contact-reaction cap or a change to commanded force.
SDK gripper effort units are not a calibrated contact-force measurement.
"""
import collections

class TemporalForceGuard:
    def __init__(self,policy):
        self.soft=policy['soft_force_n'];self.emergency=policy['simulation_emergency_contact_n']
        self.window_s=policy['window_s'];self.required=policy['sustained_windows'];self.samples=collections.deque()
        self.elapsed=0.;self.over_windows=0;self.pending=False;self.events=[];self.last={}
    def update(self,forces,dt,t):
        peak=max(forces.values(),default=0.);self.samples.append(peak);self.elapsed+=dt
        status='NORMAL';mean=sum(self.samples)/len(self.samples)
        if peak>=self.emergency:status='HARD_FORCE_STOP_EMERGENCY_SIM_ONLY'
        elif self.elapsed>=self.window_s:
            self.over_windows=self.over_windows+1 if mean>self.soft else 0
            self.samples.clear();self.elapsed=0.
            if self.over_windows>=self.required:status='HARD_FORCE_STOP_SUSTAINED'
        if status=='NORMAL' and self.over_windows>=self.required and (peak>=self.soft or mean>self.soft):status='HARD_FORCE_STOP_SUSTAINED'
        if status=='NORMAL' and peak>=self.soft:status='SOFT_FORCE_WARNING'
        if status=='SOFT_FORCE_WARNING':self.pending=True
        self.last={'status':status,'raw_peak_n':peak,'window_mean_n':mean,'consecutive_overload_windows':self.over_windows,'t':t,'emergency_scope':'SIMULATION_ONLY_NOT_HARDWARE_SAFE'}
        if status!='NORMAL':self.events.append(dict(self.last))
        return dict(self.last)
    def settled(self):return self.over_windows==0 and self.last.get('raw_peak_n',float('inf'))<self.soft
