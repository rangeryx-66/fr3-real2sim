"""Separate nominal pacing warnings from measured loss of transmission.

The original progress clock, lead, increasing-slip detector and all native
guards are retained. Only the nominal-demand soft diagnosis is disambiguated.
"""
import math
import numpy as np
from interaction_runtime.segments import TransmissionMonitor, MIN_TRANSMISSION_RATIO


def progress_coordinate(position, initial, articulation):
    p=np.asarray(position);o=np.asarray(articulation.origin_world_m)
    axis=np.asarray(articulation.axis_world)
    if articulation.joint_type=='prismatic':return float((p-initial)@axis)
    a=np.asarray(initial)-o;a-=axis*(a@axis)
    b=p-o;b-=axis*(b@axis)
    return math.atan2(float(axis@np.cross(a,b)),float(a@b))


def factory(runtime, articulation, captures):
    class MeasuredMonitor(TransmissionMonitor):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.shadow=TransmissionMonitor(*args,**kwargs)
            self.initial=runtime.tcp()[:3,3].copy();self.window_tcp=0.
            self.measured_bad=0;self.first_advisory_s=None;self.history=[]
            captures.append(self)

        def update(self,t,q,state,dt,remaining):
            # Preserve the original nominal diagnosis and its FIT cutoff.
            self.shadow.update(t,q,state,dt,remaining)
            previous=self.shadow.last
            if previous is None or self.history and self.history[-1]['t_s']==previous['t_s']:
                return None
            tcp=progress_coordinate(runtime.tcp()[:3,3],self.initial,articulation)
            # Angular unwrap relative to the last window, supporting long arcs.
            if articulation.joint_type=='revolute':
                tcp=self.window_tcp+math.atan2(math.sin(tcp-self.window_tcp),math.cos(tcp-self.window_tcp))
            actual_demand=max(0.,self.direction*(tcp-self.window_tcp));self.window_tcp=tcp
            scale=articulation.path_scale(runtime.tcp()[:3,3])
            resolved=previous['progress_rad']*scale>=1e-4*previous['window_s']
            ratio=previous['progress_rad']/max(actual_demand,1e-12)
            real_bad=bool(previous['suspect'] and (not resolved or ratio<MIN_TRANSMISSION_RATIO))
            self.measured_bad=self.measured_bad+1 if real_bad else 0
            self.last=dict(previous,nominal_suspect=previous['suspect'],
                nominal_transmission_ratio=previous['transmission_ratio'],
                actual_TCP_travel_native=actual_demand,actual_transmission_ratio=ratio,
                suspect=real_bad,advisory=bool(previous['suspect'] and not real_bad),
                transmission_ratio=ratio if previous['suspect'] else previous['transmission_ratio'])
            self.history.append(self.last.copy())
            if previous['suspect'] and self.first_advisory_s is None:
                self.first_advisory_s=max(0.,t-previous['window_s'])
            if not real_bad:self.valid_until=t
            if self.measured_bad>=2:
                self.failure=dict(self.last,first_suspect_s=t-2*self.window)
                return self.failure
            return None

        def stable(self):return self.last is not None and not self.last['suspect']
    return MeasuredMonitor
