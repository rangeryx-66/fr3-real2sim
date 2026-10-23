"""Causal stationary evidence from consecutive measured physics states.

Native projected articulation velocity may differ from integrated configuration
motion after constraint stabilization. Retain it separately; this certificate
does not replace any original per-step velocity, collision or force protection.
"""
import numpy as np


class MotionDwell:
    def __init__(self, dt, duration=.3):
        self.dt=float(dt);self.duration=float(duration);self.previous=None
        self.stable_s=0.;self.latest=None

    def update(self,t,q,tcp,coordinate):
        q=np.asarray(q,dtype=float).copy();tcp=np.asarray(tcp,dtype=float).copy()
        current=(float(t),q,tcp,float(coordinate));rates=None
        finite=q.shape==(7,) and tcp.shape==(4,4) and all(np.isfinite(x).all() for x in current)
        if finite and self.previous is not None:
            t0,q0,T0,x0=self.previous;elapsed=t-t0
            if abs(elapsed-self.dt)<=self.dt*1e-5:
                R=T0[:3,:3].T@tcp[:3,:3]
                rates=dict(joint_max_rad_s=float(np.max(abs(q-q0))/elapsed),
                    TCP_translation_m_s=float(np.linalg.norm(tcp[:3,3]-T0[:3,3])/elapsed),
                    TCP_rotation_rad_s=float(np.arctan2(np.linalg.norm(R-R.T)/2**1.5,
                                                       (np.trace(R)-1)/2)/elapsed),
                    object_native_s=abs(coordinate-x0)/elapsed)
        stable=rates is not None and rates['joint_max_rad_s']<.01 and rates['TCP_translation_m_s']<.001 and rates['TCP_rotation_rad_s']<.01 and rates['object_native_s']<1e-4
        self.stable_s=self.stable_s+self.dt if stable else 0.
        self.previous=current if finite else None
        self.latest=dict(t=float(t),stable_s=self.stable_s,rates=rates,
            ready=self.stable_s>=self.duration-1e-10,
            source='consecutive native240Hz measured joint/TCP/object configurations; causal, no future samples')
        return self.latest

    def ready(self,t,q):
        return bool(self.latest and self.latest['ready'] and self.previous is not None
            and abs(float(t)-self.previous[0])<self.dt*1e-5
            and np.max(abs(np.asarray(q)-self.previous[1]))<1e-7)
