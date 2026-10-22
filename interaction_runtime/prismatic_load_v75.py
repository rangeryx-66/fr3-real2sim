"""Causal stopping-margin advisory using the existing sustained load envelope.

It cannot relax or reset a hard stop. A predictive stop remains an interrupted
action, not target success. No centered filter or future sample is used.
"""
from collections import deque
import numpy as np


class PrismaticLoadMarginStop(RuntimeError):
    def __init__(self,evidence):
        self.evidence=evidence;super().__init__('PREDICTED_LOAD_MARGIN_REQUIRES_UNLOAD')


class LoadTrend:
    def __init__(self,policy):
        self.force=policy.data['force'];self.rows=deque()

    def update(self,t,dt,pads,wrench,stopping_time_s):
        w=np.asarray(wrench,float);x=np.array([max(pads),np.linalg.norm(w[:3]),np.linalg.norm(w[3:])])
        self.rows.append((t,dt,x))
        horizon=self.force['sustained_s']
        while self.rows and self.rows[0][0]<t-horizon+dt*.5:self.rows.popleft()
        duration=sum(r[1]for r in self.rows)
        if duration<horizon-dt*.5:return None
        mean=sum(r[1]*r[2]for r in self.rows)/duration
        times=np.array([r[0]for r in self.rows]);times-=times.mean()
        values=np.array([r[2]for r in self.rows])
        slope=times@(values-values.mean(0))/float(times@times)
        limit=np.array([self.force['pad_sustained_n'],self.force['wrist_sustained_n'],self.force['torque_sustained_nm']])
        # Predict the SAME trailing guard window at the end of deceleration:
        # retain past measured samples still in that window, append a declared
        # linear continuation of the current measured norm. This is a forecast,
        # not a future-filtered protection measurement or a changed load band.
        steps=max(1,int(np.ceil(stopping_time_s/dt)));count=len(values)
        retained=values[steps:]if steps<count else np.empty((0,3))
        first=max(1,steps-count+1);number=steps-first+1
        forecast=number*x+np.maximum(slope,0)*dt*(first+steps)*number/2
        projected=(retained.sum(0)+forecast)/count
        risk=(projected>=limit)&(slope>0)&(mean<limit)
        if not risk.any():return None
        return dict(reason='PREDICTED_LOAD_MARGIN_REQUIRES_UNLOAD',t=float(t),
            channels=['pad_N','wrist_force_N','sensor_origin_torque_Nm'],
            affected=np.flatnonzero(risk).tolist(),causal_window_mean=mean.tolist(),
            causal_window_slope_per_s=slope.tolist(),stopping_horizon_s=float(stopping_time_s),
            current_measured_norms=x.tolist(),forecast='same causal guard window at stopping horizon; linear raw-load continuation',
            projected_window_mean=projected.tolist(),unchanged_sustained_limits=limit.tolist(),
            projection_is_not_a_safety_guarantee=True,hard_guard_authoritative=True)
