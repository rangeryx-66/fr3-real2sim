"""Small downstream action selector; no simulator, asset or hidden-plant IO.

The contact table must come from measured calibration, not requested preload.
This module predicts demand; it never declares a physical slip or task failure.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class DemandModel:
    parameters: tuple  # mass kg, viscous N s/m, Coulomb N
    covariance: tuple
    residual_bound_n: float
    source: str
    breakaway_force_upper_n: float | None = None

    def upper(self, speed, acceleration, *, z=1.96):
        beta=np.asarray(self.parameters,float); cov=np.asarray(self.covariance,float)
        if (beta.shape!=(3,) or cov.shape!=(3,3) or not np.isfinite(beta).all()
                or not np.isfinite(cov).all() or np.any(beta<0)
                or np.linalg.eigvalsh((cov+cov.T)/2).min() < -1e-12
                or not math.isfinite(self.residual_bound_n) or self.residual_bound_n<0):
            raise ValueError('INVALID_MEASUREMENT_DERIVED_DEMAND_MODEL')
        phi=np.array([abs(acceleration),abs(speed),1.])
        dynamic=float(phi@beta+z*np.sqrt(max(0.,phi@cov@phi))+self.residual_bound_n)
        if self.breakaway_force_upper_n is None:return dynamic
        if not math.isfinite(self.breakaway_force_upper_n) or self.breakaway_force_upper_n<0:
            raise ValueError('INVALID_OBSERVED_BREAKAWAY_BOUND')
        return max(dynamic,self.breakaway_force_upper_n)


@dataclass(frozen=True)
class Capability:
    request_n: float
    transmitted_axis_force_lower_n: float
    measured_pad_normal_lower_n: tuple
    measured_pad_normal_upper_n: tuple
    source: str


def rest_to_rest_time(distance, speed, acceleration):
    if not all(math.isfinite(v) and v>0 for v in (distance,speed,acceleration)):
        raise ValueError('POSITIVE_FINITE_TRAJECTORY_REQUIRED')
    return (2*math.sqrt(distance/acceleration) if distance<=speed*speed/acceleration
        else distance/speed+speed/acceleration)


def choose(model, capabilities, *, distance, speeds, accelerations, motion,
           sustained_pad_n, emergency_wrist_n):
    """Fastest feasible candidate, then minimum reliable requested preload.

    No unmeasured high-preload damage/energy penalty is invented. Return the
    Pareto set too; a throughput gain and a lower preload are different claims.
    Native full-path admission and independent per-step guards remain required.
    """
    if not isinstance(model,DemandModel):raise ValueError('EXPLICIT_OBSERVED_MODEL_REQUIRED')
    candidates=[]
    for c in capabilities:
        if (not c.source or not math.isfinite(c.transmitted_axis_force_lower_n)
                or c.transmitted_axis_force_lower_n<0
                or len(c.measured_pad_normal_lower_n)!=2 or len(c.measured_pad_normal_upper_n)!=2):
            raise ValueError('MEASURED_CONTACT_CAPABILITY_REQUIRED')
        lo=np.asarray(c.measured_pad_normal_lower_n,float);hi=np.asarray(c.measured_pad_normal_upper_n,float)
        if not np.isfinite(np.r_[lo,hi]).all() or np.any(lo<0) or np.any(hi<lo):
            raise ValueError('INVALID_PAD_LOAD_CALIBRATION')
        if c.transmitted_axis_force_lower_n==0:continue
        if max(hi)>=sustained_pad_n:continue
        for v in speeds:
            for a in accelerations:
                if not (0<v<=motion['speed_m_s'] and 0<a<=motion['acceleration_m_s2']):continue
                required=model.upper(v,a)
                if required>min(c.transmitted_axis_force_lower_n,emergency_wrist_n):continue
                candidates.append(dict(requested_preload_n=float(c.request_n),reference_speed_m_s=float(v),
                    reference_acceleration_m_s2=float(a),predicted_force_upper_n=required,
                    contact_capacity_lower_n=float(c.transmitted_axis_force_lower_n),
                    predicted_motion_s=rest_to_rest_time(distance,v,a),
                    measured_pad_normal_lower_n=list(c.measured_pad_normal_lower_n),
                    model_source=model.source,contact_source=c.source))
    if not candidates:return dict(status='NO_PREDICTED_FEASIBLE_ACTION',action=None,
        physical_failure_inferred=False,episode_termination_required=False)
    candidates.sort(key=lambda x:(x['predicted_motion_s'],x['requested_preload_n'],x['predicted_force_upper_n']))
    pareto=[x for x in candidates if not any(
        y['predicted_motion_s']<=x['predicted_motion_s'] and y['requested_preload_n']<=x['requested_preload_n']
        and (y['predicted_motion_s']<x['predicted_motion_s'] or y['requested_preload_n']<x['requested_preload_n'])
        for y in candidates)]
    return dict(status='PREDICTED_ACTION_REQUIRES_NATIVE_CHECK',action=candidates[0],pareto=pareto,
        physical_failure_inferred=False,episode_termination_required=False,
        high_preload_damage_or_energy_cost_assumed=False)
