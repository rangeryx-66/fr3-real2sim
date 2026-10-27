"""SI motion evidence and recorder schema for the explicitly audited V87 policy.

An object's scalar rad/s and m/s cannot share a stationary cutoff. Convert
current measured motion through the SAME known articulation used by planning.
Use the existing stationary TCP linear/angular cells and dwell duration; no
new numeric safety envelope, manual pose, plant dynamics or command is added.
"""
import numpy as np
from agentic_interaction_v81.motion_dwell import MotionDwell


def articulation_prior(r):
    from native_g1_ft_identification import kinematic_prior
    from interaction_runtime.articulation_v7 import Articulation
    p=kinematic_prior(r.g)
    return Articulation(p['joint_type'],tuple(p['axis_world']),
        tuple(p['origin_world_m']),tuple(r.job['articulation_v7']['limits_native']))


class PhysicalMotionDwell(MotionDwell):
    def __init__(self,dt,articulation,duration=.3):
        super().__init__(dt,duration)
        self.articulation_source=articulation
        self.articulation=None
        self.physical_stable_s=0.

    def update(self,t,q,tcp,coordinate):
        previous=self.previous
        out=super().update(t,q,tcp,coordinate)
        rates=out['rates'];linear=angular=None
        if rates is not None and previous is not None:
            if self.articulation is None:self.articulation=self.articulation_source()
            point=np.r_[np.asarray(tcp)[:3,3],1.]
            delta=float(coordinate)-previous[3]
            linear=float(np.linalg.norm((self.articulation.delta(delta)@point-point)[:3])/self.dt)
            angular=abs(delta)/self.dt if self.articulation.joint_type=='revolute' else 0.
        stable=(rates is not None and linear is not None and np.isfinite([linear,angular]).all()
            and rates['joint_max_rad_s']<.01 and rates['TCP_translation_m_s']<.001
            and rates['TCP_rotation_rad_s']<.01 and linear<.001 and angular<.01)
        self.physical_stable_s=self.physical_stable_s+self.dt if stable else 0.
        self.stable_s=self.physical_stable_s
        out.update(stable_s=self.stable_s,ready=self.stable_s>=self.duration-1e-10,
            object_path_speed_m_s=linear,object_angular_speed_rad_s=angular,
            native_scalar_cutoff_is_not_a_physical_speed=True,
            source='same240Hz causal configurations; object motion in stationary TCP SI cells through known articulation')
        self.latest=out
        return out


def representable_progress(q0,q1,p0,p1,scale):
    """Conservative endpoint float32 quantization budget, not a minimum speed.

    Native state/poses are float32; reserve one ULP per endpoint/component.
    Actual motion below this representation budget cannot prove transmission.
    Sensor noise is not modeled here; no claim of hardware sensor resolution.
    """
    scalar=sum(abs(float(np.spacing(np.float32(q)))) for q in (q0,q1))*scale
    spatial=float(np.linalg.norm(abs(np.spacing(np.asarray(p0,np.float32)))+
                                  abs(np.spacing(np.asarray(p1,np.float32)))))
    return max(scalar,spatial)


def diagnostic_fields(r):
    governor=getattr(r,'coupled_governor',None) or getattr(r,'command_governor',None)
    m=getattr(governor,'latest',{})
    monitor=getattr(getattr(r,'supervisor',None),'force',None)
    feedback=getattr(monitor,'feedback',{})
    return dict(v87_constraint_speed_scale=float(feedback.get('scale',np.nan)),
        v87_load_regulation_active=bool(feedback.get('load_regulation_active',False)),
        v87_command_linear_speed_m_s=float(m.get('linear_speed_m_s',np.nan)),
        v87_command_linear_acceleration_m_s2=float(m.get('linear_acceleration_m_s2',np.nan)),
        v87_command_governor_limited=bool(m.get('limited',False)),
        v87_command_governor_valid=bool(m),
        v87_predictive_reference_scale=float(getattr(r,'v87_predictive_feedback',{}).get('scale',1.)),
        v87_predictive_advisory_active=bool(getattr(r,'v87_predictive_feedback',{}).get('active',False)))


def resource_forecast(progress,window_s,remaining,window_wall_s,resources):
    """A task strategy decision, never a physical hazard or a new timeout.

    Use actual measured throughput against already registered finite resources.
    The caller requires two complete corroborating windows; a one-window slow
    warning cannot end a useful action. No assumed dynamics or speed floor.
    """
    values=[progress,window_s,remaining,window_wall_s,resources['wall_s'],resources['sim_s']]
    if not np.isfinite(values).all() or min(progress,window_s,remaining,window_wall_s)<=0:
        return None
    eta=remaining*window_s/progress
    wall_eta=eta*window_wall_s/window_s
    if eta<=resources['sim_s'] and wall_eta<=resources['wall_s']:return None
    return dict(remaining_sim_ETA_s=eta,remaining_wall_ETA_s=wall_eta,
        remaining_resources=resources,classification='CURRENT_STRATEGY_EXCEEDS_REGISTERED_RESOURCES',
        physical_hazard=False,authority='SAFE_PARTIAL_PROGRESS_TO_AGENT')
