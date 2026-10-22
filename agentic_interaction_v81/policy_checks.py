"""Additional relocation preconditions, never replacing frozen safety guards."""


def require_station_actuation(r):
    import numpy as np
    from interaction_runtime.policy import SafetyStop
    if r.supervisor.latched is not None:raise SafetyStop('LATCHED_BEFORE_BASE_MOTION',r.supervisor.latched)
    if r.grasp_reference is not None or max(r.filtered())>r.policy.data['force']['unloaded_pad_n']:
        raise SafetyStop('BASE_MOTION_REQUIRES_VERIFIED_UNLOADED_STATE')
    if not r.supervisor.force.settled():raise SafetyStop('BASE_MOTION_FORCE_HISTORY_NOT_SETTLED')
    if r.g.get('recovery_state')!='UNLOADED_OBSERVATION':raise SafetyStop('BASE_MOTION_OUTSIDE_UNLOADED_BOUNDARY')
    dq=np.asarray(r.robot.get_joint_velocities())[r.base_indices]
    if not np.isfinite(dq).all() or np.linalg.norm(dq)>r.policy.motion('SAFE_PARK_MOTION')['measured_speed_stop_m_s']:
        raise SafetyStop('BASE_MEASURED_SPEED')
