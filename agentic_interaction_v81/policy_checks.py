"""Additional relocation preconditions, never replacing frozen safety guards."""


def settle_station(r,maximum_s=2.,stable_s=.3):
    """Finite unloaded hold; observe the same strict bounds continuously.

    Original parking position tolerance alone does not imply low joint speed.
    Do not drive back from outside the verified cell or wait under contact.
    Every hold step still executes the original collision/force/joint guards.
    """
    import math
    b=r.station_boundary(record=False)
    target=r.g.get('last_exit_plan',{}).get('parking_q',r.c['home_q'])
    def in_cell(state):
        return len(target)==len(state.arm_q) and max(abs(x-y) for x,y in zip(state.arm_q,target))<=.025
    if not in_cell(b):
        raise ValueError('STATION_SETTLE_REQUIRES_MEASURED_PARK_CELL')
    def unloaded(state):
        return state.safety_latch is None and state.released and state.unloaded_verified and state.clearance_verified
    if not unloaded(b):
        b.check()  # no new hold/contact motion; preserve the original refusal
    stable=0.
    for _ in range(math.ceil(maximum_s/r.dt)):
        r.ensure_execution_budget('PREGRASP')  # never spend exit reserve
        r.hold(r.dt)
        b=r.station_boundary(record=False)
        if not unloaded(b):b.check()
        if not in_cell(b):raise ValueError('STATION_SETTLE_LEFT_MEASURED_PARK_CELL')
        stable=stable+r.dt if b.robot_stationary and b.object_stationary else 0.
        if stable>=stable_s-1e-12:return b
    raise ValueError('STATION_STABLE_DWELL_NOT_OBSERVED_WITHIN_FINITE_HOLD')


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
