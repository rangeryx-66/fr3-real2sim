"""Preflight every original timed command before any route actuation."""


def check_samples(samples,check):
    count=0
    for q,_ in samples:
        valid,why=check(q)
        if not valid:raise RuntimeError('MOTION_PATH_UNAVAILABLE:EXECUTED_SAMPLE:'+str(why))
        count+=1
    return count


def preflight(r,path,phase,minimum_duration):
    import numpy as np
    from interaction_runtime.motion import timed_path
    r.ensure_execution_budget(phase)
    fingers=np.asarray(r.robot.get_joint_positions())[r.fingers].copy()
    stamp=None
    if getattr(r,'job',{}).get('v86_recovery_supervisor'):
        from .scene_stamp import capture
        stamp=capture(r)
    moving=r.moving().copy()
    allow=r.policy.phase(phase) in ('ESTABLISH','LOADED','RELEASE')
    def check(q):
        if r.model.margin(q)<=r.policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
        return r.collision.check(r.model.poses(q,r.base,finger_q=fingers),moving,allow)
    count=check_samples(timed_path(r.solver,r.base,[r.qtarget[r.arm].copy()]+path,r.dt,
        r.policy.motion(phase),np.asarray(r.c['arm_speed_limits'])/3,
        r.policy.data['arm_command_acceleration_rad_s2'][r.policy.phase(phase)],
        check,minimum_duration),check)
    if stamp is not None:
        from .scene_stamp import capture,same
        latest=capture(r)
        if not same(stamp,latest):raise RuntimeError('MOTION_PATH_UNAVAILABLE:SCENE_CHANGED_DURING_PLANNING')
        r.event('V86_FULL_ROUTE_CURRENT_SCENE_CHECK',scene=stamp,phase=phase,actual_timed_samples=count)
    return count
