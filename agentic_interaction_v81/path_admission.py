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
    moving=r.moving().copy()
    allow=r.policy.phase(phase) in ('ESTABLISH','LOADED','RELEASE')
    def check(q):
        if r.model.margin(q)<=r.policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
        return r.collision.check(r.model.poses(q,r.base,finger_q=fingers),moving,allow)
    return check_samples(timed_path(r.solver,r.base,[r.qtarget[r.arm].copy()]+path,r.dt,
        r.policy.motion(phase),np.asarray(r.c['arm_speed_limits'])/3,
        r.policy.data['arm_command_acceleration_rad_s2'][r.policy.phase(phase)],
        check,minimum_duration),check)
