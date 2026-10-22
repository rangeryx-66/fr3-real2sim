"""Measured-cost recovery admission before acquiring another loaded grasp.

Finite forecasts are resource scheduling, not guarantees of contact success or
larger execution deadlines. All native guards and adaptive live budgets remain.
"""
import numpy as np
from .prismatic_lifecycle_v75 import finite_admission
from .motion import timed_path
from .segments import segment_plan


def admit(r,candidate,actions,check):
    from native_g1_agent_execution import validate_execution
    from xarm7_native_gripper import GRIPPER_JOINTS
    g=r.g;c=r.v75_config;dt=r.dt;speed=np.asarray(r.c['arm_speed_limits'])/3
    transition=validate_execution(g,candidate=candidate,write=False)
    if not transition['valid']:return dict(admit=False,status='CURRENT_TRANSITION_INVALID',physical_failure=False)
    fingers=np.asarray(r.robot.get_joint_positions())[r.fingers].copy();M=r.moving().copy()
    def counted(path,phase,moving,finger_q):
        def okay(q):
            if r.model.margin(q)<=r.policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
            return r.collision.check(r.model.poses(q,r.base,finger_q=finger_q),moving,False)
        return sum(1 for _ in timed_path(r.solver,r.base,path,dt,r.policy.motion(phase),speed,
            r.policy.data['arm_command_acceleration_rad_s2'][r.policy.phase(phase)],okay))*dt
    # Execute() returns waypoint-by-waypoint; count that ACTUAL time law rather
    # than treating its whole route as one continuous spline.
    returning=[r.qtarget[r.arm].tolist()]+transition['return_path'][2:]
    approach=sum(counted([a,b],'PREGRASP',M,fingers)for a,b in zip(returning,returning[1:]))
    approach+=counted(transition['approach_audit']['preplan'],'PREGRASP',M,fingers)
    approach+=counted(transition['approach_audit']['approach'],'APPROACH',M,fingers)
    profile=r.profile
    observations=profile.calls.get('RGBD_SAM3_and_incremental_geometry',0)
    if observations<=0:return dict(admit=False,status='PERCEPTION_COST_NOT_MEASURED',physical_failure=False)
    refine_wall=profile.inclusive['RGBD_SAM3_and_incremental_geometry']/observations
    ratio=c['active_wall_per_sim_s_upper'];excess=c['healthy_segment_additive_excess_sim_s'];costs={}
    def packet(sim,source,extra_wall=0):return dict(available=True,sim_s=float(sim),wall_s=float(sim*ratio+extra_wall),source=source)
    costs['approach']=packet(approach,'actual current route timed as executed; native collision checks; measured observation cost for pregrasp refinement',refine_wall)
    costs['closure_hold']=packet(c['closure_hold_sim_s_bound'],c['closure_bound_source'])
    duration={'FIT':0.,'VALIDATION':0.};start=r.actual();motion=dict(r.policy.motion('LOADED'))
    motion.update(angular_speed_rad_s=motion['speed_m_s'],angular_acceleration_rad_s2=motion['acceleration_m_s2'])
    available=r.episode_budget.remaining(r.tick*dt)
    lookahead=motion['speed_m_s']**2/(2*motion['acceleration_m_s2'])+motion['speed_m_s']*dt
    for action in actions:
        target=float(action['target_q']);p=segment_plan(start,target,action,motion,1.,dt,available,lookahead)
        if not p['valid']:return dict(admit=False,status='PLANNED_SEGMENT_HAS_NO_FINITE_EXECUTION_BUDGET',planning=p,physical_failure=False)
        role='VALIDATION'if action['role'].startswith('VALIDATION')else'FIT'
        duration[role]+=p['nominal_duration_s']+excess;start=target
    for role in duration:costs[role]=packet(duration[role],'unchanged ProgressClock nominal + maximum measured healthy additive execution excess, all eight V7.4 episodes')
    chain=check['v75_whole_chain'];exit_plan=chain['exit'];end_M=np.asarray(chain['final_predicted_moving_pose'])
    released=np.array([r.model.native.values(r.model.native.angle(.084))[n]for n in GRIPPER_JOINTS])
    exit_sim=counted(exit_plan['clearance_path'],'CLEARANCE_RETREAT',end_M,released)
    exit_sim+=counted(exit_plan['parking_path'],'SAFE_PARK_MOTION',end_M,released)
    exit_sim+=.084/r.policy.data['jaw_release_m_s']+1.6  # maximum native jaw stroke, not CURRENT unloaded aperture
    costs['exit']=packet(exit_sim,'current predicted release/clearance/rest timed route; existing release settling and 60 s worker-close bound',exit_plan['planning_wall_s']+60.)
    available.update(reserved_wall_s=r.episode_budget.reserve_wall,reserved_sim_s=r.episode_budget.reserve_sim)
    result=finite_admission(available,costs)
    result.update(forecast_not_guarantee=True,exact_episode_deadline_unchanged=True,
        cost_prior_sha256=c['cost_prior_sha256'],includes_complete_validation=True)
    return result
