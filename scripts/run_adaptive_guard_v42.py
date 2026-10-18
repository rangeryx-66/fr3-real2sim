"""V4.2 native robot-only timing/transition/safety regression; not door contact."""
import sys,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v3 import RuntimeV3
from interaction_runtime.operations import Operations


def run(r):
    ops=Operations(r);home=r.qtarget[r.arm].copy();r.g['phase']='SETTLE';r.hold(.25);T0=r.tcp().copy();axis=np.array([0.,0.,1.]);origin=T0[:3,3]+[.2,0,0];initial=T0[:3,3]-origin
    prior=dict(joint_type='revolute',axis_world=axis.tolist(),origin_world_m=origin.tolist())
    def measured_virtual_angle():
        offset=r.tcp()[:3,3]-origin
        return math.atan2(axis@np.cross(initial,offset),initial@offset)
    # Validate the whole actual free-space arc with existing predicates first.
    seed=np.asarray(r.robot.get_joint_positions())[r.arm]
    for angle in np.linspace(0,math.radians(13),41):
        R=Rotation.from_rotvec(axis*angle).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin
        seed=r.solver.solve(D@T0,r.base,seed)
        if seed is None:raise RuntimeError('ENGINEERING_FREE_ARC_IK_UNAVAILABLE')
        ok,why=r.check_q(seed,False)
        if not ok:raise RuntimeError(why)
    r.record.metadata['scope']='ROBOT_ONLY: door_q is real unchanged object state; reference_q is declared VIRTUAL free-space arc, not door manipulation'
    results=[]
    for target in [13.,0.]:
        action=dict(role='ENGINEERING_UNLOADED_ARC',target_deg=target,profile='constant',velocity_program='constant',reference_speed_m_s=(.001 if target==13. else .008),reference_acceleration_m_s2=.04,hold=True)
        r.probe_profile=action;r.profile_started=r.tick;r.record.begin(action,'UNLOADED')
        try:result=ops.rotate_segment(target,action=action,prior=prior,measure=measured_virtual_angle)
        finally:r.probe_profile=None
        r.record.end(result);results.append(result)
        if not result['ok']:raise RuntimeError('NATIVE_ARC_INCOMPLETE:'+str(result))
        r.hold(.1)
    # A frozen virtual feedback fault must brake, then timeout, never claim goal.
    action=dict(role='ENGINEERING_STALLED_FEEDBACK',target_deg=5.,profile='constant',reference_speed_m_s=.008,reference_acceleration_m_s2=.04,max_sim_s=8.,hold=True)
    r.probe_profile=action;r.profile_started=r.tick;r.record.begin(action,'UNLOADED')
    result=ops.rotate_segment(5.,action=action,prior=prior,measure=lambda:0.)
    r.probe_profile=None;r.record.end(result);results.append(result)
    if result['ok'] or result['status']!='TRANSMISSION_FAILURE' or not result['safely_stopped']:raise RuntimeError('STALLED_FEEDBACK_NOT_STOPPED')
    r.move_path([home],'SAFE_PARK_MOTION');r.hold(.25)
    park=dict(command_q=home.tolist(),measured_q=np.asarray(r.robot.get_joint_positions())[r.arm].tolist(),pad_loads=r.filtered().tolist())
    # Explicit resource exhaustion at an unloaded boundary must prohibit new
    # work while retaining normal, collision-checked actuator parking.
    from interaction_runtime.segments import ExecutionBudgetExhausted
    r.episode_budget.wall=r.episode_budget.now()-r.episode_budget.started+r.episode_budget.reserve_wall-.01
    before=r.tick;rejected=False
    try:r.move_path([home],'PREGRASP')
    except ExecutionBudgetExhausted:rejected=True
    if not rejected or r.tick!=before:raise RuntimeError('RESOURCE_RESERVE_DID_NOT_BLOCK_NEW_WORK')
    r.move_path([home],'SAFE_PARK_MOTION');r.hold(.3)
    guarded=dict(budget_exit_only=r.budget_exit_only,new_work_rejected_before_physics=True,validated_parking_still_executable=True,safety_fault_injection='separate native_contact_02: no redundant overload injection here')
    return dict(status='NATIVE_ADAPTIVE_SHORT_COMPLETE',scope='ENGINEERING_ROBOT_ONLY_NO_DOOR_CONTACT_NO_ESTIMATOR_FIT',results=results,park_before_deliberate_guard_faults=park,guards=guarded,virtual_axis=prior)

if __name__=='__main__':sys.exit(main(RuntimeV3,run,extra_sources=('scripts/run_adaptive_guard_v42.py','scripts/update_probing_v31_model.py','scripts/xarm_ft_v31_estimator.py')))
