"""Native no-contact safety regression, including an independently applied TOOL load.

The cabinet is never forced or actuated. Known loads below are test excitation
on the gripper body only, not manipulation assistance or estimator training.
"""
import json,math
import numpy as np
from .policy import SafetyStop

def run(r):
    r.g['phase']='SETTLE';r.hold(.3);tests=[]
    command=r.qtarget[r.arm].copy();ticks=r.tick
    try:
        r.qtarget[r.arm[0]]=r.model.limits[0,1]+.1;r.step()
        raise RuntimeError('Unsafe joint command was accepted')
    except SafetyStop as e:tests.append(dict(test='joint_limit_command',stop=e.reason,physics_steps_advanced=r.tick-ticks))
    finally:r.qtarget[r.arm]=command
    v=r.g['qvelocity'].copy();r.g['qvelocity']=np.asarray(r.c['arm_speed_limits'])*2
    try:
        r.step();raise RuntimeError('Unsafe velocity command was accepted')
    except SafetyStop as e:tests.append(dict(test='velocity_command',stop=e.reason,physics_steps_advanced=r.tick-ticks))
    finally:r.g['qvelocity']=v
    rng=np.random.default_rng(1831);bad=None
    for _ in range(80):
        q=rng.uniform(r.model.limits[:,0]+.06,r.model.limits[:,1]-.06);ok,why=r.check_q(q,False)
        if not ok and 'COLLISION' in why:bad=q;break
    if bad is None:raise RuntimeError('No native colliding test configuration found; do not fabricate collision validation')
    ticks=r.tick
    rejected=None
    try:r.move_path([bad],'PREGRASP')
    except RuntimeError as e:rejected=repr(e)
    if rejected is None or r.tick!=ticks:raise RuntimeError('Invalid native collision path was not stopped before actuation')
    tests.append(dict(test='native_collision_route',native_pair=why,rejection=rejected,physics_steps_advanced=0))
    # Native authored finite caps are read back, never inferred from explicit
    # zero effort or projected joint reactions.
    caps=np.asarray(r.robot._articulation_view.get_max_efforts()).reshape(-1)
    if not np.isfinite(caps).all() or np.any(caps[r.arm]<=0) or not np.allclose(caps[r.arm],r.c['arm_effort_limits'],rtol=1e-6):raise RuntimeError('Invalid finite effort caps')
    tests.append(dict(test='native_finite_effort_readback',arm_caps_nm=caps[r.arm].tolist(),master_cap_nm=float(caps[r.controller.master]),expected_arm_caps_nm=r.c['arm_effort_limits'],motor_output_claim=False))
    load_view=r.actual_views['xarm_gripper_base_link'];force=np.zeros(3)
    def apply():
        p=r.pose('xarm_gripper_base_link')[:3,3]
        load_view.apply_forces_and_torques_at_pos(forces=force[None],torques=np.zeros((1,3)),positions=p[None],is_global=True)
    r.engineering_before_physics=apply;r.engineering_load=force;r.g['phase']='SETTLE';force[:]=[0,0,4.2]
    start=r.tick;peak=0.
    for _ in range(48):
        row=r.step();peak=max(peak,row['force_event']['wrist_force_n'])
    tests.append(dict(test='native_wrist_informational_load',applied_to='robot gripper body, never object',known_force_world_n=force.tolist(),measured_peak_wrist_n=peak,continued_steps=r.tick-start))
    force[:]=0.;r.hold(.3);force[:]=[0,0,11.]
    stopped=None;start=r.tick
    try:
        for _ in range(48):r.step()
    except SafetyStop as e:stopped=dict(reason=e.reason,evidence=e.evidence,steps_to_stop=r.tick-start)
    finally:r.engineering_before_physics=None;force[:]=0.
    if stopped is None:raise RuntimeError('Physical wrist overload was not stopped')
    tests.append(dict(test='native_wrist_emergency_load',applied_to='robot gripper body only',known_force_world_n=[0,0,11.],stop=stopped))
    result=dict(status='NATIVE_GUARD_REGRESSION_COMPLETE',tests=tests,physical_object_contact=False,object_force_applied=False,force_test_samples_not_for_identification=True)
    (r.output/'NATIVE_GUARD_REGRESSION.json').write_text(json.dumps(result,indent=2));return result
