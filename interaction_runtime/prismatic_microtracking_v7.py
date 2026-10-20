"""Finite robot-only tracking diagnostic; no grasp, object drive or perception.

An archived *observed* automatic plan supplies only an unloaded pregrasp test
configuration. The route is freshly checked; no contact pose is executed. This
is an engineering fixture, never an Agent or benchmark perception result.
"""
import json
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json
from .prismatic_feedback_v7 import PrismaticFeedbackOperations


def run(r):
    ops=PrismaticFeedbackOperations(r);r.optional_recording_disabled=True
    if r.job.get('prismatic_contact_retention'):
        raise ValueError('Robot-only diagnostic must not fabricate contact feedback')
    candidate=json.loads(Path(r.job['engineering_pregrasp_reference']).read_text())
    r.g['phase']='SETTLE';r.hold(.5)
    pre=np.array(candidate['T']);pre[:3,3]-=.04*pre[:3,2]
    measured=np.asarray(r.robot.get_joint_positions())[r.arm]
    q=r.solver.solve(pre,r.base,np.asarray(candidate['preplan'][-1]))
    if q is None:return dict(status='MICROTRACKING_PREGRASP_IK_UNAVAILABLE',grasp_attempts=0)
    from native_g1_safe_park import route_to_joint
    path,reason=route_to_joint(r.g,measured,q)
    if path is None:return dict(status='MICROTRACKING_PREGRASP_ROUTE_UNAVAILABLE',reason=reason,grasp_attempts=0)
    friction_readbacks={}
    for name in ['get_dof_friction_properties','get_dof_friction_coefficients']:
        try:
            getter=getattr(r.robot._articulation_view._physics_view,name)
            friction_readbacks[name]=dict(available=True,raw=np.asarray(getter()).tolist(),API_documentation=getter.__doc__)
        except Exception as error:friction_readbacks[name]=dict(available=False,reason=repr(error))
    atomic_json(r.output/'MICROTRACKING_PREFLIGHT.json',dict(
        fixture='archived automatic observed plan, unloaded pregrasp only',
        reference=str(r.job['engineering_pregrasp_reference']),
        fresh_native_route=True,grasp_or_door_contact_requested=False,
        source_of_force='native wrist fixed-interface F/T; no artificial load',
        physics_hz=240,robot_actuator_and_friction_unchanged=True,
        native_friction_readbacks=friction_readbacks))
    r.move_path(path[1:],'PREGRASP');r.g['phase']='APPROACH';r.hold(.5)
    axis=[]
    for native,logical in [('left_finger','gripper_link1'),('right_finger','gripper_link2')]:
        T=r.pose(native)@r.pad_frames[native]
        axis.append((T@np.r_[r.model.pads[logical].mean(0),1])[:3])
    rail=np.array(r.contact_evidence.axis);rail/=np.linalg.norm(rail)
    normal=axis[0]-axis[1];normal-=rail*(normal@rail);normal/=np.linalg.norm(normal)
    directions=[('closing_axis',normal),('bar_tangent',np.cross(rail,normal))]
    samples=[];r.record.calibration=True
    try:
        for name,direction in directions:
            origin=r.tcp().copy();r.reference_drive=None;r.begin_reference()
            # Exactly the existing 5um step once per 19-sample force window.
            period=r.filtered_loads.maxlen;n=20;amplitude=n*r.closure_policy['max_centering_step_m']
            for leg in ['out','back']:
                first=r.tcp().copy();initial_command=r.rows[-1].get('command_TCP',r.tcp()).copy()
                start=r.tick*r.dt
                for tick in range(n*period):
                    fraction=(tick//period+1)/n
                    displacement=amplitude*(fraction if leg=='out' else 1-fraction)
                    goal=origin.copy();goal[:3,3]+=displacement*direction
                    r.cartesian_step(goal,'CONTACT_PRELOAD')
                r.stop_command()
                # A predeclared diagnostic dwell may match the existing finite
                # breakaway loading horizon; it is not a drawer action timeout.
                dwell=float(r.job.get('microtracking_dwell_s',.3))
                for _ in range(int(round(dwell/r.dt))):r.cartesian_step(goal,'CONTACT_PRELOAD')
                r.stop_command()
                actual=r.tcp();command=r.rows[-1]['command_TCP']
                samples.append(dict(axis=name,leg=leg,start_sim_s=start,end_sim_s=r.tick*r.dt,
                    requested_displacement_m=amplitude*(1 if leg=='out' else -1),
                    measured_displacement_m=float((actual[:3,3]-first[:3,3])@direction),
                    commanded_FK_displacement_m=float((command[:3,3]-initial_command[:3,3])@direction),
                    diagnostic_dwell_s=dwell,
                    raw_pad_loads_n=list(r.rows[-1]['forces_n'].values()),
                    reference_world_direction=direction.tolist()))
    finally:r.record.calibration=False
    r.reference_drive=None
    exit_result=ops.release_park()
    ops.close_workers()
    result=dict(status='ROBOT_ONLY_MICROTRACKING_COMPLETE',grasp_attempts=0,
                object_joint_commands=False,segments=samples,safe_exit=exit_result)
    atomic_json(r.output/'MICROTRACKING_RESULT.json',result)
    return result
