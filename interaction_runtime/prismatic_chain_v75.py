"""Current-state prismatic whole-chain native collision feasibility.

Observed grasp scores are untouched. The known rail prior is used only for
motion/safety prediction. No hidden dynamic parameter enters this module.
The authorized CENTER envelope and original preload mapping are checked through
both directions, then original guarded release and parking are planned.
Predictions remain conditional: measured-state checks continue during execution.
"""
from types import SimpleNamespace
import numpy as np


def rail_lead_offsets(policy,dt):
    """Same two finite allowances used by PrismaticStartup.clock_feedback.

    The commanded robot can lead the actual drawer; translating both by the
    reference amount silently erased this relative collision envelope in R1.
    No lead allowance, controller or physical safety margin is changed here.
    """
    m=policy.motion('LOADED');one=m['speed_m_s']**2/(2*m['acceleration_m_s2'])+m['speed_m_s']*dt
    return [0.,-2*one,2*one]


def relative_lead_check(g,goal,seed,moving,finger_options,policy,solver,bias):
    axis=np.asarray(g['v75_motion_prior']['axis_world'],float);axis/=np.linalg.norm(axis)
    checked=0
    for lead in rail_lead_offsets(policy,g['dt']):
        if lead==0:continue
        target=np.array(goal,copy=True);target[:3,3]+=lead*axis
        q=solver.solve(target,g['base'],seed,starts=1)
        if q is None:return dict(valid=False,reason='RELATIVE_LEAD_IK',rail_lead_m=lead,checked=checked)
        for command in (q,q+bias):
            if g['model'].margin(command)<=policy.data['joint_margin_rad']:
                return dict(valid=False,reason='RELATIVE_LEAD_JOINT_MARGIN',rail_lead_m=lead,checked=checked)
            for option in finger_options:
                poses=g['model'].poses(command,g['base'],**option)
                ok,why=g['collision'].check(poses,moving,True);checked+=1
                if not ok:return dict(valid=False,reason='RELATIVE_LEAD_NATIVE_COLLISION:'+why,rail_lead_m=lead,checked=checked)
    return dict(valid=True,checked=checked,rail_lead_offsets_m=rail_lead_offsets(policy,g['dt']),
        object_pose_not_moved_with_command_lead=True)


def evaluate_chain(g,candidate,actions,policy,config,solver):
    from .articulation_v7 import Articulation
    from .segments import validate_action
    from .prismatic_park_v75 import plan_exit
    from .prismatic_planning_v73 import predicted_aperture
    from xarm7_native_gripper import GRIPPER_JOINTS
    if not actions:return dict(valid=False,reason='DECLARED_MINIMUM_CHAIN_REQUIRED')
    prior=g['v75_motion_prior'];motion=Articulation('prismatic',tuple(prior['axis_world']),tuple(prior['origin_world_m']),tuple(g['job']['articulation_v7']['limits_native']))
    for action in actions:validate_action(action);motion.validate_target(action)
    if not any(a['role'].startswith('VALIDATION')for a in actions):return dict(valid=False,reason='COMPLETE_VALIDATION_NOT_PLANNED')
    model=g['model'];base=g['base'];M=np.asarray(g['moving']());start=float(g['actual']());T=np.asarray(candidate['T'])
    initial=np.asarray(candidate['approach'][-1]);joints=np.asarray(g['robot'].get_joint_positions());bias=g['qtarget'][g['arm']]-joints[g['arm']]
    aperture=predicted_aperture(candidate);angle=model.native.angle(aperture)
    # The existing legal normal-load REQUEST is converted to position exactly
    # as in Operations.acquire. It is not assumed to be measured pad force.
    request=float(g['policy']['preload_n'])
    preload_angle=float(np.clip(angle+request*abs(model.native.width_derivative(angle))/config['gripper_kp_nm_rad'],0,.85))
    widths=[aperture,model.native.width(preload_angle)]
    P=model.poses(initial,base,width=aperture);centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3]for n in g['FINGERS']]
    jaw_axis=centers[1]-centers[0];jaw_axis/=np.linalg.norm(jaw_axis)
    bound=float(g['policy']['max_centering_displacement_m']);checked=0;records=[];final=None
    # Include every authorized centering extremum throughout pull AND reversal.
    for center in [0.,-bound,bound]:
        seed=initial.copy();previous=start
        for action in actions:
            target=float(action['target_q']);speed=min(float(action.get('reference_speed_m_s',policy.motion('LOADED')['speed_m_s'])),policy.motion('LOADED')['speed_m_s'])
            acceleration=min(float(action.get('reference_acceleration_m_s2',policy.motion('LOADED')['acceleration_m_s2'])),policy.motion('LOADED')['acceleration_m_s2'])
            stopping=speed*speed/(2*acceleration)+speed*g['dt']
            direction=np.sign(target-previous)
            # Native-coordinate stopping envelope; do not cross a physical limit.
            endpoint=target+direction*stopping
            if not motion.limits[0]<=endpoint<=motion.limits[1]:return dict(valid=False,reason='CHAIN_STOPPING_ENVELOPE_JOINT_LIMIT',role=action['role'])
            values=np.linspace(previous,endpoint,max(2,int(np.ceil(abs(endpoint-previous)/.001))+1))
            last_q=None
            for value in values:
                delta=motion.delta(float(value)-start);goal=delta@T;goal[:3,3]+=center*jaw_axis
                seed=solver.solve(goal,base,seed,starts=1)
                if seed is None:return dict(valid=False,reason='CHAIN_CURRENT_STATE_IK',role=action['role'],center_m=center,coordinate_m=float(value),checked=checked)
                for q in (seed,seed+bias):
                    if model.margin(q)<=policy.data['joint_margin_rad']:return dict(valid=False,reason='CHAIN_RETAINED_BIAS_JOINT_MARGIN',role=action['role'],checked=checked)
                    for width in widths:
                        ok,why=g['collision'].check(model.poses(q,base,width=width),delta@M,True);checked+=1
                        if not ok:return dict(valid=False,reason='CHAIN_NATIVE_COLLISION:'+why,role=action['role'],center_m=center,coordinate_m=float(value),checked=checked)
                relative=relative_lead_check(g,goal,seed,delta@M,[dict(width=w)for w in widths],policy,solver,bias)
                checked+=relative['checked']
                if not relative['valid']:return dict(valid=False,reason=relative['reason'],role=action['role'],center_m=center,coordinate_m=float(value),relative_lead=relative,checked=checked)
                # Exact stopped target is reconstructed after the envelope.
                if abs(value-target)<1e-9:last_q=seed.copy()
            delta=motion.delta(target-start);goal=delta@T;goal[:3,3]+=center*jaw_axis
            seed=solver.solve(goal,base,seed,starts=1)
            if seed is None:return dict(valid=False,reason='CHAIN_STOPPED_TARGET_IK',checked=checked)
            previous=target
            records.append(dict(role=action['role'],target_m=target,stopping_distance_m=stopping,center_m=center))
        if center==0:final=(seed.copy(),delta@M,goal.copy())
    q,Mend,Tend=final
    # Same complete jaw-release sweep, no handle collision waiver change.
    for width in np.linspace(min(widths),.084,35):
        ok,why=g['collision'].check(model.poses(q,base,width=float(width)),Mend,True)
        if not ok:return dict(valid=False,reason='CHAIN_RELEASE:'+why,checked=checked)
    released=joints.copy();released[g['arm']]=q
    released[g['fingers']]=[model.native.values(model.native.angle(.084))[n]for n in GRIPPER_JOINTS]
    context=dict(g,robot=SimpleNamespace(get_joint_positions=lambda:released.copy()),moving=lambda:Mend,tcp=lambda:Tend,qtarget=released,
        ik=lambda target,seed:solver.solve(target,base,seed,starts=1))
    exit_plan=plan_exit(context)
    return dict(valid=bool(exit_plan['valid']),reason=None if exit_plan['valid']else'CHAIN_EXIT_UNAVAILABLE',
        checked_native_configurations=checked,declared_actions=records,exit=exit_plan,
        final_nominal_arm_q=q.tolist(),final_predicted_moving_pose=Mend.tolist(),
        scope='conditional geometric safety, not predicted dynamics or contact success',
        CENTER_and_preload_mapping_unchanged=True,online_measured_revalidation_required=True)
