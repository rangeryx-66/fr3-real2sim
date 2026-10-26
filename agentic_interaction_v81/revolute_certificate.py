"""Current-state native whole-chain certificate for the isolated V8.1 gate.

Safety feasibility only: observed grasp scores, frozen controller/physics and
all online checks stay unchanged. A nominal IK arc is not this certificate.
"""
from types import SimpleNamespace
import math
import json
import numpy as np


def evaluate(g, candidate, actions, policy, config, solver):
    from interaction_runtime.articulation_v7 import Articulation
    from interaction_runtime.progress import hinge_radius, LOOKAHEAD_RAD
    from interaction_runtime.prismatic_planning_v73 import closing_envelope, predicted_aperture
    from native_g1_ft_identification import kinematic_prior
    from native_g1_safe_park import plan_exit
    from xarm7_native_gripper import GRIPPER_JOINTS

    prior = kinematic_prior(g)
    if prior['joint_type'] != 'revolute':
        raise ValueError('REVOLUTE_CERTIFICATE_ONLY')
    motion = Articulation('revolute', tuple(prior['axis_world']),
        tuple(prior['origin_world_m']), tuple(g['job']['articulation_v7']['limits_native']))
    model, base = g['model'], g['base']
    moving = np.asarray(g['moving']()).copy()
    start = float(g['actual']())
    T = np.asarray(candidate['T']).copy()
    initial = np.asarray(candidate['approach'][-1]).copy()
    aperture = predicted_aperture(candidate)
    context = dict(g, solver=solver, runtime_policy=policy)
    geometry = g.get('a').output/'agent_geometry/observed_geometry.json' if g.get('a') else None
    normal = json.loads(geometry.read_text())['normal_world'] if geometry and geometry.exists() else g.get('normal')
    if normal is None:
        return dict(valid=False, reason='CURRENT_OBSERVED_PANEL_NORMAL_REQUIRED')
    context['normal'] = np.asarray(normal)
    closing = closing_envelope(context, initial, aperture, g['policy'])
    if not closing['valid']:
        return dict(valid=False, reason=closing['reason'], closing=closing)
    joints = np.asarray(g['robot'].get_joint_positions()).copy()
    bias = np.asarray(g['qtarget'])[g['arm']] - joints[g['arm']]
    angle = model.native.angle(aperture)
    preload = float(np.clip(angle + float(g['policy']['preload_n']) *
        abs(model.native.width_derivative(angle)) / config['gripper_kp_nm_rad'], 0, .85))
    widths = [aperture, model.native.width(preload)]
    poses = model.poses(initial, base, width=aperture)
    centers = [(poses[n] @ np.r_[model.pads[n].mean(0), 1])[:3] for n in g['FINGERS']]
    jaw = centers[1] - centers[0]
    jaw /= np.linalg.norm(jaw)
    bound = float(g['policy']['max_centering_displacement_m'])
    radius = hinge_radius(T[:3, 3], prior['origin_world_m'], prior['axis_world'])
    if radius <= 1e-9:
        return dict(valid=False, reason='ZERO_PERPENDICULAR_HINGE_RADIUS')
    checked, minimum, endpoints = 0, float('inf'), []

    def check(q, M, width):
        nonlocal checked, minimum
        margin = float(model.margin(q)); minimum = min(minimum, margin)
        checked += 1
        if margin <= policy.data['joint_margin_rad']:
            return False, 'JOINT_MARGIN'
        return g['collision'].check(model.poses(q, base, width=width), M, True)

    for center in [0., -bound, bound]:
        previous, seed = start, initial.copy()
        for action in actions:
            target = motion.validate_target(action)
            speed = min(float(action.get('reference_speed_m_s', policy.motion('LOADED')['speed_m_s'])),
                policy.motion('LOADED')['speed_m_s'])
            accel = min(float(action.get('reference_acceleration_m_s2', policy.motion('LOADED')['acceleration_m_s2'])),
                policy.motion('LOADED')['acceleration_m_s2'])
            stopping = (speed * speed / (2 * accel) + speed * g['dt']) / radius
            end = target + np.sign(target - previous) * stopping
            if not motion.limits[0] <= end <= motion.limits[1]:
                return dict(valid=False, reason='REVOLUTE_STOPPING_ENVELOPE_LIMIT', role=action['role'])
            values = np.linspace(previous, end, max(2, math.ceil(abs(end - previous) / math.radians(.5)) + 1))
            for value in values:
                D = motion.delta(float(value) - start)
                for lead in [0., -LOOKAHEAD_RAD, LOOKAHEAD_RAD]:
                    command_D = motion.delta(float(value) + lead - start)
                    goal = command_D @ T
                    goal[:3, 3] += center * (command_D[:3, :3] @ jaw)
                    q = solver.solve(goal, base, seed, starts=1)
                    if q is None:
                        return dict(valid=False, reason='REVOLUTE_CHAIN_IK', role=action['role'],
                            coordinate_rad=float(value), reference_lead_rad=lead, center_m=center, checked=checked)
                    for command in (q, q + bias):
                        for width in widths:
                            ok, why = check(command, D @ moving, width)
                            if not ok:
                                return dict(valid=False, reason='REVOLUTE_CHAIN_NATIVE:' + str(why),
                                    role=action['role'], coordinate_rad=float(value), reference_lead_rad=lead,
                                    center_m=center, checked=checked, minimum_joint_margin_rad=minimum)
                    if lead == 0:seed = q.copy()
            D = motion.delta(target - start)
            goal = D @ T; goal[:3, 3] += center * (D[:3, :3] @ jaw)
            seed = solver.solve(goal, base, seed, starts=1)
            if seed is None:return dict(valid=False, reason='REVOLUTE_STOPPED_TARGET_IK', checked=checked)
            if g['job'].get('v87_staged_exit'):
                for command in (seed,seed+bias):
                    for width in widths:
                        okay,why=check(command,D @ moving,width)
                        if not okay:return dict(valid=False,reason='REVOLUTE_STOPPED_HOLD:'+str(why),
                            role=action['role'],center_m=center,checked=checked)
            previous = target
            if center != 0:continue
            if g['job'].get('v87_staged_exit'):
                # The checked loaded motion includes the stopped endpoint,
                # retained bias, centering and reference/braking envelope.
                # Future release/parking is a different measured-state task;
                # absence of its route cannot invalidate this motion proof.
                endpoints.append(dict(role=action['role'],target_rad=target,
                    stopping_envelope_rad=stopping,motion_valid=True,
                    stopped_hold_checked=True,exit_valid=None,
                    exit_planning='DEFERRED_TO_ACTUAL_STOPPED_STATE'))
                continue
            # Predict release/exit at every declared stopped boundary. Real
            # measured-state guarded exit remains authoritative in execution.
            for width in np.linspace(min(widths), .084, 35):
                ok, why = check(seed, D @ moving, float(width))
                if not ok:return dict(valid=False, reason='REVOLUTE_RELEASE:' + str(why), role=action['role'])
            released = joints.copy(); released[g['arm']] = seed
            released[g['fingers']] = [model.native.values(model.native.angle(.084))[n] for n in GRIPPER_JOINTS]
            exit_context = dict(context, robot=SimpleNamespace(get_joint_positions=lambda:released.copy()),
                moving=lambda:D @ moving, tcp=lambda:goal.copy(), qtarget=released,
                ik=lambda target, q:solver.solve(target, base, q, starts=1))
            exit_plan = plan_exit(exit_context)
            endpoints.append(dict(role=action['role'], target_rad=target,
                stopping_envelope_rad=stopping, exit_valid=bool(exit_plan['valid']),
                exit_reason=exit_plan.get('reason'), parking_region=exit_plan.get('parking_region')))
            if not exit_plan['valid']:
                return dict(valid=False, reason='REVOLUTE_CHAIN_EXIT_UNAVAILABLE', endpoints=endpoints, checked=checked)
    return dict(valid=True, closing=closing, endpoints=endpoints, checked_native_configurations=checked,
        staged_exit=bool(g['job'].get('v87_staged_exit')),
        minimum_joint_margin_rad=minimum, perpendicular_radius_m=radius,
        actual_start_rad=start, measured_finger_joints_rad=joints[g['fingers']].tolist(),
        retained_command_bias_rad=bias.tolist(), reference_lead_rad=LOOKAHEAD_RAD,
        scope='conditional current-state geometry; no physical target/retention success claimed',
        future_measured_state_revalidation_required=True, scores_and_physics_unchanged=True)
