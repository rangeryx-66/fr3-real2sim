"""Current-state coupled motion/stop proof using unchanged native collisions.

Approach/closing remains the existing fixed-station owner. This additional
certificate checks the loaded XY+arm path, centering and relative lead. It
does not certify a future exit or predict contact/dynamics success.
"""
import math
import numpy as np
from .kinematics import CoupledKinematics


def evaluate(g, candidate, actions, policy, config, solver, *, live=False):
    from native_g1_ft_identification import kinematic_prior
    from interaction_runtime.articulation_v7 import Articulation
    from interaction_runtime.prismatic_planning_v73 import predicted_aperture, closing_envelope, representable_joint_bounds
    from interaction_runtime.prismatic_chain_v75 import rail_lead_offsets
    from interaction_runtime.progress import hinge_radius, LOOKAHEAD_RAD
    from interaction_runtime.segments import validate_action
    prior = kinematic_prior(g)
    kind = prior['joint_type']
    articulation = Articulation(kind, tuple(prior['axis_world']), tuple(prior['origin_world_m']),
        tuple(g['job']['articulation_v7']['limits_native']))
    if not actions: return dict(valid=False, reason='DECLARED_CHAIN_REQUIRED')
    for action in actions: validate_action(action); articulation.validate_target(action)
    if not live and not any(a['role'].startswith('VALIDATION') for a in actions):
        return dict(valid=False, reason='COMPLETE_VALIDATION_NOT_PLANNED')
    model = g['model']; base = np.asarray(g['base']).copy()
    measured = np.asarray(g['robot'].get_joint_positions())
    names = g['robot'].dof_names
    ix = np.array([names.index(n) for n in ('mobile_x', 'mobile_y')])
    low, high = representable_joint_bounds(model.limits, policy.data['joint_margin_rad'])
    limits = np.vstack([np.tile([-1., 1.], (2, 1)), np.column_stack([low, high])])
    coupled = CoupledKinematics(solver, base, measured[ix], limits,
        np.r_[.08, .08, np.asarray(config['arm_speed_limits']) / 3])
    initial = np.asarray(candidate['approach'][-1]).copy()
    start = float(g['actual']()); moving = np.asarray(g['moving']()).copy()
    T = np.asarray(candidate['T']).copy()
    bias = np.r_[g['qtarget'][ix] - measured[ix], g['qtarget'][g['arm']] - measured[g['arm']]]
    # XY is submitted by its explicit owner rather than qtarget's legacy slots.
    runtime = g.get('v88_runtime')
    if runtime is not None: bias[:2] = runtime.base_goal - measured[ix]
    aperture = predicted_aperture(candidate)
    angle = model.native.angle(aperture)
    preload = np.clip(angle + float(g['policy']['preload_n']) * abs(model.native.width_derivative(angle))
        / config['gripper_kp_nm_rad'], 0, .85)
    options = ([dict(finger_q=measured[g['fingers']])] if live else
        [dict(width=aperture), dict(width=model.native.width(preload))])
    if not live:
        context = dict(g, solver=solver, runtime_policy=policy)
        closing = closing_envelope(context, initial, aperture, g['policy'])
        if not closing['valid']: return dict(valid=False, reason=closing['reason'], closing=closing)
    poses = model.poses(initial, base, **options[0])
    centers = [(poses[n] @ np.r_[model.pads[n].mean(0), 1])[:3] for n in g['FINGERS']]
    jaw = centers[1] - centers[0]; jaw /= np.linalg.norm(jaw)
    bound = float(g['policy']['max_centering_displacement_m'])
    radius = 1. if kind == 'prismatic' else hinge_radius(T[:3, 3], prior['origin_world_m'], prior['axis_world'])
    if radius <= 1e-9: return dict(valid=False, reason='ZERO_HINGE_RADIUS')
    checked = 0; minimum = math.inf; endpoints = []; samples = []
    motion = policy.motion('LOADED')
    leads = rail_lead_offsets(policy, g['dt']) if kind == 'prismatic' else [0., -LOOKAHEAD_RAD, LOOKAHEAD_RAD]
    for center in [0., -bound, bound]:
        seed = np.r_[measured[ix], initial]; posture = initial.copy(); previous = start
        for action in actions:
            target = float(action['target_q'])
            speed = min(action.get('reference_speed_m_s', motion['speed_m_s']), motion['speed_m_s'])
            acceleration = min(action.get('reference_acceleration_m_s2', motion['acceleration_m_s2']), motion['acceleration_m_s2'])
            stopping = (speed ** 2 / (2 * acceleration) + speed * g['dt']) / radius
            end = target + np.sign(target - previous) * stopping
            if not articulation.limits[0] <= end <= articulation.limits[1]:
                return dict(valid=False, reason='COUPLED_STOPPING_ENVELOPE_LIMIT', role=action['role'])
            interval = .001 if kind == 'prismatic' else math.radians(.5)
            values = sorted(set(np.r_[np.linspace(previous, end,
                max(2, math.ceil(abs(end - previous) / interval) + 1)), target]),
                reverse=end < previous)
            stopped = None
            for value in values:
                D = articulation.delta(float(value) - start)
                nominal = None
                for lead in leads:
                    command_D = articulation.delta(float(value) + lead - start)
                    goal = command_D @ T
                    goal[:3, 3] += center * (command_D[:3, :3] @ jaw)
                    q = coupled.solve(goal, seed, posture)
                    if q is None: return dict(valid=False, reason='COUPLED_CHAIN_IK', role=action['role'], coordinate_native=float(value), checked=checked)
                    for command in (q, q + bias):
                        minimum = min(minimum, float(model.margin(command[2:])))
                        if np.any(command <= limits[:, 0]) or np.any(command >= limits[:, 1]):
                            return dict(valid=False, reason='COUPLED_CHAIN_JOINT_LIMIT', checked=checked)
                        for option in options:
                            okay, why = g['collision'].check(model.poses(command[2:], coupled.base(command[:2]), **option), D @ moving, True)
                            checked += 1
                            if not okay: return dict(valid=False, reason='COUPLED_CHAIN_NATIVE:' + str(why), role=action['role'], coordinate_native=float(value), lead_native=float(lead), center_m=center, checked=checked)
                    if lead == 0: nominal = q.copy()
                seed = nominal
                if center == 0: samples.append(dict(coordinate_native=float(value), base_xy_m=coupled.base(seed[:2])[:2].tolist(), arm_q_rad=seed[2:].tolist()))
                if abs(value - target) < 1e-10: stopped = seed.copy()
            if stopped is None: raise AssertionError('STOPPED_TARGET_MISSING')
            seed = stopped; previous = target
            if center == 0:
                endpoints.append(dict(role=action['role'], target_rad=target, target_m=target,
                    motion_valid=True, stopped_hold_checked=True, exit_valid=None,
                    stopping_envelope_native=stopping))
    return dict(valid=True, reason=None, coordinated_xy_arm=True, staged_exit=True,
        motion_valid=True, stopped_hold_checked=True, checked_native_configurations=checked,
        declared_actions=endpoints, endpoints=endpoints, minimum_joint_margin_rad=minimum,
        trajectory=samples, reference_lead_native=leads, centering_bound_m=bound,
        exit=dict(valid=None, status='DEFERRED_TO_ACTUAL_STOPPED_STATE'),
        scope='conditional simultaneous native XY+arm loaded path and stop; actual per-step guards remain',
        no_physical_progress_claimed=True)


def current(ops, actions):
    r = ops.r; joints = np.asarray(r.robot.get_joint_positions())
    candidate = dict(T=r.tcp().tolist(), approach=[joints[r.arm].tolist()])
    return evaluate(ops.g, candidate, actions, r.policy, r.c, r.solver, live=True)
