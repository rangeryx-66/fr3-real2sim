"""Generic visual-handle deployment and bounded PiPER bar-side-pinch planning.

The setup half may inspect the prepared URDF to render/locate the INITIAL visual
handle. It exports no articulation geometry to control. The planning half uses a
robot-only model and the actual cooked PhysicalScene, never an object joint model.
Existing baseline files are not changed.
"""
from __future__ import annotations

import copy
import json
import math
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

SOURCE_Y_UP_TO_WORLD = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
FIXED_BASE = [.5, -.55, -.1, 150.]


def _read(value):
    return copy.deepcopy(value) if isinstance(value, dict) else json.loads(Path(value).read_text())


def _unit(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if length < 1e-9 or not np.all(np.isfinite(vector)):
        raise ValueError('INVALID_VISUAL_HANDLE_FRAME')
    return vector / length


def _initial_visual_root(asset_root, initial_articulation_rad=0.):
    """Scene initialization helper, before the controller data boundary starts."""
    from articulated_demo.kinematics import URDFChain
    asset_root = Path(asset_root)
    meta = _read(asset_root / 'manifest.json')
    selection = meta['interaction_geometry']['selection']
    section = min(selection['sections'], key=lambda s: abs(s.get('fraction', 0.)))
    center = np.asarray(section['anchor_root_m'], dtype=float)
    axis = _unit(selection['axis_root'])
    normal = np.asarray(selection['outward_normal_root'], dtype=float)
    normal = _unit(normal - axis * np.dot(axis, normal))
    # Prepared visual points are authored in closed root coordinates. Setup maps
    # them to the chosen initial object configuration once; no GT is sent online.
    if abs(initial_articulation_rad) > 1e-12:
        chain = URDFChain(asset_root / 'urdf' / f"{meta['asset_id']}.urdf")
        handle_link = selection['handle_link']
        before = chain.root_to_link(handle_link, {})
        after = chain.root_to_link(handle_link, {meta['joint_name']: initial_articulation_rad})
        initial = after @ np.linalg.inv(before)
        center = (initial @ np.r_[center, 1.])[:3]
        axis = initial[:3, :3] @ axis
        normal = initial[:3, :3] @ normal
    return meta, {'anchor_root_m': center, 'axis_root': axis, 'outward_normal_root': normal,
                  'dimensions_m': np.asarray(selection['dimensions_m'], dtype=float)}


def _asset_transform(meta, placement):
    R = Rotation.from_euler('z', placement['yaw_deg'], degrees=True).as_matrix() @ SOURCE_Y_UP_TO_WORLD
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [placement['x_m'], placement['y_m'],
                   -float(meta['scale_source_to_meters']) * float(meta['static_source_bounds'][0][1]) + placement['fixture_height_m']]
    return T


def _world_visual(root_visual, T, along_offset=0.):
    anchor = (T @ np.r_[root_visual['anchor_root_m'], 1.])[:3]
    axis = _unit(T[:3, :3] @ root_visual['axis_root'])
    normal = _unit(T[:3, :3] @ root_visual['outward_normal_root'])
    normal = _unit(normal - axis * np.dot(normal, axis))
    R = np.column_stack((axis, np.cross(-normal, axis), -normal))
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = anchor
    return {'schema': 'initial-visual-handle-v1', 'anchor_world_m': anchor.tolist(),
            'axis_world': axis.tolist(), 'outward_normal_world': normal.tolist(),
            'T_world_handle': H.tolist(), 'dimensions_m': root_visual['dimensions_m'].tolist(),
            'along_handle_offset_m': float(along_offset),
            'source': 'initial prepared visual handle observation; not articulation model',
            'GT_joint_type_axis_origin_exported': False}


def make_deployment_template(dev_asset_root, dev_source_report):
    """Freeze the DEV visual workspace anchor without changing the fixed base."""
    source = _read(dev_source_report)
    meta, visual = _initial_visual_root(dev_asset_root)
    T = _asset_transform(meta, source['asset_installation'])
    return {'source_report': source, 'initial_visual_handle_world': _world_visual(visual, T),
            'placement_policy': 'align visual outward yaw and anchor XY; floor fixture at zero',
            'fixed_base': copy.deepcopy(source['robot_base_pose'])}


def prepare_deployment(episode, asset_root, dev_template, output, *, base=None):
    """Resolve one preregistered configuration; never search over world offsets.

    The output `source/report.json` follows the existing bootstrap contract. The
    handle descriptor contains visual axes only, not the object's joint axis.
    A too-high object remains too high and is allowed to fail reachability.
    """
    base = list(FIXED_BASE if base is None else base)
    template = _read(dev_template)
    if list(template.get('fixed_base', base)) != base:
        raise ValueError('FROZEN_BASE_MISMATCH')
    episode = _read(episode)
    if episode.get('preflight_exclusion_reason'):
        raise ValueError(episode['preflight_exclusion_reason'])
    deployment = episode['deployment']
    if deployment.get('base_reposition_allowed', False):
        raise ValueError('BASE_REPOSITION_NOT_SUPPORTED')
    initial = float(episode.get('initialization_only', {}).get('joint_position_rad', 0.))
    meta, root_visual = _initial_visual_root(asset_root, initial)
    nominal = template['initial_visual_handle_world']
    target = np.asarray(nominal['anchor_world_m'], dtype=float)
    wanted_normal = _unit(nominal['outward_normal_world'])
    source_normal = SOURCE_Y_UP_TO_WORLD @ root_visual['outward_normal_root']
    if min(np.linalg.norm(wanted_normal[:2]), np.linalg.norm(source_normal[:2])) < 1e-6:
        raise ValueError('VISUAL_OUTWARD_YAW_UNOBSERVABLE')
    yaw = math.atan2(wanted_normal[1], wanted_normal[0]) - math.atan2(source_normal[1], source_normal[0])
    yaw += math.radians(float(deployment.get('yaw_perturbation_deg', 0.)))
    R = Rotation.from_euler('z', yaw).as_matrix() @ SOURCE_Y_UP_TO_WORLD
    nominal_frame = np.asarray(nominal['T_world_handle'], dtype=float)[:3, :3]
    perturbation = nominal_frame @ np.asarray(deployment.get('translation_in_initial_handle_frame_m', [0., 0., 0.]), dtype=float)
    target = target + perturbation
    rotated_anchor = R @ root_visual['anchor_root_m']
    static_floor = float(meta['scale_source_to_meters']) * float(meta['static_source_bounds'][0][1])
    requested_fixture = target[2] - rotated_anchor[2] + static_floor
    fixture = max(0., requested_fixture)
    placement = {'x_m': float(target[0] - rotated_anchor[0]),
                 'y_m': float(target[1] - rotated_anchor[1]),
                 'yaw_deg': float(math.degrees(yaw)), 'fixture_height_m': float(fixture)}
    source = copy.deepcopy(template['source_report'])
    source.update(asset_id=str(meta['asset_id']), robot_base_pose=base, asset_installation=placement)
    # Keep camera/scene configuration but remove prior object's experiment outcomes.
    for key in ('success', 'status', 'door_angle_deg', 'actual_door_angle_deg', 'joint_axis', 'joint_origin'):
        source.pop(key, None)
    source['benchmark_setup'] = {'episode_id': episode['episode_id'], 'visual_placement_only': True,
                                 'per_asset_world_offset': False, 'robot_base_frozen': True,
                                 'fixture_floor_applied': bool(requested_fixture < 0.),
                                 'requested_fixture_height_m': float(requested_fixture),
                                 'asset_geometry_scaled_for_reachability': False}
    T = _asset_transform(meta, placement)
    visual = _world_visual(root_visual, T, deployment.get('along_handle_offset_m', 0.))
    visual['candidate_grid'] = copy.deepcopy(episode.get('candidate_grid', {}))
    visual['anchor_target_error_m'] = (np.asarray(visual['anchor_world_m']) - target).tolist()
    output = Path(output)
    (output / 'source').mkdir(parents=True, exist_ok=True)
    (output / 'source' / 'report.json').write_text(json.dumps(source, indent=2) + '\n')
    (output / 'initial_visual_handle_world.json').write_text(json.dumps(visual, indent=2) + '\n')
    result = {'source': str((output / 'source').resolve()), 'source_report': source,
              'initial_visual_handle_world': visual, 'placement': placement,
              'visual_root_setup_only': {key: value.tolist() for key, value in root_visual.items()},
              'fixture_floor_applied': bool(requested_fixture < 0.)}
    (output / 'deployment.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def plan_grasps(model, scene, moving_initial, initial_visual, base, *, seed=61,
                budget=12, output=None, stop_after_first=False, wall_clock_s=600., candidate_grid=None):
    """At most twelve exact-IK, cooked-geometry, home-to-grasp plans.

    `model` must be robot_only_model(); `scene` is PhysicalScene from the native
    cooked export. Contact closure is deliberately not certified by this planner.
    The bounded family is the manifest candidate_grid: 2 swaps x 2 depths x 3 slides.
    """
    from piper_mobile_demo.model import Model
    if not 1 <= int(budget) <= 12:
        raise ValueError('GRASP_CANDIDATE_BUDGET_EXCEEDED')
    if getattr(model, 'asset', None) is not None:
        raise ValueError('ROBOT_ONLY_MODEL_REQUIRED')
    started = time.monotonic()
    deadline = started + float(wall_clock_s)
    model.rng = np.random.default_rng(int(seed))
    base = list(base)
    moving_initial = np.asarray(moving_initial, dtype=float)
    initial_visual = _read(initial_visual)
    axis = _unit(initial_visual['axis_world'])
    normal = _unit(initial_visual['outward_normal_world'])
    normal = _unit(normal - axis * np.dot(normal, axis))
    anchor = np.asarray(initial_visual['anchor_world_m'], dtype=float)
    R = np.column_stack((axis, np.cross(-normal, axis), -normal))
    P = model.poses(np.asarray(model.home, dtype=float), base, width=.04)
    tcp_inv = np.linalg.inv(P['tcp_link'])
    pad_offset = np.mean([(tcp_inv @ P[name] @ np.r_[model.pads[name].mean(0), 1.])[:3]
                          for name in ('gripper_link1', 'gripper_link2')], axis=0)
    rows, trials = [], []
    q_home = np.asarray(model.home, dtype=float)
    minimum_margin = .05

    def check(q, _base=base):
        if time.monotonic() > deadline:
            raise TimeoutError('GRASP_PLANNING_WALL_CLOCK_BUDGET')
        q = np.asarray(q, dtype=float)
        if model.margin(q) <= minimum_margin:
            return False, 'LOW_JOINT_MARGIN', None
        ok, reason = scene.check(model.poses(q, _base, finger_q=[.05, -.05]), moving_initial, allow_handle=False)
        return bool(ok), reason, None

    class PlanningView:
        def __getattr__(self, name):
            return getattr(model, name)
        def check(self, q, _base):
            return check(q, _base)

    view = PlanningView()
    home_valid, home_reason, _ = check(q_home)
    slide_center = float(initial_visual.get('along_handle_offset_m', 0.))
    pad_length = float(np.max([bounds[1, 1] - bounds[0, 1] for bounds in model.pads.values()]))
    usable_slide = max(0., (float(initial_visual['dimensions_m'][0]) - pad_length) / 2)
    from interactive_twin.manifest import DEFAULT_POLICY
    grid = copy.deepcopy(candidate_grid or initial_visual.get('candidate_grid') or DEFAULT_POLICY['candidate_grid'])
    candidates = [(float(swap), float(depth), slide_center + float(delta))
                  for swap in grid['finger_swap_degrees']
                  for depth in grid['depth_offsets_m']
                  for delta in grid['along_handle_offsets_m']]
    if len(candidates) > 12:
        raise ValueError('FROZEN_CANDIDATE_GRID_EXCEEDS_BUDGET')
    candidates = candidates[:int(budget)]
    for index, (swap, depth, slide) in enumerate(candidates):
        target = np.eye(4)
        target[:3, :3] = R @ Rotation.from_euler('z', swap, degrees=True).as_matrix()
        target[:3, 3] = anchor + axis * slide - normal * depth - target[:3, :3] @ pad_offset
        row = {'candidate_index': index, 'variant': f'bar_side_pinch_{index:02d}',
               'family': 'bar-side-pinch', 'base': base, 'T': target.tolist(),
               'grasp_local': {'finger_swap_deg': swap, 'depth_m': depth, 'along_handle_m': slide},
               'status': 'NO_IK', 'checks': {}, 'seed': int(seed)}
        rows.append(row)
        try:
            if abs(slide) > usable_slide + 1e-12:
                row['status'] = 'BAD_GRASP_GEOMETRY_OUTSIDE_BAR_REGION'
                continue
            if not home_valid:
                row['status'] = 'HOME_' + home_reason
                continue
            if time.monotonic() > deadline:
                raise TimeoutError('GRASP_PLANNING_WALL_CLOCK_BUDGET')
            grasp_q = model.ik(target, base, starts=5)
            if grasp_q is None:
                continue
            row['checks']['exact_ik'] = True
            row['grasp_joint_margin_rad'] = float(model.margin(grasp_q))
            ok, reason, _ = check(grasp_q)
            if not ok:
                row['status'] = 'GRASP_' + reason
                continue
            row['checks']['open_grasp_collision_free'] = True
            pre = target.copy()
            pre[:3, 3] -= .04 * target[:3, 2]
            q = model.ik(pre, base, seed=grasp_q, starts=3)
            if q is None:
                row['status'] = 'NO_PREGRASP_IK'
                continue
            approach = []
            for fraction in np.linspace(0., 1., 21):
                pose = target.copy()
                pose[:3, 3] = (1. - fraction) * pre[:3, 3] + fraction * target[:3, 3]
                q = model.ik(pose, base, seed=q, starts=1)
                if q is None:
                    row['status'] = 'NO_APPROACH_IK'
                    break
                ok, reason, _ = check(q)
                if not ok:
                    row['status'] = 'APPROACH_' + reason
                    break
                if approach:
                    previous = np.asarray(approach[-1], dtype=float)
                    count = max(2, int(np.ceil(np.max(np.abs(q - previous)) / .025)) + 1)
                    failed_edge = next((reason for state in np.linspace(previous, q, count)
                                        for ok, reason, _ in [check(state)] if not ok), None)
                    if failed_edge:
                        row['status'] = 'APPROACH_EDGE_' + failed_edge
                        break
                approach.append(np.asarray(q, dtype=float).tolist())
            else:
                row['checks']['open_approach_collision_free'] = True
                path = Model.joint_plan(view, q_home, np.asarray(approach[0]), base, iterations=1500)
                if path is None:
                    row['status'] = 'NO_HOME_PLAN'
                    continue
                row.update(status='PENDING_REAL_CLOSURE', q_pre=approach[0], q_grasp=approach[-1],
                           approach=approach, preplan=path,
                           minimum_joint_margin_rad=min(float(model.margin(np.asarray(state))) for state in approach + path))
                row['checks']['home_plan_collision_free'] = True
                trials.append(copy.deepcopy(row))
                if stop_after_first:
                    break
        except TimeoutError as exc:
            row['status'] = str(exc)
            break
        finally:
            row['elapsed_planning_seconds'] = time.monotonic() - started
    result = {'schema': 'interactive-twin-grasp-plan-v1', 'base': base, 'rows': rows,
              'trial_candidates': trials, 'best': trials[0] if trials else None,
              'planning_complete': not any('WALL_CLOCK_BUDGET' in r['status'] for r in rows),
              'candidate_budget': int(budget), 'candidate_grid': grid, 'candidates_attempted': len(rows),
              'family': 'generic bar-side-pinch; finite local depth/slide and symmetric finger swap',
              'collision_representation': 'native cooked PhysicalScene, official unsplit fingers',
              'raw_intersections': 'diagnostic only; not an execution veto',
              'joint_margin_minimum_rad': minimum_margin, 'RRT_iterations': 1500,
              'real_closure_verified': False, 'elapsed_seconds': time.monotonic() - started}
    if output is not None:
        output = Path(output)
        output = output if output.suffix == '.json' else output / 'plan.json'
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + '\n')
    return result
