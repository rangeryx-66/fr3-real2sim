"""Bounded SE(2) recovery from the initial visual handle frame, never a GT arc."""
import ast
import copy
import hashlib
import json
import math
import time
from functools import lru_cache
from pathlib import Path

import fcl
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


@lru_cache(maxsize=2)
def physical_scene_class(root):
    # Execute the exact frozen class without importing Isaac-only report APIs.
    from piper_mobile_demo.owned_scene import Shape, meshes, intersects
    path = Path(root) / 'articulated_interaction/physical_baseline.py'
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PhysicalScene')
    namespace = dict(np=np, Shape=Shape, meshes=meshes, intersects=intersects,
                     FINGERS=('gripper_link1', 'gripper_link2'))
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['PhysicalScene']


def transform(base):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('z', base[3], degrees=True).as_matrix()
    T[:3, 3] = base[:3]
    return T


def at_base(export, initial, base):
    """Move the existing support, not the object or any robot collider."""
    result = copy.deepcopy(export)
    delta = transform(base) @ np.linalg.inv(transform(initial))
    for entry in result['shapes']:
        if 'pedestal' in entry['path']:
            entry['world_transform'] = (delta @ np.asarray(entry['world_transform'])).tolist()
    return result


def scene_at(root, export, model, initial, base):
    from piper_mobile_demo.owned_scene import Shape
    data = at_base(export, initial, base)
    allowed = [e['path'] for e in data['shapes']
               if 'handlepiece' in e['path'].replace('_', '').lower()]
    scene = physical_scene_class(root)(data, model, allowed)
    # Existing mobile platform dimensions, explicitly simulated in the new run.
    B = transform(base)
    B[:3, 3] = [base[0], base[1], -.66]
    scene.scene.append(Shape(trimesh.creation.box([.34, .30, .20]), B, True,
                             '/World/mobile_chassis', 'mobile_chassis'))
    return scene


def distance(a, b):
    return max(0., float(fcl.distance(a.object, b.object)))


def home_valid(scene, model, base):
    from piper_mobile_demo.owned_scene import intersects
    P = model.poses(np.asarray(model.home, float), base, finger_q=[.05, -.05])
    ok, why = scene.check(P, scene.moving_reference, False)
    if not ok:
        return False, why, 0.
    chassis = next(s for s in scene.scene if s.path == '/World/mobile_chassis')
    env = [s for s in scene.scene if s is not chassis and 'pedestal' not in s.path]
    for s in env:
        if intersects(chassis, s):
            return False, 'CHASSIS_ENVIRONMENT_COLLISION:' + s.path, 0.
    return True, 'SAFE', min([distance(chassis, s) for s in env] or [1.])


class MobileRuntimeScene:
    """Reuse identical cooked FCL solids instead of rebuilding them each tick."""
    def __init__(self, root, export, model, initial):
        self.initial = list(initial)
        scene = scene_at(root, export, model, initial, initial)
        self.scene = scene
        self.platform = [s for s in scene.scene if 'pedestal' in s.path or s.path=='/World/mobile_chassis']
        scene.scene = [s for s in scene.scene if s not in self.platform]
        self.moving_reference = scene.moving_reference

    def check(self, poses, base):
        from piper_mobile_demo.owned_scene import intersects
        ok, why = self.scene.check(poses, self.moving_reference, False)
        if not ok:
            return ok, why
        delta = transform(base) @ np.linalg.inv(transform(self.initial))
        for support in self.platform:
            support.place(delta)
            for robot in self.scene.robot:
                if robot.body in ('base_link','link1') and 'pedestal' in support.path:
                    continue
                if intersects(robot,support):
                    return False,'DANGEROUS_GEOMETRY_COLLISION:'+robot.body+':'+support.path
            if support.path=='/World/mobile_chassis':
                for environment in self.scene.scene:
                    if intersects(support,environment):
                        return False,'CHASSIS_ENVIRONMENT_COLLISION:'+environment.path
        return True,'SAFE'

    def contact_guard(self, contacts, poses):
        return self.scene.contact_guard(contacts,poses)


def route(root, export, model, initial, target, spacing=.025):
    """Conservative rotate/translate route alternatives with arm held at home."""
    yaw_delta = (target[3] - initial[3] + 180.) % 360. - 180.
    xy_count = max(2, int(np.ceil(np.linalg.norm(np.subtract(target[:2], initial[:2])) / spacing)) + 1)
    yaw_count = max(2, int(np.ceil(abs(yaw_delta) / 3.)) + 1)
    alternatives = []
    for rotate_first in (False, True):
        intermediate = list(initial)
        if rotate_first:
            rotation = [[initial[0], initial[1], initial[2], initial[3] + f*yaw_delta]
                        for f in np.linspace(0., 1., yaw_count)]
            translation = [[*(np.asarray(initial[:2])*(1-f)+np.asarray(target[:2])*f),
                            initial[2], initial[3]+yaw_delta] for f in np.linspace(0., 1., xy_count)]
            waypoints = rotation + translation[1:]
        else:
            translation = [[*(np.asarray(initial[:2])*(1-f)+np.asarray(target[:2])*f),
                            initial[2], initial[3]] for f in np.linspace(0., 1., xy_count)]
            rotation = [[target[0], target[1], initial[2], initial[3]+f*yaw_delta]
                        for f in np.linspace(0., 1., yaw_count)]
            waypoints = translation + rotation[1:]
        reason = None
        clearance = float('inf')
        for pose in waypoints:
            scene = scene_at(root, export, model, initial, pose)
            ok, why, gap = home_valid(scene, model, pose)
            clearance = min(clearance, gap)
            if not ok:
                reason = why
                break
        alternatives.append({'rotate_first': rotate_first, 'failure': reason})
        if reason is None:
            return {'valid': True, 'waypoints': [list(map(float, p)) for p in waypoints],
                    'chassis_clearance_m': clearance, 'alternatives': alternatives,
                    'translation_m': float(np.linalg.norm(np.subtract(target[:2], initial[:2]))),
                    'rotation_deg': abs(yaw_delta), 'arm_locked_at_home': True,
                    'mode': 'simulated kinematic SE2 platform; not wheel/navigation dynamics'}
    return {'valid': False, 'alternatives': alternatives, 'reason': reason}


def candidate_bases(initial, visual, policy):
    normal = np.asarray(visual['outward_normal_world'], float)[:2]
    normal /= np.linalg.norm(normal)
    anchor = np.asarray(visual['anchor_world_m'], float)[:2]
    result = []
    for angle in policy['azimuth_offsets_deg']:
        a = math.radians(angle)
        direction = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]) @ normal
        for standoff in policy['standoff_m']:
            xy = anchor + float(standoff) * direction
            heading = math.degrees(math.atan2(*(anchor-xy)[::-1]))
            for yaw_offset in policy['yaw_offsets_deg']:
                pose = [float(xy[0]), float(xy[1]), float(initial[2]), float(heading+yaw_offset)]
                result.append(pose)
    # Fixed ordering, nearest travel first; no per-asset coordinates.
    result.sort(key=lambda b: np.linalg.norm(np.subtract(b[:2], initial[:2])))
    return result


def workspace_check(model, scene, trial, base, visual, span):
    """Local unknown-joint probe room, never a GT opening arc certificate."""
    T0 = np.asarray(trial['T'], float)
    normal = np.asarray(visual['outward_normal_world'], float)
    lateral = np.asarray(visual['axis_world'], float)
    directions = [normal, normal+.25*lateral, normal-.25*lateral, lateral]
    margins = []
    accepted = []
    for index, direction in enumerate(directions):
        direction /= np.linalg.norm(direction)
        seed = np.asarray(trial['q_grasp'], float)
        states = []
        for amount in np.linspace(0., span, 6)[1:]:
            T = T0.copy(); T[:3, 3] += amount*direction
            q = model.ik(T, base, seed=seed, starts=1)
            if q is None or model.margin(q) <= .05:
                break
            moving = T @ np.linalg.inv(T0) @ scene.moving_reference
            ok, _ = scene.check(model.poses(q, base, finger_q=[.05, -.05]), moving, True)
            if not ok:
                break
            states.append(float(model.margin(q))); seed = q
        if len(states) == 5:
            margins.extend(states); accepted.append(index)
    return {'passed': bool(accepted), 'directions_with_workspace': accepted,
            'span_m': span, 'minimum_joint_margin_rad': min(margins) if margins else None,
            'geometry_prediction': 'retained-grasp local rigid transform; native contact remains authoritative',
            'full_opening_arc_certified': False}


def recover(root, export, fixed_plan, visual, initial, policy, *, seed, deadline):
    from interaction_identification.contact_probe import robot_only_model
    from interactive_twin.planning import plan_grasps
    model = robot_only_model(Path(root) / 'config/piper.urdf')
    model.rng = np.random.default_rng(seed)
    started = time.monotonic()
    rows = []
    ranked = []
    for index, base in enumerate(candidate_bases(initial, visual, policy)):
        if time.time() >= deadline or time.monotonic()-started > policy['search_wall_s']:
            break
        scene = scene_at(root, export, model, initial, base)
        ok, why, clearance = home_valid(scene, model, base)
        row = {'base_index': index, 'base': base, 'status': why, 'coarse_exact_ik': 0}
        rows.append(row)
        if not ok:
            continue
        feasible = []
        # All original 12 poses are unchanged; only the robot base is different.
        for candidate in fixed_plan['rows']:
            if time.time() >= deadline or time.monotonic()-started > policy['search_wall_s']:
                break
            T = np.asarray(candidate['T'], float)
            q = model.ik(T, base, starts=5)
            if q is None:
                continue
            row['coarse_exact_ik'] += 1
            ok, why = scene.check(model.poses(q, base, finger_q=[.05, -.05]), scene.moving_reference, False)
            if ok and model.margin(q) > .05:
                feasible.append((float(model.margin(q)), candidate['candidate_index']))
        if not feasible:
            row['status'] = 'NO_COLLISION_FREE_EXACT_GRASP_IK'
            continue
        row['status'] = 'COARSE_GRASP_FEASIBLE'
        row['coarse_best_margin_rad'] = max(x[0] for x in feasible)
        row['clearance_m'] = clearance
        row['travel_m'] = float(np.linalg.norm(np.subtract(base[:2], initial[:2])))
        ranked.append(row)
    ranked.sort(key=lambda r: (-r['coarse_best_margin_rad'], -r['clearance_m'], r['travel_m']))
    choices = []
    for row in ranked[:policy['full_plan_base_budget']]:
        if time.time() >= deadline:
            break
        base = row['base']; path = route(root, export, model, initial, base)
        row['route'] = path
        if not path['valid']:
            row['status'] = 'NO_COLLISION_FREE_BASE_ROUTE'
            continue
        scene = scene_at(root, export, model, initial, base)
        plan = plan_grasps(model, scene, scene.moving_reference, visual, base,
                           seed=seed, budget=12, wall_clock_s=policy['full_plan_wall_s'],
                           candidate_grid=fixed_plan['candidate_grid'])
        qualified = []
        for trial in plan['trial_candidates']:
            probe = workspace_check(model, scene, trial, base, visual, policy['probe_workspace_m'])
            trial['unknown_probe_workspace'] = probe
            if not probe['passed']:
                continue
            margin = min(trial['minimum_joint_margin_rad'], probe['minimum_joint_margin_rad'])
            q = np.asarray(trial['q_grasp'], float)
            J = np.column_stack([(model.poses(q+np.eye(6)[j]*1e-5, base)['tcp_link'][:3, 3]
                                 -model.poses(q, base)['tcp_link'][:3, 3])/1e-5 for j in range(6)])
            singular = float(np.linalg.svd(J, compute_uv=False)[-1])
            trial['recovery_score'] = [margin, path['chassis_clearance_m'], -path['translation_m'], singular]
            qualified.append(trial)
        row['full_plan_candidate_count'] = len(qualified)
        row['status'] = 'RECOVERY_PREFLIGHT_PASSED' if qualified else 'NO_APPROACH_OR_PROBE_WORKSPACE'
        if qualified:
            qualified.sort(key=lambda t: t['recovery_score'], reverse=True)
            plan['trial_candidates'] = qualified; plan['best'] = qualified[0]
            choices.append({'base': base, 'route': path, 'plan': plan, 'score': qualified[0]['recovery_score']})
    choices.sort(key=lambda c: c['score'], reverse=True)
    return {'status': 'RECOVERY_PREFLIGHT_PASSED' if choices else 'NO_MOBILE_RECOVERY',
            'selected': choices[0] if choices else None, 'search_rows': rows,
            'bases_checked': len(rows), 'search_elapsed_s': time.monotonic()-started,
            'policy': policy, 'GT_articulation_used': False,
            'official_robot_or_contact_model_changed': False}


def eligible(fixed_plan):
    statuses = [r['status'] for r in fixed_plan['rows']]
    if fixed_plan.get('trial_candidates'):
        return False
    return any(s in ('NO_IK', 'NO_PREGRASP_IK', 'NO_APPROACH_IK', 'NO_HOME_PLAN')
               or 'LOW_JOINT_MARGIN' in s or 'SELF_COLLISION' in s
               or s.startswith(('HOME_', 'APPROACH_')) for s in statuses)
