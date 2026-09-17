"""Deterministic, robot-specific local grasp refinement.

The AnyGrasp pose, score and rank remain immutable.  Refinements are execution
metadata around a parent pose and are ordered only within that parent.
"""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from mesh_hand_geometry import evaluate, target_mesh

SEED = 20260911
SAMPLES = 256
MOVEIT_LIMIT = 12
PAD_AREA_M2 = .0175 * .0185
TRANSLATION_BOUNDS_M = np.array([.005, .005, .010])
ROTATION_BOUNDS_DEG = np.array([10., 10., 15.])
WEIGHTS = dict(com_lever=.35, gravity_roll_torque=.25,
               normal_mismatch=.20, patch_deficit=.15,
               contact_asymmetry=.05)


def _lhs(n=SAMPLES-1, dimensions=6, seed=SEED):
    rng = np.random.default_rng(seed)
    result = np.empty((n, dimensions))
    for j in range(dimensions):
        result[:, j] = (rng.permutation(n) + .5) / n
    return result * 2 - 1


def local_samples():
    values = np.vstack([np.zeros(6), _lhs()])
    values[:, :3] *= TRANSLATION_BOUNDS_M
    values[:, 3:] *= ROTATION_BOUNDS_DEG
    return values


def apply_offset(parent, offset):
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_euler('xyz', offset[3:], degrees=True).as_matrix()
    delta[:3, 3] = offset[:3]
    return np.asarray(parent, dtype=float) @ delta


def _patch_centroid(side):
    points = [x['point_TCP'] for x in side.get('contacts', [])
              if x.get('in_first_patch') and x.get('in_friction_cone')]
    return np.mean(points, axis=0) if points else None


def torque_metrics(name, hand, target, mesh_result):
    mesh = target_mesh(name)
    target_to_hand = np.linalg.inv(hand) @ target
    com = target_to_hand[:3, :3] @ np.asarray(mesh.center_mass) + target_to_hand[:3, 3]
    centers = [_patch_centroid(x) for x in mesh_result['sides']]
    if any(x is None for x in centers):
        return dict(valid=False, reason='MISSING_BILATERAL_PATCH')
    left, right = centers
    line = right-left
    length = np.linalg.norm(line)
    if length < 1e-6:
        return dict(valid=False, reason='DEGENERATE_CONTACT_LINE')
    line /= length
    midpoint = (left+right)/2
    arm = com-midpoint
    lever = float(np.linalg.norm(arm-line*np.dot(arm, line)))
    gravity = hand[:3, :3].T @ np.array([0., 0., -1.])
    roll_lever = float(abs(np.dot(np.cross(arm, gravity), line)))
    opposition = float(np.clip(-mesh_result['normal_opposition'], -1, 1))
    patch = min(x.get('normal_patch_area_m2', 0.) for x in mesh_result['sides'])
    asymmetry = float(mesh_result['first_contact_asymmetry_m'])
    components = dict(
        com_lever=float(np.clip(lever/.08, 0, 1)),
        gravity_roll_torque=float(np.clip(roll_lever/.08, 0, 1)),
        normal_mismatch=float(np.clip((1-opposition)/2, 0, 1)),
        patch_deficit=float(np.clip(1-patch/PAD_AREA_M2, 0, 1)),
        contact_asymmetry=float(np.clip(asymmetry/.010, 0, 1)),
    )
    cost = sum(WEIGHTS[k]*v for k, v in components.items())
    return dict(valid=True, contact_centers_TCP_m=[left.tolist(), right.tolist()],
                target_COM_TCP_m=com.tolist(), contact_line_length_m=float(length),
                COM_contact_line_lever_m=lever, gravity_roll_lever_m=roll_lever,
                min_normal_patch_area_m2=float(patch), normal_opposition=opposition,
                first_contact_asymmetry_m=asymmetry, normalized=components,
                torque_cost=float(cost))


def generate(name, parent, target):
    rows = []
    for i, offset in enumerate(local_samples()):
        hand = apply_offset(parent, offset)
        mesh = evaluate(name, hand, target)
        torque = torque_metrics(name, hand, target, mesh)
        rows.append(dict(refinement_id=i, parent_is_original=i == 0,
                         offset_translation_TCP_m=offset[:3].tolist(),
                         offset_rotation_TCP_deg=offset[3:].tolist(),
                         T_B_TCP=hand.tolist(), mesh_hand_geometry=mesh,
                         torque=torque,
                         geometry_passed=bool(mesh['passed'] and torque['valid'])))
    valid = [x for x in rows if x['geometry_passed']]
    valid.sort(key=lambda x: (x['torque']['torque_cost'], x['refinement_id']))
    for rank, row in enumerate(valid): row['torque_rank'] = rank
    return dict(target=name, seed=SEED, samples=SAMPLES, moveit_limit=MOVEIT_LIMIT,
                translation_bounds_m=TRANSLATION_BOUNDS_M.tolist(),
                rotation_bounds_deg=ROTATION_BOUNDS_DEG.tolist(), weights=WEIGHTS,
                candidates=rows, moveit_queue=[x['refinement_id'] for x in valid[:MOVEIT_LIMIT]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', required=True)
    parser.add_argument('--parent', required=True)
    parser.add_argument('--target-pose', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    parent = np.asarray(json.loads(Path(args.parent).read_text())['T_B_TCP'])
    target = np.asarray(json.loads(Path(args.target_pose).read_text())['T_B_target'])
    Path(args.output).write_text(json.dumps(generate(args.target, parent, target), indent=2))


if __name__ == '__main__': main()
