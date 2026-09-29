"""Identical target, approach, and Dex1 geometry filters for all providers."""

import numpy as np
from scipy.spatial import cKDTree


def filter_candidates(candidates, scene, collision_checker, *, target_radius_m=.04,
                      approach_C=None, approach_max_angle_deg=180, max_candidates=100):
    target = scene.target_C
    tree = cKDTree(target)
    direction = None if approach_C is None else np.asarray(approach_C, dtype=float)
    if direction is not None:
        direction /= np.linalg.norm(direction)
    counts = {'raw': len(candidates), 'target': 0, 'approach': 0,
              'collision_free': 0, 'collision': 0, 'collision_unknown': 0,
              'low_clearance': 0, 'width_unreachable': 0}
    accepted = []
    for candidate in candidates:
        # Providers disagree on grasp origin (wrist vs. contact centre). Test
        # both the native origin and the calibrated Dex1 pad TCP.
        distance = float(min(tree.query(candidate.T_C_G[:3, 3])[0],
                             tree.query(candidate.T_C_TCP[:3, 3])[0]))
        candidate.checks['target_distance_m'] = distance
        if distance > target_radius_m:
            candidate.checks['target'] = 'OUTSIDE_TARGET'
            continue
        candidate.checks['target'] = 'PASS'
        counts['target'] += 1
        if direction is not None:
            approach = candidate.T_C_TCP[:3, 2]
            cosine = float(np.clip(np.dot(approach, direction), -1, 1))
            angle = float(np.degrees(np.arccos(cosine)))
            candidate.checks['approach_angle_deg'] = angle
            if angle > approach_max_angle_deg:
                candidate.checks['approach'] = 'REJECTED'
                continue
        candidate.checks['approach'] = 'PASS'
        counts['approach'] += 1
        collision = collision_checker.check(candidate.T_B_TCP, candidate.width_m)
        candidate.checks['dex1_scene_collision'] = collision
        if collision['status'] == 'COLLISION':
            counts['collision'] += 1
            continue
        if collision['status'] == 'WIDTH_UNREACHABLE':
            counts['width_unreachable'] += 1
            continue
        if collision['status'] == 'LOW_CLEARANCE':
            counts['low_clearance'] += 1
        elif collision['status'] != 'FREE':
            counts['collision_unknown'] += 1
        else:
            counts['collision_free'] += 1
        accepted.append(candidate)
    accepted.sort(key=lambda c: (c.checks['dex1_scene_collision']['status'] == 'FREE', c.score),
                  reverse=True)
    return accepted[:max_candidates], counts
