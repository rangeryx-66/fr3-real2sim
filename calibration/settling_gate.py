"""Settling-aware grasp stability metrics from the existing 240 Hz trace.

This module is deliberately independent of Isaac/ROS so that the exact online
decision can be replayed byte-for-byte on saved traces.  Quaternion fields use
Isaac's wxyz convention and all transforms are active transforms.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Iterable
import math
import numpy as np


WINDOWS_S = (0.1, 0.2, 0.3)
MIN_SETTLE_S = 0.5
MAX_SETTLE_S = 1.0
STABLE_DWELL_S = 0.3
CONTACT_FORCE_N = 0.1
MAX_CONTACT_GAP_S = 0.05
MAX_CUM_TRANSLATION_M = 0.010
MAX_CUM_ROTATION_DEG = 15.0


@dataclass(frozen=True)
class GateThresholds:
    translation_velocity_m_s: float = 0.00025
    angular_velocity_deg_s: float = 0.5
    following_residual_m: float = 0.001
    min_vertical_follow_ratio: float = 0.8


def quat_multiply(a, b):
    aw, ax, ay, az = np.asarray(a, dtype=float)
    bw, bx, by, bz = np.asarray(b, dtype=float)
    return np.array([
        aw*bw-ax*bx-ay*by-az*bz,
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
    ])


def quat_inverse(q):
    q = np.asarray(q, dtype=float)
    return np.r_[q[0], -q[1:]] / np.dot(q, q)


def quat_rotate(q, v):
    return quat_multiply(quat_multiply(q, np.r_[0.0, v]), quat_inverse(q))[1:]


def quat_angle_deg(a, b):
    q = quat_multiply(quat_inverse(a), b)
    return math.degrees(2.0 * math.atan2(np.linalg.norm(q[1:]), abs(q[0])))


def relative_pose(record):
    tq = np.asarray(record['tcp_quat'], dtype=float)
    bq = np.asarray(record['box_quat'], dtype=float)
    p = quat_rotate(quat_inverse(tq), np.asarray(record['box'])-np.asarray(record['tcp']))
    return p, quat_multiply(quat_inverse(tq), bq)


def _theil_sen(t, values):
    """Median pairwise slope. Windows contain <=72 samples at 240 Hz."""
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)
    slopes = []
    for i in range(len(t)-1):
        dt = t[i+1:]-t[i]
        slopes.append((values[i+1:]-values[i])/dt[:, None])
    return np.median(np.concatenate(slopes), axis=0) if slopes else np.zeros(values.shape[1])


def _angular_speed(t, quaternions):
    q0 = quaternions[0]
    angles = np.array([quat_angle_deg(q0, q) for q in quaternions])
    # Angle magnitude is adequate for the short, bounded gate window and is
    # robust to quaternion sign flips.
    return abs(float(_theil_sen(t, angles[:, None])[0]))


def _longest_false_gap(t, good):
    longest = current = 0.0
    for i in range(1, len(t)):
        current = 0.0 if good[i] else current + float(t[i]-t[i-1])
        longest = max(longest, current)
    return longest


def _contact_centroid(record, side):
    contacts = record.get('finger_contacts', [])
    if side >= len(contacts):
        return None
    item = contacts[side]
    points = np.asarray(item.get('points_world_m', []), dtype=float).reshape(-1, 3)
    forces = np.asarray(item.get('normal_force_N', []), dtype=float).reshape(-1)
    keep = forces > CONTACT_FORCE_N
    return points[keep].mean(axis=0) if keep.any() else None


def compute_metrics(records: Iterable[dict], reference=None):
    rows = list(records)
    if len(rows) < 3:
        raise ValueError('stability gate requires at least three trace samples')
    t = np.asarray([r['t'] for r in rows], dtype=float)
    rel = [relative_pose(r) for r in rows]
    relp = np.asarray([x[0] for x in rel])
    relq = np.asarray([x[1] for x in rel])
    refp, refq = rel[0] if reference is None else reference
    cumulative_translation = np.linalg.norm(relp-np.asarray(refp), axis=1)
    cumulative_rotation = np.asarray([quat_angle_deg(refq, q) for q in relq])
    forces = np.asarray([r.get('forces', [0, 0]) for r in rows], dtype=float)
    bilateral = np.min(forces, axis=1) > CONTACT_FORCE_N

    windows = {}
    for width in WINDOWS_S:
        start = int(np.searchsorted(t, t[-1]-width))
        wt = t[start:]
        velocity = _theil_sen(wt, relp[start:])
        contact_speeds = []
        for side in range(2):
            points, times = [], []
            for r in rows[start:]:
                p = _contact_centroid(r, side)
                if p is not None:
                    points.append(p); times.append(r['t'])
            if len(points) >= 3:
                contact_speeds.append(float(np.linalg.norm(_theil_sen(times, points))))
        windows[f'{int(width*1000)}ms'] = {
            'relative_translation_velocity_m_s': float(np.linalg.norm(velocity)),
            'relative_angular_velocity_deg_s': _angular_speed(wt, relq[start:]),
            'max_contact_point_velocity_m_s': max(contact_speeds, default=None),
        }

    tcp_delta = np.asarray(rows[-1]['tcp'])-np.asarray(rows[0]['tcp'])
    target_delta = np.asarray(rows[-1]['box'])-np.asarray(rows[0]['box'])
    vertical_ratio = float(target_delta[2]/tcp_delta[2]) if abs(tcp_delta[2]) > 1e-5 else None
    support_height = min(float(r['box'][2]) for r in rows)
    return {
        'duration_s': float(t[-1]-t[0]),
        'cumulative_translation_m': float(cumulative_translation[-1]),
        'cumulative_rotation_deg': float(cumulative_rotation[-1]),
        'max_cumulative_translation_m': float(cumulative_translation.max()),
        'max_cumulative_rotation_deg': float(cumulative_rotation.max()),
        'windows': windows,
        'max_contact_gap_s': float(_longest_false_gap(t, bilateral)),
        'terminal_bilateral': bool(bilateral[-1]),
        'bilateral_fraction': float(bilateral.mean()),
        'tcp_delta_m': tcp_delta.tolist(),
        'target_delta_m': target_delta.tolist(),
        'vertical_follow_ratio': vertical_ratio,
        'following_residual_m': float(np.linalg.norm(target_delta-tcp_delta)),
        'min_target_center_z_m': support_height,
    }


def evaluate(records, thresholds=GateThresholds(), old_gate=None, reference=None):
    m = compute_metrics(records, reference=reference)
    w = m['windows']['200ms']
    reasons = []
    if not m['terminal_bilateral'] or m['max_contact_gap_s'] > MAX_CONTACT_GAP_S:
        reasons.append('CONTACT_LOSS')
    if m['max_cumulative_translation_m'] > MAX_CUM_TRANSLATION_M:
        reasons.append('CONTINUOUS_SLIP')
    if m['max_cumulative_rotation_deg'] > MAX_CUM_ROTATION_DEG:
        reasons.append('ROTATIONAL_INSTABILITY')
    if w['relative_angular_velocity_deg_s'] > thresholds.angular_velocity_deg_s:
        reasons.append('ROTATIONAL_INSTABILITY')
    if w['relative_translation_velocity_m_s'] > thresholds.translation_velocity_m_s:
        reasons.append('CONTINUOUS_SLIP')
    ratio = m['vertical_follow_ratio']
    if ratio is not None and ratio < thresholds.min_vertical_follow_ratio:
        reasons.append('CONTINUOUS_SLIP')
    if m['following_residual_m'] > thresholds.following_residual_m:
        reasons.append('CONTINUOUS_SLIP')
    priority = ['CONTACT_LOSS', 'DROP', 'ROTATIONAL_INSTABILITY', 'CONTINUOUS_SLIP']
    category = next((x for x in priority if x in reasons), None)
    passed = category is None
    old_passed = None if old_gate is None else bool(old_gate.get('passed'))
    if passed:
        category = 'SETTLING_THEN_STABLE' if old_passed is False else 'STABLE'
    return {
        'passed': passed,
        'category': category,
        'reasons': sorted(set(reasons), key=lambda x: priority.index(x)),
        'thresholds': asdict(thresholds),
        'metrics': m,
    }


def thresholds_from_controls(control_metrics):
    """Freeze control-derived thresholds exactly as specified by the protocol."""
    trans = [x['windows']['200ms']['relative_translation_velocity_m_s'] for x in control_metrics]
    angular = [x['windows']['200ms']['relative_angular_velocity_deg_s'] for x in control_metrics]
    residual = [x['following_residual_m'] for x in control_metrics]
    return GateThresholds(
        translation_velocity_m_s=max(0.00025, 5*float(np.percentile(trans, 99))),
        angular_velocity_deg_s=max(0.5, 5*float(np.percentile(angular, 99))),
        following_residual_m=max(0.001, 5*float(np.percentile(residual, 99))),
        min_vertical_follow_ratio=0.8,
    )
