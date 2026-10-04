"""Bounded Cartesian command proposals from a saved EE-estimated model.

Only external directional drive references are proposed. The unchanged native
feedback controller computes efforts using its own q/qdot; no q replay occurs.
Each conditional trial starts once from the same observable grasp snapshot.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def make_tape(snapshot, estimate, supporting_poses, spec):
    a = np.asarray(estimate['revolute']['axis'], float)
    c = np.asarray(estimate['revolute']['point_on_axis'], float)
    poses = np.asarray(supporting_poses)
    rv = Rotation.from_matrix(poses[-1, :3, :3]@poses[0, :3, :3].T).as_rotvec()
    sign = 1. if rv@a >= 0 else -1.
    x = np.asarray(snapshot['T_ee'], float)[:3, 3].copy()
    def tangent(p):
        d = sign*np.cross(a, p-c)
        return d/np.linalg.norm(d)
    d = tangent(x); frames = []; dt = 1/240
    template = {'compliant': True, 'arm_position': snapshot['arm_position_command'],
                'arm_velocity': [0.]*6, 'arm_effort': [0.]*6, 'finger_mode': 'effort',
                'finger_position': snapshot['finger_position_command'],
                'finger_effort': snapshot['finger_effort_n'], 'retention_armed': True,
                'diagnostics': {}}
    total = .5+spec['duration_s']+2.
    last_direction = 1.; ramp = 0.
    for i in range(round(total/dt)):
        t = i*dt-.5
        phase = 'COMPLIANT_SETTLE' if t < 0 else ('FINAL_HOLD' if t >= spec['duration_s'] else spec['probe_id'])
        direction = 1.; active = False
        for lo, hi, sgn in spec['windows']:
            if lo <= t < hi: active = True; direction = float(sgn)
        if direction != last_direction and active: ramp = 0.
        ramp += dt
        if active:
            d = direction*tangent(x)
            x += d*spec['speed_m_s']*min(1., ramp/2.)*dt
            last_direction = direction
        frames.append({**template, 't': i*dt, 'phase': phase,
                       'cartesian_input': {'direction_world': d.tolist(),
                                           'reference_world_m': x.tolist(), 'active': active}})
    return frames


def action_travel(log, spec):
    t = np.asarray(log['time_s']); p = np.asarray(log['signals']['ee_T_world_tcp'])[:, :3, 3]
    selection = np.ones(len(t), bool)
    if spec['name'] == 'reverse':
        lo, hi, _ = next(w for w in spec['windows'] if w[2] < 0)
        selection = (t >= lo+.5)&(t <= hi-.5)
    selected = p[selection]
    return {'excursion_m': float(np.max(np.linalg.norm(selected-selected[0], axis=1))) if len(selected) else 0.,
            'path_length_m': float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
            'window_is_reverse_only': spec['name'] == 'reverse'}
