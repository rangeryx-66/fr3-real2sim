"""Bounded rail-transverse contact retention; no jaw/force/physics changes.

Development primitive: reuse the existing closure's contact-stability band,
geometric centering sign, displacement and step limits. Apply at most one small
centering step per existing force-filter window, rather than integrating the
same delayed load imbalance at every physics step. The safety supervisor and
finite loading budget remain separate and authoritative.
"""
import numpy as np


class PrismaticContactBalance:
    def __init__(self, policy, rail_axis, filter_steps, initial_center_offset, dt=1/240):
        self.policy = policy
        self.rail = np.asarray(rail_axis, float).copy(); self.rail /= np.linalg.norm(self.rail)
        self.period = max(1, int(filter_steps)); self.last_tick = None
        self.step = min(policy['max_centering_step_m'], policy['max_centering_speed_m_s'] * self.period * dt)
        self.initial = np.asarray(initial_center_offset, float).copy()
        self.offset = np.zeros(3); self.corrections = 0

    def update(self, tick, forces, measured_pad_centers):
        if self.last_tick is not None and tick - self.last_tick < self.period:
            return None
        self.last_tick = int(tick)
        f = np.asarray(forces, float); centers = np.asarray(measured_pad_centers, float)
        if f.shape!=(2,) or centers.shape!=(2,3) or not np.isfinite(f).all() or not np.isfinite(centers).all():
            raise ValueError('RETENTION_FEEDBACK_NONFINITE_OR_WRONG_SHAPE')
        stable_load = .8 * self.policy['preload_n']  # existing FORCE_HOLD band
        if min(f) >= stable_load or max(f) < stable_load:
            return None  # no symmetry requirement and no new execution veto
        axis = centers[0] - centers[1]
        axis -= self.rail * (axis @ self.rail)
        length = np.linalg.norm(axis)
        if not np.isfinite(length) or length < 1e-12:
            return dict(applied=False, reason='NO_TRANSVERSE_CENTERING_DIRECTION')
        # Move toward the more-loaded pad, unloading it and bringing the
        # opposite pad toward contact. Direction derives from measured frames.
        delta = axis / length * np.sign(f[0] - f[1]) * self.step
        current = self.initial + self.offset
        bound = self.policy['max_centering_displacement_m']
        # Clip only the new transverse step; never rescale the old displacement
        # and accidentally introduce a rail command at the spherical bound.
        a = delta @ delta; b = current @ delta; c = current @ current - bound**2
        fraction = 0. if c > 1e-15 else float(np.clip((-b + np.sqrt(max(0., b*b - a*c))) / a, 0, 1))
        actual = delta * fraction; self.offset += actual
        applied = bool(np.linalg.norm(actual) > 1e-12)
        self.corrections += int(applied)
        return dict(applied=applied, reason='EXISTING_CENTERING_BOUND' if not applied else 'RETAIN_CONTACT_WITHOUT_MORE_SQUEEZE',
                    pad_loads_n=f.tolist(), step_world_m=actual.tolist(),
                    offset_world_m=self.offset.tolist(),
                    total_center_offset_world_m=(self.initial+self.offset).tolist(),
                    correction_count=self.corrections, jaw_command_changed=False,
                    rail_component_m=float(self.offset @ self.rail),
                    force_filter_window_steps=self.period)
