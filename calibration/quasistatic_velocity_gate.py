"""Configuration-derived velocity for STATIC acceptance only.

Isaac's native velocity is retained separately; this does not replace dynamic
PayloadID velocities. Require consecutive 240 Hz samples, fail closed on gaps.
"""
import numpy as np

def configuration_velocity(t, q, expected_dt=1/240):
    t, q = np.asarray(t, float), np.asarray(q, float)
    if t.ndim != 1 or q.shape != (len(t), 7) or len(t) < 2:
        raise ValueError('INVALID_VELOCITY_SAMPLE_SHAPE')
    if not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError('NONFINITE_VELOCITY_SAMPLE')
    dt = np.diff(t)
    if not np.allclose(dt, expected_dt, rtol=1e-4, atol=1e-8):
        raise ValueError('NONCONSECUTIVE_SIMULATION_TIMESTAMPS')
    return np.diff(q, axis=0) / dt[:, None]
