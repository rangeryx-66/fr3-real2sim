"""One SE(3) and score contract for all candidate generators."""

from dataclasses import dataclass, field
from typing import Any
import numpy as np


def rigid(value, name='transform'):
    T = np.asarray(value, dtype=np.float64)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f'{name} must be a finite 4x4 matrix')
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError(f'{name} has an invalid homogeneous row')
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-3) or abs(np.linalg.det(R) - 1) > 1e-3:
        raise ValueError(f'{name} rotation is not SO(3)')
    return T


@dataclass
class GraspCandidate:
    model: str
    rank: int
    score: float
    T_C_G: np.ndarray
    T_G_TCP: np.ndarray
    T_B_C: np.ndarray
    width_m: float | None = None
    depth_m: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)
    checks: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        self.T_C_G = rigid(self.T_C_G, 'T_C_G')
        self.T_G_TCP = rigid(self.T_G_TCP, 'T_G_TCP')
        self.T_B_C = rigid(self.T_B_C, 'T_B_C')
        self.score = float(self.score)
        if not np.isfinite(self.score):
            raise ValueError('score must be finite')
        for key in ('width_m', 'depth_m'):
            value = getattr(self, key)
            if value is not None and (not np.isfinite(value) or value < 0):
                raise ValueError(f'{key} must be nonnegative or null')

    @property
    def T_C_TCP(self):
        return self.T_C_G @ self.T_G_TCP

    @property
    def T_B_TCP(self):
        return self.T_B_C @ self.T_C_TCP

    def to_dict(self):
        return {
            'model': self.model, 'rank': self.rank, 'score': self.score,
            'T_C_G': self.T_C_G.tolist(), 'T_G_TCP': self.T_G_TCP.tolist(),
            'T_C_TCP': self.T_C_TCP.tolist(), 'T_B_C': self.T_B_C.tolist(),
            'T_B_TCP': self.T_B_TCP.tolist(), 'width_m': self.width_m,
            'depth_m': self.depth_m, 'metadata': self.metadata, 'checks': self.checks,
        }


def graspnet_to_tcp(depth_m):
    """Reuse the GraspNet/AnyGrasp convention in src/r1a7_frames.py."""
    T = np.eye(4)
    T[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    T[0, 3] = float(depth_m)
    return T
