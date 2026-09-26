"""Piper mounting/workspace transform without changing grasp poses or policy.

Nominal coordinates are the frozen benchmark coordinates in which the robot
base was at the origin.  ``T_B_N`` maps those nominal coordinates into the
coordinates of a newly mounted Piper base.  The optional workspace shift moves
the complete camera/object/clutter layout together and therefore preserves all
object-relative geometry.
"""
from __future__ import annotations
from dataclasses import dataclass
import json
import os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class MountConfig:
    base_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_yaw_deg: float = 0.0
    workspace_shift_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def T_base_nominal(self) -> np.ndarray:
        """Transform shifted nominal scene coordinates into Piper base."""
        T_nominal_base=np.eye(4)
        T_nominal_base[:3,:3]=Rotation.from_euler('z',self.base_yaw_deg,degrees=True).as_matrix()
        T_nominal_base[:3,3]=self.base_xyz_m
        T_nominal_shift=np.eye(4);T_nominal_shift[:3,3]=self.workspace_shift_xyz_m
        return np.linalg.inv(T_nominal_base) @ T_nominal_shift

    def as_dict(self) -> dict:
        return dict(base_xyz_m=list(self.base_xyz_m),base_yaw_deg=float(self.base_yaw_deg),
                    workspace_shift_xyz_m=list(self.workspace_shift_xyz_m))


def from_dict(data: dict) -> MountConfig:
    return MountConfig(tuple(map(float,data.get('base_xyz_m',(0,0,0)))),
                       float(data.get('base_yaw_deg',0.0)),
                       tuple(map(float,data.get('workspace_shift_xyz_m',(0,0,0)))))


def load_mount(path: str | Path | None = None) -> MountConfig:
    path=path or os.environ.get('PIPER_MOUNT_CONFIG')
    if not path:return MountConfig()
    return from_dict(json.loads(Path(path).read_text()))


def transform_matrix(T_nominal: np.ndarray, config: MountConfig) -> np.ndarray:
    return config.T_base_nominal @ np.asarray(T_nominal,dtype=float)


def transform_pose(position, quaternion_wxyz, config: MountConfig):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(quaternion_wxyz,-1)).as_matrix();T[:3,3]=position
    out=transform_matrix(T,config)
    return out[:3,3],np.roll(Rotation.from_matrix(out[:3,:3]).as_quat(),1)


def transform_point(position, config: MountConfig) -> np.ndarray:
    return transform_matrix(np.vstack((np.column_stack((np.eye(3),position)),[0,0,0,1])),config)[:3,3]


def transform_grasp_data(data: dict, config: MountConfig) -> dict:
    """Return a copy with only the saved camera extrinsic re-expressed."""
    out=dict(data)
    out['T_B_C']=transform_matrix(np.asarray(data['T_B_C'],dtype=float),config).tolist()
    return out
