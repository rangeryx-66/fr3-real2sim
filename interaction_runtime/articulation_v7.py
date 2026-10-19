"""Explicit SI articulation prior. No geometry completion or dynamic parameters.

Twists use [linear, angular] world coordinates at the supplied contact origin;
wrenches use [force, moment] at that SAME origin. Their dot product is power.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Articulation:
    joint_type: str
    axis_world: tuple
    origin_world_m: tuple
    limits: tuple
    joint_name: str = ''

    def __post_init__(self):
        if self.joint_type not in ('revolute', 'prismatic'):
            raise ValueError('Only one-axis revolute/prismatic priors supported')
        axis = np.asarray(self.axis_world, float)
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
            raise ValueError('Finite nonzero world axis required')
        if np.shape(self.origin_world_m) != (3,) or not np.isfinite(self.origin_world_m).all():
            raise ValueError('Finite world origin required')
        if len(self.limits) != 2 or not np.isfinite(self.limits).all() or self.limits[0] >= self.limits[1]:
            raise ValueError('Ordered finite native-coordinate limits required')
        object.__setattr__(self, 'axis_world', tuple(axis / np.linalg.norm(axis)))

    @property
    def units(self):
        return dict(coordinate='rad', effort='N m', inertial='kg m^2', damping='N m s/rad') if self.joint_type == 'revolute' else dict(coordinate='m', effort='N', inertial='kg', damping='N s/m')

    def delta(self, displacement):
        """Rigid displacement from q0 to q0+displacement, not an object command."""
        a = np.asarray(self.axis_world)
        D = np.eye(4)
        if self.joint_type == 'revolute':
            R = Rotation.from_rotvec(a * displacement).as_matrix()
            o = np.asarray(self.origin_world_m)
            D[:3, :3] = R
            D[:3, 3] = o - R @ o
        else:
            D[:3, 3] = a * displacement
        return D

    def jacobian(self, point_world):
        a = np.asarray(self.axis_world)
        if self.joint_type == 'prismatic':
            return np.r_[a, np.zeros(3)]
        return np.r_[np.cross(a, np.asarray(point_world) - self.origin_world_m), a]

    def effort(self, wrench_sensor, T_world_sensor):
        """Robot-on-object compensated wrench, not native object torque."""
        T = np.asarray(T_world_sensor)
        w = np.asarray(wrench_sensor)
        return float(self.jacobian(T[:3, 3]) @ np.r_[T[:3, :3] @ w[:3], T[:3, :3] @ w[3:]])

    def path_scale(self, point_world):
        """Metres per native coordinate: hinge perpendicular radius or rail 1."""
        return float(np.linalg.norm(self.jacobian(point_world)[:3]))

    def coordinate_limits(self, point_world, motion):
        scale = self.path_scale(point_world)
        if self.joint_type == 'prismatic':
            return float(motion['speed_m_s']), float(motion['acceleration_m_s2'])
        if scale < 1e-9:
            raise ValueError('Contact on hinge axis has no pulling lever arm')
        return min(motion['speed_m_s']/scale, motion['angular_speed_rad_s']), min(motion['acceleration_m_s2']/scale, motion['angular_acceleration_rad_s2'])

    def validate_target(self, action):
        """No implicit degree/linear conversion at the command boundary."""
        if action.get('coordinate_unit') != self.units['coordinate']:
            raise ValueError('Explicit coordinate_unit must match articulation')
        target = float(action['target_q'])
        if not np.isfinite(target) or not self.limits[0] <= target <= self.limits[1]:
            raise ValueError('Target outside authored joint limits')
        return target

    def packet(self):
        return dict(joint_type=self.joint_type, axis_world=list(self.axis_world), origin_world_m=list(self.origin_world_m), limits_native=list(self.limits), joint_name=self.joint_name, units=self.units, source='explicit known-kinematics prior; dynamics excluded')
