"""URDF joint geometry used to predict a grasped part's rigid motion."""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    out[:3, 3] = xyz
    return out


def _vector(element, key, default):
    return np.fromstring(element.get(key, default), sep=' ', dtype=float)


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None

    def pose(self, q):
        motion = np.eye(4)
        if self.kind in ('revolute', 'continuous'):
            motion[:3, :3] = Rotation.from_rotvec(self.axis * q).as_matrix()
        elif self.kind == 'prismatic':
            motion[:3, 3] = self.axis * q
        elif self.kind != 'fixed':
            raise ValueError(f'unsupported URDF joint type {self.kind}')
        return self.origin @ motion


class URDFChain:
    def __init__(self, path):
        self.path = Path(path)
        robot = ET.parse(self.path).getroot()
        self.joints = {}
        for item in robot.findall('joint'):
            origin = item.find('origin')
            origin = origin if origin is not None else ET.Element('origin')
            axis = item.find('axis')
            axis = axis if axis is not None else ET.Element('axis')
            limit = item.find('limit')
            kind = item.get('type')
            vector = _vector(axis, 'xyz', '1 0 0')
            if np.linalg.norm(vector) == 0:
                if kind != 'fixed':
                    raise ValueError('zero joint axis')
                vector = np.array([1., 0., 0.])
            vector /= np.linalg.norm(vector)
            joint = Joint(item.get('name'), kind, item.find('parent').get('link'),
                          item.find('child').get('link'),
                          transform(_vector(origin, 'xyz', '0 0 0'),
                                    _vector(origin, 'rpy', '0 0 0')),
                          vector,
                          float(limit.get('lower')) if limit is not None and limit.get('lower') is not None else None,
                          float(limit.get('upper')) if limit is not None and limit.get('upper') is not None else None)
            if joint.child in self.joints:
                raise ValueError(f'multiple parents for {joint.child}')
            self.joints[joint.child] = joint
        links = {link.get('name') for link in robot.findall('link')}
        roots = links - set(self.joints)
        if len(roots) != 1:
            raise ValueError(f'expected one URDF root link, got {roots}')
        self.root = roots.pop()

    def root_to_link(self, link, positions):
        result = np.eye(4)
        chain = []
        while link != self.root:
            joint = self.joints[link]
            chain.append(joint)
            link = joint.parent
        for joint in reversed(chain):
            result = result @ joint.pose(positions.get(joint.name, 0.0))
        return result

    def target_tcp(self, *, moving_link, joint_name, q_now, q_target,
                   T_world_moving_now, T_world_tcp_now):
        joint = next(j for j in self.joints.values() if j.name == joint_name)
        if joint.kind not in ('revolute', 'prismatic'):
            raise ValueError('target joint must be revolute or prismatic')
        if joint.lower is not None and q_target < joint.lower - 1e-6:
            raise ValueError('target below joint limit')
        if joint.upper is not None and q_target > joint.upper + 1e-6:
            raise ValueError('target above joint limit')
        now = self.root_to_link(moving_link, {joint_name: q_now})
        future = self.root_to_link(moving_link, {joint_name: q_target})
        T_world_root = T_world_moving_now @ np.linalg.inv(now)
        T_moving_tcp = np.linalg.inv(T_world_moving_now) @ T_world_tcp_now
        return T_world_root @ future @ T_moving_tcp
