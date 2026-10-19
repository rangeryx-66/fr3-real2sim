"""Dex1-1 collision meshes and sweep volumes from the pinned Unitree URDF.

The R1-to-Dex1 mount and pad TCP remain the provisional values in the existing
calibration file. No Robotiq geometry or joint convention is used here.
"""

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .candidate import rigid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
URDF = ROOT / 'third_party/unitree_ros/robots/dexterous_hand_description/dex1_1/dex1_1.urdf'
CALIBRATION = ROOT / 'config/end_effector_calibration.yaml'


def origin(node):
    T = np.eye(4)
    if node is not None:
        T[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
        T[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return T


def transform_points(T, points):
    return np.asarray(points) @ T[:3, :3].T + T[:3, 3]


class Dex1Geometry:
    def __init__(self, urdf=URDF, calibration=CALIBRATION):
        self.urdf = Path(urdf)
        self.calibration_path = Path(calibration)
        self.root = ET.parse(self.urdf).getroot()
        self.calibration = json.loads(self.calibration_path.read_text())
        tcp = self.calibration['dex1_base_to_tcp']
        self.T_D_TCP = origin(ET.Element('origin', xyz=' '.join(map(str, tcp['xyz_m'])),
                                         rpy=' '.join(map(str, tcp['rpy_rad']))))
        self.T_D_TCP = rigid(self.T_D_TCP)
        # GraspGenX requires +Z approach and +X closing; the project TCP uses
        # +Z approach and +Y closing. Keep the gripper origin at dex1_base_link.
        self.T_D_GX = np.eye(4)
        self.T_D_GX[:3, :3] = self.T_D_TCP[:3, :3] @ Rotation.from_euler('z', 90, degrees=True).as_matrix()
        self.T_GX_TCP = np.linalg.inv(self.T_D_GX) @ self.T_D_TCP
        self.links = {link.get('name'): link for link in self.root.findall('link')}
        self.joints = {joint.get('name'): joint for joint in self.root.findall('joint')}
        self.children = {}
        for joint in self.joints.values():
            self.children.setdefault(joint.find('parent').get('link'), []).append(joint)
        self.finger_joints = (self.joints['Joint1_1'], self.joints['Joint2_1'])
        self.q_min = float(self.finger_joints[0].find('limit').get('lower'))
        self.q_max = float(self.finger_joints[0].find('limit').get('upper'))
        self.closed_width = float(self.calibration['simulated_opening']['closed_width_m'])
        self.open_width = float(self.calibration['simulated_opening']['open_width_m'])
        self.sha256 = hashlib.sha256(self.urdf.read_bytes()).hexdigest()

    def q_for_width(self, width_m):
        from r1a7_calibration import width_to_finger_q
        return width_to_finger_q(float(width_m), self.urdf)

    def width_for_q(self, q):
        return self.open_width - 2 * (float(q) - self.q_min)

    def link_poses(self, width_m, finger_q=None):
        q = self.q_for_width(width_m)
        poses = {'base_link': np.eye(4)}
        pending = ['base_link']
        while pending:
            parent = pending.pop()
            for joint in self.children.get(parent, []):
                child = joint.find('child').get('link')
                T = poses[parent] @ origin(joint.find('origin'))
                if joint.get('type') == 'prismatic':
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    slide = np.eye(4); slide[:3, 3] = axis * (q if finger_q is None else float(finger_q[0 if joint.get('name') == 'Joint1_1' else 1]))
                    T = T @ slide
                elif joint.get('type') == 'revolute':
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    angle = np.eye(4); angle[:3, :3] = Rotation.from_rotvec(axis * q).as_matrix()
                    T = T @ angle
                poses[child] = T
                pending.append(child)
        return poses

    def finger_translations_in_tcp(self, finger_q):
        reference=self.link_poses(self.open_width)
        actual=self.link_poses(self.open_width,finger_q)
        rotation=np.linalg.inv(self.T_D_TCP)[:3,:3]
        out={}
        for name in reference:
            if not np.allclose(reference[name][:3,:3],actual[name][:3,:3],atol=1e-12):
                raise ValueError('translation-only collision reuse requires prismatic fingers')
            out[name]=rotation@(actual[name][:3,3]-reference[name][:3,3])
        return out

    @lru_cache(maxsize=128)
    def meshes_in_tcp(self, width_m, finger_q=None):
        import trimesh
        from dex1_collision_profile import enabled, proxy_path
        poses = self.link_poses(width_m, finger_q)
        pieces = []
        for name, link in self.links.items():
            if name not in poses:
                continue
            for item in link.findall('collision'):
                mesh_node = item.find('geometry/mesh')
                if mesh_node is None:
                    continue
                mesh_path = (self.urdf.parent / mesh_node.get('filename')).resolve()
                if not mesh_path.is_file():
                    raise FileNotFoundError(mesh_path)
                if enabled():
                    mesh_path = proxy_path(name)
                mesh = trimesh.load(mesh_path, force='mesh', process=True)
                scale = mesh_node.get('scale')
                if scale:
                    mesh.apply_scale(np.fromstring(scale, sep=' '))
                mesh.apply_transform(np.linalg.inv(self.T_D_TCP) @ poses[name] @ origin(item.find('origin')))
                pieces.append((name, mesh))
        if not pieces:
            raise RuntimeError('official Dex1 URDF has no loadable collision meshes')
        return pieces

    def measured_pad_aperture(self, finger_q):
        pads=sorted([mesh.bounds for name,mesh in self.meshes_in_tcp(self.open_width,tuple(finger_q))
                     if name in ('Link1_3','Link2_3')],key=lambda b:b[:,1].mean())
        return float(pads[1][0,1]-pads[0][1,1])

    def sweep_params(self):
        """Derive GraspGenX open/mid free-space boxes from official pad meshes."""
        def box(width):
            pieces = self.meshes_in_tcp(width)
            pads = [mesh for name, mesh in pieces if name in ('Link1_3', 'Link2_3')]
            if len(pads) != 2:
                raise RuntimeError('Dex1 terminal pad collision meshes unavailable')
            # Terminal pad bounds are converted from TCP to the required GX base
            # frame; the aperture itself is derived from URDF joint travel.
            bounds = []
            for mesh in pads:
                vertices = transform_points(self.T_GX_TCP, mesh.vertices)
                bounds.append(np.stack((vertices.min(axis=0), vertices.max(axis=0))))
            low = np.maximum(bounds[0][0], bounds[1][0])
            high = np.minimum(bounds[0][1], bounds[1][1])
            if np.any(high[1:] <= low[1:]):
                raise RuntimeError('terminal pad meshes have no shared Y/Z region')
            extent = np.array([width, high[1] - low[1], high[2] - low[2]])
            offset = np.array([0, (high[1] + low[1]) / 2, (high[2] + low[2]) / 2])
            return extent, offset, max(b[1, 2] for b in bounds)

        open_ext, open_off, fingertip = box(self.open_width)
        mid_width = self.width_for_q((self.q_min + self.q_max) / 2)
        mid_ext, mid_off, _ = box(mid_width)
        if fingertip <= 0:
            raise RuntimeError('Dex1 fingertip is not in +Z of GraspGenX frame')
        return {'extents_open': open_ext.tolist(), 'offset_open': open_off.tolist(),
                'extents_mid': mid_ext.tolist(), 'offset_mid': mid_off.tolist(),
                'gripper_type': 0, 'fingertip_depth': float(fingertip)}

    def provenance(self):
        return {'urdf': str(self.urdf), 'urdf_sha256': self.sha256,
                'collision_links': [name for name, _ in self.meshes_in_tcp(self.open_width)],
                'collision_profile': __import__('os').environ.get('DEX1_COLLISION_PROFILE', 'official_stl'),
                'mount_status': self.calibration['link7_to_dex1_base']['status'],
                'tcp_status': self.calibration['dex1_base_to_tcp']['status']}
