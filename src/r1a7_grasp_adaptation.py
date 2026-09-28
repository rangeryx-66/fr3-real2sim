"""Bounded Dex1 grasp search around an unchanged AnyGrasp TCP target."""
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import struct
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Variant:
    transform: np.ndarray
    label: str
    translation_m: float
    rotation_rad: float


def variants(raw, limit=80):
    """Ordered local alternatives; all distances are metres and angles degrees."""
    raw = np.asarray(raw, dtype=float)
    yield Variant(raw.copy(), 'raw', 0., 0.)
    count = 1
    # The first ring lifts the low Dex1 fingers off the table. Later rings
    # explore insertion depth, lateral centring, and small wrist rotations.
    rings = [
        (dz, depth, side, roll, pitch, yaw)
        for dz in (.005, .010, .015, .020, .025)
        for depth, side, roll, pitch, yaw in (
            (0, 0, 0, 0, 0), (.010, 0, 0, 0, 0), (-.010, 0, 0, 0, 0),
            (0, .005, 0, 0, 0), (0, -.005, 0, 0, 0),
            (0, 0, 10, 0, 0), (0, 0, -10, 0, 0),
            (0, 0, 0, 10, 0), (0, 0, 0, -10, 0),
            (0, 0, 0, 0, 10), (0, 0, 0, 0, -10),
            (.010, 0, 0, 0, 10), (.010, 0, 0, 0, -10),
            (.010, 0, 0, 10, 0), (.010, 0, 0, -10, 0),
            (.020, 0, 0, 10, 0), (.020, 0, 0, -10, 0),
            (.010, 0, 0, 20, 0), (.010, 0, 0, -20, 0),
            (0, 0, 15, 0, 0), (0, 0, -15, 0, 0),
        )
    ]
    for dz, depth, side, roll, pitch, yaw in rings:
        T = raw.copy()
        T[:3, :3] = raw[:3, :3] @ Rotation.from_euler('xyz', (roll, pitch, yaw), degrees=True).as_matrix()
        T[:3, 3] += np.array([0., 0., dz]) + raw[:3, 2]*depth + raw[:3, 1]*side
        dp = float(np.linalg.norm(T[:3, 3] - raw[:3, 3]))
        dr = float((Rotation.from_matrix(raw[:3, :3]).inv()*Rotation.from_matrix(T[:3, :3])).magnitude())
        yield Variant(T, f'z{dz:+.3f}_depth{depth:+.3f}_side{side:+.3f}_rpy{roll:+d},{pitch:+d},{yaw:+d}', dp, dr)
        count += 1
        if count >= limit:
            return


def contact_geometry(T, box_center, box_size=(.045, .045, .05), shape='box', object_yaw=0.):
    """Necessary parallel-jaw contact conditions for the Dex1 terminal pads.

    The official URDF puts the two pad link origins at +/-25.03 mm across the
    TCP jaw axis at zero finger travel. Both pad origins lie on the TCP plane.
    The official pad mesh occupies TCP-local x=-28.4..0 mm and
    z=-2..47.5 mm. We place the object centre inside that pad region and
    between the fingers. Isaac contact is still
    the final test; this geometric proxy cannot assert force closure.
    """
    center = np.asarray(box_center, dtype=float)
    local = T[:3, :3].T @ (center - T[:3, 3])
    jaw = T[:3, 1]
    local_jaw = Rotation.from_euler('z',-object_yaw).apply(jaw)
    if shape=='cylinder':
        projected_width = float(box_size[0]*np.linalg.norm(local_jaw[:2])+box_size[2]*abs(local_jaw[2]))
    else:
        projected_width = float(np.sum(np.abs(local_jaw) * np.asarray(box_size)))
    if not -.026 <= local[0] <= .002:
        return False, 'object outside finger-pad height', 0.
    if abs(local[1]) > .014:
        return False, 'object off jaw centre', 0.
    if not .004 <= local[2] <= .042:
        return False, 'insufficient grasp depth', 0.
    if abs(jaw[2]) > .35 or not .018 <= projected_width <= .085:
        return False, 'jaw alignment/opening', 0.
    score = (max(0., 1.-abs(local[1])/.014) *
             max(0., 1.-abs(local[0]+.014)/.018) *
             max(0., 1.-abs(local[2]-.022)/.030))
    return True, 'pad plane intersects target and jaws bracket centre', float(score)


@lru_cache(maxsize=1)
def open_finger_pad_vertices():
    """Official Dex1 terminal collision vertices at the benchmark's 90 mm opening."""
    mesh_dir=ROOT/'third_party/unitree_ros/robots/dexterous_hand_description/dex1_1/meshes'
    vertices=[]
    for name,origin_x in (('Link1_3',.0450335),('Link2_3',-.0450335)):
        data=(mesh_dir/f'{name}.STL').read_bytes()
        triangles=struct.unpack_from('<I',data,80)[0]
        mesh=np.array([struct.unpack_from('<3f',data,84+50*i+12*j)
                       for i in range(triangles) for j in (1,2,3)])
        # URDF fixed-joint chain and r1a7_tcp_joint: TCP local axes are
        # (Dex base Z, Dex base X, Dex base Y), with the TCP at the pad plane.
        vertices.append(np.stack((mesh[:,2],origin_x+mesh[:,0],mesh[:,1]),axis=1))
    return np.concatenate(vertices)


def pad_table_penetration(T):
    """Guaranteed terminal-mesh/table overlap, used only as an early reject."""
    world=open_finger_pad_vertices()@T[:3,:3].T+T[:3,3]
    inside=(world[:,0]>=.15)&(world[:,0]<=.85)&(world[:,1]>=-.35)&(world[:,1]<=.35)&(world[:,2]>=-.05)&(world[:,2]<0.)
    return float(-world[inside,2].min()) if inside.any() else 0.


def path_targets(grasp):
    pre = grasp.copy(); pre[:3, 3] -= .08*grasp[:3, 2]
    micro = grasp.copy(); micro[2, 3] += .02
    lift = grasp.copy(); lift[2, 3] += .10
    return pre, micro, lift
