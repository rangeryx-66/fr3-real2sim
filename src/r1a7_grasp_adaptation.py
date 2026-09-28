"""Bounded Dex1 grasp search around an unchanged AnyGrasp TCP target."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


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


def contact_geometry(T, box_center, box_size=(.045, .045, .05)):
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
    projected_width = float(np.sum(np.abs(jaw) * np.asarray(box_size)))
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


def path_targets(grasp):
    pre = grasp.copy(); pre[:3, 3] -= .08*grasp[:3, 2]
    micro = grasp.copy(); micro[2, 3] += .02
    lift = grasp.copy(); lift[2, 3] += .10
    return pre, micro, lift
