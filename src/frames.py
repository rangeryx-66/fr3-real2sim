"""Active transforms: T_A_B maps coordinates B -> A, meters, column vectors."""
import numpy as np
from scipy.spatial.transform import Rotation

def rigid(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('TF_ERROR: invalid transform')
    if not np.allclose(T[3], [0,0,0,1], atol=1e-6) or not np.allclose(T[:3,:3].T @ T[:3,:3], np.eye(3), atol=1e-5) or abs(np.linalg.det(T[:3,:3])-1)>1e-5:
        raise ValueError('TF_ERROR: transform is not SE(3)')
    return T

def grasp_to_tcp(T_B_C, rotation, translation, depth):
    # GraspNet: +x approach, +y jaw separation, +z height.
    # Franka Hand TCP: +z approach, +y jaw separation; x_H = -z_G.
    T_C_G = np.eye(4)
    T_C_G[:3,:3] = rotation
    T_C_G[:3,3] = translation
    T_G_H = np.eye(4)
    T_G_H[:3,:3] = [[0,0,1],[0,1,0],[-1,0,0]]
    # GraspNet finger tips are at x_G=depth (graspnetAPI/utils/utils.py).
    # Official FR3 finger collision tip: .0584+.04525+.0185/2=.1129 m
    # in hand coordinates; official TCP is z=.1034 m, hence 9.5 mm behind tip.
    T_G_H[0,3] = float(depth) - 0.0095
    T_B_H = rigid(T_B_C) @ rigid(T_C_G) @ rigid(T_G_H)
    pre = T_B_H.copy()
    pre[:3,3] -= 0.08 * T_B_H[:3,2]
    lift = T_B_H.copy()
    lift[2,3] += 0.10
    return pre, T_B_H, lift

def pose_values(T):
    T = rigid(T)
    return T[:3,3].tolist(), Rotation.from_matrix(T[:3,:3]).as_quat().tolist()
