"""Unchanged AnyGrasp grasp frame to the Dex1 fingertip TCP convention."""
import numpy as np
from frames import rigid


def grasp_to_r1a7_tcp(T_B_C, rotation, translation, depth):
    # GraspNet: +X approach, +Y jaw separation. Dex1 TCP: +Z approach,
    # +Y jaw separation. TCP is defined at the official terminal-pad plane.
    T_C_G = np.eye(4)
    T_C_G[:3, :3] = rotation
    T_C_G[:3, 3] = translation
    T_G_TCP = np.eye(4)
    T_G_TCP[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    T_G_TCP[0, 3] = float(depth)
    grasp = rigid(T_B_C) @ rigid(T_C_G) @ rigid(T_G_TCP)
    pre = grasp.copy()
    pre[:3, 3] -= .08 * grasp[:3, 2]
    micro = grasp.copy()
    micro[2, 3] += .02
    lift = grasp.copy()
    lift[2, 3] += .10
    return pre, grasp, micro, lift
