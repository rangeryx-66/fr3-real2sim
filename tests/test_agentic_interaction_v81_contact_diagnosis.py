import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v81.contact_diagnosis import point_velocity,transform


class MaterialVelocityTests(unittest.TestCase):
    def test_translating_drawer(self):
        old=np.eye(4);new=old.copy();new[0,3]=.001
        np.testing.assert_allclose(point_velocity(old,new,[.2,.3,.4],.01),[.1,0,0])

    def test_rotating_door_not_axis_times_angular_velocity(self):
        old=np.eye(4);new=old.copy();new[:3,:3]=Rotation.from_rotvec([0,0,.01]).as_matrix()
        np.testing.assert_allclose(point_velocity(old,new,[.3,0,0],.01),[0,.3,0],atol=1e-12)

    def test_pose_roundtrip(self):
        T=transform([[1,2,3,0,0,0,1]])[0]
        np.testing.assert_allclose(T[:3,3],[1,2,3]);np.testing.assert_allclose(T[:3,:3],np.eye(3))


if __name__=='__main__':unittest.main()
