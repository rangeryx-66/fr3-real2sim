"""Known rigid motions distinguish actuator tracking from contact motion."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v81.feedback import motion_errors


class CartesianFeedbackTests(unittest.TestCase):
    def test_quarter_turn_is_not_1570mm_of_drawer_travel(self):
        prior = dict(joint_type='revolute', axis_world=[0, 0, 1], origin_world_m=[1, 2, 3])
        q = np.linspace(0, np.pi/2, 31)
        measured = np.array([1, 2, 3]) + Rotation.from_rotvec(q[:, None]*[0, 0, 1]).apply([.4, 0, .2])
        command = measured + [.001, -.002, 0]
        result = motion_errors(command, measured, q, prior)
        np.testing.assert_allclose(result['measured_minus_rigid_articulation_world_m'], 0, atol=1e-14)
        np.testing.assert_allclose(np.linalg.norm(result['command_minus_measured_world_m'], axis=1), np.sqrt(5)*.001)
        np.testing.assert_allclose(result['incremental_servo_error_world_m'], 0, atol=1e-14)

    def test_real_relative_motion_is_not_assigned_to_servo_tracking(self):
        q = np.array([0., .3, .7])
        measured = Rotation.from_rotvec(q[:, None]*[0, 0, 1]).apply([.4, 0, 0])
        measured[-1] += [.003, 0, 0]
        result = motion_errors(measured.copy(), measured, q,
            dict(joint_type='revolute', axis_world=[0, 0, 1], origin_world_m=[0, 0, 0]))
        np.testing.assert_allclose(result['command_minus_measured_world_m'], 0)
        self.assertAlmostEqual(np.linalg.norm(result['measured_minus_rigid_articulation_world_m'][-1]), .003)

    def test_linear_native_metres_and_nonzero_anchor(self):
        q = np.array([.02, .04, .08])
        measured = np.c_[np.zeros(3), -q, np.ones(3)]
        result = motion_errors(measured, measured, q,
            dict(joint_type='prismatic', axis_world=[0, -3, 0], origin_world_m=[0, 0, 0]), anchor=1)
        np.testing.assert_allclose(result['measured_minus_rigid_articulation_world_m'], 0)


if __name__ == '__main__':
    unittest.main()
