"""Coupled controller fault tests; no simulated operation is claimed here."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v88.kinematics import CoupledKinematics
from agentic_interaction_v87.command_governor import CommandGovernor

MOTION = dict(speed_m_s=.012, angular_speed_rad_s=.08,
    acceleration_m_s2=.04, angular_acceleration_rad_s2=.2)


class LinearCoupled(CoupledKinematics):
    def __init__(self):
        self.limits = np.tile([-1., 1.], (9, 1))
        self.speeds = np.r_[.08, .08, np.full(7, .3)]
        self.latest = {}

    def fk(self, q):
        pose = np.eye(4); pose[:3, 3] = [q[0] + .3 * q[2], q[1] + .3 * q[3], .3 * q[4]]
        pose[:3, :3] = Rotation.from_rotvec(q[5:8]).as_matrix()
        return pose

    def jacobian(self, q):
        J = np.zeros((6, 9)); J[:2, :2] = np.eye(2); J[:3, 2:5] = .3 * np.eye(3)
        J[3:, 5:8] = np.eye(3)
        return J


class CoordinationTests(unittest.TestCase):
    def test_translation_is_primarily_base_with_arm_compensation(self):
        model = LinearCoupled(); q = np.zeros(9); target = np.eye(4); target[0, 3] = .02
        result = model.solve(target, q)
        self.assertIsNotNone(result)
        self.assertGreater(result[0], .018)
        self.assertLess(abs(.3 * result[2]), .002)
        self.assertLess(np.linalg.norm(model.fk(result)[:3, 3] - target[:3, 3]), 1e-5)

    def test_arc_translation_and_orientation_are_solved_in_same_vector(self):
        model = LinearCoupled(); q = np.zeros(9)
        target = np.eye(4); target[:2, 3] = [.01, .015]
        target[:3, :3] = Rotation.from_rotvec([0, 0, .1]).as_matrix()
        result = model.solve(target, q)
        self.assertIsNotNone(result); self.assertGreater(np.linalg.norm(result[:2]), .015)
        self.assertGreater(abs(result[7]), .09)
        np.testing.assert_allclose(model.fk(result), target, atol=1e-4)

    def test_base_limit_changes_allocation_without_crossing_native_limit(self):
        model = LinearCoupled(); q = np.zeros(9); q[0] = .999
        target = model.fk(q); target[0, 3] += .01
        result = model.solve(target, q)
        self.assertIsNotNone(result)
        self.assertLessEqual(result[0], 1.)
        self.assertGreater(result[2], .02)

    def test_nonlinear_governor_counts_base_and_arm_tcp_velocity_together(self):
        model = LinearCoupled(); governor = CommandGovernor(model.fk, 1/240)
        q = np.zeros(9); velocity = np.zeros(9)
        for direction in [1, -1]:
            for _ in range(100):
                wanted = np.zeros(9); wanted[0] = .03 * direction; wanted[2] = .03 * direction
                previous = velocity.copy()
                velocity = governor.velocity(q, wanted, previous, model.speeds, .04, MOTION)
                q += velocity / 240
                self.assertLessEqual(governor.latest['linear_speed_m_s'], MOTION['speed_m_s'] * (1+1e-12))
                self.assertLessEqual(governor.latest['linear_acceleration_m_s2'], MOTION['acceleration_m_s2'] * (1+1e-12))
                self.assertLessEqual(np.max(abs(velocity-previous)), .04/240+1e-12)

    def test_canceling_joint_motion_is_not_treated_as_tcp_overspeed(self):
        model = LinearCoupled(); governor = CommandGovernor(model.fk, 1/240)
        wanted = np.zeros(9); wanted[0] = .0001; wanted[2] = -.0001/.3
        velocity = governor.velocity(np.zeros(9), wanted, np.zeros(9), model.speeds, 1., MOTION)
        np.testing.assert_allclose(velocity, wanted)
        self.assertLess(governor.latest['linear_speed_m_s'], 1e-12)

    def test_invalid_native_state_rejects_joint_solve(self):
        model = LinearCoupled(); target = np.eye(4)
        q = np.zeros(9); q[0] = np.nan
        with self.assertRaises((ValueError, RuntimeError)):
            model.velocity(target, q, np.zeros(9), 1/240, .04, MOTION, np.zeros(7))

    def test_combined_collision_check_uses_proposed_base_not_old_station(self):
        from agentic_interaction_v88.runtime import runtime_class
        Runtime = runtime_class(object); r = Runtime.__new__(Runtime)
        r._coord_active = True; r.base_goal = np.array([.1, .2]); r.fingers = [7, 8]
        r.robot = NS(get_joint_positions=lambda: np.zeros(9))
        r.policy = NS(data={'joint_margin_rad': .05})
        r.model = NS(margin=lambda q: .5, poses=Mock(return_value='combined_poses'))
        r.coupled = NS(base=lambda xy: np.r_[xy, 0., 0.])
        r.collision = NS(check=Mock(return_value=(False, 'SUPPORT_SCENE_COLLISION')))
        r.moving = lambda: 'actual_object'; r.intentional_contact = lambda: True
        self.assertEqual(r.check_q(np.zeros(7)), (False, 'SUPPORT_SCENE_COLLISION'))
        np.testing.assert_allclose(r.model.poses.call_args.args[1], [.1, .2, 0., 0.])
        r.collision.check.assert_called_once_with('combined_poses', 'actual_object', True)

    def test_joint_stop_brakes_base_and_arm_under_last_loaded_limits(self):
        from agentic_interaction_v88.runtime import runtime_class
        Runtime = runtime_class(object); r = Runtime.__new__(Runtime)
        r._coord_active = True; r.dt = 1/240; r.arm = np.arange(7)
        r.qtarget = np.zeros(7); r.base_goal = np.zeros(2)
        r.base_velocity = np.array([.006, 0.]); r.g = dict(phase='RELEASE', qvelocity=np.zeros(7))
        r._v87_last_command_phase = 'OPEN_CONTINUE'; r._v87_last_command_motion = MOTION
        r.policy = NS(data={'arm_command_acceleration_rad_s2': {'LOADED': .8}}, phase=lambda p: 'LOADED')
        r.coupled = LinearCoupled(); r.coupled_governor = CommandGovernor(r.coupled.fk, r.dt)
        r.coupled_governor.previous_twist[0] = .006
        r.check_q = lambda q: (True, None); seen = []
        r.step = lambda: seen.append((r.g['phase'], r.base_velocity.copy(), r.g['qvelocity'].copy()))
        r.stop_command()
        self.assertTrue(seen); self.assertEqual({s[0] for s in seen}, {'OPEN_CONTINUE'})
        self.assertLess(np.max(abs(np.r_[r.base_velocity, r.g['qvelocity']])), 1e-12)
        self.assertEqual(r.g['phase'], 'RELEASE')
        self.assertLess(r.base_goal[0], .001)

    def test_actual_native_base_overspeed_latches_and_is_never_cleared(self):
        from agentic_interaction_v88.runtime import runtime_class
        from interaction_runtime.policy import SafetyStop
        class Parent:
            def step(self): return self.next_row
        Runtime = runtime_class(Parent); r = Runtime.__new__(Runtime)
        r._install_coord_fields = lambda: None; r.coupled = object()
        r.base_goal = np.zeros(2); r.base_velocity = np.zeros(2)
        r.mount = {'native_limits_m': [-1., 1.]}; r.base_indices = np.array([0, 1])
        r.next_row = dict(q=np.zeros(9), joint_velocities=np.r_[.081, np.zeros(8)], phase='OPEN_CONTINUE', t=1.)
        r.supervisor = NS(latched=None); r.event = Mock()
        with self.assertRaisesRegex(SafetyStop, 'NATIVE_BASE_VELOCITY'): r.step()
        self.assertEqual(r.supervisor.latched['reason'], 'NATIVE_BASE_VELOCITY')
        self.assertEqual(r.supervisor.latched['base_velocity_m_s'], [.081, 0.])

    def test_moving_base_tool_inertia_and_wrench_shift_use_world_geometry(self):
        from xarm_wrist_ft import tool_inertial_world, wrench_at
        body = dict(mass_kg=2., com_acceleration_world_m_s2=[.1, 0., 0.],
            omega_world_rad_s=[0., 0., 0.], alpha_world_rad_s2=[0., 0., 0.],
            inertia_world_kg_m2=np.eye(3), com_world_m=[1., 2., 3.])
        wrench = tool_inertial_world([body], np.array([1., 2., 2.]))
        np.testing.assert_allclose(wrench, [.2, 0., 0., 0., .2, 0.])
        sensor = np.eye(4); sensor[:3, 3] = [1., 2., 2.]
        shifted = wrench_at(wrench, sensor, np.array([1., 2., 0.]))
        np.testing.assert_allclose(shifted, [.2, 0., 0., 0., .6, 0.])


if __name__ == '__main__': unittest.main()
