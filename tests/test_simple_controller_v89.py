import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v89.controller import (
    Bounds, CommandOwner, StopState, WeightedDLS, pose_error, pose_xyzw)


class LinearRobot:
    """XY + arm translational cancellation, three rotational DOFs, redundancy."""
    J = np.zeros((6, 9))
    J[:2, :2] = np.eye(2)
    J[:3, 2:5] = np.eye(3)
    J[3:, 5:8] = np.eye(3)

    def fk(self, q):
        x = self.J @ q
        return pose_xyzw(x[:3], Rotation.from_rotvec(x[3:]).as_quat())

    def jacobian(self, q):
        return self.J.copy()


def bounds():
    return Bounds(np.tile([-2., 2.], (9, 1)), np.r_[.08, .08, np.full(7, .5)],
                  np.r_[.04, .04, np.full(7, .8)], .012, .2, .04, .8)


class SimpleControllerTests(unittest.TestCase):
    def test_pose_convention_and_short_rotation(self):
        p = pose_xyzw([.1, .2, .3], [0, 0, 0, 1])
        np.testing.assert_array_equal(pose_error(p, p), np.zeros(6))
        a = pose_xyzw([0, 0, 0], Rotation.from_euler('z', 179, degrees=True).as_quat())
        b = pose_xyzw([0, 0, 0], Rotation.from_euler('z', -179, degrees=True).as_quat())
        self.assertAlmostEqual(abs(pose_error(a, b)[5]), np.deg2rad(2))
        self.assertAlmostEqual(pose_error(p, np.eye(4))[0], .1)

    def test_dls_zero_singularity_and_allocation(self):
        robot = LinearRobot()
        kernel = WeightedDLS(robot)
        b = bounds()
        np.testing.assert_array_equal(kernel.velocity(np.eye(4), np.zeros(9), b), np.zeros(9))
        target = np.eye(4); target[0, 3] = .001
        v = kernel.velocity(target, np.zeros(9), b)
        self.assertGreater(v[0], 0)
        self.assertGreater(v[2], 0)
        self.assertTrue(np.isfinite(v).all())
        fixed = kernel.velocity(target, np.zeros(9), b, mobile=False)
        np.testing.assert_array_equal(fixed[:2], np.zeros(2))
        robot.J = np.zeros((6, 9))  # rank zero, not an optimizer exception
        np.testing.assert_array_equal(kernel.velocity(target, np.zeros(9), b), np.zeros(9))

    def test_candidate_rejection_and_stale_commit_do_not_mutate(self):
        o = CommandOwner(LinearRobot(), np.zeros(9), 1/240)
        before = o.command
        p = o.propose(np.full(9, np.nan), bounds())
        self.assertFalse(p.accepted)
        with self.assertRaises(ValueError): o.commit(p)
        self.assertIs(o.command, before)
        p = o.propose(np.r_[.001, np.zeros(8)], bounds())
        self.assertTrue(p.accepted, p.reason)
        o.commit(p)
        with self.assertRaises(ValueError): o.commit(p)
        q = o.command.position.copy()
        p = o.propose(np.zeros(9), bounds(), lambda q: (False, 'wall'))
        self.assertFalse(p.accepted)
        np.testing.assert_array_equal(o.command.position, q)
        self.assertTrue(o.stop.valid)

    def test_dt_gain_and_motion_time(self):
        endpoints = []
        for dt in (1/60, 1/240):
            robot = LinearRobot(); o = CommandOwner(robot, np.zeros(9), dt)
            kernel = WeightedDLS(robot)
            target = np.eye(4); target[0, 3] = .004
            for _ in range(round(2/dt)):
                p = o.track(kernel, target, o.command.position, bounds())
                self.assertTrue(p.accepted, p.reason)
                o.commit(p)
            endpoints.append(robot.fk(o.command.position)[0, 3])
        self.assertLess(abs(endpoints[0] - endpoints[1]), 5e-5)
        self.assertGreater(endpoints[1], .0039)

    def test_joint_margin_reversal_and_tracking_lag(self):
        o = CommandOwner(LinearRobot(), np.r_[1.9999, np.zeros(8)], 1/240)
        for sign in (1., -1.):
            for _ in range(30):
                p = o.propose(np.r_[sign*.08, np.zeros(8)], bounds())
                self.assertTrue(p.accepted, p.reason)
                o.commit(p)
                self.assertLessEqual(o.command.position[0], 2.)
        # Measured lag never overwrites the command integration origin.
        before = o.command.position.copy()
        target = np.eye(4); target[0, 3] = .02
        p = o.track(WeightedDLS(o.kinematics), target, np.zeros(9), bounds())
        self.assertTrue(p.accepted, p.reason)
        np.testing.assert_allclose(p.command.position, before + p.command.velocity*o.dt)

    def test_coordinated_cancellation_brake_and_measured_dwell(self):
        v = np.r_[.02, 0, -.02, np.zeros(6)]
        o = CommandOwner(LinearRobot(), np.zeros(9), 1/240, velocity=v)
        plan = o.brake(o.command, bounds())
        self.assertTrue(plan.valid, plan.reason)
        o.stop = plan
        for c in o.start_stop():
            np.testing.assert_allclose(c.velocity[0], -c.velocity[2], atol=1e-15)
            self.assertLess(np.linalg.norm(c.twist), 1e-12)
            o.commit_brake(c)
        self.assertTrue(o.command_zero)
        self.assertFalse(o.measured_stationary)
        o.settling()
        for _ in range(100):
            self.assertFalse(o.observe_stop(v, 0., 0.))
        for _ in range(38): o.observe_stop(np.zeros(9), 0., 0.)
        self.assertEqual(o.state, StopState.STOPPED)
        # Restart needs no reset or latch clearing.
        p = o.propose(np.r_[.001, np.zeros(8)], bounds())
        self.assertTrue(p.accepted, p.reason)
        o.commit(p)
        self.assertEqual(o.state, StopState.RUNNING)

    def test_stop_geometry_refusal_is_not_a_motion_commit(self):
        o = CommandOwner(LinearRobot(), np.zeros(9), 1/240)
        before = o.command
        p = o.propose(np.r_[.01, np.zeros(8)], bounds(), lambda q: (q[0] < 1e-9, 'wall'))
        self.assertFalse(p.accepted)
        self.assertIs(o.command, before)


if __name__ == '__main__': unittest.main()
