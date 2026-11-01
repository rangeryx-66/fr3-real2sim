import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v89.controller import (
    Bounds, CommandOwner, MeasuredMotion, StopState, WeightedDLS, pose_error, pose_xyzw)


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

    def test_stop_observation_uses_actual_motion_and_rejects_rotating_or_moving_base(self):
        m=MeasuredMotion(1/240);o=CommandOwner(LinearRobot(),np.zeros(9),1/240)
        o.settling()
        for i in range(40):
            # A .02rad/s projected native dq can coexist with static integrated
            # poses; it is retained by runtime, not used as a pose derivative.
            motion=m.update(i/240,np.zeros(9),np.eye(4),0.)
            if motion:o.observe_stop(**motion)
        self.assertTrue(o.measured_stationary)
        for i in range(40,80):
            T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([0,0,.02*(i-40)/240]).as_matrix()
            motion=m.update(i/240,np.zeros(9),T,0.)
            o.observe_stop(**motion)
        self.assertFalse(o.measured_stationary)
        self.assertEqual(o.state,StopState.SETTLING)
        # Stationary TCP from XY/arm cancellation still is NOT measured rest.
        v=np.r_[.001,0,-.001,np.zeros(6)]
        for _ in range(40): self.assertFalse(o.observe_stop(v,0.,0.))
        self.assertIsNone(m.update(10.,np.zeros(9),np.eye(4),0.))

    def test_float32_native_readback_cannot_create_second_proposal_during_brake(self):
        from interaction_runtime.policy import Policy, ForceMonitor
        from agentic_interaction_v89.runtime import runtime_class
        class Parent:
            def __init__(r, job):
                r.job=job; r.dt=1/240; r.base=np.zeros(4)
                r.base_indices=np.array([0,1]); r.arm=np.arange(2,9)
                r.qtarget=np.full(10, .1234567, dtype=np.float32)
                r.base_goal=r.qtarget[:2].copy();r.base_velocity=np.zeros(2)
                r.g=dict(qtarget=r.qtarget,qvelocity=np.zeros(7),phase='SETTLE')
                r.mount=dict(native_limits_m=[-1.,1.])
                r.policy=Policy.load(Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json')
                r.model=SimpleNamespace(limits=np.tile([-2.,2.],(7,1)))
                r.robot=SimpleNamespace(get_joint_positions=lambda:np.full(10,.1234567,dtype=np.float32))
                r.c=dict(arm_speed_limits=np.full(7,.3));r.solver=None
                r.record=SimpleNamespace(coordinate_fields=lambda row:{},metadata={'v81_base':{}},end=lambda *a,**kw:None)
                r.supervisor=SimpleNamespace(force=ForceMonitor(r.policy))
                r.controller=SimpleNamespace(apply_coordinated=Mock());r.jaw_width=.084
                r.phase='SETTLE'
            def step(r):
                r.submit_drive()
                return dict(q=r.robot.get_joint_positions(),joint_velocities=np.zeros(10),phase=r.phase,t=0.)
        def geometry(solver, base, xy, limits, speed):
            k=LinearRobot(); k.limits=limits; k.speeds=speed
            return k
        with patch('agentic_interaction_v89.runtime.WorldKinematics',side_effect=geometry):
            r=runtime_class(Parent)({'v89_controller':'coordinated_dls_v1'})
        self.assertEqual(r.qtarget.dtype,np.float64)
        self.assertIs(r.g['qtarget'],r.qtarget)
        o=r.command_owner
        proposal=o.propose(np.r_[1e-3,np.zeros(8)],bounds())
        self.assertTrue(proposal.accepted,proposal.reason)
        o.commit(proposal);r._write_command()
        generation=o.command.generation
        r.step()
        self.assertEqual(o.command.generation,generation)
        for c in o.start_stop():
            o.commit_brake(c);r._write_command();r.step()
            self.assertEqual(o.command.generation,c.generation)
        self.assertTrue(o.command_zero)
        self.assertGreater(r.controller.apply_coordinated.call_count,1)


if __name__ == '__main__': unittest.main()
