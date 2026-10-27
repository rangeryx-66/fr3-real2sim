import unittest
from unittest.mock import patch
import numpy as np
from agentic_interaction_v87.command_governor import CommandGovernor
from agentic_interaction_v88.stopping import check_stop


class StoppingTests(unittest.TestCase):
    @staticmethod
    def fk(q):
        T=np.eye(4);T[0,3]=q[0]+q[1];return T

    def test_small_tcp_motion_does_not_hide_base_stopping_collision(self):
        dt=1/240;g=CommandGovernor(self.fk,dt,preserve_direction=True)
        g.previous_twist=np.array([.001,0,0,0,0,0]);g.calls=17
        q=np.array([.008,0.]);v=np.array([.02,-.019])
        motion=dict(speed_m_s=.012,acceleration_m_s2=.04,angular_speed_rad_s=.08,
            angular_acceleration_rad_s2=.2)
        self.assertLess(q[0],.01)
        result=check_stop(q,v,g,np.array([.08,.3]),np.array([.04,.8]),motion,
            lambda command:(command[0]<.01,'static obstacle'))
        self.assertFalse(result['valid']);self.assertEqual(result['reason'],'static obstacle')
        self.assertGreater(result['checked'],1)
        np.testing.assert_array_equal(q,[.008,0.]);np.testing.assert_array_equal(v,[.02,-.019])
        np.testing.assert_array_equal(g.previous_twist,[.001,0,0,0,0,0]);self.assertEqual(g.calls,17)

    def test_safe_stop_is_previewed_under_original_bounds(self):
        g=CommandGovernor(self.fk,1/240,preserve_direction=True)
        g.previous_twist=np.array([.004,0,0,0,0,0]);seen=[]
        motion=dict(speed_m_s=.006,acceleration_m_s2=.008,angular_speed_rad_s=.08,
            angular_acceleration_rad_s2=.2)
        def check(q):seen.append(q.copy());return True,'safe'
        result=check_stop(np.zeros(2),np.array([.004,0]),g,[.08,.3],[.008,.8],motion,check)
        self.assertTrue(result['valid']);self.assertGreater(len(seen),2)
        self.assertLess(seen[-1][0],.0011)

    def test_speed_boundary_has_a_feasible_interior_shared_brake(self):
        g=CommandGovernor(self.fk,1/240,preserve_direction=True)
        previous=np.array([.0027407,.00000134]);speed=float(previous.sum())
        g.previous_twist=np.array([speed,0,0,0,0,0])
        motion=dict(speed_m_s=speed/(1-1e-6),acceleration_m_s2=.008,
            angular_speed_rad_s=.08,angular_acceleration_rad_s2=.2)
        # Both clipped endpoints can fail the tighter numerical target. The
        # interior shared fraction must succeed without a multivariable solve.
        with patch('agentic_interaction_v87.command_governor.minimize',
                side_effect=AssertionError('shared scalar brake should suffice')):
            velocity=g.velocity(np.zeros(2),np.zeros(2),previous,[.08,.3],[.008,.8],motion)
        self.assertTrue(np.all(velocity>=0));self.assertTrue(np.all(velocity<previous))
        self.assertLessEqual(g.latest['linear_speed_m_s'],motion['speed_m_s'])
        self.assertLessEqual(g.latest['linear_acceleration_m_s2'],motion['acceleration_m_s2']*(1+1e-12))
        self.assertTrue(np.all(abs(velocity-previous)<=[.008/240+1e-12,.8/240+1e-12]))

    def test_inconsistent_unsafe_twist_is_still_rejected(self):
        g=CommandGovernor(self.fk,1/240,preserve_direction=True)
        g.previous_twist=np.array([.1,0,0,0,0,0])
        motion=dict(speed_m_s=.003,acceleration_m_s2=.008,
            angular_speed_rad_s=.08,angular_acceleration_rad_s2=.2)
        with self.assertRaisesRegex(RuntimeError,'TCP_PROJECTION_INFEASIBLE'):
            g.velocity(np.zeros(2),np.zeros(2),np.array([.002,0]),[.08,.3],[.008,.8],motion)


if __name__=='__main__':unittest.main()
