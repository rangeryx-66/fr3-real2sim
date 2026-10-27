import unittest
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


if __name__=='__main__':unittest.main()
