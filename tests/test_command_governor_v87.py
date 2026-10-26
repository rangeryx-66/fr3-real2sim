import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v87.command_governor import CommandGovernor


MOTION=dict(speed_m_s=.012,angular_speed_rad_s=.08,
    acceleration_m_s2=.04,angular_acceleration_rad_s2=.2)


def fk(q):
    t=np.eye(4);t[:3,3]=q[:3];t[:3,:3]=Rotation.from_rotvec(q[3:6]).as_matrix();return t


class GovernorTests(unittest.TestCase):
    def test_replayed_49mm_s_ik_request_is_bounded_with_acceleration(self):
        g=CommandGovernor(fk,1/240);q=np.zeros(7);v=np.zeros(7)
        for i in range(150):
            v=g.velocity(q,np.array([.0497445087,0,0,0,0,0,0]),v,np.ones(7),.8,MOTION)
            q+=v*g.dt
            self.assertLessEqual(g.latest['linear_speed_m_s'],.012+1e-9)
            self.assertLessEqual(g.latest['linear_acceleration_m_s2'],.04+1e-8)
        self.assertGreater(q[0],.004);self.assertGreater(g.limited,0)

    def test_direction_reversal_preserves_slew_and_branch_request(self):
        g=CommandGovernor(fk,1/240);q=np.zeros(7);v=np.zeros(7)
        for direction in [1,-1,1]:
            for i in range(100):
                old=v.copy();wanted=np.array([direction*.04,0,0,0,0,0,0])
                v=g.velocity(q,wanted,v,np.ones(7),.8,MOTION);q+=v*g.dt
                self.assertLessEqual(np.max(np.abs(v-old)),.8*g.dt+1e-10)
                self.assertLessEqual(g.latest['linear_acceleration_m_s2'],.04+1e-8)

    def test_rotational_limit_and_joint_limit_are_independent(self):
        g=CommandGovernor(fk,1/240);q=np.zeros(7);v=np.zeros(7)
        for i in range(120):
            v=g.velocity(q,[0,0,0,.3,0,0,.5],v,np.full(7,.15),.4,MOTION);q+=v*g.dt
            self.assertLessEqual(g.latest['angular_speed_rad_s'],.08+1e-9)
            self.assertLessEqual(g.latest['angular_acceleration_rad_s2'],.2+1e-7)
            self.assertLessEqual(np.max(np.abs(v)),.15+1e-10)

    def test_infeasible_rate_change_rejects_without_committing_state(self):
        g=CommandGovernor(fk,1/240);q=np.zeros(7)
        with self.assertRaisesRegex(RuntimeError,'CONTROLLED_EXECUTION_INTERRUPT'):
            g.velocity(q,[1,0,0,0,0,0,0],[.1,0,0,0,0,0,0],np.ones(7),.8,MOTION)
        self.assertEqual(g.calls,0);np.testing.assert_array_equal(g.previous_twist,np.zeros(6))

    def test_invalid_state_is_never_coerced_to_zero(self):
        g=CommandGovernor(fk,1/240)
        with self.assertRaises(ValueError):g.velocity(np.full(7,np.nan),np.zeros(7),np.zeros(7),np.ones(7),.8,MOTION)

    def test_internal_optimizer_headroom_is_not_a_new_physical_stop_limit(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        g=CommandGovernor(fk,1/240);q=np.zeros(7);v=np.zeros(7);v[0]=MOTION['speed_m_s']
        g.previous_twist[0]=v[0]
        # A numerical optimizer may leave its tightened target unsatisfied
        # while the actual FK command remains within every declared limit.
        result=SimpleNamespace(x=np.zeros(7),success=False)
        with patch('agentic_interaction_v87.command_governor.minimize',return_value=result):
            accepted=g.velocity(q,[.04,0,0,0,0,0,0],v,np.ones(7),.8,MOTION)
        np.testing.assert_allclose(accepted,v)
        self.assertLess(g.latest['minimum_normalized_slack'],0)
        self.assertGreaterEqual(g.latest['minimum_declared_slack'],-1e-12)

    def test_optimizer_success_cannot_authorize_exceeding_original_limit(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        g=CommandGovernor(fk,1/240);q=np.zeros(7);v=np.zeros(7);v[0]=.013
        g.previous_twist[0]=v[0];before=g.previous_twist.copy()
        with patch('agentic_interaction_v87.command_governor.minimize',
                return_value=SimpleNamespace(x=np.zeros(7),success=True)):
            with self.assertRaisesRegex(RuntimeError,'TCP_PROJECTION_INFEASIBLE'):
                g.velocity(q,[.04,0,0,0,0,0,0],v,np.ones(7),.8,MOTION)
        np.testing.assert_array_equal(g.previous_twist,before);self.assertEqual(g.calls,0)

    def test_loaded_motion_brakes_before_entering_slower_release_phase(self):
        from types import SimpleNamespace as NS
        from agentic_interaction_v87.runtime import runtime_class
        Runtime=runtime_class(object);r=Runtime.__new__(Runtime)
        r.dt=1/240;r.arm=np.arange(7);r.qtarget=np.zeros(7)
        r.g=dict(phase='RELEASE',qvelocity=np.array([.012,0,0,0,0,0,0]))
        r.command_governor=CommandGovernor(fk,r.dt)
        r.command_governor.previous_twist[0]=.012
        r.c=dict(arm_speed_limits=np.full(7,3.))
        r.policy=NS(data=dict(arm_command_acceleration_rad_s2=dict(LOADED=.8)),phase=lambda p:'LOADED')
        r._v87_cartesian_active=True;r._v87_last_command_phase='OPEN_CONTINUE'
        r._v87_last_command_motion=dict(MOTION);r.event=lambda *a,**kw:None
        r.check_q=lambda q:(True,None);phases=[]
        r.step=lambda:phases.append(r.g['phase'])
        # Imposing RELEASE's 8mm/s bound on the existing 12mm/s command is
        # infeasible; the previous implementation did exactly that.
        r.motion_limits=lambda phase:dict(MOTION,speed_m_s=.008)
        r.stop_command()
        self.assertTrue(phases);self.assertEqual(set(phases),{'OPEN_CONTINUE'})
        self.assertEqual(r.g['phase'],'RELEASE')
        self.assertLessEqual(np.max(abs(r.g['qvelocity'])),1e-12)
        self.assertLess(r.qtarget[0],.002)


if __name__=='__main__':unittest.main()
