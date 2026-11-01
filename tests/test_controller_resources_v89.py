"""Empirical scheduling forecasts cannot substitute for actual/native limits."""
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np


class ResourceAuthorityTests(unittest.TestCase):
    def test_cost_forecast_is_advisory_but_real_budget_still_binds(self):
        from agentic_interaction_v81.replanning import admission
        available=dict(wall_s=60.,sim_s=100.)
        r=NS(job={'v81_integrated_repair':True},tick=0,dt=1/240,actual=lambda:0.,
            episode_budget=NS(remaining=lambda t:available.copy()))
        ops=NS(r=r,_v81_phase_costs=dict(acquire=100.,release=100.,
            healthy_motion_wall_s_per_native=10000.))
        with patch('agentic_interaction_v81.recovery_batch.measured_costs',
                return_value={'observed_max_operation_wall_s':{}}):
            old=admission(ops,[{'target_q':.012}])
            self.assertFalse(old['admitted'])
            r.job['v89_controller']='coordinated_dls_v1'
            new=admission(ops,[{'target_q':.012}])
            self.assertTrue(new['admitted']);self.assertFalse(new['forecast_admitted'])
            self.assertEqual(new['current_attempt_estimate_wall_s'],old['current_attempt_estimate_wall_s'])
            self.assertEqual(new['remaining_work'],available)
            available['wall_s']=0.
            self.assertFalse(admission(ops,[{'target_q':.012}])['admitted'])
            available.update(wall_s=60.,sim_s=0.)
            self.assertFalse(admission(ops,[{'target_q':.012}])['admitted'])

    def test_staged_admission_keeps_native_transition_and_current_proof(self):
        from agentic_interaction_v87.staged_cost import admit_staged_motion
        available=dict(wall_s=60.,sim_s=100.)
        r=NS(job={'v89_controller':'coordinated_dls_v1'},g={},v75_config={},
            dt=1/240,tick=0,c={'arm_speed_limits':np.ones(7)},
            episode_budget=NS(remaining=lambda t:available.copy()))
        native=Mock(return_value={'valid':False})
        with patch.dict(sys.modules,{
                'native_g1_agent_execution':NS(validate_execution=native),
                'xarm7_native_gripper':NS(GRIPPER_JOINTS=[])}),\
             patch('agentic_interaction_v87.staged_cost.timed_path',
                side_effect=AssertionError('no stale empirical timed-cost veto')):
            self.assertFalse(admit_staged_motion(r,{},[],{'valid':True})['admit'])
            native.return_value={'valid':True}
            self.assertFalse(admit_staged_motion(r,{},[],{'valid':False})['admit'])
            out=admit_staged_motion(r,{},[],{'valid':True})
            self.assertTrue(out['admit']);self.assertTrue(out['native_transition_checked'])
            self.assertTrue(out['exact_episode_deadline_unchanged'])
            available['wall_s']=0.
            self.assertFalse(admit_staged_motion(r,{},[],{'valid':True})['admit'])


if __name__=='__main__':unittest.main()
