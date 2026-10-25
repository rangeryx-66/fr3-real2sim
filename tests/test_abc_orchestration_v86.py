import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from agentic_interaction_v81.integration import apply,ABC_ORCHESTRATION_PROFILE,ABC_PROFILE
from agentic_interaction_v81.orchestration_extension import renew_parked_wait
from interaction_runtime.policy import SafetyStop


class AuthorizedABCWaitTests(unittest.TestCase):
    def test_explicit_profile_composes_existing_abc_and_authenticated_credit(self):
        auth=dict(human_authorization_ref='user-explicit-continue',max_total_extra_wall_s=14400,parked_decision_renewal_s=1800)
        for kind in ['revolute','prismatic']:
            job={'articulation_v7':{'joint_type':kind},'episode_sim_budget_s':3600,'safe_exit_sim_reserve_s':120}
            apply(dict(phase='engineering',integration_profile=ABC_ORCHESTRATION_PROFILE,orchestration_authorization=auth),job)
            self.assertTrue(job['v84_abc']);self.assertTrue(job['v81_active_recovery']);self.assertTrue(job['v83_orchestration_extension'])
            self.assertEqual(job['episode_sim_budget_s'],3600);self.assertEqual(job['safe_exit_sim_reserve_s'],120)
        with self.assertRaises(ValueError):
            apply(dict(phase='engineering',integration_profile=ABC_PROFILE,orchestration_authorization=auth),{})

    def test_expired_unparked_wait_must_execute_and_verify_original_park_before_credit(self):
        with tempfile.TemporaryDirectory() as folder:
            manager=SimpleNamespace(authorization={'parked_decision_renewal_s':1800},renew_wait=Mock(return_value={'extended':True,'renewed_wait_s':1800}))
            r=SimpleNamespace(output=Path(folder),job={'v84_abc':True},stopped=False,orchestration_extension=manager,tick=10,dt=1/240,world=SimpleNamespace(pause=Mock()))
            ops=SimpleNamespace(r=r,release_park=Mock(return_value={'safe':True}),current_plan_ready=True,_v81_plan_ticket='old')
            with patch('agentic_interaction_v81.task_recovery.unloaded',return_value=True),\
                 patch('agentic_interaction_v81.session_status.write'),\
                 patch('agentic_interaction_v81.orchestration_extension.measured_evidence',side_effect=[ValueError('VERIFIED_NATIVE_PARK_REQUIRED'),{'verified':True}]):
                result=renew_parked_wait(ops,1000,sequence=4)
            self.assertTrue(result['extended']);self.assertTrue(result['guarded_park_recovery'])
            ops.release_park.assert_called_once();manager.renew_wait.assert_called_once_with({'verified':True},1000)
            self.assertFalse(ops.current_plan_ready);self.assertIsNone(ops._v81_plan_ticket)

    def test_unknown_state_unconfirmed_park_and_hard_stop_cannot_receive_credit(self):
        for outcome in ['unknown','unconfirmed','hard']:
            with self.subTest(outcome=outcome),tempfile.TemporaryDirectory() as folder:
                manager=SimpleNamespace(authorization={'parked_decision_renewal_s':1800},renew_wait=Mock())
                r=SimpleNamespace(output=Path(folder),job={'v84_abc':True},stopped=False,orchestration_extension=manager,tick=10,dt=1/240,world=SimpleNamespace(pause=Mock()))
                park=Mock(side_effect=SafetyStop('HARD')) if outcome=='hard' else Mock(return_value={'safe':False})
                ops=SimpleNamespace(r=r,release_park=park)
                reason='SENSOR_STATE_UNTRUSTED'if outcome=='unknown'else'VERIFIED_NATIVE_PARK_REQUIRED'
                with patch('agentic_interaction_v81.task_recovery.unloaded',return_value=True),\
                     patch('agentic_interaction_v81.session_status.write'),\
                     patch('agentic_interaction_v81.orchestration_extension.measured_evidence',side_effect=ValueError(reason)):
                    if outcome=='hard':
                        with self.assertRaises(SafetyStop):renew_parked_wait(ops,1000,sequence=4)
                    else:self.assertFalse(renew_parked_wait(ops,1000,sequence=4)['extended'])
                manager.renew_wait.assert_not_called()
                if outcome=='unknown':park.assert_not_called()


if __name__=='__main__':unittest.main()
