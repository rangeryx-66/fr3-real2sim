import copy
import unittest
from types import SimpleNamespace
from agentic_interaction_v81.declaration_preflight import check
from agentic_interaction_v81.operations import operations_class
from interaction_runtime.articulation_v7 import Articulation


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.prior=Articulation('prismatic',(0,1,0),(0,0,0),(0,.3))
        self.actions=[dict(role='FIT_OPEN',target_q=.05,coordinate_unit='m',
            profile='constant',fit_intervals_s=[[0,30]],hold=True),
            dict(role='VALIDATION_REVERSE',target_q=.01,coordinate_unit='m',
                 profile='constant',hold=True)]

    def test_complete_chain_accepted_without_mutation(self):
        before=copy.deepcopy(self.actions)
        self.assertTrue(check(self.actions,self.prior)['valid'])
        self.assertEqual(before,self.actions)

    def test_missing_validation_caught_before_expensive_owner(self):
        class Owner:
            def articulation(s):return self.prior
            def request_plan(s,c):raise AssertionError('expensive owner called')
        ops=operations_class(Owner)()
        result=ops.request_plan(dict(planned_segments=self.actions[:1]))
        self.assertFalse(result['plan'])
        self.assertIn('COMPLETE_VALIDATION_NOT_PLANNED',result['reason'])
        self.assertFalse(result['expensive_planning_started'])
        self.assertFalse(hasattr(ops,'_v81_declared_chain'))

    def test_native_units_limits_and_validation_leakage(self):
        for change in [dict(coordinate_unit='rad'),dict(target_q=.4),
                       dict(fit_intervals_s=[[0,10]])]:
            actions=copy.deepcopy(self.actions);actions[-1].update(change)
            with self.subTest(change=change),self.assertRaises((ValueError,TypeError)):
                check(actions,self.prior)

    def test_actual_protocol_rejects_oversized_chain(self):
        with self.assertRaises(ValueError):check(self.actions*3,self.prior)


if __name__=='__main__':unittest.main()
