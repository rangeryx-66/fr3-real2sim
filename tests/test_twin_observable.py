import unittest
from unittest.mock import patch
import numpy as np
from interactive_twin_observable.uncertainty import observable, build
from interactive_twin_observable.policy import action_score, choose_next


class ObservablePolicyTests(unittest.TestCase):
    def test_short_reverse_does_not_invalidate_long_forward(self):
        T=np.repeat(np.eye(4)[None],30,axis=0);T[:,0,3]=np.linspace(0,.00069,30)
        self.assertFalse(observable(T))
        T[:,0,3]=np.linspace(0,.02,30)
        self.assertTrue(observable(T))

    def test_heldout_not_accepted_by_estimator(self):
        with self.assertRaisesRegex(ValueError,'HELDOUT_ACCESS_DENIED'):
            action_score({'split':'heldout'},{'split':'train'},None)

    def test_unavailable_reverse_does_not_block_next_forward(self):
        state={'support':[{'candidate_id':'a'},{'candidate_id':'b'}]}
        bank={k:{'reverse':{'safe':True,'log':{}},'high_forward':{'safe':True,'log':{}}} for k in ('a','b')}
        specs={k:{'name':k,'duration_s':8} for k in ('reverse','high_forward')}
        with patch('interactive_twin_observable.policy.action_travel',return_value={'excursion_m':.00069}),patch('interactive_twin_observable.policy._signature',side_effect=[np.zeros(3),np.ones(3)]):
            d=choose_next(state,bank,{'low_forward':{}},specs,None)
        self.assertEqual(d['selected'],'high_forward')
        self.assertEqual(d['options'][0]['reason'],'REVERSE_NOT_RELIABLY_OBSERVABLE_AT_1MM')

    def test_failed_prediction_is_not_executed(self):
        result=choose_next({'support':[{'candidate_id':'a'}]},
                           {'a':{'low_forward':{'safe':False,'log':None}}},{},
                           {'low_forward':{'duration_s':8}},None)
        self.assertIsNone(result['selected'])

if __name__=='__main__':unittest.main()
