import unittest
from unittest.mock import patch
from interactive_twin_conditional.selection import choose,compare_actions

class ConditionalSelectionTests(unittest.TestCase):
    def setUp(self):
        self.config={'selection':{'profile_loss_delta':1.,'max_validation_rms_noise_units':10.},
                     'wrong_prior':{'tau_c':.012,'b':1.},'physics_grid':{'tau_c':[0,.004,.012],'b':[0,.3,1.]}}
    def test_test_action_is_rejected(self):
        with self.assertRaises(ValueError):choose({'P1':{},'P2':{},'P3':{},'P4':{}},[],None,self.config)
        with self.assertRaises(ValueError):compare_actions({'P4':{}},{'P4':{}},None)
    def test_axis_resistance_tradeoff_retains_interval(self):
        logs={'P1':{},'P2':{},'P3':{}}
        items=[{'candidate_id':'a','structure_id':'S0','tau_c':0.,'b':0.,'logs':logs,'status':'OK'},
               {'candidate_id':'b','structure_id':'S2','tau_c':.004,'b':.3,'logs':logs,'status':'OK'},
               {'candidate_id':'p','structure_id':'S0','tau_c':.012,'b':1.,'logs':logs,'status':'OK'}]
        with patch('interactive_twin_conditional.selection.compare_actions',return_value={'train_loss':1.,'validation_loss':1.}):
            result=choose(logs,items,None,self.config)
        self.assertEqual(result['status'],'STRUCTURE_PHYSICS_CONFUNDED')
        self.assertEqual(result['parameter_intervals']['tau_c'],[0.,.012])
        self.assertFalse(result['test_read'])
    def test_failed_model_does_not_cancel_remaining_grid(self):
        logs={'P1':{},'P2':{},'P3':{}}
        items=[{'candidate_id':'bad','structure_id':'S0','tau_c':0.,'b':0.,'logs':{},'status':'CONTACT_LOSS'},
               {'candidate_id':'good','structure_id':'S1','tau_c':.012,'b':1.,'logs':logs,'status':'OK'}]
        with patch('interactive_twin_conditional.selection.compare_actions',return_value={'train_loss':1.,'validation_loss':1.}):
            result=choose(logs,items,None,self.config)
        self.assertEqual(result['methods']['uncertain_structure_calibrated']['candidate_id'],'good')
        self.assertEqual(result['failures'][0]['status'],'CONTACT_LOSS')

if __name__=='__main__':unittest.main()
