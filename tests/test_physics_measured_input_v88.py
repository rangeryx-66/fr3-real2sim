import unittest
import numpy as np
from agentic_interaction_v88.measured_input import summarize


class FitIsolationTests(unittest.TestCase):
    def model(self):return dict(constant_model=dict(joint_type='prismatic',intervals=0,
        parameters=[2.,3.,.4],conditional_covariance=np.eye(3).tolist(),algebraic_rank=3,
        conditionally_supported=[True]*3),selected=dict(intervals=0))

    def piece(self,role,v,effort):return dict(segments=[role],valid=np.ones(4,bool),
        qdd=np.array([.001,-.001,.002,-.002]),qd=np.full(4,v),direction=np.full(4,np.sign(v)),
        unfiltered_torque=np.full(4,effort))

    def test_heldout_force_cannot_change_model_uncertainty_or_control_inputs(self):
        fit=[self.piece('FIT_1',.003,.42),self.piece('FIT_2',-.003,-.42),
            self.piece('FIT_3',.002,.41)]
        clean=summarize(self.model(),fit)
        dirty=summarize(self.model(),fit+[self.piece('VALIDATION_NEVER_FIT',.01,10000.)])
        self.assertEqual(clean,dirty);self.assertTrue(clean['model_usable'])

    def test_single_complete_fit_and_partial_reverse_are_not_a_qualified_m0(self):
        result=summarize(self.model(),[self.piece('FIT_ONLY_FORWARD',.003,.42)])
        self.assertFalse(result['model_usable']);self.assertFalse(result['actual_bidirectional_FIT'])


if __name__=='__main__':unittest.main()
