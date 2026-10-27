import unittest
import numpy as np
from agentic_interaction_v88.contact_calibration import point_normal_sums,bounds,residual_axis_capacity


class ContactCalibrationTests(unittest.TestCase):
    def test_opposing_normals_do_not_create_an_infinite_friction_bound(self):
        n,b,seen=point_normal_sums(np.array([10]),np.array([10,10]),np.array([0,0]),
            np.array([[0,0,1.],[0,0,-1.]]),np.array([[0,0,1.],[0,0,-1.]]),np.array([1.,0,0]),1.)
        self.assertEqual(n[0,0],2.);self.assertEqual(b[0,0],2.);self.assertTrue(seen[0,0])

    def test_friction_is_a_lower_bound_and_heldout_data_cannot_inflate_it(self):
        n=np.ones((3,2));budget=n.copy();friction=np.zeros((3,2,3));friction[:2,:,0]=.2
        friction[2,:,0]=1000.;mask=np.array([True,True,False])
        r=bounds(n,budget,friction,n,mask,resolved_normal_n=.05)
        self.assertAlmostEqual(r['used_traction_coefficient_lower'],.2)
        self.assertFalse(r['friction_point_value_identified'])

    def test_transverse_friction_consumes_axis_transmission_budget(self):
        np.testing.assert_allclose(residual_axis_capacity([[1.,1.]],[[.3,.3]],.5),[.8])
        np.testing.assert_allclose(residual_axis_capacity([[1.,1.]],[[0.,0.]],.5),[1.])

    def test_exhausted_predicted_budget_is_not_a_scripted_slip_outcome(self):
        np.testing.assert_allclose(residual_axis_capacity([[1.,1.]],[[.6,.6]],.5),[0.])


if __name__=='__main__':unittest.main()
