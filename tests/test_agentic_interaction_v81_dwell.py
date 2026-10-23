"""Actual-motion station certificate; no physics/model/velocity guard bypass."""
import unittest
import numpy as np
from agentic_interaction_v81.motion_dwell import MotionDwell


class DwellTests(unittest.TestCase):
    def run_states(self, fn):
        d=MotionDwell(1/240)
        for n in range(120):
            t=n/240;q,T,x=fn(n,t);d.update(t,q,T,x)
        return d,q

    def test_true_static_pose_needs_continuous_physics_samples(self):
        d,q=self.run_states(lambda n,t:(np.zeros(7),np.eye(4),0.))
        self.assertTrue(d.ready(119/240,q))
        self.assertFalse(d.ready(120/240,q))
        self.assertFalse(d.ready(119/240,np.ones(7)*.00001))

    def test_real_joint_motion_rejected_even_if_reported_velocity_would_be_zero(self):
        d,q=self.run_states(lambda n,t:(np.ones(7)*t*.02,np.eye(4),0.))
        self.assertFalse(d.ready(119/240,q))

    def test_high_frequency_returning_oscillation_is_not_stationary(self):
        d,q=self.run_states(lambda n,t:(np.ones(7)*(.0002 if n%2 else -.0002),np.eye(4),0.))
        self.assertFalse(d.ready(119/240,q))

    def test_small_translation_rotation_and_object_motion_have_independent_checks(self):
        def frames(kind):
            def frame(n,t):
                T=np.eye(4);q=np.zeros(7);x=0.
                if kind=='TCP':T[0,3]=t*.002
                if kind=='rotation':
                    a=t*.02;T[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
                if kind=='object':x=t*.0002
                return q,T,x
            return frame
        for kind in ['TCP','rotation','object']:
            with self.subTest(kind=kind):
                d,q=self.run_states(frames(kind));self.assertFalse(d.ready(119/240,q))

    def test_gap_or_nonfinite_data_clears_certificate(self):
        for bad in ['gap','nan']:
            d,q=self.run_states(lambda n,t:(np.zeros(7),np.eye(4),0.))
            d.update(1.,q,np.eye(4),0.) if bad=='gap' else d.update(.5,np.full(7,np.nan),np.eye(4),0.)
            self.assertFalse(d.latest['ready'])

    def test_single_sample_below_speed_threshold_is_not_enough(self):
        d=MotionDwell(1/240);d.update(0,np.zeros(7),np.eye(4),0.)
        self.assertFalse(d.ready(0,np.zeros(7)))


if __name__=='__main__':unittest.main()
