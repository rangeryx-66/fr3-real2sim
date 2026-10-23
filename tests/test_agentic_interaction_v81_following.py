import unittest
import numpy as np
from agentic_interaction_v81.contact_following import RateConsistentFollowing,selection
from interaction_runtime.prismatic_unloading_v751 import TransverseUnloading


class FollowingTests(unittest.TestCase):
    def setUp(self):
        self.policy=dict(max_centering_speed_m_s=.002,max_centering_step_m=5e-6,
                         max_centering_displacement_m=.002)
        self.centres=np.array([[0,0,.01],[0,0,-.01]])

    def test_rate_changes_without_new_motion_authority(self):
        legacy=TransverseUnloading(self.policy,[0,1,0],19,[0,0,0])
        new=RateConsistentFollowing(self.policy,[0,1,0],19,[0,0,0])
        for k in range(240):
            previous=new.offset.copy();new.update(k,1/240,[0,0,1],self.centres)
            legacy.update(k,1/240,[0,0,1],self.centres)
            self.assertLessEqual(np.linalg.norm(new.offset-previous),5e-6+1e-12)
            self.assertLessEqual(np.linalg.norm(new.offset),.002+1e-12)
            self.assertAlmostEqual(new.offset[1],0)
        self.assertGreater(np.linalg.norm(new.offset),10*np.linalg.norm(legacy.offset))
        self.assertEqual(new.load_window_steps,19)

    def test_existing_ball_and_closure_offset_remain_authoritative(self):
        f=RateConsistentFollowing(self.policy,[0,1,0],19,[.0019,0,0])
        for k in range(5000):
            f.update(k,1/240,[0,0,1],self.centres)
            self.assertLessEqual(np.linalg.norm(f.initial+f.offset),.002+1e-12)

    def test_no_task_axis_jaw_axis_and_invalid_measurement(self):
        f=RateConsistentFollowing(self.policy,[0,0,1],19,[0,0,0])
        self.assertFalse(f.update(0,1/240,[0,0,1],self.centres)['applied'])
        with self.assertRaises(ValueError):f.update(1,1/240,[np.nan,0,0],self.centres)

    def test_revolute_and_implicit_activation_rejected(self):
        self.assertEqual(selection({},'revolute'),'legacy_window')
        with self.assertRaises(ValueError):selection(dict(contact_following='rate_consistent'),'prismatic')
        with self.assertRaises(ValueError):selection(dict(contact_following='rate_consistent',transverse_unloading=True),'revolute')


if __name__=='__main__':unittest.main()
