import unittest
import numpy as np
from interaction_runtime.prismatic_unloading_v751 import TransverseUnloading,causal_world_wrench


class UnloadingTest(unittest.TestCase):
    def make(self,offset=None):
        p=dict(max_centering_speed_m_s=.002,max_centering_step_m=5e-6,max_centering_displacement_m=.002)
        return TransverseUnloading(p,[0,-1,0],19,np.zeros(3)if offset is None else offset)
    def test_unloads_real_transverse_load_but_never_commands_rail(self):
        c=self.make();r=c.update(0,1/240,[0,.6,-3.7],[[0,0,.01],[0,0,-.01]])
        self.assertGreater(r['step_world_m'][2],0);self.assertEqual(r['rail_component_m'],0)
        self.assertLessEqual(np.linalg.norm(r['step_world_m']),5e-6)
        self.assertIsNone(c.update(1,1/240,[0,.6,-3.7],[[0,0,.01],[0,0,-.01]]))
    def test_force_direction_uses_world_not_left_right_names(self):
        for centres in [[[0,0,.01],[0,0,-.01]],[[0,0,-.01],[0,0,.01]]]:
            c=self.make();c.update(0,1/240,[0,.6,-3.7],centres)
            self.assertGreater(c.offset[2],0)
    def test_rail_only_load_does_not_correct(self):
        c=self.make();c.update(0,1/240,[0,.6,0],[[0,0,.01],[0,0,-.01]])
        np.testing.assert_array_equal(c.offset,0)
    def test_closure_and_new_offset_share_original_displacement_bound(self):
        c=self.make(np.array([0,0,.001999]))
        for i in range(100):c.update(i*19,1/240,[0,0,-3],[[0,0,1],[0,0,-1]])
        self.assertLessEqual(np.linalg.norm(c.initial+c.offset),.002000000001)
    def test_bad_measurement_stops_no_silent_force_substitute(self):
        c=self.make()
        with self.assertRaises(ValueError):c.update(0,1/240,[0,np.nan,0],np.zeros((2,3)))
    def test_single_causal_compensation_no_future_or_native_impulses(self):
        r=dict(wrist_ft=dict(T_world_sensor=np.eye(4),raw_wrench_sensor=[1,2,3,4,5,6],tool_gravity_wrench_world=[0,0,7,0,0,0]))
        np.testing.assert_allclose(causal_world_wrench(r,[1,0,0,0,0,0]),[0,2,10,4,5,6])


if __name__=='__main__':unittest.main()
