"""Numerical/drive-interface tests, distinct from native physical tests."""
import unittest
import numpy as np
from agentic_interaction_v81.excitation import preview
from agentic_interaction_v81.mobile_mount import drive_class,BASE_MAX_EFFORT_N,BASE_KP,BASE_KD


class NumericTests(unittest.TestCase):
    def test_native_float32_boundary_commitment(self):
        from agentic_interaction_v81.station import Boundary
        b=Boundary('native',0,0,np.float32(1),tuple(np.zeros(4,np.float32)),
            tuple(np.zeros(7,np.float32)),tuple(np.zeros(6,np.float32)),np.float32(.01),'m',
            True,True,True,True,True,None,'a'*64)
        self.assertEqual(len(b.commitment()),64)

    def test_profile_variation_adds_independent_acceleration_opportunity(self):
        m=dict(speed_m_s=.012,acceleration_m_s2=.04,angular_speed_rad_s=.08,angular_acceleration_rad_s2=.2)
        a=dict(role='FIT_TEST',reference_speed_m_s=.008,reference_acceleration_m_s2=.02)
        c=preview(a,0,.15,joint_type='prismatic',radius=1,motion=m,scales=[1,3,.3])
        p=preview(dict(a,velocity_program='plateaus',low_speed_fraction=.25,ramp_s=.5,plateau_s=.8),
                  0,.15,joint_type='prismatic',radius=1,motion=m,scales=[1,3,.3])
        self.assertTrue(p['nominal_target_reached'])
        self.assertEqual(p['information']['algebraic_rank'],3)
        self.assertGreater(p['information']['independent_acceleration_rms_native'],
                           c['information']['independent_acceleration_rms_native'])
        self.assertFalse(p['physical_feasibility_verified'])

    def test_preview_does_not_turn_too_short_motion_into_usable_fit(self):
        m=dict(speed_m_s=.012,acceleration_m_s2=.04,angular_speed_rad_s=.08,angular_acceleration_rad_s2=.2)
        r=preview(dict(role='FIT_TEST'),0,.00001,joint_type='prismatic',radius=1,motion=m,scales=[1,3,.3])
        self.assertEqual(r['eligible_sample_opportunity'],0)

    def test_reverse_regressors_use_real_sign_not_role_name(self):
        m=dict(speed_m_s=.012,acceleration_m_s2=.04,angular_speed_rad_s=.08,angular_acceleration_rad_s2=.2)
        r=preview(dict(role='FIT_PULL',reference_speed_m_s=.006),.15,.05,
                  joint_type='prismatic',radius=1,motion=m,scales=[1,3,.3])
        self.assertGreater(r['information']['direction_counts']['negative'],0)
        self.assertEqual(r['information']['direction_counts']['positive'],0)

    def test_g1_cap_rewrite_cannot_erase_base_or_change_arm_authority(self):
        class Raw:
            def __init__(self):self.kp=np.arange(10,dtype=float);self.kd=np.arange(10,dtype=float)+.5
            def get_gains(self):return self.kp.copy(),self.kd.copy()
            def set_gains(self,kps,kds,**kw):self.kp=kps.copy();self.kd=kds.copy()
        class Robot:
            def __init__(self):self._articulation_view=self;self.speeds=np.ones((1,10))
            def set_max_efforts(self,c):self.cap=c.copy()
            def get_max_joint_velocities(self):return self.speeds.copy()
            def set_max_joint_velocities(self,v):self.speeds=v.copy()
        class Driver:
            def __init__(self,robot,c,output):
                self.robot=robot;self.names=[f'joint{i}' for i in range(7)]+['drive_joint','mobile_x','mobile_y']
                self.raw=Raw();self.caps=np.array([50.,50.,30.,30.,30.,20.,20.,.5,0.,0.])
            def command_width(self,width):
                self.caps[7]=width;self.robot._articulation_view.set_max_efforts(self.caps.reshape(1,-1))
            def set_gains(self,*args,**kw):
                self.raw.set_gains(np.arange(10,dtype=float),np.arange(10,dtype=float)+.5)
        import tempfile
        robot=Robot()
        with tempfile.TemporaryDirectory() as folder:d=drive_class(Driver)(robot,{},folder)
        np.testing.assert_array_equal(robot.speeds[0,:8],np.ones(8))
        np.testing.assert_array_equal(robot.speeds[0,8:],[.08,.08])
        for width in [.1,.8,.25]:
            d.command_width(width)
            np.testing.assert_array_equal(robot.cap[0,:7],[50,50,30,30,30,20,20])
            np.testing.assert_array_equal(robot.cap[0,8:],[BASE_MAX_EFFORT_N]*2)
            self.assertEqual(robot.cap[0,7],width)
        d.set_gains()
        np.testing.assert_array_equal(d.raw.kp[:8],np.arange(8,dtype=float))
        np.testing.assert_array_equal(d.raw.kp[8:],[BASE_KP]*2)
        np.testing.assert_array_equal(d.raw.kd[8:],[BASE_KD]*2)


if __name__=='__main__':unittest.main()
