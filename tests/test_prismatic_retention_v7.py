import unittest
from types import SimpleNamespace
import numpy as np
from interaction_runtime.prismatic_retention_v7 import PrismaticContactBalance


P = dict(preload_n=.5, max_centering_step_m=5e-6, max_centering_displacement_m=.002, max_centering_speed_m_s=.002)


class ContactRetentionTests(unittest.TestCase):
    def test_measured_frame_sign_and_no_rail_motion(self):
        b = PrismaticContactBalance(P, [0,-1,0], 19, [0,0,0])
        r = b.update(0, [.8,.1], [[.01,.01,0],[-.01,-.01,0]])
        self.assertGreater(r['step_world_m'][0], 0)
        self.assertAlmostEqual(r['step_world_m'][1], 0)
        r = b.update(19, [.1,.8], [[.01,.01,0],[-.01,-.01,0]])
        self.assertLess(r['step_world_m'][0], 0)

    def test_no_continuous_symmetry_gate(self):
        b = PrismaticContactBalance(P, [0,1,0], 19, [0,0,0])
        self.assertIsNone(b.update(0, [.45,.7], [[0,0,1],[0,0,-1]]))
        self.assertIsNone(b.update(19, [.02,.03], [[0,0,1],[0,0,-1]]))

    def test_does_not_reuse_delayed_load_at_every_physics_step(self):
        b = PrismaticContactBalance(P, [0,1,0], 19, [0,0,0])
        for tick in range(19):b.update(tick, [.8,.1], [[0,0,1],[0,0,-1]])
        np.testing.assert_allclose(b.offset, [0,0,5e-6])

    def test_existing_total_bound_cannot_reset_each_segment(self):
        b = PrismaticContactBalance(P, [0,1,0], 19, [0,0,.00199])
        for tick in range(0, 10000, 19):b.update(tick, [.8,.1], [[0,0,1],[0,0,-1]])
        self.assertLessEqual(np.linalg.norm(b.initial+b.offset), .002+1e-12)
        self.assertLessEqual(b.offset[2], .00001+1e-12)

    def test_bound_does_not_create_axial_offset(self):
        b = PrismaticContactBalance(P, [0,1,0], 19, [0,.002,0])
        b.update(0, [.8,.1], [[0,0,1],[0,0,-1]])
        np.testing.assert_allclose(b.offset, 0, atol=1e-12)

    def test_quasistatic_response_range_not_one_fitted_asset(self):
        # Diagnostic surrogate only, not a native-contact success assertion.
        for stiffness in [4000., 10000., 20000., 40000.]:
            b = PrismaticContactBalance(P, [0,1,0], 19, [0,0,0])
            for tick in range(0, 240, 19):
                imbalance=.7-2*stiffness*b.offset[2]
                forces=np.clip([.5+imbalance/2,.5-imbalance/2], 0, None)
                b.update(tick, forces, [[0,0,1],[0,0,-1]])
            self.assertLess(abs(.7-2*stiffness*b.offset[2]), .3)
            self.assertAlmostEqual(b.offset @ b.rail, 0)

    def test_hold_is_live_original_duration_and_does_not_command_jaw(self):
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackRuntime
        r=PrismaticFeedbackRuntime.__new__(PrismaticFeedbackRuntime)
        r.g={'phase':'FORCE_HOLD'};r.phase='FORCE_HOLD';r.dt=1/240
        r.contact_retention=True;r.grasp_reference=np.eye(4)
        r.policy=SimpleNamespace(phase=lambda p:'ESTABLISH')
        r.jaw_width=.01;steps=[];stops=[]
        r.world=SimpleNamespace(play=lambda:None);r.stop_command=lambda:stops.append(True)
        r.begin_reference=lambda:setattr(r,'reference_drive',SimpleNamespace(pose=np.eye(4)))
        r.cartesian_step=lambda goal,phase:steps.append((goal.copy(),phase))
        r.hold(1.)
        self.assertEqual(len(steps),240);self.assertEqual(len(stops),2)
        self.assertEqual(r.jaw_width,.01)

    def test_actual_frame_correction_is_not_replayed_after_handover(self):
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackRuntime
        r=PrismaticFeedbackRuntime.__new__(PrismaticFeedbackRuntime)
        r.contact_retention=True;r.grasp_reference=np.eye(4)
        r.policy=SimpleNamespace(phase=lambda p:'LOADED')
        r.reference_drive=object();r.retention=PrismaticContactBalance(P,[0,1,0],19,[0,0,0])
        r.retention.offset=np.array([0,0,.00002]);r.retention_epoch=r.retention.offset.copy()
        r.retention.last_tick=0;r.tick=1
        r.model=SimpleNamespace(pads={'gripper_link1':np.zeros((2,3)),'gripper_link2':np.zeros((2,3))})
        r.pad_frames={'left_finger':np.eye(4),'right_finger':np.eye(4)}
        r.pose=lambda n:np.eye(4);r.filtered=lambda:np.array([.5,.5])
        np.testing.assert_array_equal(r.retention_goal(np.eye(4),'OPEN_CONTINUE'),np.eye(4))

    def test_no_retention_motion_in_exit_or_without_verified_grasp(self):
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackRuntime
        r=PrismaticFeedbackRuntime.__new__(PrismaticFeedbackRuntime)
        r.contact_retention=True;r.grasp_reference=None
        r.policy=SimpleNamespace(phase=lambda p:'RELEASE')
        goal=np.eye(4)
        self.assertIs(r.retention_goal(goal,'RELEASE'),goal)
        self.assertFalse(r.retention_phase('FORCE_HOLD'))

    def test_constant_rail_goal_gets_bounded_transverse_steps_without_accumulating_input(self):
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackRuntime
        r=PrismaticFeedbackRuntime.__new__(PrismaticFeedbackRuntime)
        r.contact_retention=True;r.grasp_reference=np.eye(4);r.retention=None
        r.retention_closure=SimpleNamespace(command=np.eye(4),origin=np.eye(4))
        r.closure_policy=P;r.contact_evidence=SimpleNamespace(axis=[0,1,0])
        r.filtered_loads=SimpleNamespace(maxlen=19);r.dt=1/240;r.tick=0
        r.reference_drive=object();r.retention_epoch=np.zeros(3)
        r.policy=SimpleNamespace(phase=lambda p:'LOADED')
        r.model=SimpleNamespace(pads={'gripper_link1':np.zeros((2,3)),'gripper_link2':np.zeros((2,3))})
        r.pad_frames={'left_finger':np.eye(4),'right_finger':np.eye(4)}
        def pose(name):
            T=np.eye(4);T[2,3]=.01 if name=='left_finger' else -.01;return T
        r.pose=pose;r.filtered=lambda:np.array([.8,.1]);r.event=lambda *a,**kw:None
        goal=np.eye(4);one=r.retention_goal(goal,'CONTACT_PRELOAD');r.tick=1
        two=r.retention_goal(goal,'CONTACT_PRELOAD')
        np.testing.assert_allclose(one[:3,3],[0,0,5e-6]);np.testing.assert_array_equal(one,two)
        np.testing.assert_array_equal(goal,np.eye(4))
        r.tick=19;three=r.retention_goal(goal,'CONTACT_PRELOAD')
        np.testing.assert_allclose(three[:3,3],[0,0,1e-5])


if __name__ == '__main__':unittest.main()
