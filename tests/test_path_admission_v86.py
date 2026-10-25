import unittest
import importlib.util
from unittest.mock import Mock,patch
from types import SimpleNamespace
import numpy as np
from agentic_interaction_v81.path_admission import check_samples,preflight
from interaction_runtime.policy import SafetyStop


class PathAdmissionTests(unittest.TestCase):
    def test_unchecked_intermediate_timed_sample_is_rejected_without_actuation(self):
        samples=[(np.array([x]),np.zeros(1))for x in [0.,.1,.111,.4]]
        with self.assertRaisesRegex(RuntimeError,'EXECUTED_SAMPLE:JOINT_MARGIN'):
            check_samples(samples,lambda q:(not .110<q[0]<.112,'JOINT_MARGIN'))

    def test_valid_original_sample_stream_remains_unchanged_and_hard_error_propagates(self):
        samples=[(np.array([.1]),np.array([.2])),(np.array([.3]),np.array([0.]))]
        self.assertEqual(check_samples(iter(samples),lambda q:(True,'SAFE')),2)
        with self.assertRaises(SafetyStop):check_samples(iter(samples),Mock(side_effect=SafetyStop('HARD')))

    @unittest.skipUnless(importlib.util.find_spec('scipy'),'server numerical dependencies required')
    def test_native_margin_failure_occurs_before_motion_or_target_mutation(self):
        target=np.zeros(7)
        policy=SimpleNamespace(data={'joint_margin_rad':.05,'arm_command_acceleration_rad_s2':{'FREE':1.6}},phase=lambda p:'FREE',motion=lambda p:{})
        r=SimpleNamespace(ensure_execution_budget=Mock(),robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(13)),
            fingers=np.arange(7,13),arm=np.arange(7),moving=lambda:np.eye(4),policy=policy,
            model=SimpleNamespace(margin=lambda q:.049),collision=SimpleNamespace(check=Mock()),
            qtarget=target.copy(),solver=None,base=np.zeros(4),dt=1/240,c={'arm_speed_limits':[1]*7},step=Mock())
        with patch('interaction_runtime.motion.timed_path',return_value=iter([(np.ones(7),np.zeros(7))])):
            with self.assertRaisesRegex(RuntimeError,'JOINT_MARGIN'):preflight(r,[np.ones(7)],'PREGRASP',0.)
        np.testing.assert_array_equal(r.qtarget,target);r.step.assert_not_called();r.collision.check.assert_not_called()

    def test_preserves_original_two_argument_calibration_owner(self):
        from agentic_interaction_v81.runtime import runtime_class
        class Parent:
            def move_path(self,path,phase):return ('original-owner',path,phase)
        cls=runtime_class(Parent);r=cls.__new__(cls)
        with patch('agentic_interaction_v81.path_admission.preflight',return_value=2)as check:
            result=r.move_path([1,2],'APPROACH')
        self.assertEqual(result,('original-owner',[1,2],'APPROACH'))
        check.assert_called_once_with(r,[1,2],'APPROACH',0.)

    @unittest.skipUnless(importlib.util.find_spec('scipy'),'server numerical dependencies required')
    def test_native_float32_assignment_cannot_cross_original_margin_after_approval(self):
        # Recorded xArm joint2 lower=-2.059, last good command approaches -2.009.
        # Solver precision passes .05; assigning the endpoint into native float32
        # produces .04999994 and used to trip COMMAND_JOINT_MARGIN at execution.
        endpoint=np.zeros(7);endpoint[1]=np.nextafter(-2.009,np.inf)
        margin=lambda q:float(q[1])+2.059
        self.assertGreater(margin(endpoint),.05)
        self.assertLess(margin(endpoint.astype(np.float32)),.05)
        policy=SimpleNamespace(data={'joint_margin_rad':.05,'arm_command_acceleration_rad_s2':{'FREE':1.6}},
            phase=lambda p:'FREE',motion=lambda p:{})
        r=SimpleNamespace(ensure_execution_budget=Mock(),robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(13)),
            fingers=np.arange(7,13),arm=np.arange(7),moving=lambda:np.eye(4),policy=policy,
            model=SimpleNamespace(margin=margin),collision=SimpleNamespace(check=Mock()),
            qtarget=np.zeros(7,dtype=np.float32),solver=None,base=np.zeros(4),dt=1/240,
            c={'arm_speed_limits':[1]*7},step=Mock())
        with patch('interaction_runtime.motion.timed_path',return_value=iter([(endpoint,np.zeros(7))])):
            with self.assertRaisesRegex(RuntimeError,'EXECUTED_SAMPLE:JOINT_MARGIN'):
                preflight(r,[endpoint],'PREGRASP',0.)
        r.step.assert_not_called();r.collision.check.assert_not_called()
        np.testing.assert_array_equal(r.qtarget,np.zeros(7,dtype=np.float32))

    @unittest.skipUnless(importlib.util.find_spec('scipy'),'server numerical dependencies required')
    def test_scene_audit_uses_original_event_reserved_fields_without_aborting_motion(self):
        events=[]
        # Same keyword ownership as Runtime.event, rather than a permissive Mock.
        def event(name,**data):events.append(dict(event=name,t=0.,phase='PREGRASP',**data))
        policy=SimpleNamespace(data={'joint_margin_rad':.05,'arm_command_acceleration_rad_s2':{'FREE':1.6}},
            phase=lambda p:'FREE',motion=lambda p:{})
        r=SimpleNamespace(job={'v86_recovery_supervisor':True},ensure_execution_budget=Mock(),
            robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(13)),
            fingers=np.arange(7,13),arm=np.arange(7),moving=lambda:np.eye(4),actual=lambda:0.,
            grasp_reference=None,base_generation=0,policy=policy,event=event,
            model=SimpleNamespace(margin=lambda q:1.,poses=lambda *a,**k:np.eye(4)),
            collision=SimpleNamespace(check=lambda *a:(True,'SAFE')),
            qtarget=np.zeros(7,dtype=np.float32),solver=None,base=np.zeros(4),dt=1/240,
            c={'arm_speed_limits':[1]*7},step=Mock())
        with patch('interaction_runtime.motion.timed_path',return_value=iter([(np.zeros(7),np.zeros(7))])):
            self.assertEqual(preflight(r,[np.zeros(7)],'PREGRASP',0.),1)
        self.assertEqual(events[0]['motion_phase'],'PREGRASP')
        self.assertEqual(events[0]['phase'],'PREGRASP');r.step.assert_not_called()


if __name__=='__main__':unittest.main()
