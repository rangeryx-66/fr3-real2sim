"""Finite recovery of unloaded route planning; hard protections stay final."""
import sys,unittest
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from agentic_interaction_v81.exit_routes import release_park,timed_returns
from interaction_runtime.policy import SafetyStop


@contextmanager
def no_routes(r):yield []


def runtime():
    return SimpleNamespace(supervisor=SimpleNamespace(latched=None,force=SimpleNamespace(settled=lambda:True)),
        grasp_reference=None,rows=[dict(aperture_m=.084)],filtered=lambda:[0.,0.],
        event=Mock(),stop_command=Mock(),hold=Mock())


class ExitTests(unittest.TestCase):
    def test_verified_unloaded_route_rejection_can_replan_once(self):
        r=runtime();operation=Mock(side_effect=[dict(safe=False,reason='TIMED_RETURN_ROUTE_UNAVAILABLE:collision'),dict(safe=True)])
        with patch('agentic_interaction_v81.exit_routes.timed_returns',no_routes):result=release_park(r,operation)
        self.assertTrue(result['safe']);self.assertEqual(operation.call_count,2)
        r.stop_command.assert_called_once();r.hold.assert_called_once_with(.3)
        self.assertFalse(result['v81_exit_planning']['force_history_reset'])

    def test_two_failed_plans_end_without_indefinite_exit_motion(self):
        r=runtime();op=Mock(return_value=dict(safe=False,reason='MOTION_PATH_UNAVAILABLE:collision'))
        with patch('agentic_interaction_v81.exit_routes.timed_returns',no_routes):result=release_park(r,op)
        self.assertFalse(result['safe']);self.assertEqual(op.call_count,2)

    def test_hard_stop_exception_propagates_without_replan(self):
        r=runtime();op=Mock(side_effect=SafetyStop('COLLISION'))
        with patch('agentic_interaction_v81.exit_routes.timed_returns',no_routes):
            with self.assertRaises(SafetyStop):release_park(r,op)
        self.assertEqual(op.call_count,1);r.stop_command.assert_not_called()

    def test_unstable_loaded_unreleased_or_unknown_failure_cannot_resume(self):
        for kind in ['latch','reference','load','aperture','force','unknown']:
            r=runtime()
            if kind=='latch':r.supervisor.latched={'reason':'hard'}
            if kind=='reference':r.grasp_reference=np.eye(4)
            if kind=='load':r.filtered=lambda:[.06,0.]
            if kind=='aperture':r.rows[-1]['aperture_m']=.015
            if kind=='force':r.supervisor.force.settled=lambda:False
            reason='UNKNOWN'if kind=='unknown'else'TIMED_RETURN_ROUTE_UNAVAILABLE:collision'
            op=Mock(return_value=dict(safe=False,reason=reason))
            with self.subTest(kind=kind),patch('agentic_interaction_v81.exit_routes.timed_returns',no_routes):
                self.assertFalse(release_park(r,op)['safe']);self.assertEqual(op.call_count,1)
                r.stop_command.assert_not_called()

    def test_original_timed_checker_rejects_a_coarse_free_edge(self):
        owner=SimpleNamespace(route_to_joint=Mock(return_value=([[0.]*7,[.4]*7],'DIRECT')))
        g=dict(robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(7)),arm=np.arange(7),
            fingers=np.array([],int),qtarget=np.zeros(7),moving=lambda:np.eye(4),base=np.zeros(4),
            model=SimpleNamespace(margin=lambda q:1.,poses=lambda q,*a,**k:q),
            collision=SimpleNamespace(check=lambda q,*a:(not(.10<q[0]<.12),'narrow native obstacle')))
        policy=SimpleNamespace(data={'joint_margin_rad':.05,'arm_command_acceleration_rad_s2':{'PARK':.5}},
            phase=lambda p:'PARK',motion=lambda p:dict(speed_m_s=.08,acceleration_m_s2=.18,
                angular_speed_rad_s=.4,angular_acceleration_rad_s2=.8))
        r=SimpleNamespace(solver=None,base=np.zeros(4),dt=1/240,policy=policy,c={'arm_speed_limits':[1]*7})
        with patch.dict(sys.modules,{'native_g1_safe_park':owner}):
            original=owner.route_to_joint
            with patch('agentic_interaction_v81.exit_clearance.ParkingClearance',lambda scene:scene):
                with timed_returns(r)as history:path,why=owner.route_to_joint(g,np.zeros(7),np.ones(7)*.4)
            self.assertIs(owner.route_to_joint,original)
        self.assertIsNone(path);self.assertTrue(why.startswith('TIMED_RETURN_ROUTE_UNAVAILABLE:'))
        self.assertFalse(history[0]['timed_valid'])

    def test_every_actual_timed_sample_is_checked_before_motion(self):
        owner=SimpleNamespace(route_to_joint=Mock(return_value=([[0.]*7,[.4]*7],'DIRECT')))
        collision=SimpleNamespace(check=lambda q,*a:(not(.110<q[0]<.112),'thin obstacle'))
        g=dict(robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(7)),arm=np.arange(7),
            fingers=np.array([],int),qtarget=np.zeros(7),moving=lambda:np.eye(4),base=np.zeros(4),
            model=SimpleNamespace(margin=lambda q:1.,poses=lambda q,*a,**k:q),collision=collision)
        policy=SimpleNamespace(data={'joint_margin_rad':.05,'arm_command_acceleration_rad_s2':{'PARK':.5}},
            phase=lambda p:'PARK',motion=lambda p:{})
        r=SimpleNamespace(solver=None,base=np.zeros(4),dt=1/240,policy=policy,c={'arm_speed_limits':[1]*7})
        samples=[(np.full(7,.111),np.zeros(7))]
        with patch.dict(sys.modules,{'native_g1_safe_park':owner}),\
             patch('agentic_interaction_v81.exit_clearance.ParkingClearance',lambda scene:scene),\
             patch('interaction_runtime.motion.timed_path',return_value=iter(samples)):
            with timed_returns(r)as history:path,why=owner.route_to_joint(g,np.zeros(7),np.ones(7)*.4)
        self.assertIsNone(path);self.assertIn('EXECUTED_PARK_SAMPLE',why)
        self.assertFalse(history[0]['timed_valid'])


if __name__=='__main__':unittest.main()
