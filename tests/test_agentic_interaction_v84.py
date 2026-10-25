"""Operational regressions: real loss still stops; pacing alone is advisory."""
import math,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
import numpy as np
from interaction_runtime.articulation_v7 import Articulation
from interaction_runtime.progress import ProgressClock,LOOKAHEAD_RAD
from interaction_runtime.segments import TransmissionMonitor
from agentic_interaction_v84.monitor import factory
from agentic_interaction_v84.operations import operations_class,contact_trusted
from agentic_interaction_v84.transport import validate
from agent_command_protocol_v12 import ProtocolError


def action():return dict(role='FIT_LONG',coordinate_unit='rad',target_q=.4,
    profile='constant',hold=True,reference_speed_m_s=.008,reference_acceleration_m_s2=.025)


class FeedbackTests(unittest.TestCase):
    def replay(self,object_rate,tcp_rate):
        a=Articulation('revolute',(0,0,1),(0,0,0),(-2,2));pose=np.eye(4);pose[0,3]=.4
        r=S(tcp=lambda:pose.copy());captures=[];m=factory(r,a,captures)(0,1,action())
        old=TransmissionMonitor(0,1,action());fail=None;legacy=None
        for i in range(1,1000):
            t=i/240;angle=tcp_rate*t;pose[:3,3]=[.4*math.cos(angle),.4*math.sin(angle),0]
            state=dict(requested_rate_rad_s=.025,lag_rad=LOOKAHEAD_RAD*.99)
            fail=m.update(t,object_rate*t,state,1/240,1) or fail
            legacy=old.update(t,object_rate*t,state,1/240,1) or legacy
        return m,fail,legacy

    def test_coupled_slow_motion_warns_without_false_loss(self):
        monitor,fail,legacy=self.replay(.0005,.0005)
        self.assertIsNotNone(legacy);self.assertIsNone(fail)
        self.assertTrue(monitor.last['advisory']);self.assertTrue(monitor.stable())
        self.assertIsNotNone(monitor.first_advisory_s)

    def test_real_slip_returns_recoverable_failure(self):
        monitor,fail,_=self.replay(0,.01)
        self.assertIsNotNone(fail);self.assertFalse(monitor.stable())

    def test_complete_stall_is_not_ignored(self):
        _,fail,_=self.replay(0,0);self.assertIsNotNone(fail)

    def test_reference_and_original_half_degree_bound_unchanged(self):
        c=ProgressClock(0,1)
        for _ in range(3000):c.advance(0,.01,.4,1/240,.3,.03,.5)
        # Original trapezoidal discrete integration can use one acceleration
        # cell beyond the analytic braking boundary; do not alter that owner.
        self.assertLessEqual(c.reference,LOOKAHEAD_RAD+(.03/.4)*(1/240)**2)
        self.assertEqual(c.lookahead,LOOKAHEAD_RAD)

class ContactTests(unittest.TestCase):
    def fixture(self):
        rows=[dict(t=i/240,aperture_m=.014,relative_translation_slip_m=.003+i*1e-7,
            joint_coordinate=.3,wrist_ft=dict(valid=True)) for i in range(121)]
        r=S(supervisor=S(latched=None),grasp_reference=np.eye(4),rows=rows,
            closure_policy=dict(max_slip_m=.001),filtered=lambda:np.array([0.,1.2]),
            record=S(segments=[dict(result=dict(status='TARGET_REACHED',physical_transmission_valid=True))]))
        class Parent:
            def articulation(self):return S(joint_type='revolute')
            def release_park(self):
                self.called=True;self.g['recovery_state']='UNLOADED_OBSERVATION';return dict(safe=True)
        cls=operations_class(Parent);o=object.__new__(cls);o.r=r;o.g={};o._abc_retain=True;o.called=False
        return o

    def test_nonzero_slip_and_one_pad_do_not_veto_contact(self):
        o=self.fixture();self.assertTrue(contact_trusted(o))
        self.assertIs(type(contact_trusted(o)),bool)
        result=o.release_park();self.assertTrue(result['contact_retained'])
        self.assertFalse(result['safe']);self.assertFalse(o.called)

    def test_hard_latch_never_retains_or_clears(self):
        o=self.fixture();o.r.supervisor.latched={'reason':'HARD'}
        self.assertFalse(contact_trusted(o));self.assertTrue(o.release_park()['safe'])
        self.assertTrue(o.called);self.assertEqual(o.r.supervisor.latched,{'reason':'HARD'})

    def test_untrusted_sensor_returns_to_native_exit(self):
        o=self.fixture();o.r.rows[-1]['wrist_ft']['valid']=False
        self.assertFalse(contact_trusted(o));o.release_park();self.assertTrue(o.called)

class ContractTests(unittest.TestCase):
    def command(self):return dict(protocol_version=1,sequence=2,operation='continue_motion',
        reason='remaining measured task progress',contact_generation=1,segments=[action()],retain_contact=True)
    def test_finite_native_contract(self):validate(self.command(),2)
    def test_stale_missing_generation_rejected(self):
        c=self.command();c.pop('contact_generation')
        with self.assertRaises(ProtocolError):validate(c,2)
    def test_no_manual_grasp_or_safety_override(self):
        c=self.command();c['grasp_transform']=np.eye(4).tolist()
        with self.assertRaises(ProtocolError):validate(c,2)
    def test_no_validation_fit_leak(self):
        c=self.command();c['segments'][0].update(role='VALIDATION_LONG',fit_intervals_s=[[0,1]])
        with self.assertRaises(ProtocolError):validate(c,2)
    def test_no_unregistered_strategy(self):
        c=self.command();c['segments'][0]['execution_strategy']='ignore_all_warnings'
        with self.assertRaises(ProtocolError):validate(c,2)

class QualityTests(unittest.TestCase):
    def test_operation_success_can_be_excluded_from_fit(self):
        from agentic_interaction_v84.quality import selection_scope
        import update_probing_v7_model as model
        s=dict(role='FIT_LONG',completed=True,result=dict(ok=True,status='TARGET_REACHED',
            abc_feedback=dict(FIT_exclude_entire_segment=True)))
        with selection_scope():
            ix,q=model.selection(s,{},np.arange(500),True)
        self.assertEqual(len(ix),0);self.assertTrue(q['action_ok'])

class FiniteHoldTests(unittest.TestCase):
    def boundary_fixture(self,d):
        o=ContactTests().fixture();r=o.r;r.output=Path(d);r.tick=0;r.dt=1/240
        r.job=dict(episode_id='test',safe_exit_wall_reserve_s=300)
        r.base_generation=0;r.g=dict(observation_generations=[])
        r.episode_budget=S(remaining=lambda sim:dict(wall_s=100,sim_s=100))
        r.stopped=False;r.hold=lambda seconds:setattr(r,'tick',r.tick+12)
        o.g['recovery_state']='LOADED_CRITIQUE'
        return o

    def test_real_numpy_packet_serializes_at_loaded_boundary(self):
        import json
        from agentic_interaction_v84.session import loaded_command
        with tempfile.TemporaryDirectory() as d:
            o=self.boundary_fixture(d);o.packet=lambda:dict(contact_trusted=np.bool_(True))
            directory=o.r.output/'agent_commands';directory.mkdir();(directory/'004.json').write_text('{}')
            self.assertTrue(loaded_command(o,4));self.assertFalse(o.called)
            self.assertIs(json.loads((o.r.output/'ABC_CONTACT_PACKET.json').read_text())['contact_trusted'],True)

    def test_metadata_error_unloads_and_preserves_episode_goal(self):
        import json
        from agentic_interaction_v84.session import loaded_command
        with tempfile.TemporaryDirectory() as d:
            o=self.boundary_fixture(d)
            def broken():raise TypeError('injected JSON presentation failure')
            o.packet=broken
            self.assertIsNone(loaded_command(o,4));self.assertTrue(o.called)
            v=json.loads((o.r.output/'ABC_C_METADATA_RECOVERY_004.json').read_text())
            self.assertTrue(v['safe_exit']['safe']);self.assertTrue(v['original_goal_preserved'])

    def test_contact_timeout_steps_physics_then_unloads_without_ending_task(self):
        from agentic_interaction_v84.session import loaded_command
        with tempfile.TemporaryDirectory() as d:
            o=ContactTests().fixture();r=o.r;r.output=Path(d);r.tick=0;r.dt=1/240
            r.job=dict(episode_id='test',safe_exit_wall_reserve_s=300)
            r.base_generation=0;r.g=dict(observation_generations=[])
            r.episode_budget=S(remaining=lambda sim:dict(wall_s=100,sim_s=100))
            r.stopped=False;r.hold=lambda seconds:setattr(r,'tick',r.tick+12)
            o.packet=lambda:dict(state='C');o.g['contact_attempts']=[]
            result=loaded_command(o,4)
            self.assertIsNone(result);self.assertGreater(r.tick,0);self.assertTrue(o.called)
            self.assertTrue((r.output/'ABC_C_TIMEOUT_004.json').exists())

class PlanningScheduleTests(unittest.TestCase):
    def test_identical_winner_fewer_checks_without_pruning_invalid_winner(self):
        from agentic_interaction_v84.planning import dominated_checker
        candidates=[dict(shared_selection_utility=x,good=good) for x,good in
            [(10,False),(8,True),(7,True),(9,True),(9,True),(6,True)]]
        calls=[];audit=[];pool=[];owner=S()
        def native(owner,c):calls.append(c['shared_selection_utility']);return dict(valid=c['good'])
        checked=dominated_checker(native,audit,pool);winner=None
        for c in candidates:
            result=checked(owner,c)
            if result['valid'] and (winner is None or c['shared_selection_utility']>winner):winner=c['shared_selection_utility']
        self.assertEqual(winner,9);self.assertEqual(calls,[10,8,9])
        self.assertEqual(len(pool),3)
        self.assertTrue(all(x['native_checks_not_claimed'] for x in audit if x.get('pruned_not_collision_failure')))

class RecoveryRouteTests(unittest.TestCase):
    def test_local_ik_failure_after_verified_unload_is_recoverable(self):
        from agentic_interaction_v81.task_recovery import classify
        r=S(supervisor=S(latched=None,force=S(settled=lambda:True)),
            job={'v84_abc':True},
            grasp_reference=None,actual=lambda:.3,filtered=lambda:np.array([0.,0.]),
            policy=S(data={'force':{'unloaded_pad_n':.05}}))
        ops=S(r=r,g={'recovery_state':'UNLOADED_OBSERVATION'})
        self.assertEqual(classify(ops,dict(error="RuntimeError('LOCAL_IK_UNAVAILABLE')")),
            'OPERATION_REPLAN_REQUIRED')
        self.assertEqual(classify(ops,dict(error="RuntimeError('UNCONFIRMED_CONTROLLER_FAULT')")),
            'CONTROL_STATE_UNTRUSTED')
        r.job['v84_abc']=False
        self.assertEqual(classify(ops,dict(error="RuntimeError('LOCAL_IK_UNAVAILABLE')")),
            'CONTROL_STATE_UNTRUSTED')
        r.job['v84_abc']=True
        r.supervisor.latched={'reason':'COLLISION'}
        self.assertEqual(classify(ops,dict(error="RuntimeError('LOCAL_IK_UNAVAILABLE')")),'HARD_STOP')

if __name__=='__main__':unittest.main()
