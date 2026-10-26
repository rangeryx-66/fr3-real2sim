"""Physical-unit, quantization, finite-resource and actual session-boundary faults."""
import math,tempfile,unittest,json,sys
from pathlib import Path
from types import SimpleNamespace as NS,ModuleType
from unittest.mock import Mock,patch
import numpy as np
from interaction_runtime.articulation_v7 import Articulation
from agentic_interaction_v81.motion_dwell import MotionDwell
from agentic_interaction_v87.motion_policy import PhysicalMotionDwell,resource_forecast
from agentic_interaction_v87.constraint_feedback import POLICY


class SIHoldTests(unittest.TestCase):
    def replay(self,kind,rate,radius=.4):
        a=Articulation(kind,(0,0,1),(0,0,0),(-2,2));old=MotionDwell(1/240)
        new=PhysicalMotionDwell(1/240,lambda:a);T=np.eye(4);T[0,3]=radius
        for i in range(241):
            t=i/240;q=rate*t
            before=old.update(t,np.zeros(7),T,q);after=new.update(t,np.zeros(7),T,q)
        return before,after

    def test_slow_door_release_uses_surface_SI_speed_not_native_rad_cutoff(self):
        before,after=self.replay('revolute',.0003)
        self.assertFalse(before['ready']);self.assertTrue(after['ready'])
        self.assertAlmostEqual(after['object_path_speed_m_s'],.00012,places=8)

    def test_same_physical_motion_has_same_hold_authority_on_rail(self):
        before,after=self.replay('prismatic',.0005)
        self.assertFalse(before['ready']);self.assertTrue(after['ready'])
        self.assertAlmostEqual(after['object_path_speed_m_s'],.0005)

    def test_surface_speed_and_rotation_still_deny_uncontrolled_motion(self):
        for kind,rate,radius in [('revolute',.0003,10.),('revolute',.02,.01),
                                 ('prismatic',.002,.4)]:
            with self.subTest(kind=kind,rate=rate,radius=radius):
                self.assertFalse(self.replay(kind,rate,radius)[1]['ready'])

    def test_missing_consecutive_state_or_fast_arm_is_not_a_hold(self):
        a=Articulation('prismatic',(1,0,0),(0,0,0),(-1,1))
        d=PhysicalMotionDwell(1/240,lambda:a)
        for i in range(100):out=d.update(i/120,np.ones(7)*i*.01,np.eye(4),0.)
        self.assertFalse(out['ready'])


class MeasurementAndResourceTests(unittest.TestCase):
    def replay(self,object_rate,tcp_rate,resources=False):
        from agentic_interaction_v84.monitor import factory
        from interaction_runtime.progress import LOOKAHEAD_RAD
        a=Articulation('revolute',(0,0,1),(0,0,0),(-2,2));T=np.eye(4);T[0,3]=.4
        r=NS(job={'v87_constraint_policy':POLICY},tcp=lambda:T.copy(),tick=0,dt=1/240)
        if resources:r.episode_budget=NS(remaining=lambda t:{'sim_s':100.,'wall_s':100.})
        clock=[0.];captures=[];fail=None
        with patch('agentic_interaction_v84.monitor.time.monotonic',side_effect=lambda:clock[0]):
            monitor=factory(r,a,captures)(0,1,{'role':'FIT','profile':'constant'})
            for i in range(1,1001):
                t=i/240;clock[0]=t;r.tick=i
                T[:3,3]=[.4*math.cos(tcp_rate*t),.4*math.sin(tcp_rate*t),0]
                fail=monitor.update(t,object_rate*t,dict(requested_rate_rad_s=.025,
                    lag_rad=LOOKAHEAD_RAD*.99),1/240,1.) or fail
        return monitor,fail

    def test_resolved_coupled_motion_below_empirical_speed_floor_is_advisory(self):
        m,fail=self.replay(.00001,.00001)
        self.assertIsNone(fail);self.assertTrue(m.last['advisory']);self.assertTrue(m.stable())

    def test_actual_slip_and_complete_stall_still_end_action(self):
        for actual,tcp in [(0.,.01),(0.,0.)]:
            with self.subTest(actual=actual,tcp=tcp):self.assertIsNotNone(self.replay(actual,tcp)[1])

    def test_two_resource_forecasts_request_strategy_change_not_contact_failure(self):
        m,fail=self.replay(.00001,.00001,True)
        self.assertEqual(fail['classification'],'CURRENT_STRATEGY_EXCEEDS_REGISTERED_RESOURCES')
        self.assertTrue(fail['actual_co_motion_confirmed']);self.assertFalse(fail['suspect'])
        self.assertEqual(len(fail['resource_forecast']),2)

    def test_finite_budget_is_not_an_arbitrary_minimum_speed(self):
        self.assertIsNone(resource_forecast(.0001,2.,.001,2.,{'wall_s':100.,'sim_s':100.}))
        out=resource_forecast(.0001,2.,1.,2.,{'wall_s':100.,'sim_s':100.})
        self.assertFalse(out['physical_hazard'])
        self.assertEqual(out['authority'],'SAFE_PARTIAL_PROGRESS_TO_AGENT')


class RecorderSchemaTests(unittest.TestCase):
    def test_first_constructor_row_and_later_governed_row_survive_actual_compact_writer(self):
        from interaction_runtime.compact_v2 import CompactRecorder
        from agentic_interaction_v87.runtime import runtime_class
        cls=runtime_class(object);r=object.__new__(cls)
        r.supervisor=NS(force=NS());r._v87_diagnostics_installed=False
        with tempfile.TemporaryDirectory() as folder:
            record=CompactRecorder(folder);r.record=record
            record.coordinate_fields=lambda row:{'step':np.uint32(row['step']),'joint_q':np.float64(0.)}
            r._install_diagnostics();first=record.coordinate_fields({'step':0})
            r.command_governor=NS(latest={'linear_speed_m_s':.01,'linear_acceleration_m_s2':.02})
            r.supervisor.force.feedback={'scale':.8,'load_regulation_active':True}
            second=record.coordinate_fields({'step':1})
            record.pending=[first,second];record.count=2;record.close()
            with np.load(Path(folder)/'00000000.npz') as a:
                self.assertTrue(np.isnan(a['v87_constraint_speed_scale'][0]))
                self.assertAlmostEqual(a['v87_constraint_speed_scale'][1],.8)
                self.assertFalse(a['v87_command_governor_valid'][0])
                self.assertTrue(a['v87_command_governor_valid'][1])


class UnconfirmedLifecycleDispatchTests(unittest.TestCase):
    def test_real_session_routes_unconfirmed_hold_to_diagnostic_owner(self):
        from test_recovery_supervisor_v86 import fixture
        from agentic_interaction_v81.session import serve
        with tempfile.TemporaryDirectory() as folder:
            ops=fixture(folder);r=ops.r;r.job['v87_continuous_recovery']=True
            world=r.world;world.close=Mock();world.reset=Mock()
            ops.decision_packet=lambda *args:{'unloaded':True}
            ops.articulation=lambda:NS(units={},validate_target=lambda a:True)
            ops.close_workers=Mock()
            def fail(command):
                r.g['recovery_state']='FINITE_CONTACT_OPERATION'
                return dict(status='CONTROL_STATE_UNTRUSTED',outcome='UNRECOVERABLE',
                    reason='SAFE_HOLD_OR_RELEASE_NOT_CONFIRMED')
            ops.prepare_grasp=fail
            (r.output/'agent_commands').mkdir();(r.output/'agent_commands/000.json').write_text('{}')
            command={'operation':'prepare_grasp','sequence':0,'reason':'fault injection'}
            observed=ModuleType('interaction_runtime.registered_observation_v71');observed.initialize=Mock()
            owner=ModuleType('interaction_runtime.probing_v7');owner.OperationsV7=object
            with patch.dict(sys.modules,{'interaction_runtime.registered_observation_v71':observed,
                    'interaction_runtime.probing_v7':owner}), \
                 patch('agentic_interaction_v81.session.operations_class',return_value=lambda runtime:ops), \
                 patch('agentic_interaction_v81.session.window',return_value=(20,{})), \
                 patch('agentic_interaction_v81.session.status'), \
                 patch('agentic_interaction_v81.session.receive',return_value=(command,{})), \
                 patch('agentic_interaction_v81.session.complete'), \
                 patch('agentic_interaction_v87.lifecycle.retain',return_value={'status':'DIAGNOSTIC_OWNER'}) as retain:
                out=serve(r)
            self.assertEqual(out['status'],'DIAGNOSTIC_OWNER')
            self.assertIs(retain.call_args.args[0].r.world,world)
            self.assertEqual(retain.call_args.args[1],1)
            world.close.assert_not_called();world.reset.assert_not_called()


class FinalSubmissionTests(unittest.TestCase):
    def runtime(self,collision=True):
        from test_constraint_feedback_v87 import fixture
        from agentic_interaction_v87.runtime import runtime_class
        from agentic_interaction_v87.command_governor import CommandGovernor
        class Parent:
            def motion_limits(self,phase):return dict(self.policy.motion(phase))
            def check_q(self,q):
                self.checked.append(np.asarray(q).copy());return collision,'injected prediction'
            def cartesian_step(self,goal,phase):
                self.native_retention_and_IK_called=True;self.g['phase']=phase
                self.native_motion_limits=self.motion_limits(phase)
                proposal=self.qtarget[self.arm]+np.array([.03,0,0,0,0,0,0])
                okay,why=self.check_q(proposal)
                if not okay:
                    from interaction_runtime.policy import SafetyStop
                    raise SafetyStop('COMMAND_COLLISION:'+why)
                self.qtarget[self.arm]=proposal;self.g['qvelocity']=np.ones(7)*3
                self.seed=proposal.copy()
                return self.step()
            def step(self):self.sent.append(self.qtarget.copy());return {}
        cls=runtime_class(Parent);r=object.__new__(cls);base=fixture()
        r.__dict__.update(base.__dict__);r.g['qvelocity']=np.zeros(7);r.c={'arm_speed_limits':np.ones(7)*3}
        r.dt=1/240;r.tick=0;r.seed=np.zeros(7);r.reference_bias=np.zeros(7)
        r.ensure_execution_budget=Mock();r._v87_fixture_done=False;r.checked=[];r.sent=[]
        def fk(q):
            T=np.eye(4);T[0,3]=q[0];return T
        r.command_governor=CommandGovernor(fk,r.dt)
        return r

    def test_native_reference_pipeline_survives_and_only_final_command_is_submitted(self):
        r=self.runtime();r.cartesian_step(np.eye(4),'OPEN_CONTINUE')
        self.assertTrue(r.native_retention_and_IK_called)
        self.assertEqual(len(r.sent),1)
        self.assertLess(np.linalg.norm(r.sent[0]),.012/240)
        self.assertTrue(all(np.linalg.norm(q)<.012/240 for q in r.checked))
        self.assertIsNone(r._v87_submission)

    def test_rejected_proposal_restores_last_accepted_command_and_governor_state(self):
        r=self.runtime(False);before=r.qtarget.copy()
        with self.assertRaisesRegex(RuntimeError,'CONTROLLED_EXECUTION_INTERRUPT'):
            r.cartesian_step(np.eye(4),'OPEN_CONTINUE')
        self.assertEqual(r.sent,[]);np.testing.assert_array_equal(r.qtarget,before)
        np.testing.assert_array_equal(r.command_governor.previous_twist,np.zeros(6))
        self.assertEqual(r.command_governor.calls,0)
        self.assertIsNone(r.supervisor.latched)
