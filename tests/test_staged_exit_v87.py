"""Explicit fault injection; native physical acceptance is a separate result."""
import ast,math,sys,tempfile,unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from test_agentic_interaction_v81_revolute_certificate import CertificateTests
from test_recovery_supervisor_v86 import fixture
from agentic_interaction_v81.revolute_certificate import evaluate
from agentic_interaction_v81.replanning import coverage
from agentic_interaction_v81 import recovery_supervisor as supervisor
from agentic_interaction_v87 import staged_exit as staged
from interaction_runtime.policy import SafetyStop


@contextmanager
def untimed(r):yield []


class MotionAdmissionTests(unittest.TestCase):
    def test_ninety_degree_motion_does_not_require_a_predicted_parking_route(self):
        g,c,a,p,s,m,records,close,exit_check=CertificateTests().fixture()
        g['job']['v87_staged_exit']=True
        a=[dict(role='FIT_OPEN',target_q=math.pi/2,coordinate_unit='rad'),a[-1]]
        exit_check.side_effect=AssertionError('future parking must not run during motion admission')
        with patch.dict(sys.modules,m):out=evaluate(g,c,a,p,dict(gripper_kp_nm_rad=10.),s)
        self.assertTrue(out['valid']);exit_check.assert_not_called();close.assert_called_once()
        self.assertTrue(all(e['exit_valid'] is None for e in out['endpoints']))
        plan=dict(path=[dict(state=0.),dict(state=.1)],execution_feasibility=dict(v81_revolute_chain=out))
        self.assertTrue(coverage(plan,[math.pi/2])['full_requested_actions_feasible'])
        self.assertGreater(len(records),100);self.assertEqual(g['actual'](),0.)

    def test_stop_envelope_collision_margin_and_ik_still_reject(self):
        for kind in ['collision','margin','ik','stopping_limit']:
            g,c,a,p,s,m,records,close,exit_check=CertificateTests().fixture()
            g['job']['v87_staged_exit']=True
            if kind=='collision':g['collision'].check=lambda *args:(False,'original collision')
            if kind=='margin':g['model'].margin=lambda q:.049
            if kind=='ik':s.solve=lambda *args,**kw:None
            if kind=='stopping_limit':g['job']['articulation_v7']['limits_native']=[0.,.1]
            with self.subTest(kind=kind),patch.dict(sys.modules,m):
                self.assertFalse(evaluate(g,c,a,p,dict(gripper_kp_nm_rad=10.),s)['valid'])

    def test_an_exit_deferral_without_a_stopped_motion_proof_is_not_coverage(self):
        for endpoint in [dict(target_rad=1.,exit_valid=None),
                dict(target_rad=1.,exit_valid=None,motion_valid=True,stopped_hold_checked=False)]:
            plan=dict(path=[dict(state=0.)],execution_feasibility=dict(v81_revolute_chain=
                dict(valid=True,staged_exit=True,endpoints=[endpoint])))
            self.assertFalse(coverage(plan,[1.])['full_requested_actions_feasible'])

    def test_prismatic_motion_preserves_collisions_margins_and_exact_stopped_hold(self):
        from agentic_interaction_v87.prismatic_motion import evaluate_prismatic_motion
        for fault in [None,'collision','margin','ik','exact_stop_collision']:
            g,c,a,p,s,m,records,close,exit_check=CertificateTests().fixture()
            g['job']['articulation_v7']['limits_native']=[0.,.25]
            g['v75_motion_prior']=dict(axis_world=[1.,0.,0.],origin_world_m=[0.,0.,0.])
            for action in a:action['coordinate_unit']='m'
            original=g['model'].poses
            def poses(q,*args,**kw):
                P=original(q,*args,**kw);P['tcp_link'][0,3]=q[0];return P
            g['model'].poses=poses
            s.solve=lambda T,*args,**kw:np.r_[T[0,3],np.zeros(6)]
            if fault=='collision':g['collision'].check=lambda *args:(False,'native collision')
            if fault=='margin':g['model'].margin=lambda q:.049
            if fault=='ik':s.solve=lambda *args,**kw:None
            if fault=='exact_stop_collision':
                g['collision'].check=lambda P,*args:(abs(P['tcp_link'][0,3]-1.1)>1e-10,'exact stopped collision')
            with self.subTest(fault=fault),patch.dict(sys.modules,m):
                out=evaluate_prismatic_motion(g,c,a,p,dict(gripper_kp_nm_rad=10.),s)
                self.assertEqual(out['valid'],fault is None)
                if fault is None:self.assertIsNone(out['exit']['valid'])


class StagedExitTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.ops=fixture(self.tmp.name);r=self.ops.r
        r.job.update(v87_staged_exit=True,v87_continuous_recovery=True)
        r.fingers=np.arange(7,13);r.g['qvelocity']=np.zeros(7)
        r.collision=NS(check=Mock(return_value=(True,'native clear')))
        r.intentional_contact=Mock(return_value=True)  # existing RELEASE phase
        r.model.poses=Mock(return_value={});r.event=Mock();r.tcp=lambda:np.eye(4)
        r.move_path=Mock();r.step=Mock();r.policy.data['jaw_release_m_s']=.01
        self.ops.articulation=lambda:NS(joint_type='revolute')
        self.park=Mock(side_effect=lambda g,p:dict(reached=True,unloaded=True,
            measured_collision_check=dict(valid=True),region=p['parking_region']))
        self.route=Mock(side_effect=[(None,'native obstacle'),([[0.]*7,[.1]*7],'DIRECT')])
        self.modules={'native_g1_safe_park':NS(verify_park=self.park,route_to_joint=self.route),
            'native_g1_contact_recovery':NS(release_geometry=Mock(return_value=dict(valid=True)))}
        self.routes=Mock(return_value=iter([([[0.]*7,[.04]*7],{})]))
        self.targets=Mock(return_value=iter([('specified',np.zeros(7)),('alternative',np.full(7,.1))]))

    def run_exit(self,time_scope=untimed):
        with patch.dict(sys.modules,self.modules),patch.object(staged,'local_routes',self.routes),\
             patch.object(staged,'parking_targets',self.targets),\
             patch('agentic_interaction_v81.exit_clearance.ParkingClearance',lambda s:s),\
             patch('agentic_interaction_v81.exit_routes.timed_returns',time_scope):
            return staged.release_park(self.ops)

    def test_actual_ninety_then_release_local_retreat_fresh_observation_and_alternative_park(self):
        # This explicit fixture represents completed motion; it is not a
        # simulation measurement or a claim that native physics reached90deg.
        r=self.ops.r;r.coordinate=math.pi/2;world=r.world
        order=[]
        r.move_path.side_effect=lambda path,phase:order.append(phase)
        self.ops.observe.side_effect=lambda:order.append('FRESH_RGBD')or dict(fresh=True)
        out=self.run_exit()
        self.assertTrue(out['safe']);self.assertTrue(out['release']['confirmed'])
        self.assertTrue(out['retreat']['reached']);self.assertTrue(out['parking']['reached'])
        self.assertEqual(out['park']['region'],'alternative')
        self.assertEqual(order,['CLEARANCE_RETREAT','FRESH_RGBD','SAFE_PARK_MOTION'])
        self.assertEqual([s['state']['coordinate_native']for s in out['stages']],[math.pi/2]*5)
        self.assertIs(r.world,world);self.assertEqual(r.coordinate,math.pi/2)

    def test_no_parking_route_retains_distinct_release_and_clearance_results(self):
        self.route.side_effect=None;self.route.return_value=(None,'collision')
        out=self.run_exit();self.assertEqual(out['status'],staged.EXIT_PATH_NOT_FOUND)
        self.assertFalse(out['safe']);self.assertTrue(out['release']['confirmed'])
        self.assertTrue(out['retreat']['reached']);self.assertFalse(out['parking']['reached'])
        self.assertTrue(out['hold_confirmed'])
        r=self.ops.r;result=dict(segments=[dict(status='TARGET_REACHED',gain_native=.5)],safe_exit=out)
        finished=supervisor.complete(self.ops,dict(operation='acquire_execute'),result)
        self.assertEqual(finished['segments'],result['segments'])
        self.assertFalse(finished['episode_termination_required']);self.assertTrue(self.ops._v86_exit_pending)
        self.ops.release_park.assert_not_called()

    def test_local_direction_failure_can_use_independently_native_checked_parking(self):
        self.routes.return_value=iter([])
        world=self.ops.r.world;coordinate=self.ops.r.coordinate
        out=self.run_exit()
        self.assertTrue(out['safe']);self.assertFalse(out['retreat']['reached'])
        self.assertTrue(out['retreat']['independent_parking_search'])
        self.assertTrue(out['release']['confirmed']);self.assertTrue(out['parking']['reached'])
        self.assertEqual(self.ops.r.move_path.call_count,1)
        self.assertEqual(self.ops.r.move_path.call_args.args[1],'SAFE_PARK_MOTION')
        self.ops.observe.assert_called_once();self.assertIs(self.ops.r.world,world)
        self.assertEqual(self.ops.r.coordinate,coordinate)

    def test_local_failure_cannot_bypass_original_timed_parking_rejection(self):
        self.routes.return_value=iter([])
        self.route.side_effect=None;self.route.return_value=(None,'TIMED_RETURN_ROUTE_UNAVAILABLE:native obstacle')
        out=self.run_exit()
        self.assertFalse(out['safe']);self.assertEqual(out['status'],staged.LOCAL_RETREAT_NOT_FOUND)
        self.assertTrue(out['release']['confirmed']);self.assertFalse(out['parking']['reached'])
        self.ops.r.move_path.assert_not_called();self.assertEqual(self.route.call_count,2)

    def test_rest_heading_search_keeps_other_joints_and_native_limits(self):
        r=self.ops.r;home=np.array([0.,-.5,0.,1.,0.,.5,0.])
        limits=np.tile([-math.pi,math.pi],(7,1));r.model.limits=limits
        r.model.margin=lambda q:float(np.minimum(q-limits[:,0],limits[:,1]-q).min())
        r.robot.get_joint_positions=lambda:np.r_[1.4,np.zeros(12)]
        def original(g):
            for yaw in [0.,-math.radians(20),math.radians(20)]:
                q=home.copy();q[0]=yaw;yield 'original',q
        self.modules['native_g1_safe_park'].parking_targets=original
        with patch.dict(sys.modules,self.modules):targets=list(staged.parking_targets(self.ops))
        self.assertGreater(len(targets),3);self.assertLessEqual(len(targets),24)
        self.assertTrue(any(abs(q[0]-1.4)<1e-10 for _,q in targets))
        self.assertEqual([q[0] for _,q in targets[:3]],[0.,-math.radians(20),math.radians(20)])
        for _,q in targets:
            np.testing.assert_array_equal(q[1:],home[1:]);self.assertGreater(r.model.margin(q),.05)
        self.assertEqual(len({tuple(q)for _,q in targets}),len(targets))

    def test_sensing_failure_does_not_veto_independently_checked_safety_exit(self):
        self.ops.observe.side_effect=OSError('segmentation worker unavailable')
        out=self.run_exit()
        self.assertTrue(out['safe']);self.assertIn('unavailable',out['post_clearance_observation_error'])
        self.assertTrue(out['parking']['reached'])

    def test_physical_hard_fault_during_sensing_never_attempts_parking(self):
        self.ops.observe.side_effect=SafetyStop('INVALID_WRIST_FT')
        with self.assertRaises(SafetyStop):self.run_exit()
        self.route.assert_not_called()

    def test_release_sweep_rejection_keeps_grasp_and_never_opens_jaw(self):
        r=self.ops.r;r.grasp_reference=np.eye(4);r.g['reference']=np.eye(4)
        self.modules['native_g1_contact_recovery'].release_geometry.return_value=dict(valid=False,reason='native collision')
        with patch('agentic_interaction_v84.operations.contact_trusted',return_value=True):out=self.run_exit()
        self.assertEqual(out['status'],staged.SAFE_RELEASE_NOT_CONFIRMED)
        self.assertTrue(out['contact_retained']);self.assertEqual(self.ops.abc_state,'C')
        self.assertIsNotNone(r.grasp_reference);r.step.assert_not_called();r.move_path.assert_not_called()

    def test_unstable_object_or_arm_cannot_release(self):
        self.ops.r.station_motion.ready=lambda *a:False
        out=self.run_exit();self.assertEqual(out['status'],staged.SAFE_HOLD_NOT_ESTABLISHED)
        self.modules['native_g1_contact_recovery'].release_geometry.assert_not_called()
        self.ops.r.step.assert_not_called();self.ops.r.move_path.assert_not_called()

    def test_partial_acquisition_contact_does_not_require_verified_grasp_reference(self):
        r=self.ops.r
        self.assertIsNone(r.grasp_reference)
        r.collision.check.side_effect=lambda poses,moving,allow:(bool(allow),'finger/allowed handle')
        self.assertTrue(staged.safe_hold(self.ops))
        self.assertTrue(r.collision.check.call_args.args[2])
        r.step.assert_not_called();r.move_path.assert_not_called()

    def test_failed_bilateral_acquisition_can_release_and_recover_same_world(self):
        r=self.ops.r;world=r.world;coordinate=r.coordinate
        r.rows[-1]['aperture_m']=.02;r.jaw_width=.02
        r.filtered=lambda:np.array([.2,0.]) if r.rows[-1]['aperture_m']<.083 else np.zeros(2)
        r.collision.check.side_effect=lambda poses,moving,allow:(
            bool(allow or r.rows[-1]['aperture_m']>=.083),'finger/allowed handle')
        r.step.side_effect=lambda:r.rows[-1].update(aperture_m=r.jaw_width)
        def move_only_released(path,phase):
            self.assertGreaterEqual(r.rows[-1]['aperture_m'],.083)
            self.assertEqual(float(max(r.filtered())),0.)
        r.move_path.side_effect=move_only_released
        out=self.run_exit()
        self.assertTrue(out['safe']);self.assertTrue(out['release']['confirmed'])
        result=supervisor.complete(self.ops,dict(operation='acquire_execute'),dict(
            error="RuntimeError('BILATERAL_HOLD_NOT_ESTABLISHED')",segments=[],safe_exit=out))
        self.assertTrue(result['recoverable']);self.assertFalse(result['episode_termination_required'])
        self.assertIsNone(r.supervisor.latched);self.assertIs(r.world,world)
        self.assertEqual(r.coordinate,coordinate);self.assertEqual(r.job['v81_task_goal_native'],1.57)

    def test_phase_contact_scope_does_not_waive_other_native_collisions(self):
        self.ops.r.collision.check.return_value=(False,'arm/table collision')
        self.assertFalse(staged.safe_hold(self.ops))
        self.ops.r.step.assert_not_called();self.ops.r.move_path.assert_not_called()

    def test_unloaded_phase_cannot_reuse_stale_verified_grasp_contact_scope(self):
        r=self.ops.r;r.grasp_reference=np.eye(4);r.g['reference']=np.eye(4)
        r.intentional_contact.return_value=False
        r.collision.check.side_effect=lambda poses,moving,allow:(bool(allow),'finger/handle')
        self.assertFalse(staged.safe_hold(self.ops))

    def test_original_hard_latch_blocks_all_exit_motion_and_is_not_cleared(self):
        r=self.ops.r;latch=dict(reason='original hard');r.supervisor.latched=latch
        out=self.run_exit();self.assertEqual(out['status'],staged.HARD_SAFETY_STOP)
        self.assertIs(r.supervisor.latched,latch);r.stop_command.assert_not_called()
        r.hold.assert_not_called();r.move_path.assert_not_called()

    def test_hard_safety_during_retreat_propagates_without_trying_alternative(self):
        self.ops.r.move_path.side_effect=SafetyStop('NATIVE_HARD_COLLISION')
        with self.assertRaises(SafetyStop):self.run_exit()
        self.route.assert_not_called();self.ops.observe.assert_not_called()

    def test_identical_failed_parking_edges_are_not_resubmitted(self):
        self.route.side_effect=None;self.route.return_value=(None,'collision')
        self.run_exit();self.assertEqual(self.route.call_count,2)
        self.routes.return_value=iter([([[0.]*7,[.04]*7],{})])
        self.targets.return_value=iter([('specified',np.zeros(7)),('alternative',np.full(7,.1))])
        self.run_exit();self.assertEqual(self.route.call_count,2)

    def test_current_motion_collision_prevents_even_local_clearance(self):
        self.ops.r.collision.check.return_value=(False,'original collision')
        out=self.run_exit();self.assertEqual(out['status'],staged.SAFE_HOLD_NOT_ESTABLISHED)
        self.ops.r.move_path.assert_not_called()

    def test_registered_preferred_route_rejection_is_once_only_after_real_goal_record(self):
        r=self.ops.r
        r.job['v87_staged_exit_fixture']=dict(kind='reject_preferred_parking_route_once_after_task_goal')
        r.v81_task_progress=dict(maximum_measured_native=r.job['v81_task_goal_native'])
        self.route.side_effect=None;self.route.return_value=([[0.]*7,[.1]*7],'DIRECT')
        out=self.run_exit()
        self.assertTrue(out['safe']);self.assertEqual(out['park']['region'],'alternative')
        self.assertTrue(out['parking']['candidates'][0]['engineering_injected'])
        self.assertTrue(self.ops._v87_park_rejection_injected)
        self.assertEqual(self.ops.r.move_path.call_count,2)  # local + alternative only

    def test_actual_parking_calls_the_original_timed_route_wrapper(self):
        native=self.modules['native_g1_safe_park'];rejections=Mock(return_value=(None,'TIMED_RETURN_ROUTE_UNAVAILABLE:native obstacle'))
        @contextmanager
        def enforced(r):
            with patch.object(native,'route_to_joint',rejections):yield []
        out=self.run_exit(enforced)
        self.assertEqual(out['status'],staged.EXIT_PATH_NOT_FOUND)
        self.assertEqual(rejections.call_count,2);self.route.assert_not_called()
        self.assertEqual(self.ops.r.move_path.call_count,1)  # local only

    def test_exit_dwell_uses_original_reserved_exit_phase_after_work_is_exhausted(self):
        from interaction_runtime.runtime import Runtime
        r=self.ops.r;r.g['phase']='OPEN_CONTINUE';r._decelerating=False;r.budget_exit_only=True
        r.episode_budget.remaining=lambda t:dict(wall_s=0.,sim_s=0.)
        r.hold.side_effect=lambda seconds:Runtime.ensure_execution_budget(r,r.g['phase'])
        out=self.run_exit()
        self.assertTrue(out['safe']);self.assertTrue(r.budget_exit_only)
        self.assertEqual(r.g['phase'],'RELEASE')  # fake move_path does not change phases


class StagedBudgetTests(unittest.TestCase):
    def test_unknown_future_route_keeps_exit_reserve_and_complete_motion_budget(self):
        from agentic_interaction_v87.staged_cost import admit_staged_motion
        for available,expected in [(1000.,True),(0.,False)]:
            with self.subTest(available=available),tempfile.TemporaryDirectory()as tmp:
                ops=fixture(tmp);r=ops.r
                r.c=dict(arm_speed_limits=[1.]*7);r.base=np.zeros(4);r.solver=None;r.fingers=np.arange(7,13)
                r.v75_config=dict(active_wall_per_sim_s_upper=2.,healthy_segment_additive_excess_sim_s=1.,
                    closure_hold_sim_s_bound=2.,closure_bound_source='original recorded bound')
                r.profile=NS(calls={'RGBD_SAM3_and_incremental_geometry':1},
                    inclusive={'RGBD_SAM3_and_incremental_geometry':2.})
                r.policy.motion=lambda phase:dict(speed_m_s=.01,acceleration_m_s2=.03)
                r.policy.phase=lambda phase:phase
                r.policy.data.update(arm_command_acceleration_rad_s2={'PREGRASP':.5,'APPROACH':.5})
                r.episode_budget=NS(remaining=lambda t:dict(wall_s=available,sim_s=available),reserve_wall=300.,reserve_sim=60.)
                transition=dict(valid=True,return_path=[[0.]*7,[0.]*7,[0.]*7],
                    approach_audit=dict(preplan=[[0.]*7,[0.]*7],approach=[[0.]*7,[0.]*7]))
                native=NS(validate_execution=Mock(return_value=transition))
                actions=[dict(role='FIT_OPEN',target_q=.18),dict(role='VALIDATION_REVERSE',target_q=.08)]
                with patch.dict(sys.modules,{'native_g1_agent_execution':native,'xarm7_native_gripper':NS(GRIPPER_JOINTS=[])}),\
                     patch('agentic_interaction_v87.staged_cost.timed_path',return_value=iter([])),\
                     patch('agentic_interaction_v87.staged_cost.segment_plan',return_value=dict(valid=True,nominal_duration_s=4.)):
                    out=admit_staged_motion(r,{},actions,dict(v75_whole_chain=dict(valid=True,exit=dict(valid=None))))
                self.assertEqual(out['admit'],expected)
                self.assertTrue(out['future_exit_cost_unknown']);self.assertTrue(out['includes_complete_validation'])
                self.assertEqual(out['cost_sources']['exit']['wall_s'],300.)
                self.assertEqual(out['cost_sources']['exit']['sim_s'],60.)
                self.assertEqual(r.episode_budget.reserve_wall,300.)
                native.validate_execution.assert_called_once()


class LauncherRegistrationTests(unittest.TestCase):
    def test_strict_v87_entry_accepts_flags_but_frozen_v8_compat_does_not_receive_them(self):
        import agentic_interaction_v81.launch as launcher
        import json
        class BeforePhysicalPrepare(Exception):pass
        with tempfile.TemporaryDirectory()as tmp:
            folder=Path(tmp);spec=folder/'spec.json';env=folder/'env.json';env.write_text('{}')
            data=dict(schema='agentic-interaction/v81',episode_id='unit_only',phase='engineering',
                scene='unit',physics_file='/unused',physics_sha256='0'*64,identification_case=0,
                integration_profile='v87_abc_hybrid_recovery_v1',staged_exit=True,
                staged_exit_fixture=dict(kind='reject_preferred_parking_route_once_after_task_goal'),
                orchestration_authorization=dict(human_authorization_ref='unit explicit task',
                    max_total_extra_wall_s=1800,parked_decision_renewal_s=1800))
            spec.write_text(json.dumps(data))
            prepare=Mock(side_effect=BeforePhysicalPrepare)
            # Read-only unit mocks stop BEFORE prepare can export assets,
            # reserve GPUs, create an Actor or issue a native control command.
            with patch.object(launcher,'baseline_audit'),patch.object(launcher,'prepare',prepare),\
                 patch('agentic_interaction_v81.terminal_safety.registry_path',return_value=folder/'safety'),\
                 patch('agentic_interaction_v81.terminal_safety.require_clearance'):
                with self.assertRaises(BeforePhysicalPrepare):launcher._launch(spec,env)
                compat=json.loads(prepare.call_args.args[0].read_text())
                self.assertEqual(compat['schema'],'agentic-interaction/v8')
                self.assertNotIn('staged_exit',compat);self.assertNotIn('staged_exit_fixture',compat)
                data['integration_profile']='v84_abc_continuous_v1';spec.write_text(json.dumps(data))
                prepare.reset_mock()
                with self.assertRaisesRegex(ValueError,'STRICT_V81_SPEC_REQUIRED'):launcher._launch(spec,env)
                prepare.assert_not_called()


if __name__=='__main__':unittest.main()
