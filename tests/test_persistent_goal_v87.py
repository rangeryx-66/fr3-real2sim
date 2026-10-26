"""Goal ownership faults at the actual dispatcher; no physical-success claims."""
import hashlib,json,tempfile,time,unittest,sys
from pathlib import Path
from types import SimpleNamespace as NS,ModuleType
from unittest.mock import Mock,patch
from test_recovery_supervisor_v86 import fixture
from agentic_interaction_v87 import persistent_goal as persistent
from agentic_interaction_v81 import task_recovery
from interaction_runtime.segments import EpisodeBudget


class PersistentGoalTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.ops=fixture(self.tmp.name);self.r=self.ops.r
        self.r.job[persistent.FLAG]=dict(deadline_unix_s=15400.)
        self.r.job['agent_wait_timeout_s']=600.;self.r.event=Mock()

    def test_every_soft_reason_and_exhausted_alternative_assessment_keeps_goal_active(self):
        reasons=['NO_IK','ALL_CANDIDATES_REJECTED','GRASP_FAILED','SLIP','CONTACT_DEGRADED',
            'NO_CURRENT_PATH','JOINT_MARGIN_WARNING','SOFT_SPEED_WARNING','LOAD_WARNING',
            'EXIT_PATH_NOT_FOUND','RECOVERY_COUNT_REACHED','AGENT_UNCERTAIN']
        self.r.episode_budget.remaining=lambda t:dict(wall_s=0.,sim_s=0.)
        self.ops._v81_phase_costs=dict(acquire=1e9,release=1e9)
        with patch('time.time',return_value=1000.):
            for reason in reasons:
                command=dict(operation='finish',reason=reason,owner_requested_stop=True,
                    stop_assessment=dict(kind='NO_SAFE_CONTINUATION',considered=[]))
                with self.subTest(reason=reason):
                    out=task_recovery.finish_check(self.ops,command)
                    self.assertFalse(out['allowed']);self.assertTrue(out['recoverable'])
                    self.assertEqual(out['reason'],'LOCAL_FAILURE_DOES_NOT_END_REGISTERED_GOAL')

    def test_complete_goal_and_true_deadline_are_distinct_from_planning_failure(self):
        done=dict(actual_goal_reached=True,meaningful_reverse=True,complete_independent_validation=True)
        with patch('time.time',return_value=1000.):self.assertTrue(persistent.finish_assessment(self.ops,done)['task_complete'])
        done['complete_independent_validation']=False
        with patch('time.time',return_value=15400.):
            out=persistent.finish_assessment(self.ops,done)
            self.assertTrue(out['allowed']);self.assertFalse(out['task_complete'])

    def test_real_latch_is_retained_and_never_reset(self):
        latch=dict(reason='EMERGENCY_WRIST_TORQUE');self.r.supervisor.latched=latch
        out=persistent.finish_assessment(self.ops,{})
        self.assertTrue(out['allowed']);self.assertFalse(out['task_complete'])
        self.assertIs(self.r.supervisor.latched,latch);self.r.hold.assert_not_called()

    def test_reasoning_subtimeout_renews_only_within_absolute_deadline(self):
        with patch('time.time',return_value=15000.),patch('time.perf_counter',return_value=20.):
            renewed=persistent.renew_decision(self.r,0.,10.,3)
        self.assertEqual(renewed,(420.,15400.));self.r.event.assert_called_once()
        record=json.loads((self.r.output/'PERSISTENT_STRATEGY_SWITCH_REQUIRED.json').read_text())
        self.assertFalse(record['useful_search_claimed']);self.assertFalse(record['world_recreated'])
        with patch('time.time',return_value=15400.):self.assertIsNone(persistent.renew_decision(self.r,0.,10.,3))

    def test_launcher_enforces_absolute_deadline_without_credit(self):
        from agentic_interaction_v81.orchestration_extension import wait_actor
        actor=NS(wait=Mock(return_value=0))
        with patch('time.time',return_value=1200.):
            self.assertEqual(wait_actor(actor,self.r.job,self.r.output),0)
        actor.wait.assert_called_once_with(timeout=14200.)

    def test_explicit_registration_counts_startup_and_rejects_another_asset(self):
        p=Path(self.tmp.name)/'registration.json';p.write_text(json.dumps(dict(fixed_cases=[dict(asset_id='32086',wall_budget_s=14400)])))
        spec=dict(phase='engineering',scene='asset_32086',wall_budget_s=14400,sim_budget_s=14400,staged_exit=True,
            persistent_goal=dict(kind=persistent.KIND,registration=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),formal_start_unix_s=1000.))
        job={}
        with patch('time.time',return_value=1100.):
            persistent.activate(spec,job);budget=EpisodeBudget(job,now=lambda:10.)
        self.assertEqual(job[persistent.FLAG]['deadline_unix_s'],15400.)
        self.assertEqual(budget.wall,14300.)
        with patch('time.time',return_value=1100.),self.assertRaisesRegex(ValueError,'FROZEN_ASSET'):
            persistent.activate(dict(spec,scene='asset_45936'),{})


class RealDispatcherTests(unittest.TestCase):
    def test_soft_failure_then_premature_finish_then_inspect_stays_same_world_until_deadline(self):
        from agentic_interaction_v81.session import serve
        for reason in ['NO_IK','ALL_CANDIDATES_REJECTED','GRASP_FAILED','SLIP',
                       'CONTACT_DEGRADED','NO_PATH','EXIT_PATH_NOT_FOUND','RECOVERY_COUNT','UNCERTAIN']:
            with self.subTest(reason=reason),tempfile.TemporaryDirectory()as folder:
                ops=fixture(folder);r=ops.r;world=r.world
                world.close=Mock();world.reset=Mock();r.job.update(v87_continuous_recovery=True,
                    v87_persistent_goal=dict(deadline_unix_s=15400.),agent_wait_timeout_s=600.)
                ops._abc_retain=False;ops.close_workers=Mock()
                ops.decision_packet=lambda *args:{'unloaded':True}
                ops.prepare_grasp=lambda c:dict(plan=False,reason=reason)
                ops.articulation=lambda:NS(units={},validate_target=lambda a:True)
                (r.output/'agent_commands').mkdir()
                commands=[dict(operation=name,sequence=i,reason=reason)for i,name in enumerate(['prepare_grasp','finish','inspect','finish'])]
                for i in range(4):(r.output/'agent_commands'/('%03d.json'%i)).write_text('{}')
                clock=[1000.]
                def receive(*args):
                    c=commands.pop(0)
                    if c['sequence']==3:clock[0]=15400.
                    return c,{}
                observed=ModuleType('interaction_runtime.registered_observation_v71');observed.initialize=Mock()
                owner=ModuleType('interaction_runtime.probing_v7');owner.OperationsV7=object
                with patch.dict(sys.modules,{'interaction_runtime.registered_observation_v71':observed,'interaction_runtime.probing_v7':owner}), \
                     patch('agentic_interaction_v81.session.operations_class',return_value=lambda runtime:ops), \
                     patch('agentic_interaction_v81.session.window',return_value=(600,{})), \
                     patch('agentic_interaction_v81.session.status'), \
                     patch('agentic_interaction_v81.session.receive',side_effect=receive), \
                     patch('agentic_interaction_v81.session.complete'), \
                     patch('time.time',side_effect=lambda:clock[0]):
                    out=serve(r)
                self.assertEqual(len(out['trace']),4)
                self.assertEqual(out['trace'][1]['result']['status'],'CONTINUATION_REQUIRED')
                self.assertEqual(out['trace'][2]['command']['operation'],'inspect')
                self.assertEqual(out['trace'][3]['result']['outcome'],'EPISODE_STOPPED')
                self.assertIs(r.world,world);world.close.assert_not_called();world.reset.assert_not_called()
                self.assertEqual(r.coordinate,.55);self.assertIsNone(r.supervisor.latched)
