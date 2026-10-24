import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace,ModuleType
from unittest.mock import patch

from agentic_interaction_v81 import task_recovery as recovery


def fixture(folder):
    g=dict(recovery_state='UNLOADED_OBSERVATION',agent_observations=[],at_safe_park=True)
    r=SimpleNamespace(output=Path(folder),job=dict(v81_active_recovery=True,
        v81_task_goal_native=1.5707963267948966,v81_minimum_reverse_native=.17453292519943295,
        runtime_acquisition_budget=6,articulation_v7=dict(joint_type='revolute'),agent_wait_timeout_s=10),
        actual=lambda:.55,base_generation=1,grasp_reference=None,g=g,
        base=SimpleNamespace(tolist=lambda:[0,0,0,0]),
        supervisor=SimpleNamespace(latched=None,force=SimpleNamespace(settled=lambda:True)),
        policy=SimpleNamespace(data=dict(force=dict(unloaded_pad_n=.05))),filtered=lambda:[0.,0.],
        tick=0,dt=1/240,record=SimpleNamespace(segments=[]),
        episode_budget=SimpleNamespace(remaining=lambda t:dict(wall_s=2000,sim_s=1000)),
        v81_task_progress=dict(maximum_measured_native=1.1))
    return SimpleNamespace(r=r,g=g,current_plan_ready=True,acquisitions=6)


class TaskRecoveryTests(unittest.TestCase):
    def test_early_probe_reverse_cannot_replace_post_goal_reverse_and_validation(self):
        from agentic_interaction_v81.integration import progress
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d);ops.r.job['v81_integrated_repair']=True
            ops.r.v81_task_progress=None
            ops.r.record.count=100
            progress(ops.r,dict(joint_coordinate=1.571))
            ops.r.record.count=120
            progress(ops.r,dict(joint_coordinate=1.2))
            self.assertEqual(ops.r.v81_task_progress['first_goal_record'],100)
            result=dict(status='TARGET_REACHED',physical_transmission_valid=True,
                        requested_target_reached=True,gain_native=-.3)
            ops.r.record.segments=[dict(role='VALIDATION_EARLY',first_step=50,result=result)]
            done=recovery.completed(ops)
            self.assertTrue(done['actual_goal_reached'])
            self.assertFalse(done['meaningful_reverse'])
            self.assertFalse(done['complete_independent_validation'])
            self.assertFalse(recovery.finish_check(ops,{})['allowed'])
            ops.r.record.segments.append(dict(role='VALIDATION_AFTER_GOAL',first_step=110,result=result))
            self.assertTrue(recovery.finish_check(ops,{})['task_complete'])

    def test_broken_worker_publication_is_not_a_controller_fault(self):
        from agentic_interaction_v81.validation_queue import model_state
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d);ops.r.record.segments=[dict(id=1,role='FIT_OPEN')]
            (Path(d)/'model').mkdir();(Path(d)/'model/MODEL_PUBLICATION.json').write_text('{')
            result=model_state(ops)
            self.assertEqual(result['state'],'WORKER_FAILED')
            self.assertFalse(result['qualified_model_approved'])
            self.assertTrue(result['manipulation_may_continue'])
            self.assertEqual(recovery.classify(ops,dict(pending_validation=result)),'READY')

    def test_post_contact_observation_retires_old_angle_and_view_count_is_not_a_stop(self):
        from agentic_interaction_v81.operations import operations_class
        from unittest.mock import Mock
        class Owner:
            def observe(self):
                self.g['agent_observations'].append(dict(folder='new',world_sim_s=20.))
                return dict(observation=True)
        with tempfile.TemporaryDirectory() as d:
            f=fixture(d);ops=operations_class(Owner)();ops.__dict__.update(f.__dict__)
            old=[dict(folder='old',world_sim_s=5.)]
            ops.g.update(agent_observations=old,observation_generations=[],wrist_routes=[],
                         agent_moves=4,wrist_config=dict(settle_s=.2))
            ops._last_contact_finished_world_s=10.
            ops.r.move_path=Mock();ops.r.hold=Mock()
            self.assertFalse(ops.move_and_observe(dict(feasible=False))['executed'])
            ops.r.move_path.assert_not_called()
            self.assertTrue(ops.move_and_observe(dict(feasible=True,route=dict(q_path=[[0],[1]])))['observation'])
            ops.r.move_path.assert_called_once_with([[1]],'PREGRASP')
            self.assertEqual(ops.g['observation_generations'],[old])
            self.assertEqual(ops.g['agent_observations'],[dict(folder='new',world_sim_s=20.)])
            self.assertFalse(ops.current_plan_ready)
            self.assertEqual(ops.g['agent_moves'],5)

    def test_failed_validation_is_history_not_completion_or_fit(self):
        from agentic_interaction_v81.validation_queue import enqueue,mark,requests
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d);ops.r.job['episode_id']='test'
            action=dict(role='VALIDATION_REVERSE',target_q=.8)
            command=dict(segments=[action],reason='fresh native route',inspected_image_sha256='a'*64)
            enqueue(ops,command);mark(ops,action,'PHYSICAL_FAILURE',result=dict(status='TRANSMISSION_FAILURE'))
            enqueue(ops,command)
            row=requests(ops)['requests'][0]
            self.assertEqual(row['status'],'PENDING_REPLAN')
            self.assertEqual(row['failed_attempts'][0]['result']['status'],'TRANSMISSION_FAILURE')
            self.assertFalse(recovery.completed(ops)['complete_independent_validation'])

    def test_transmission_failure_keeps_goal_and_requires_unloaded_recovery(self):
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d);command=dict(operation='attempt_grasp_and_probe',sequence=3)
            result=dict(segments=[dict(status='TRANSMISSION_FAILURE',gain_native=.2)],safe_exit=dict(safe=True))
            self.assertEqual(recovery.after(ops,command,result)['task_recovery']['status'],'CONTACT_RECOVERY_REQUIRED')
            self.assertEqual(recovery.state(ops)['goal_native'],1.5707963267948966)
            self.assertFalse(recovery.before(ops,dict(operation='finish',sequence=4))['allowed'])
            ops.r.grasp_reference='still loaded'
            self.assertEqual(recovery.classify(ops,result),'SAFE_BOUNDARY_UNAVAILABLE')

    def test_same_failed_request_requires_changed_evidence_or_action(self):
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d);c=dict(operation='build_geometry',sequence=0,planning_target_q=1.1,reason='first')
            self.assertIsNone(recovery.before(ops,c))
            recovery.after(ops,c,dict(plan=False,reason='NO_IK'))
            c.update(sequence=1,reason='same request with different prose')
            self.assertEqual(recovery.before(ops,c)['status'],'STRATEGY_CHANGE_REQUIRED')
            c['planning_target_q']=.8
            self.assertIsNone(recovery.before(ops,c))
            ops.g['agent_observations'].append(dict(folder='fresh',world_sim_s=7.))
            c['planning_target_q']=1.1
            self.assertIsNone(recovery.before(ops,c))

    def test_action_timeout_is_not_episode_budget_exhaustion(self):
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d)
            self.assertEqual(recovery.classify(ops,dict(status='BUDGET_EXHAUSTED')),'OPERATION_REPLAN_REQUIRED')
            ops.r.episode_budget.remaining=lambda t:dict(wall_s=0,sim_s=100)
            self.assertEqual(recovery.classify(ops,dict(status='BUDGET_EXHAUSTED')),'WORK_BUDGET_EXHAUSTED')
            self.assertTrue(recovery.finish_check(ops,{})['allowed'])

    def test_hard_latch_and_unknown_controller_fault_never_recover(self):
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d)
            self.assertEqual(recovery.classify(ops,dict(error="RuntimeError('NATIVE_DT_INVALID')")),
                             'CONTROL_STATE_UNTRUSTED')
            self.assertEqual(recovery.classify(ops,dict(error="RuntimeError('APPROACH_IK')")),
                             'OPERATION_REPLAN_REQUIRED')
            ops.r.supervisor.latched={'reason':'FORCE_HARD_LIMIT'}
            self.assertEqual(recovery.classify(ops,{}),'HARD_STOP')
            with self.assertRaises(ValueError):recovery.acquisition_allowance(ops,{'sequence':9})
            self.assertEqual(ops.r.supervisor.latched,{'reason':'FORCE_HARD_LIMIT'})

    def test_initial_acquisition_count_is_review_not_termination(self):
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d);recovery.acquisition_allowance(ops,dict(sequence=9,reason='fresh different candidate'))
            self.assertEqual(ops.r.job['runtime_acquisition_budget'],7)
            self.assertEqual(recovery.remaining(ops),dict(wall_s=2000,sim_s=1000))
            self.assertFalse(recovery.finish_check(ops,{})['allowed'])

    def test_false_stop_assessment_cannot_replace_measured_progress(self):
        with tempfile.TemporaryDirectory() as d:
            ops=fixture(d)
            for i,op in enumerate(('evaluate_view','build_geometry','redeclare_actions','evaluate_station')):
                recovery.after(ops,dict(sequence=i,operation=op),
                    dict(plan=False,reason='native rejection'))
            assessment=dict(kind='NO_SAFE_CONTINUATION',considered=[dict(strategy=s,reason='checked and rejected',
                evidence_sequences=[i])for i,s in enumerate(('change_view','change_candidate','change_motion','relocate_station'))])
            self.assertTrue(recovery.finish_check(ops,dict(stop_assessment=assessment))['allowed'])
            ops.r.base_generation=2
            self.assertFalse(recovery.finish_check(ops,dict(stop_assessment=assessment))['allowed'])
            ops.r.base_generation=1
            assessment['considered'][-1]['evidence_sequences']=[99]
            self.assertFalse(recovery.finish_check(ops,dict(stop_assessment=assessment))['allowed'])
            self.assertTrue(recovery.finish_check(ops,dict(owner_requested_stop=True))['allowed'])

    def test_real_session_continues_after_timeout_and_rejected_premature_finish(self):
        from agentic_interaction_v81.session import serve
        from interaction_runtime.segments import ExecutionBudgetExhausted
        with tempfile.TemporaryDirectory() as d:
            original=fixture(d);r=original.r;r.stopped=False
            commands=[dict(operation='observe',sequence=0),dict(operation='finish',sequence=1),
                      dict(operation='inspect',sequence=2),dict(operation='finish',sequence=3,owner_requested_stop=True)]
            (r.output/'agent_commands').mkdir()
            for i,c in enumerate(commands):(r.output/'agent_commands'/f'{i:03d}.json').write_text(json.dumps(c))
            class Ops:
                def __init__(self,r):self.__dict__.update(original.__dict__);self.cache={}
                def decision_packet(self,*a):return {'unloaded':True}
                def observe(self):raise ExecutionBudgetExhausted('finite action ceiling')
                def release_park(self):return {'safe':True}
                def close_workers(self):pass
            owner=ModuleType('interaction_runtime.probing_v7');owner.OperationsV7=object
            observed=ModuleType('interaction_runtime.registered_observation_v71');observed.initialize=lambda *a:None
            with patch.dict('sys.modules',{'interaction_runtime.probing_v7':owner,
                    'interaction_runtime.registered_observation_v71':observed}), \
                 patch('agentic_interaction_v81.session.operations_class',return_value=Ops), \
                 patch('agentic_interaction_v81.session.status'), \
                 patch('agentic_interaction_v81.session.receive',side_effect=lambda p,i,res:(commands[i],{})):
                result=serve(r)
            self.assertEqual(result['status'],'SESSION_COMPLETE')
            self.assertEqual(len(result['trace']),4)
            self.assertEqual(result['trace'][0]['result']['status'],'OPERATION_BUDGET_EXHAUSTED')
            self.assertEqual(result['trace'][1]['result']['status'],'CONTINUATION_REQUIRED')


if __name__=='__main__':unittest.main()
