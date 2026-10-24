import copy
import math
import sys
import json
import tempfile
from pathlib import Path
import unittest
from types import SimpleNamespace,ModuleType
from unittest.mock import patch
from agentic_interaction_v81.replanning import coverage,successor,alternatives,candidate_gate,admission
from agentic_interaction_v81.transport import validate,ProtocolError


def action(q,role='FIT_REQUEST'):
    return dict(target_q=q,coordinate_unit='rad',role=role,profile='constant',hold=True)


class ReplanningTests(unittest.TestCase):
    def ops(self):
        r=SimpleNamespace(job={},actual=lambda:.2,base_generation=0,tick=0,dt=1/240,
            supervisor=SimpleNamespace(latched=None),
            record=SimpleNamespace(segments=[dict(result=dict(status='TARGET_REACHED',
                safely_stopped=True,physical_transmission_valid=True))]),
            episode_budget=SimpleNamespace(remaining=lambda t:dict(wall_s=1000,sim_s=1000)))
        return SimpleNamespace(r=r,articulation=lambda:SimpleNamespace(validate_target=lambda a:a['target_q']))

    def test_81_point_1_is_not_90_in_either_direction(self):
        p=dict(path=[dict(state=math.radians(x)) for x in (10,40,81.1)])
        c=coverage(p,[math.pi/2],math.pi/2)
        self.assertFalse(c['full_requested_actions_feasible'])
        self.assertFalse(c['full_requested_horizon_feasible'])
        self.assertTrue(coverage(p,[math.radians(20)],math.pi/2)['full_requested_actions_feasible'])
        self.assertFalse(coverage(p,[0.])['full_requested_actions_feasible'])
        self.assertFalse(coverage({},[.1])['full_requested_actions_feasible'])

    def test_prismatic_whole_chain_certificate_preserves_cross_start_reversal(self):
        p=dict(coordinate_unit='m',path=[dict(state=.02),dict(state=.01)],
            execution_feasibility=dict(v75_whole_chain=dict(valid=True,
                declared_actions=[dict(target_m=.01),dict(target_m=.03),dict(target_m=.025)])))
        self.assertTrue(coverage(p,[.01,.03,.025])['full_requested_actions_feasible'])
        self.assertFalse(coverage(p,[.04])['full_requested_actions_feasible'])
        p['execution_feasibility']['v75_whole_chain']['valid']=False
        self.assertFalse(coverage(p,[.03])['full_requested_actions_feasible'])

    def test_native_rejection_then_different_checked_plan_continues(self):
        ops=self.ops();a=action(.8);a['path_alternatives']=[action(.5,'FIT_SHORTER')];calls=[]
        def native(a):
            calls.append(a['target_q'])
            return dict(valid=a['target_q']==.5,reason='DANGEROUS_GEOMETRY_COLLISION:link3:original31')
        result=successor(ops,a,native)
        self.assertTrue(result['valid']);self.assertEqual(calls,[.8,.5])
        self.assertEqual(a['target_q'],.5)
        self.assertFalse(result['full_original_target_feasible'])
        self.assertIn('original31',result['rejected_request']['reason'])

    def test_all_rejected_stays_recoverable_not_global_infeasibility(self):
        ops=self.ops();a=action(.8);a['path_alternatives']=[action(.5,'FIT_SHORTER')]
        result=successor(ops,a,lambda a:dict(valid=False,reason='native collision'))
        self.assertEqual(result['status'],'PLANNING_REJECTED')
        self.assertFalse(result['episode_termination_required']);self.assertEqual(a['target_q'],.8)

    def test_latch_never_searches_or_clears(self):
        ops=self.ops();ops.r.supervisor.latched={'reason':'overload'};calls=[]
        a=action(.8);a['path_alternatives']=[action(.5,'FIT_SHORTER')]
        result=successor(ops,a,lambda a:calls.append(a)or dict(valid=False,reason='collision'))
        self.assertFalse(result['valid']);self.assertEqual(len(calls),1)
        self.assertEqual(ops.r.supervisor.latched,{'reason':'overload'})

    def test_unstable_contact_cannot_take_alternative(self):
        ops=self.ops();ops.r.record.segments[-1]['result']['status']='TRANSMISSION_FAILURE'
        a=action(.8);a['path_alternatives']=[action(.5,'FIT_SHORTER')];calls=[]
        result=successor(ops,a,lambda a:calls.append(a)or dict(valid=False,reason='collision'))
        self.assertFalse(result['valid']);self.assertEqual(len(calls),1)

    def test_validation_never_substituted_and_nested_alternatives_rejected(self):
        a=action(.8,'VALIDATION_PRIMARY');a['path_alternatives']=[action(.5,'FIT_SHORTER')]
        with self.assertRaises(ValueError):alternatives(a)
        a=action(.8);b=action(.5,'FIT_SHORTER');b['path_alternatives']=[];a['path_alternatives']=[b]
        with self.assertRaises(ValueError):alternatives(a)
        with self.assertRaises(ProtocolError):validate(dict(operation='attempt_grasp_and_probe',protocol_version=1,sequence=0,segments=[a]),0)

    def test_injected_rejection_is_one_shot_not_reported_as_native_collision(self):
        ops=self.ops();ops.r.job=dict(v81_phase='engineering',v81_path_rejection_fixture=True)
        a=action(.8);a['path_alternatives']=[action(.5,'FIT_SHORTER')]
        result=successor(ops,a,lambda a:dict(valid=True,target_q=a['target_q']))
        self.assertTrue(result['valid']);self.assertTrue(result['requested_path_rejected'])
        self.assertTrue(result['rejected_request']['native_rejection_preserved']['engineering_injected'])
        self.assertTrue(successor(ops,action(.6),lambda a:dict(valid=True))['valid'])

    def test_candidate_fallback_keeps_native_gate(self):
        calls=[]
        class Native:
            def __init__(self):self.audit=[]
            def __call__(self,c):calls.append(c);return dict(valid=c.get('collision_free',False))
        module=ModuleType('native_g1_execution_candidates');module.CandidateCheck=Native
        partial=dict(path=[dict(state=0),dict(state=.6)],section_fraction=0,insertion_m=0,roll_deg=0,pitch_deg=0)
        with patch.dict(sys.modules,{'native_g1_execution_candidates':module}),candidate_gate([action(.8)],.8):
            checker=Native();self.assertFalse(checker(copy.deepcopy(partial))['valid'])
            full=dict(partial,path=[dict(state=0),dict(state=.8)])
            self.assertFalse(checker(full)['valid'])
            self.assertTrue(checker(dict(full,collision_free=True))['valid'])
        self.assertEqual(len(calls),2)

    def test_prismatic_delegates_to_frozen_whole_chain_not_first_arc_only(self):
        class Native:
            def __init__(self):self.audit=[];self.g={'job':{'articulation_v7':{'joint_type':'prismatic'}}}
            def __call__(self,c):return dict(valid=True)
        module=ModuleType('native_g1_execution_candidates');module.CandidateCheck=Native
        partial=dict(coordinate_unit='m',path=[dict(state=.02),dict(state=.01)],
            section_fraction=0,insertion_m=0,roll_deg=0,pitch_deg=0)
        a=dict(action(.03),coordinate_unit='m')
        with patch.dict(sys.modules,{'native_g1_execution_candidates':module}),candidate_gate([a],.01) as audit:
            self.assertTrue(Native()(partial)['valid'])
            self.assertTrue(audit[0]['deferred_to_original_prismatic_whole_chain'])
        # The eventual approval still fails without the full native certificate.
        self.assertFalse(coverage(partial,[.03])['full_requested_actions_feasible'])

    def test_recovery_reserve_does_not_consume_exit_reserve(self):
        ops=self.ops();cost=dict(observed_max_operation_wall_s=dict(reobserve_and_replan=80,
            prepare_grasp=100,attempt_grasp_and_probe=400))
        with patch('agentic_interaction_v81.recovery_batch.measured_costs',return_value=cost):
            result=admission(ops);self.assertTrue(result['admitted'])
            ops.r.episode_budget.remaining=lambda t:dict(wall_s=970,sim_s=1000)
            self.assertFalse(admission(ops)['admitted'])

    def test_partial_target_never_reaches_physical_owner(self):
        from agentic_interaction_v81.operations import operations_class
        calls=[]
        class Owner:
            def attempt(self,command):calls.append(command);return {}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'visual_plan.json').write_text(json.dumps(dict(
                path=[dict(state=0),dict(state=math.radians(81.1))])))
            ops=operations_class(Owner)();ops.current_plan_ready=True;ops.r=SimpleNamespace(output=p)
            result=ops.attempt(dict(segments=[action(math.pi/2)]))
            self.assertEqual(result['status'],'PLANNING_REJECTED');self.assertEqual(calls,[])

    def test_session_keeps_accepting_commands_after_rejected_path(self):
        from agentic_interaction_v81.session import serve
        events=[]
        class Ops:
            def __init__(self,r):self.cache={}
            def decision_packet(self,*args):return {'unloaded':True}
            def attempt(self,c):return dict(status='PLANNING_REJECTED',recoverable=True,
                episode_termination_required=False,safe_exit={'safe':True})
            def release_park(self):events.append('park');return {'safe':True}
            def close_workers(self):events.append('closed')
        observed=ModuleType('interaction_runtime.registered_observation_v71');observed.initialize=lambda *a:None
        owner=ModuleType('interaction_runtime.probing_v7');owner.OperationsV7=object
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'agent_commands').mkdir()
            commands=[dict(operation='attempt_grasp_and_probe',segments=[action(.5)]),
                dict(operation='inspect'),dict(operation='finish')]
            for i,c in enumerate(commands):(p/'agent_commands'/('%03d.json'%i)).write_text(json.dumps(c))
            r=SimpleNamespace(job={'articulation_v7':{'joint_type':'revolute'},'maximum_agent_operations':4},
                g={'recovery_state':'UNLOADED_OBSERVATION'},output=p,base_generation=0,
                base=SimpleNamespace(tolist=lambda:[0,0,0,0]),supervisor=SimpleNamespace(latched=None))
            # The real session dispatch/termination logic is tested; the
            # physical owner is a fixture, not a claimed Isaac contact test.
            Ops.articulation=lambda self:SimpleNamespace(validate_target=lambda a:True)
            with patch.dict(sys.modules,{'interaction_runtime.registered_observation_v71':observed,
                    'interaction_runtime.probing_v7':owner}), \
                 patch('agentic_interaction_v81.session.operations_class',return_value=Ops), \
                 patch('agentic_interaction_v81.session.window',return_value=(20,{})), \
                 patch('agentic_interaction_v81.session.status'), \
                 patch('agentic_interaction_v81.session.receive',side_effect=lambda path,i,res:(commands[i],{})), \
                 patch('agentic_interaction_v81.session.complete'):
                result=serve(r)
            self.assertEqual(result['status'],'SESSION_COMPLETE')
            self.assertEqual(len(result['trace']),3)
            self.assertEqual(result['trace'][0]['result']['status'],'PLANNING_REJECTED')
            self.assertEqual(events,['park','closed'])


if __name__=='__main__':unittest.main()
