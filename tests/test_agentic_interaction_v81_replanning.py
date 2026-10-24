import copy
import math
import sys
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

    def test_recovery_reserve_does_not_consume_exit_reserve(self):
        ops=self.ops();cost=dict(observed_max_operation_wall_s=dict(reobserve_and_replan=80,
            prepare_grasp=100,attempt_grasp_and_probe=400))
        with patch('agentic_interaction_v81.recovery_batch.measured_costs',return_value=cost):
            result=admission(ops);self.assertTrue(result['admitted'])
            ops.r.episode_budget.remaining=lambda t:dict(wall_s=970,sim_s=1000)
            self.assertFalse(admission(ops)['admitted'])


if __name__=='__main__':unittest.main()
