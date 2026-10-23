"""Dropped combinations, stale approvals and bounded search isolation."""
import hashlib,json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from agentic_interaction_v81.recovery_search import batch,family,check,scope,retire_plan,current_packet,complete,skip_dominated_checks


class RecoverySearchTests(unittest.TestCase):
    def test_second_batch_reaches_omitted_combinations_without_pose_edits(self):
        pool=[dict(section=s,insertion_m=d,roll=r,pitch=p,contact=dict(score=i))
            for i,(s,d,r,p) in enumerate((s,d,r,p)for s in [-.3,0,.3]
                for d in [-.01,0,.01]for r in [0,180]for p in [0,-15,15])]
        choose=lambda xs,n:sorted(xs,key=lambda x:x['contact']['score'],reverse=True)[:n]
        one=batch(pool,18,0,choose);two=batch(pool,18,1,choose)
        self.assertEqual(len(two),18)
        self.assertFalse({family(x)for x in one}&{family(x)for x in two})
        self.assertTrue(all(any(x is y for y in pool)for x in one+two))
        with self.assertRaises(ValueError):batch(pool,18,2,choose)

    def test_loaded_latched_revolute_and_missing_first_failure_rejected(self):
        for kind,state,latch in [('revolute','UNLOADED_OBSERVATION',None),
                ('prismatic','LOADED',None),('prismatic','UNLOADED_OBSERVATION','hard'),
                ('prismatic','UNLOADED_OBSERVATION',None)]:
            ops=SimpleNamespace(r=SimpleNamespace(job={'articulation_v7':{'joint_type':kind}},
                supervisor=SimpleNamespace(latched=latch)),g={'recovery_state':state,'agent_observations':[]})
            with self.assertRaises(ValueError):check(ops,dict(recovery_search_round=1))
        self.assertEqual(check(SimpleNamespace(),{}),0)

    def test_rejected_plan_image_is_retired_and_not_offered_as_current(self):
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);(p/'visual_planning').mkdir()
            for name in ['visual_plan.json','AGENT_GRASP_PLAN_INSPECTION.png',
                         'V8_PLAN_INSPECTION_PROVENANCE.json','visual_planning/candidate.json']:
                (p/name).write_bytes(name.encode())
            raw=p/'rgb.png';raw.write_bytes(b'original observation')
            ops=SimpleNamespace(r=SimpleNamespace(output=p),current_plan_ready=True,_v81_plan_ticket={})
            retire_plan(ops)
            self.assertFalse(ops.current_plan_ready);self.assertIsNone(ops._v81_plan_ticket)
            self.assertFalse((p/'visual_plan.json').exists())
            self.assertEqual(len(list((p/'plan_history/000').iterdir())),4)
            self.assertEqual(raw.read_bytes(),b'original observation')
            packet=current_packet(ops,dict(plan_board='old.png'))
            self.assertIsNone(packet['plan_board'])

    def test_failure_and_observation_identity_are_required(self):
        ops=SimpleNamespace(r=SimpleNamespace(job={'articulation_v7':{'joint_type':'prismatic'}},
            supervisor=SimpleNamespace(latched=None)),g={'recovery_state':'UNLOADED_OBSERVATION'})
        identity=dict(observations='new',station_generation=1,acquisitions=1)
        ops._v81_failed_search=dict(round=0,reason='BOUNDED_VISUAL_APPROACH_IK_SEARCH_EXHAUSTED',identity=identity)
        with patch('agentic_interaction_v81.boundary_operations.wrist_evidence',return_value=True), \
             patch('agentic_interaction_v81.recovery_search.observation_identity',return_value=identity):
            self.assertEqual(check(ops,dict(recovery_search_round=1,reason='inspect omitted measured families')),1)
            ops._v81_failed_search['identity']={}
            with self.assertRaises(ValueError):check(ops,dict(recovery_search_round=1,reason='fresh'))

    def test_original_repr_reason_is_not_confused_with_a_safety_fault(self):
        ops=SimpleNamespace()
        with patch('agentic_interaction_v81.recovery_search.observation_identity',return_value={}):
            result=dict(plan=False,reason="RuntimeError('BOUNDED_VISUAL_APPROACH_IK_SEARCH_EXHAUSTED')")
            complete(ops,0,result)
            self.assertEqual(ops._v81_failed_search['reason'],'BOUNDED_VISUAL_APPROACH_IK_SEARCH_EXHAUSTED')
            complete(ops,0,dict(plan=False,reason='SAFETY_STOP'))
            self.assertEqual(ops._v81_failed_search['reason'],'SAFETY_STOP')

    def test_scoped_shortlist_restores_frozen_owner_after_an_error(self):
        with tempfile.TemporaryDirectory()as d:
            owner=SimpleNamespace(diverse_shortlist=lambda pool,limit=18:pool[:limit])
            original=owner.diverse_shortlist
            ops=SimpleNamespace(r=SimpleNamespace(output=Path(d)))
            pool=[dict(section=i,insertion_m=0,roll=0,pitch=0)for i in range(40)]
            with patch.dict(sys.modules,{'native_g1_robust_contact_ranking':owner}), \
                 patch('agentic_interaction_v81.recovery_search.observation_identity',return_value={}):
                with self.assertRaises(RuntimeError):
                    with scope(ops,1):
                        self.assertEqual(owner.diverse_shortlist(pool),pool[18:36])
                        raise RuntimeError('native collision remains authoritative')
                self.assertIs(owner.diverse_shortlist,original)
                self.assertTrue((Path(d)/'RECOVERY_SEARCH_V81.json').exists())

    def test_utility_pruning_preserves_the_original_strict_winner(self):
        class Checker:
            def __call__(self,c):self.calls.append(c['id']);return dict(valid=c['feasible'])
            def __init__(self):self.calls=[]
        owner=SimpleNamespace(PrismaticCandidateCheckV75=Checker)
        candidates=[dict(id=i,shared_selection_utility=k,feasible=ok)for i,(k,ok)in
            enumerate([(10,False),(8,True),(7,True),(8,True),(9,False),(8.1,True),(6,True)])]
        expected=max((c for c in candidates if c['feasible']),key=lambda c:c['shared_selection_utility'])['id']
        original=Checker.__call__;checker=Checker();best=None
        with patch.dict(sys.modules,{'interaction_runtime.prismatic_candidate_v75':owner}):
            with skip_dominated_checks():
                for c in candidates:
                    result=checker(c)
                    if result['valid'] and (best is None or c['shared_selection_utility']>best['shared_selection_utility']):best=c
        self.assertEqual(best['id'],expected);self.assertEqual(checker.calls,[0,1,4,5])
        self.assertIs(Checker.__call__,original)


if __name__=='__main__':unittest.main()
