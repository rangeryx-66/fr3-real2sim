import ast,copy,json,tempfile,unittest,sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from interaction_runtime.prismatic_memory_v75_r3 import recovery_memory,stamp_catalog,resolve_candidate


def failure():
    return dict(physical_grasp_started=True,failed_contact_candidate=True,short_success=False,
        family=[0,0,0,0],acquisition=dict(acquired=True),segments=[dict(status='TRANSMISSION_FAILURE')],
        safe_exit=dict(safe=True,park=dict(reached=True,unloaded=True)))


class MemoryTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.p=Path(self.tmp.name);(self.p/'attempt_inputs').mkdir()
        self.write(0,1.)
    def tearDown(self):self.tmp.cleanup()
    def write(self,i,request):
        (self.p/'attempt_inputs'/('%02d.json'%i)).write_text(json.dumps(dict(requested_preload_n=request)))
    def test_different_legal_request_preserves_failure_but_reconsiders_family(self):
        a=[failure()];old=copy.deepcopy(a);blocked,m=recovery_memory(a,self.p,1.5)
        self.assertEqual(blocked,set());self.assertEqual(a,old)
        self.assertTrue(m['failed_conditions'][0]['reconsideration_eligible'])
        self.assertTrue(m['failed_conditions'][0]['prior_input_sha256'])
        self.assertEqual(recovery_memory(a,self.p,1.)[0],{(0,0,0,0)})
    def test_current_request_failed_before_stays_blocked(self):
        self.write(1,1.5)
        self.assertEqual(recovery_memory([failure(),failure()],self.p,1.5)[0],{(0,0,0,0)})
    def test_unsafe_unacquired_error_or_unknown_conditions_are_not_retry_permission(self):
        for field,value in [('safe_exit',dict(safe=False)),('acquisition',dict(acquired=False)),
            ('operation_status','SAFETY_STOP'),('error','collision'),('acquisition',None),('safe_exit',None),
            ('segments',[dict(status='SAFE_PARTIAL_PROGRESS')])]:
            a=failure();a[field]=value
            self.assertEqual(recovery_memory([a],self.p,1.5)[0],{(0,0,0,0)})
        (self.p/'attempt_inputs/00.json').unlink()
        self.assertEqual(recovery_memory([failure()],self.p,1.5)[0],{(0,0,0,0)})
    def test_no_physical_attempt_is_not_a_failed_acquisition(self):
        a=failure();a['physical_grasp_started']=False
        self.assertEqual(recovery_memory([a],self.p,1.)[0],set())
    def test_legal_menu_and_stale_request_catalog(self):
        for v in [True,2.,float('nan')]:
            with self.assertRaises(ValueError):recovery_memory([],self.p,v)
        _,m=recovery_memory([failure()],self.p,1.5)
        c=stamp_catalog(dict(catalog_sha256='old',observation_sha256='obs',candidates=[]),m)
        with self.assertRaisesRegex(ValueError,'CHANGED_RECOVERY'):
            resolve_candidate({},c,[],[failure()],self.p,1.)
        # Same catalogue semantics, automatic unchanged candidate coordinates.
        from interaction_runtime.recovery_candidates_v7 import observation_key
        c=stamp_catalog(dict(observation_sha256=observation_key([]),candidates=[dict(id='0',family=[0,0,0,0],eligible=True)]),m)
        req=dict(catalog_sha256=c['catalog_sha256'],candidate_id='0',reason='measured traction saturated; safe different legal request')
        self.assertEqual(resolve_candidate(req,c,[],[failure()],self.p,1.5),(0,0,0,0))
    def test_unchanged_scores_and_preferred_candidate_selection(self):
        from interaction_runtime.recovery_candidates_v7 import order
        a=dict(section=0,insertion_m=0,roll=0,pitch=0,score=.1)
        b=dict(section=.3,insertion_m=0,roll=0,pitch=0,score=.9)
        self.assertEqual(order([a,b],set(),(0,0,0,0)),[a])
        self.assertEqual(order([a,b],{(0,0,0,0)}),[b])


class ParkAndInterfaceTest(unittest.TestCase):
    def fake(self):
        from interaction_runtime.prismatic_unloaded_park_v75_r3 import plan_unloaded_exit
        g=dict(filtered=lambda:np.zeros(2),force_guard=SimpleNamespace(settled=lambda:True),
            robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(2)),arm=[0,1],qtarget=np.zeros(2),moving=lambda:np.eye(4))
        park=SimpleNamespace(PARK_DEFINITION=dict(unloaded_pad_n=.05),
            parking_targets=lambda g:[('rest',np.ones(2))],route_to_joint=lambda g,a,b:([a.tolist(),b.tolist()],'DIRECT'))
        recovery=SimpleNamespace(checked_route=lambda g,p:dict(valid=True,samples=len(p)))
        return plan_unloaded_exit,g,park,recovery
    def test_unloaded_joint_route_without_cartesian_ik_keeps_original_verifier_contract(self):
        fn,g,park,recovery=self.fake()
        with patch.dict(sys.modules,{'native_g1_safe_park':park,'native_g1_contact_recovery':recovery}):
            r=fn(g,lambda g:dict(valid=False,reason='NO_RETREAT_IK'))
        self.assertTrue(r['valid']);self.assertEqual(r['parking_region'],'rest')
        self.assertFalse(r['unloaded_joint_return']['contact_waiver'])
        self.assertEqual(r['parking_q'],[1.,1.]);self.assertEqual(len(r['clearance_path']),2)
    def test_loaded_unsettled_or_collision_route_remains_unavailable(self):
        for kind in ['loaded','unsettled','collision','no_route']:
            fn,g,park,recovery=self.fake()
            if kind=='loaded':g['filtered']=lambda:np.ones(2)
            if kind=='unsettled':g['force_guard'].settled=lambda:False
            if kind=='collision':recovery.checked_route=lambda g,p:dict(valid=False,reason='NATIVE_COLLISION')
            if kind=='no_route':park.route_to_joint=lambda *a:(None,'NO_ROUTE')
            with patch.dict(sys.modules,{'native_g1_safe_park':park,'native_g1_contact_recovery':recovery}):
                self.assertFalse(fn(g,lambda g:dict(valid=False))['valid'])
    def test_session_safety_control_flow_is_exact_original(self):
        root=Path(__file__).resolve().parents[1]
        def read(p,n):return next(x for x in ast.parse((root/p).read_text()).body if isinstance(x,ast.FunctionDef)and x.name==n)
        old=read('interaction_runtime/prismatic_session_v75.py','serve_v75')
        new=read('interaction_runtime/prismatic_session_v75_r3.py','serve_v75_r3');new.name=old.name
        for n in ast.walk(new):
            if isinstance(n,ast.Name)and n.id=='PrismaticOperationsV75R3':n.id='PrismaticEvidenceOperationsV75'
        self.assertEqual(ast.dump(old),ast.dump(new))
    def test_scoped_factory_restored_after_planning_error(self):
        from interaction_runtime.prismatic_operations_v75_r3 import PrismaticOperationsV75R3
        from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75
        sentinel=lambda g:None;module=SimpleNamespace(build_geometry=sentinel)
        with patch.dict(sys.modules,{'native_g1_contact_recovery':module}),patch.object(PrismaticEvidenceOperationsV75,'build_geometry',side_effect=ValueError('native check')):
            with self.assertRaisesRegex(ValueError,'native check'):
                PrismaticOperationsV75R3.build_geometry(PrismaticOperationsV75R3.__new__(PrismaticOperationsV75R3))
        self.assertIs(module.build_geometry,sentinel)

    def test_hard_latch_prevents_even_unloaded_exit_planning(self):
        from interaction_runtime.prismatic_operations_v75_r3 import PrismaticOperationsV75R3
        from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75
        ops=PrismaticOperationsV75R3.__new__(PrismaticOperationsV75R3)
        ops.r=SimpleNamespace(supervisor=SimpleNamespace(latched='ORIGINAL_HARD_STOP'))
        with patch.object(PrismaticEvidenceOperationsV75,'release_park')as called:
            self.assertFalse(ops.release_park()['safe']);called.assert_not_called()

    def test_exit_factory_restored_after_original_executor_error(self):
        from interaction_runtime.prismatic_operations_v75_r3 import PrismaticOperationsV75R3
        from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75
        ops=PrismaticOperationsV75R3.__new__(PrismaticOperationsV75R3)
        ops.r=SimpleNamespace(supervisor=SimpleNamespace(latched=None))
        sentinel=lambda g:None;module=SimpleNamespace(plan_exit=sentinel)
        with patch.dict(sys.modules,{'native_g1_safe_park':module}),patch.object(PrismaticEvidenceOperationsV75,'release_park',side_effect=ValueError('native stop')):
            with self.assertRaisesRegex(ValueError,'native stop'):ops.release_park()
        self.assertIs(module.plan_exit,sentinel)


if __name__=='__main__':unittest.main()
