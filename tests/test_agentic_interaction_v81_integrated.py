import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from agentic_interaction_v81.preload_extension import request,for_job,instrument
from agentic_interaction_v81.model_packets import snapshot
from agentic_interaction_v81.validation_queue import enqueue,admit,model_state,requests


class PhysicsLifecycleTests(unittest.TestCase):
    def test_paused_observation_resumes_before_native_exit_step(self):
        from agentic_interaction_v81.physics_lifecycle import resume_for_exit
        from interaction_runtime.policy import SafetyStop
        class World:
            playing=False
            def play(self):self.playing=True
            def step(self):return 1/240 if self.playing else 0.
        world=World();r=SimpleNamespace(world=world,supervisor=SimpleNamespace(latched=None))
        self.assertEqual(world.step(),0.)  # reproduces native dt=0 precondition
        resume_for_exit(r)
        self.assertEqual(world.step(),1/240)
        world.playing=False;r.supervisor.latched={'reason':'real collision'}
        with self.assertRaises(SafetyStop):resume_for_exit(r)
        self.assertFalse(world.playing)


class PreloadTests(unittest.TestCase):
    def test_real_prismatic_mro_memory_planning_and_acquisition_admit_same_menu(self):
        try:
            import trimesh, fcl, pinocchio
        except ModuleNotFoundError as error:
            self.skipTest('Full frozen planner import requires server dependency environment: '+str(error))
        from agentic_interaction_v81.preload_extension import menu_scope
        from native_g1_radial_families_v751 import radial_operations_class
        from interaction_runtime.probing_v7 import OperationsV7
        from interaction_runtime.prismatic_preload_v71 import PreloadOperations
        from interaction_runtime.prismatic_v73 import PrismaticOperationsV73
        import interaction_runtime.prismatic_memory_v75_r3 as memory
        import interaction_runtime.prismatic_operations_v75_r3 as conditioned
        from interaction_runtime.articulation_v7 import Articulation
        actions=[dict(role=role,coordinate_unit='m',target_q=q,profile='constant',hold=True,
            **({'fit_intervals_s':[[.6,10.]]} if role.startswith('FIT') else {}))
            for role,q in [('FIT_A',.03),('FIT_B',.01),('VALIDATION_C',.02)]]
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'visual_plan.json').write_text('{}')
            cls=radial_operations_class();ops=cls.__new__(cls)
            ops.g={'recovery_state':'UNLOADED_OBSERVATION','contact_attempts':[],
                'agent_observations':[]}
            ops.r=SimpleNamespace(job={'v81_integrated_repair':True,
                'v81_evidence_approved_preloads':[2.,2.5]},output=p,
                closure_policy={'preload_n':1.,'finger_effort_limit_n':10.},
                supervisor=SimpleNamespace(latched=None,force=SimpleNamespace(settled=lambda:True)),
                grasp_reference=None,filtered=lambda:[0.,0.],
                policy=SimpleNamespace(data={'force':{'unloaded_pad_n':.05}}),actual=lambda:0.)
            ops.articulation=lambda:Articulation('prismatic',(0.,-1.,0.),(0.,0.,0.),(0.,.2))
            ops.feedback=lambda:{'contact_feedback':{'current':True}}
            for value in (2.,2.5,3.):
                ops.r.job['v81_preload_study']=value==3.
                command={'native_g1_preload_request_n':value,'planned_segments':actions}
                with menu_scope(ops):
                    self.assertEqual(memory.recovery_memory([],p,value)[1]['requested_preload_n'],value)
                    self.assertIn(value,conditioned.LEGAL_REQUESTS)
                    with patch.object(OperationsV7,'request_plan',return_value={'plan':False,'reason':'geometry_stub'})as downstream:
                        result=ops.request_plan(command)
                        downstream.assert_called_once()
                        self.assertEqual(result['reason'],'geometry_stub')
                    ops.current_plan_ready=True
                    with patch.object(PreloadOperations,'attempt',return_value={'physical_grasp_started':True})as execution:
                        result=PrismaticOperationsV73.attempt(ops,{'native_g1_preload_request_n':value})
                        execution.assert_called_once()
                        self.assertTrue(result['physical_grasp_started'])
                        recorded=json.loads((p/'attempt_inputs/00.json').read_text())
                        self.assertEqual(recorded['requested_preload_n'],value)
                self.assertEqual(memory.LEGAL_REQUESTS,(.5,.75,1.,1.5))
                self.assertEqual(conditioned.LEGAL_REQUESTS,(.5,.75,1.,1.5))
            ops.r.job['v81_preload_study']=False
            with menu_scope(ops),self.assertRaises(ValueError):
                PrismaticOperationsV73.attempt(ops,{'native_g1_preload_request_n':3.})

    def test_no_silent_menu_clipping_or_unverified_agent_activation(self):
        for value in (2.,2.5,3.):
            self.assertEqual(for_job({'v81_preload_study':True},value),value)
            with self.assertRaises(ValueError):for_job({},value)
        self.assertEqual(for_job({'v81_evidence_approved_preloads':[2.]},2.),2.)
        for value in (True,float('nan'),4.,30.):
            with self.assertRaises(ValueError):request(value,study=True)

    def test_passive_telemetry_does_not_change_closure_commands(self):
        class Closure:
            def __init__(self,*a):pass
            def update(self,*a):return dict(mode='effort',state='FORCE_HOLD',closing_effort_n=3.,T=np.eye(4))
        geometry=SimpleNamespace(width_derivative=lambda a:-.07,angle=lambda w:.5)
        r=SimpleNamespace(Closure=Closure,record=SimpleNamespace(coordinate_fields=lambda row:{}),
            robot=SimpleNamespace(get_joint_positions=lambda:np.array([.4])),
            controller=SimpleNamespace(master=0,geometry=geometry,caps=np.array([2.])),
            c={'gripper_kp_nm_rad':10.},jaw_width=.014)
        expected=Closure().update();instrument(r);actual=r.Closure(None,None).update()
        for key in ('mode','state','closing_effort_n'):self.assertEqual(actual[key],expected[key])
        self.assertTrue(np.array_equal(actual['T'],expected['T']))
        row=r.record.coordinate_fields({'q':[.4]})
        self.assertAlmostEqual(row['mapping_unclipped_master_rad'],.421)
        self.assertEqual(r.controller.caps[0],2.)


class PacketTests(unittest.TestCase):
    def test_snapshot_excludes_future_validation_and_copies_no_raw_arrays(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'telemetry').mkdir();raw=p/'telemetry/000.npz'
            np.savez_compressed(raw,step=np.arange(20),segment=np.r_[np.ones(10),2*np.ones(10)])
            first=dict(id=1,role='FIT_A',first_step=0,last_step=9,completed=True)
            validation=dict(id=2,role='VALIDATION_FUTURE',first_step=10,last_step=19,completed=True)
            index=dict(metadata={},chunks=[dict(file='000.npz',first_step=0,last_step=19,
                sha256=hashlib.sha256(raw.read_bytes()).hexdigest())],segments=[first,validation])
            (p/'telemetry/index.json').write_text(json.dumps(index));(p/'identification_input.json').write_text('{}')
            out=p/'packets/001';manifest=snapshot(p,out,[first],'episode')
            snapshot_bytes=(out/'telemetry/index.json').read_bytes()
            result=json.loads(snapshot_bytes)
            self.assertEqual([s['id']for s in result['segments']],[1])
            self.assertEqual((out/'telemetry'/result['chunks'][0]['file']).resolve(),raw.resolve())
            self.assertEqual(list(out.rglob('*.npz')),[])
            index['segments'].append({'id':3});(p/'telemetry/index.json').write_text(json.dumps(index))
            self.assertEqual((out/'telemetry/index.json').read_bytes(),snapshot_bytes)
            self.assertEqual(manifest['raw_array_copies'],0)


class ValidationTests(unittest.TestCase):
    def ops(self,p):
        self.holds=[]
        r=SimpleNamespace(output=p,job={'episode_id':'test'},dt=1/240,tick=0,
            episode_budget=SimpleNamespace(remaining=lambda t:dict(wall_s=100.,sim_s=100.)),
            supervisor=SimpleNamespace(latched=None),
            record=SimpleNamespace(segments=[dict(id=1,role='FIT_A',first_step=0,result=dict(
                status='TARGET_REACHED',safely_stopped=True,physical_transmission_valid=True))]),
            rows=[dict(aperture_m=.014,relative_translation_slip_m=0.)],
            closure_policy={'max_slip_m':.001},actual=lambda:.1)
        r.hold=lambda t:self.holds.append(t)
        return SimpleNamespace(r=r,check_successor=lambda a:dict(valid=True))

    def publication(self,p,fit=1,ready=True):
        folder=p/'model';folder.mkdir(exist_ok=True);packet=p/'immutable';packet.mkdir(exist_ok=True)
        model=packet/'estimate.json';model.write_text(json.dumps(dict(status='updated')))
        digest=hashlib.sha256(model.read_bytes()).hexdigest();r=packet/'ready.json'
        r.write_text(json.dumps(dict(qualified_bidirectional=ready,episode_id='test',model_sha256=digest)))
        pub=dict(model=str(model),readiness=str(r),model_sha256=digest,latest_FIT_id=fit)
        (folder/'MODEL_PUBLICATION.json').write_text(json.dumps(pub))
        return dict(state='READY',publication=pub,readiness=json.loads(r.read_text()))

    def action(self):return dict(role='VALIDATION_ORIGINAL',target_q=.2,coordinate_unit='rad',profile='constant',hold=True)

    def test_statuses_separate_computing_insufficient_failed_and_stale(self):
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);ops=self.ops(p)
            self.assertEqual(model_state(ops)['state'],'MODEL_COMPUTING')
            self.publication(p,ready=False)
            self.assertEqual(model_state(ops)['state'],'DATA_INSUFFICIENT')
            self.publication(p,fit=0)
            self.assertEqual(model_state(ops)['state'],'MODEL_COMPUTING')
            (p/'model/WORKER_LIFECYCLE.json').write_text(json.dumps(dict(state='WORKER_FAILED',latest_FIT_id=1)))
            self.assertEqual(model_state(ops)['state'],'WORKER_FAILED')
            self.publication(p)
            self.assertEqual(model_state(ops)['state'],'READY')
            (p/'immutable/estimate.json').write_text('{}')
            self.assertEqual(model_state(ops)['state'],'WORKER_FAILED')

    def test_finite_physics_hold_then_original_validation_sealed_automatically(self):
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);ops=self.ops(p);a=self.action();enqueue(ops,dict(segments=[a],reason='approved'))
            ready=self.publication(p)
            with patch('agentic_interaction_v81.validation_queue.model_state',side_effect=[{'state':'MODEL_COMPUTING'},ready]):
                result=admit(ops,a)
            self.assertTrue(result['ready']);self.assertGreater(sum(self.holds),0)
            self.assertLessEqual(sum(self.holds),1.)
            self.assertEqual(requests(ops)['requests'][0]['action'],a)
            self.assertEqual(requests(ops)['requests'][0]['status'],'SEALED_READY')

    def test_unstable_contact_does_not_wait_or_lose_pending_request(self):
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);ops=self.ops(p);a=self.action();enqueue(ops,dict(segments=[a]))
            ops.r.record.segments[0]['result']['physical_transmission_valid']=False
            with patch('agentic_interaction_v81.validation_queue.model_state',return_value={'state':'MODEL_COMPUTING'}):
                self.assertFalse(admit(ops,a)['ready'])
            self.assertEqual(self.holds,[]);self.assertEqual(requests(ops)['requests'][0]['action'],a)

    def test_measured_worker_latency_allows_finite_stable_handover(self):
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);ops=self.ops(p);a=self.action();enqueue(ops,dict(segments=[a]))
            ready=self.publication(p)
            pub=ready['publication'];pub['worker_wall_s']=4.
            (p/'model/MODEL_PUBLICATION.json').write_text(json.dumps(pub))
            with patch('agentic_interaction_v81.validation_queue.model_state',
                       side_effect=[{'state':'MODEL_COMPUTING'}]*30+[ready]):
                self.assertTrue(admit(ops,a)['ready'])
            self.assertGreater(sum(self.holds),1.)
            self.assertLessEqual(sum(self.holds),3.)
            self.assertEqual(requests(ops)['requests'][0]['action'],a)

    def test_contact_degradation_interrupts_numerical_wait(self):
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);ops=self.ops(p);a=self.action();enqueue(ops,dict(segments=[a]))
            def drift(t):
                self.holds.append(t);ops.r.rows[-1]['aperture_m']+=.0006
            ops.r.hold=drift
            with patch('agentic_interaction_v81.validation_queue.model_state',return_value={'state':'MODEL_COMPUTING'}):
                result=admit(ops,a)
            self.assertFalse(result['ready']);self.assertEqual(result['reason'],'CONTACT_UNSTABLE')
            self.assertAlmostEqual(sum(self.holds),.05)
            self.assertEqual(requests(ops)['requests'][0]['action'],a)




class CurrentPoolUnitsTests(unittest.TestCase):
    def test_prismatic_replan_preserves_cross_start_whole_chain_certificate(self):
        import sys
        from agentic_interaction_v81.candidate_pool import select
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);candidate=dict(path=[{'state':.05},{'state':.15}],
                shared_selection_utility=1.,section_fraction=0.,insertion_m=0.,roll_deg=0.,pitch_deg=0.)
            ops=SimpleNamespace(r=SimpleNamespace(output=p,job={'articulation_v7':{'joint_type':'prismatic'}},
                policy=None,c=None,solver=None),g={},_v81_current_candidates=dict(identity={'same':True},
                horizon=.15,candidates=[candidate]),current_plan_ready=False)
            class Checker:
                def __init__(self,*args):pass
                def __call__(self,c):return dict(valid=True,v75_whole_chain=dict(valid=True,
                    declared_actions=[dict(target_m=.12),dict(target_m=.02)]))
            actions=[dict(role='FIT_PULL',target_q=.12),dict(role='VALIDATION_REVERSE',target_q=.02)]
            with patch('agentic_interaction_v81.candidate_pool.identity',return_value={'same':True}),\
                 patch('agentic_interaction_v81.boundary_operations.wrist_evidence',return_value=True),\
                 patch('agentic_interaction_v81.preload_extension.select'),\
                 patch.dict(sys.modules,{'interaction_runtime.prismatic_candidate_v75':SimpleNamespace(PrismaticCandidateCheckV75=Checker)}):
                result=select(ops,dict(planned_segments=actions,reason='same current observed pool'))
            self.assertTrue(result['plan'])
            self.assertEqual(json.loads((p/'visual_plan.json').read_text())['coordinate_unit'],'m')


if __name__=='__main__':unittest.main()
