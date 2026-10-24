import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

from interaction_runtime.segments import EpisodeBudget
from agentic_interaction_v81 import orchestration_extension as extension
from agentic_interaction_v81.debug_budget import window
from agentic_interaction_v81.integration import apply, REVOLUTE_VALIDATION_PROFILE
from agentic_interaction_v81.transport import validate


class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def unix(self):return 1000.+self.t


def evidence():
    return dict(unloaded=True,park_reached=True,robot_stationary=True,object_stationary=True,
        released=True,no_grasp_reference=True,force_settled=True,finite_state=True,
        sensors_trusted=True,safety_latch=None,sim_s=10.,closed=False)


def fixture(folder, renewal=600):
    clock=Clock()
    original=EpisodeBudget(dict(episode_wall_budget_s=1000,episode_sim_budget_s=2000,
        safe_exit_wall_reserve_s=100,safe_exit_sim_reserve_s=120),now=clock.now)
    budget=extension.WallCredit(original)
    binding=dict(episode_id='new_episode',source_commit='a'*40,actor_pid=123,session_nonce='b'*32)
    authorization=dict(human_authorization_ref='human-turn:explicit-continue',
        max_total_extra_wall_s=1800,parked_decision_renewal_s=renewal)
    key='c'*64
    manager=extension.ActorLedger(folder,binding,authorization,budget,key,unix_now=clock.unix)
    def request(n=1,delta=600):
        return dict(operation=extension.OPERATION,protocol_version=1,sequence=0,
            binding=copy.deepcopy(binding),request_nonce='%032x'%n,
            human_authorization_ref=authorization['human_authorization_ref'],delta_wall_s=delta,
            reason='Explicit human request to keep working from verified unloaded park')
    return clock,original,budget,manager,request,key


class OrchestrationTests(unittest.TestCase):
    def test_authorized_clock_credit_continues_consuming_after_original_clamp(self):
        with tempfile.TemporaryDirectory() as folder:
            clock,original,budget,manager,request,key=fixture(folder)
            clock.t=800;manager.accept(request(),evidence(),clock.unix()+100)
            clock.t=950
            self.assertEqual(original.remaining(10)['wall_s'],0)
            self.assertEqual(budget.remaining(10)['wall_s'],550)
            clock.t=1490;self.assertEqual(budget.remaining(10)['wall_s'],10)
            clock.t=1501;self.assertEqual(budget.remaining(10)['wall_s'],0)
            self.assertEqual(original.started,0)
            self.assertEqual(original.wall,1000)
            self.assertEqual(budget.remaining(10)['sim_s'],original.remaining(10)['sim_s'])
            self.assertEqual((budget.reserve_wall,budget.reserve_sim),(100,120))
            self.assertEqual(extension.accepted_total(folder,manager.binding,manager.authorization,key),600)

    def test_all_actor_identity_components_and_human_reference_are_enforced(self):
        with tempfile.TemporaryDirectory() as folder:
            clock,_,_,manager,request,_=fixture(folder)
            for key in manager.binding:
                cmd=request();cmd['binding'][key]=456 if key=='actor_pid' else 'wrong'
                with self.assertRaisesRegex(ValueError,'BINDING'):manager.accept(cmd,evidence(),1100)
            cmd=request();cmd['human_authorization_ref']='agent-invented-reference'
            with self.assertRaisesRegex(ValueError,'AUTHORIZATION_REFERENCE'):manager.accept(cmd,evidence(),1100)
            cmd=request();cmd['reason']=''
            with self.assertRaises(ValueError):manager.accept(cmd,evidence(),1100)
            self.assertEqual(manager.records,0)

    def test_nonce_replay_single_limit_and_cumulative_cap(self):
        with tempfile.TemporaryDirectory() as folder:
            clock,_,_,manager,request,_=fixture(folder)
            for delta in (0,1801,float('inf'),True,1.5):
                with self.assertRaises(ValueError):manager.accept(request(delta=delta),evidence(),1100)
            manager.accept(request(delta=1200),evidence(),1100)
            with self.assertRaisesRegex(ValueError,'NONCE_REPLAY'):manager.accept(request(delta=1200),evidence(),1100)
            with self.assertRaisesRegex(ValueError,'CUMULATIVE'):manager.accept(request(2,601),evidence(),1100)
            manager.accept(request(2,600),evidence(),1100)
            self.assertEqual(manager.budget.total_extra_wall_s,1800)
            self.assertEqual(len(list((Path(folder)/extension.LEDGER).glob('*.json'))),2)

    def test_loaded_hard_untrusted_nonfinite_and_closed_refuse_without_credit(self):
        with tempfile.TemporaryDirectory() as folder:
            _,_,budget,manager,request,_=fixture(folder)
            for field in ('unloaded','park_reached','robot_stationary','object_stationary',
                          'released','no_grasp_reference','force_settled','finite_state','sensors_trusted'):
                state=evidence();state[field]=False
                with self.assertRaises(ValueError):manager.accept(request(),state,1100)
            for updates in ({'safety_latch':'HARD_STOP'},{'closed':True},{'object_q':float('nan')}):
                state=dict(evidence(),**updates)
                with self.assertRaises(ValueError):manager.accept(request(),state,1100)
            manager.closed=True
            with self.assertRaisesRegex(ValueError,'CLOSED'):manager.accept(request(),evidence(),1100)
            self.assertEqual(budget.total_extra_wall_s,0)

    def test_expired_manual_boundary_refuses_and_standing_renewal_requires_opt_in(self):
        with tempfile.TemporaryDirectory() as folder:
            clock,_,budget,manager,request,_=fixture(folder,renewal=0)
            with self.assertRaisesRegex(ValueError,'EXPIRED'):manager.accept(request(),evidence(),clock.unix())
            with self.assertRaisesRegex(ValueError,'NOT_EXPLICITLY'):manager.renew_wait(evidence(),clock.unix())
            self.assertEqual(budget.total_extra_wall_s,0)

    def test_standing_renewal_at_original_floor_uses_unclamped_elapsed_and_is_finite(self):
        with tempfile.TemporaryDirectory() as folder:
            clock,original,budget,manager,request,_=fixture(folder)
            clock.t=901
            result=manager.renew_wait(evidence(),clock.unix()-1)
            self.assertEqual(result['renewed_wait_s'],600)
            self.assertEqual(budget.remaining(10)['wall_s'],599)
            self.assertEqual(original.started,0)
            manager.closed=True
            with self.assertRaisesRegex(ValueError,'CLOSED'):manager.renew_wait(evidence(),clock.unix())

    def test_signed_ledger_tampering_or_wrong_launcher_binding_cannot_grant_time(self):
        with tempfile.TemporaryDirectory() as folder:
            _,_,_,manager,request,key=fixture(folder)
            manager.accept(request(),evidence(),1100)
            with self.assertRaises(ValueError):extension.accepted_total(folder,dict(manager.binding,actor_pid=124),manager.authorization,key)
            path=Path(folder)/extension.LEDGER/'0001.json';saved=path.read_text()
            changed=json.loads(saved);changed['total_extra_wall_s']=1800;path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError,'AUTHENTICATION'):extension.accepted_total(folder,manager.binding,manager.authorization,key)
            path.write_text(saved)
            with self.assertRaises(FileExistsError):extension.immutable_json(path,{})
            with self.assertRaisesRegex(ValueError,'FRESH_ACTOR_LEDGER'):extension.ActorLedger(folder,manager.binding,manager.authorization,manager.budget,key)

    def test_profile_defaults_and_opt_in_schema(self):
        job=dict(articulation_v7=dict(joint_type='revolute'))
        apply(dict(phase='engineering',integration_profile=REVOLUTE_VALIDATION_PROFILE),job)
        self.assertNotIn(extension.FLAG,job)
        old=object();runtime=SimpleNamespace(job=job,episode_budget=old)
        extension.install(runtime);self.assertIs(runtime.episode_budget,old)
        authorization=dict(human_authorization_ref='human-turn:continue',max_total_extra_wall_s=3600,parked_decision_renewal_s=600)
        spec=dict(phase='engineering',integration_profile=extension.PROFILE,orchestration_authorization=authorization)
        new=dict(articulation_v7=dict(joint_type='revolute'));apply(spec,new)
        self.assertTrue(new[extension.FLAG]);self.assertEqual(new['v83_continuous_video_hz'],5)
        self.assertEqual(new['v81_task_goal_native'],np.pi/2)
        for invalid in ({},dict(authorization,max_total_extra_wall_s=float('inf')),dict(authorization,parked_decision_renewal_s=1801)):
            with self.assertRaises(ValueError):apply(dict(spec,orchestration_authorization=invalid),{})
        with self.assertRaises(ValueError):apply(dict(spec,integration_profile=REVOLUTE_VALIDATION_PROFILE),{})

    def test_transport_and_default_actor_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            _,_,_,manager,request,_=fixture(folder)
            self.assertEqual(validate(request(),0),request())
            bad=request();bad['safe_exit_wall_reserve_s']=0
            with self.assertRaises(ValueError):validate(bad,0)
            self.assertFalse(extension.handle(SimpleNamespace(r=SimpleNamespace()),request(),1100)['extended'])

    def test_measured_gate_rechecks_native_park_and_rejects_hidden_nan_without_motion(self):
        from agentic_interaction_v81.station import Boundary
        with tempfile.TemporaryDirectory() as folder:
            _,_,_,manager,request,_=fixture(folder)
            b=Boundary('new_episode',0,0,1.,(0,0,0,0),(0,)*7,(0,)*6,.3,'rad',
                True,True,True,True,True,None,'a'*64)
            row=dict(joint_coordinate_velocity=0.,aperture_m=.084,T_tcp=np.eye(4),
                wrist_ft=dict(valid=True,raw_wrench_sensor=np.zeros(6),gravity_compensated_wrench_world=np.zeros(6)))
            r=SimpleNamespace(output=Path(folder),stopped=False,rows=[row],tick=240,dt=1/240,
                qtarget=np.zeros(15),grasp_reference=None,orchestration_extension=manager,
                robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(15),get_joint_velocities=lambda:np.zeros(15)),
                supervisor=SimpleNamespace(latched=None,force=SimpleNamespace(settled=lambda:True)),
                station_boundary=lambda record=False:b,filtered=lambda:np.zeros(2))
            ops=SimpleNamespace(r=r,g=dict(recovery_state='UNLOADED_OBSERVATION',at_safe_park=True,last_exit_plan={},reference=None))
            ops.g['last_exit_plan']={'parking_q':[0.]*7}
            verified=dict(reached=True,unloaded=True)
            with patch.dict(sys.modules,{'native_g1_safe_park':SimpleNamespace(verify_park=lambda *a:verified)}):
                self.assertTrue(extension.measured_evidence(ops)['finite_state'])
                row['wrist_ft']['raw_wrench_sensor'][0]=float('nan')
                self.assertFalse(extension.handle(ops,request(),1100)['extended'])
                row['wrist_ft']['raw_wrench_sensor'][0]=0.
                verified['reached']=False
                self.assertFalse(extension.handle(ops,request(),1100)['extended'])
                verified['reached']=True
                (r.output/'PROCESS_COMPLETION.json').write_text('{}')
                self.assertFalse(extension.handle(ops,request(),1100)['extended'])
            self.assertEqual(manager.records,0)

    def test_launcher_uses_same_signed_credit_and_default_keeps_old_timeout(self):
        with tempfile.TemporaryDirectory() as folder:
            clock,_,_,manager,request,key=fixture(folder)
            class Actor:
                pid=123;args=['offline-test-actor']
                def poll(self):return 0 if clock.t>=1300 else None
                def wait(self,timeout):
                    clock.t+=timeout
                    if clock.t>=800 and not manager.records:manager.accept(request(),evidence(),clock.unix()+100)
                    result=self.poll()
                    if result is None:raise subprocess.TimeoutExpired(self.args,timeout)
                    return result
            job=dict(episode_wall_budget_s=1000,episode_id=manager.binding['episode_id'],
                v83_orchestration_source_commit=manager.binding['source_commit'],
                v83_orchestration_session_nonce=manager.binding['session_nonce'],
                orchestration_authorization=manager.authorization,v83_orchestration_extension=True)
            self.assertEqual(extension.wait_actor(Actor(),job,folder,key,now=clock.now,poll_s=10),0)
            self.assertGreater(clock.t,1180)  # original launcher deadline passed
            class Old:
                def wait(self,timeout):self.timeout=timeout;return 5
            old=Old();self.assertEqual(extension.wait_actor(old,{'episode_wall_budget_s':1000},folder),5)
            self.assertEqual(old.timeout,1180)

    def test_launcher_denies_forged_credit_but_retains_original_owned_timeout(self):
        with tempfile.TemporaryDirectory() as folder:
            clock,_,_,manager,request,key=fixture(folder)
            manager.accept(request(),evidence(),1100)
            path=Path(folder)/extension.LEDGER/'0001.json';record=json.loads(path.read_text())
            record['hmac_sha256']='0'*64;path.write_text(json.dumps(record))
            class Actor:
                pid=123;args=['offline-test-actor']
                def poll(self):return None
                def wait(self,timeout):clock.t+=timeout;raise subprocess.TimeoutExpired(self.args,timeout)
            job=dict(episode_wall_budget_s=1000,episode_id=manager.binding['episode_id'],
                v83_orchestration_source_commit=manager.binding['source_commit'],
                v83_orchestration_session_nonce=manager.binding['session_nonce'],
                orchestration_authorization=manager.authorization,v83_orchestration_extension=True)
            with self.assertRaises(subprocess.TimeoutExpired):extension.wait_actor(Actor(),job,folder,key,now=clock.now,poll_s=100)
            self.assertEqual(clock.t,1180)
            self.assertTrue((Path(folder)/'ORCHESTRATION_WATCHDOG_FAULT.json').exists())

    def test_actual_session_wait_loop_renews_parked600_and_default_still_times_out(self):
        from agentic_interaction_v81 import session,task_recovery
        for enabled in (False,True):
            with self.subTest(enabled=enabled),tempfile.TemporaryDirectory() as folder:
                clock,_,budget,manager,_,_=fixture(folder)
                # Enough original task time: reproduce B02's decision600 issue.
                budget.original.wall=7200
                output=Path(folder)
                r=SimpleNamespace(job=dict(articulation_v7=dict(joint_type='revolute'),v81_active_recovery=True,agent_wait_timeout_s=600),
                    g=dict(recovery_state='UNLOADED_OBSERVATION'),supervisor=SimpleNamespace(latched=None),
                    episode_budget=budget if enabled else budget.original,output=output,
                    tick=0,dt=1/240,base_generation=0,base=np.zeros(4),stopped=False)
                if enabled:r.orchestration_extension=manager
                ops=SimpleNamespace(r=r,g=r.g,decision_packet=lambda *a:{},release_park=lambda:{'safe':True},close_workers=lambda:None)
                def sleep(_):
                    clock.t+=10
                    if clock.t>=650:
                        path=output/'agent_commands/000.json';path.write_text('{}')
                fake_owner=SimpleNamespace(OperationsV7=object)
                fake_init=SimpleNamespace(initialize=lambda *a:None)
                with patch.dict(sys.modules,{'interaction_runtime.probing_v7':fake_owner,'interaction_runtime.registered_observation_v71':fake_init}), \
                    patch.object(session,'operations_class',lambda _:lambda _:ops), \
                    patch.object(session,'status',lambda *a,**k:None), \
                    patch.object(session,'receive',lambda *a:({'operation':'finish'},{})), \
                    patch.object(session,'complete',lambda *a:None), \
                    patch.object(task_recovery,'enabled',lambda _:False), \
                    patch.object(task_recovery,'before',lambda *a:None), \
                    patch.object(task_recovery,'after',lambda ops,cmd,result:result), \
                    patch.object(extension,'measured_evidence',lambda _:evidence()), \
                    patch.object(session.time,'perf_counter',clock.now), \
                    patch.object(session.time,'time',clock.unix),patch.object(session.time,'sleep',sleep):
                    result=session.serve(r)
                self.assertEqual(result['status'],'SESSION_COMPLETE' if enabled else 'ORCHESTRATION_TIMEOUT')
                if enabled:
                    waiting=json.loads((output/'WAITING_FOR_AGENT.json').read_text())
                    self.assertEqual(waiting['timeout_s'],1200)
                    self.assertEqual(waiting['deadline_unix_s'],2200)
                    self.assertEqual(manager.budget.total_extra_wall_s,600)
                    packet=json.loads((output/'DECISION_PACKET.json').read_text())
                    self.assertEqual(packet['orchestration_extension']['effective_wall_s'],7800)
                    self.assertEqual(packet['remaining_resources']['wall_s'],7100)
                    self.assertTrue(manager.closed)
                else:self.assertEqual(clock.t,600)


if __name__=='__main__':unittest.main()
