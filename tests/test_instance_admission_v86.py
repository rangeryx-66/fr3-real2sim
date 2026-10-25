"""No Isaac imports/physics: closed incident, resource conflict and fresh admission."""
import fcntl,json,os,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
from agentic_interaction_v81 import instance_admission as m
from agentic_interaction_v81.terminal_safety import require_clearance,preserve_unverified_process_hold

def dump(p,d):
    p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d))

class InstanceAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        p=self.p=Path(self.tmp.name).resolve();self.lock=p/'owned.lock';self.lock.touch()
        self.old=p/'old_fault';self.old.mkdir();dump(self.old/'job.json',{})
        dump(self.old/'PROCESS.json',dict(pid=99999999,job_sha256=m.sha(self.old/'job.json'),
            command=[sys.executable,'-u','-m','agentic_interaction_v81.entry','--job',str(self.old/'job.json')]))
        dump(self.old/'PROCESS_COMPLETION.json',dict(actor_exited=True,returncode=0))
        self.audit=p/'fault_audit.json';dump(self.audit,dict(episode_id=self.old.name,
            status='HARD_STOP_UNPARKED',simulation_closed=True,safe_park_verified=False))
        self.hold=Path(str(self.lock)+'.safety_hold.json')
        dump(self.hold,dict(episode_id=self.old.name,status='HARD_STOP_UNPARKED',evidence=str(self.audit)))
        self.original=self.hold.read_bytes()
        domain=p/'domain.json';dump(domain,dict(execution='isaac_sim_only',actor_lock=str(self.lock),
            python=sys.executable,output_parent=str(p/'new')))
        self.env=dict(actor_lock=str(self.lock),python=sys.executable,output_root=str(p/'new/episodes'),
            simulation_isolation=dict(schema=m.SCHEMA,destroyed_instances=[dict(hold_path=str(self.hold),
            hold_sha256=m.sha(self.hold),episode=str(self.old),audit_sha256=m.sha(self.audit),
            process_sha256=m.sha(self.old/'PROCESS.json'),completion_sha256=m.sha(self.old/'PROCESS_COMPLETION.json'),
            owned_pids=[99999999],evidence_files=[dict(file='job.json',sha256=m.sha(self.old/'job.json'))])]))
        self.domain_patch=patch.object(m,'DOMAIN',domain);self.domain_patch.start();self.addCleanup(self.domain_patch.stop)
        self.owner_patch=patch.object(m,'no_live_owners');self.owners=self.owner_patch.start();self.addCleanup(self.owner_patch.stop)

    def test_destroyed_incident_preserved_new_uuid_admitted_without_old_resume(self):
        with m.reserve(self.env,'fresh') as i:
            self.assertNotEqual(i['uuid'],'fresh');self.assertEqual(i['history'][0]['status'],'NOT_PASS_SAFE_PARK_UNCONFIRMED')
            self.assertFalse(Path(i['safety_registry']).exists())
            self.assertEqual(self.hold.read_bytes(),self.original)
            with self.assertRaises(RuntimeError):require_clearance(self.hold)  # legacy remains blocked
            self.assertNotEqual(Path(i['safety_registry']),self.hold)
            preserve_unverified_process_hold(i['output'],0,i['safety_registry'])
            self.assertTrue(Path(i['safety_registry']).exists())
        self.assertEqual(self.hold.read_bytes(),self.original)

    def test_shared_resource_conflict_blocks_before_new_instance(self):
        with self.lock.open('a') as f:
            fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                with m.reserve(self.env,'fresh'):self.fail('resource conflict admitted')
        self.assertFalse((self.p/'new').exists())

    def test_unknown_global_and_hardware_stops_block_new_uuid(self):
        for suffix in ['.global_stop.json','.hardware_stop.json']:
            p=Path(str(self.lock)+suffix);p.write_text('{}')
            with self.assertRaisesRegex(RuntimeError,'GLOBAL_SAFETY'):
                with m.reserve(self.env,'fresh'):self.fail('global stop bypassed')
            p.unlink()  # isolated temporary fixture only
        self.hold.write_text('{}')
        with self.assertRaisesRegex(RuntimeError,'HOLD_CHANGED'):
            with m.reserve(self.env,'fresh'):self.fail('changed hold admitted')

    def test_live_or_reused_old_owner_is_not_destroyed(self):
        with patch.object(m,'identity',return_value=dict(pid=99999999,start_tick='new')):
            with self.assertRaisesRegex(RuntimeError,'NOT_PROVEN_TERMINATED'):
                with m.reserve(self.env,'fresh'):self.fail('live owner admitted')

    def test_no_hardware_or_relocated_global_lock_profile(self):
        for change in [dict(robot_ip='192.0.2.1'),dict(actor_lock=str(self.p/'alternate.lock'))]:
            env=dict(self.env,**change)
            with self.assertRaises(ValueError):
                with m.reserve(env,'fresh'):self.fail('hardware or changed resource domain admitted')

    def test_missing_history_or_resume_old_episode_denied(self):
        with self.assertRaisesRegex(ValueError,'OLD_INSTANCE_ID'):
            with m.reserve(self.env,self.old.name):self.fail('old episode resumed')
        self.hold.unlink()  # private temporary fixture only; not a reset API
        with self.assertRaisesRegex(RuntimeError,'EVIDENCE_MISSING'):
            with m.reserve(self.env,'fresh'):self.fail('missing incident accepted')

    def test_active_controller_scan_blocks_and_does_not_create_directory(self):
        self.owners.side_effect=RuntimeError('LIVE_SIMULATION_CONTROLLER')
        with self.assertRaisesRegex(RuntimeError,'LIVE_SIMULATION'):
            with m.reserve(self.env,'fresh'):self.fail('active controller ignored')
        self.assertFalse((self.p/'new').exists())

    def test_uuid_and_lease_cannot_be_reused_or_direct_entry_substituted(self):
        with m.reserve(self.env,'fresh') as i:
            with self.assertRaises(BlockingIOError):
                with m.reserve(self.env,'other'):self.fail('concurrent owner admitted')
            job=dict(instance_uuid=i['uuid'],instance_directory=i['directory'],episode_id='fresh',output=i['output'],instance_environment=self.env)
            with self.assertRaisesRegex(RuntimeError,'OWNERSHIP_UNVERIFIED'):m.check_child(job)
        self.assertEqual(self.hold.read_bytes(),self.original)

    def fake_runtime(self,FT=True):
        folder=self.p/'fake_instance';folder.mkdir();output=self.p/'fake_episode';output.mkdir()
        world=type('World',(),{'__module__':'isaacsim.core.api.world'})
        w=world();w.play=Mock();w.pause=Mock()
        boundary=S(check=Mock())
        # asdict() is patched only for this no-physics native-admission harness.
        r=S(job=dict(instance_directory=str(folder),instance_uuid='uid',episode_id='fake_episode'),
            output=output,world=w,supervisor=S(latched=None,force=S(settled=lambda:True)),
            grasp_reference=None,g={},hold=Mock(),rows=[dict(wrist_ft=dict(valid=FT))],tick=84,
            robot=S(get_joint_positions=lambda:[0]*9),arm=[0,1,2,3,4,5,6],fingers=[7,8],base=[0]*4,
            model=S(poses=lambda *a,**kw:{}),collision=S(check=lambda *a:(True,'')),
            moving=lambda:None,actual=lambda:0.,filtered=lambda:__import__('numpy').zeros(2),
            record=S(flush=Mock(),queue=S(join=Mock())),close=Mock())
        return r,boundary

    def test_online_lease_check_does_not_open_legacy_history_resource_path(self):
        with m.reserve(self.env,'fresh') as i:
            job=dict(instance_uuid=i['uuid'],instance_directory=i['directory'],
                episode_id='fresh',output=i['output'],instance_environment=self.env)
            # Simulate the child having the already verified live parent.
            launcher=json.loads((Path(i['directory'])/'INSTANCE.json').read_text())['launcher']
            launcher=launcher or dict(pid=123,start_tick='fixture')
            manifest=Path(i['directory'])/'INSTANCE.json';value=json.loads(manifest.read_text())
            value['launcher']=launcher;dump(manifest,value)
            original=Path.open
            def no_history(path,*args,**kwargs):
                if path==self.lock:raise AssertionError('online history IO')
                return original(path,*args,**kwargs)
            with patch.object(m,'identity',return_value=launcher),patch.object(Path,'open',no_history),patch.object(m,'global_and_history') as history:
                m.check_child(job,verify_history=False)
                history.assert_not_called()

    def test_native_unloaded_admission_required_before_interaction(self):
        r,b=self.fake_runtime()
        with patch.object(m,'check_child'),patch('agentic_interaction_v81.policy_checks.settle_station',return_value=b),patch('dataclasses.asdict',return_value={}):
            certificate=m.native_unloaded_check(r)
        self.assertTrue(certificate['passed']);r.hold.assert_called_once_with(.35)
        b.check.assert_called_once();r.close.assert_not_called()
        self.assertTrue(json.loads((r.output/'UNLOADED_ADMISSION.json').read_text())['passed'])

    def test_native_selftest_failure_never_marks_pass_or_enters_ABC(self):
        r,b=self.fake_runtime(FT=False)
        with patch.object(m,'check_child'),patch('agentic_interaction_v81.policy_checks.settle_station',return_value=b),patch('agentic_interaction_v81.terminal_safety.persist') as persist:
            with self.assertRaisesRegex(RuntimeError,'WRIST_FT'):m.native_unloaded_check(r)
        self.assertFalse(json.loads((r.output/'UNLOADED_ADMISSION.json').read_text())['passed'])
        persist.assert_called_once();r.close.assert_called_once()

if __name__=='__main__':unittest.main()

