"""No physics imports or hardware commands: injected terminal-state regressions."""
import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch
from agentic_interaction_v81.terminal_safety import classify,persist,process_outcome,require_clearance,preserve_unverified_process_hold
from agentic_interaction_v81.runtime import runtime_class
from interaction_runtime.policy import SafetyStop


def good_exit():
    return dict(safe=True,park=dict(reached=True,unloaded=True,
        measured_collision_check=dict(valid=True)))


class TerminalSafetyTests(unittest.TestCase):
    def test_park_flag_or_rc_zero_never_proves_park(self):
        self.assertFalse(classify(None,{},True)['safe_park_verified'])
        with tempfile.TemporaryDirectory() as d:
            v=process_outcome(d,0)
            self.assertFalse(v['physical_safe_park_verified']);self.assertTrue(v['requires_intervention'])

    def test_latch_dominates_even_old_valid_park(self):
        v=classify({'reason':'OVERLOAD'},good_exit(),True)
        self.assertEqual(v['status'],'HARD_STOP_UNPARKED')
        self.assertFalse(v['safe_park_verified']);self.assertFalse(v['controller_stop_verified'])
        self.assertTrue(classify(None,good_exit())['safe_park_verified'])

    def runtime(self,d):
        world=S(pause=Mock(),is_playing=lambda:False,play=Mock())
        return S(output=Path(d),job=dict(episode_id='fault',v84_abc=True,
            terminal_safety_registry=str(Path(d)/'hold.json')),world=world,g={},
            supervisor=S(latched={'reason':'OVERLOAD'}),arm=[0,1],
            rows=[dict(t=2.,joint_coordinate=.0438,joint_velocities=[.2,-.1])])

    def test_pause_not_faked_as_controller_ack_or_zero_velocity(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.runtime(d);latch=r.supervisor.latched.copy()
            v=persist(r,dict(safe_exit=dict(safe=False)),pause=True)
            r.world.pause.assert_called_once();r.world.play.assert_not_called()
            self.assertEqual(r.supervisor.latched,latch)
            self.assertEqual(v['last_arm_velocity_abs_max'],.2)
            self.assertTrue(v['simulation_paused_verified'])
            self.assertFalse(v['controller_stop_verified']);self.assertFalse(v['safe_park_verified'])
            with self.assertRaises(RuntimeError):require_clearance(Path(d)/'hold.json')
            v=process_outcome(d,0);self.assertTrue(v['requires_intervention'])

    def test_malformed_or_old_registry_fails_closed(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'hold';p.write_text('broken')
            with self.assertRaises(RuntimeError):require_clearance(p)
            require_clearance(Path(d)/'absent')

    def test_step_fault_pauses_without_retry_or_driving(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.runtime(d)
            class Parent:
                def step(self):raise SafetyStop('OVERLOAD',{'x':1})
            obj=object.__new__(runtime_class(Parent));obj.__dict__.update(r.__dict__)
            with self.assertRaisesRegex(SafetyStop,'OVERLOAD'):obj.step()
            obj.world.pause.assert_called_once();obj.world.play.assert_not_called()
            self.assertTrue((Path(d)/'TERMINAL_SAFETY.json').exists())

    def test_export_failure_preserves_original_hard_fault(self):
        with tempfile.TemporaryDirectory() as d:
            class Parent:
                def step(self):raise SafetyStop('OVERLOAD',{})
            o=object.__new__(runtime_class(Parent));o.__dict__.update(self.runtime(d).__dict__)
            with patch('agentic_interaction_v81.terminal_safety.persist',side_effect=OSError('disk')):
                with self.assertRaisesRegex(SafetyStop,'OVERLOAD') as caught:o.step()
            self.assertTrue(any('export failed' in n for n in caught.exception.__notes__))

    def test_missing_terminal_record_rc_zero_creates_hold(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);hold=p/'hold.json'
            v=preserve_unverified_process_hold(p,0,hold)
            self.assertTrue(v['requires_intervention']);self.assertTrue(hold.exists())
            with self.assertRaises(RuntimeError):require_clearance(hold)

    def test_corrupt_or_stale_terminal_evidence_cannot_release_gate(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);record=p/'TERMINAL_SAFETY.json';record.write_text('broken')
            self.assertTrue(process_outcome(p,0)['requires_intervention'])
            record.write_text('[1]');self.assertTrue(process_outcome(p,0)['requires_intervention'])
            record.write_text(json.dumps(dict(episode_id='old',status='SAFE_PARK_VERIFIED',safe_park_verified=True,requires_intervention=False)))
            self.assertTrue(process_outcome(p,0)['requires_intervention'])

    def test_verified_exit_does_not_create_false_hold(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);hold=p/'hold.json'
            (p/'TERMINAL_SAFETY.json').write_text(json.dumps(dict(episode_id=p.name,status='SAFE_PARK_VERIFIED',safe_park_verified=True,requires_intervention=False,latch_present=False)))
            v=preserve_unverified_process_hold(p,0,hold)
            self.assertTrue(v['physical_safe_park_verified']);self.assertFalse(hold.exists())

    def test_launcher_refuses_before_resource_preparation_or_actor_spawn(self):
        from agentic_interaction_v81.launch import launch
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);profile=p/'env.json';lock=p/'actor.lock'
            profile.write_text(json.dumps(dict(actor_lock=str(lock))))
            Path(str(lock)+'.safety_hold.json').write_text('{}')
            with patch('agentic_interaction_v81.launch.baseline_audit'), \
                 patch('agentic_interaction_v81.launch.prepare') as prepare, \
                 patch('agentic_interaction_v81.launch.subprocess.Popen') as spawn:
                with self.assertRaisesRegex(RuntimeError,'UNRESOLVED_TERMINAL'):
                    launch(p/'unused-spec.json',profile)
                prepare.assert_not_called();spawn.assert_not_called()

    def test_latest_hard_exit_beats_initial_park_in_trace(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.runtime(d)
            result=dict(trace=[dict(result=dict(safe_exit=good_exit())),
                dict(result=dict(safe_exit=dict(safe=False,reason='HARD_STOP')))])
            v=persist(r,result)
            self.assertFalse(v['safe_park_verified']);self.assertEqual(v['unload']['reason'],'HARD_STOP')


if __name__=='__main__':unittest.main()
