import math
import tempfile
import unittest
import copy
from pathlib import Path
from types import SimpleNamespace
from agentic_interaction_v81.integration import apply, progress, ACTIVE_PROFILE, PRISMATIC_ACTIVE_PROFILE, REVOLUTE_VALIDATION_PROFILE
from agentic_interaction_v81 import task_recovery
from test_agentic_interaction_v81_task_recovery import fixture


class PrismaticRecoveryProfileTests(unittest.TestCase):
    def test_revolute_validation_profile_adds_only_recording_to_original_r02(self):
        original=dict(articulation_v7=dict(joint_type='revolute'))
        validation=copy.deepcopy(original)
        apply(dict(phase='engineering',integration_profile=ACTIVE_PROFILE),original)
        apply(dict(phase='engineering',integration_profile=REVOLUTE_VALIDATION_PROFILE),validation)
        self.assertEqual(validation.pop('v83_continuous_video_hz'),5)
        self.assertEqual(validation,original)
        self.assertEqual(validation['v81_task_goal_native'],math.pi/2)
        self.assertEqual(validation['v81_minimum_reverse_native'],math.radians(10))
        with self.assertRaisesRegex(ValueError,'REQUIRES_REVOLUTE'):
            apply(dict(phase='engineering',integration_profile=REVOLUTE_VALIDATION_PROFILE),dict(articulation_v7=dict(joint_type='prismatic')))

    def test_video_override_changes_only_recording_mode(self):
        from agentic_interaction_v81.video_evidence import configure
        with tempfile.TemporaryDirectory() as directory:
            original = dict(force=dict(emergency=10), phases=dict(loaded=.012),
                            modes=dict(FAST_DEV=dict(continuous_video=False, video_hz=15)))
            runtime = SimpleNamespace(job=dict(v83_continuous_video_hz=5),
                mode='FAST_DEV', policy=SimpleNamespace(data=copy.deepcopy(original), sha256='a'*64),
                render=SimpleNamespace(c=None), output=Path(directory))
            configure(runtime)
            self.assertEqual(runtime.policy.data['force'], original['force'])
            self.assertEqual(runtime.policy.data['phases'], original['phases'])
            self.assertTrue(runtime.render.c['continuous_video'])
            self.assertEqual(runtime.render.c['video_hz'], 5)
            configure(runtime)
            self.assertTrue((Path(directory)/'V83_VIDEO_EVIDENCE.json').exists())

    def test_joint_specific_units_and_preserved_r02_profile(self):
        for profile, joint, goal, reverse in [(ACTIVE_PROFILE, 'revolute', math.pi/2, math.radians(10)),
                (PRISMATIC_ACTIVE_PROFILE, 'prismatic', .18, .04)]:
            job = dict(articulation_v7=dict(joint_type=joint))
            apply(dict(phase='engineering', integration_profile=profile), job)
            self.assertEqual(job['v81_task_goal_native'], goal)
            self.assertEqual(job['v81_minimum_reverse_native'], reverse)
            self.assertTrue(job['v81_active_recovery'])
        for profile, joint in [(ACTIVE_PROFILE, 'prismatic'), (PRISMATIC_ACTIVE_PROFILE, 'revolute')]:
            with self.assertRaises(ValueError):
                apply(dict(phase='engineering', integration_profile=profile),
                      dict(articulation_v7=dict(joint_type=joint)))

    def test_current_prismatic_state_survives_failure_and_requires_post_goal_reverse(self):
        with tempfile.TemporaryDirectory() as directory:
            ops = fixture(directory)
            ops.r.job['articulation_v7']['joint_type'] = 'prismatic'
            apply(dict(phase='engineering', integration_profile=PRISMATIC_ACTIVE_PROFILE), ops.r.job)
            ops.r.actual = lambda: .16
            ops.r.v81_task_progress = None
            ops.r.record.count = 40
            progress(ops.r, dict(joint_coordinate=.16))
            result = task_recovery.after(ops, dict(sequence=1, operation='attempt_grasp_and_probe'),
                dict(segments=[dict(status='TRANSMISSION_FAILURE')], safe_exit=dict(safe=True)))
            self.assertEqual(result['task_recovery']['status'], 'CONTACT_RECOVERY_REQUIRED')
            self.assertEqual(task_recovery.state(ops)['goal_native'], .18)
            self.assertEqual(ops.r.v81_task_progress['current_measured_native'], .16)
            self.assertEqual(ops.r.v81_task_progress['unit'], 'm')
            self.assertFalse(task_recovery.finish_check(ops, {})['allowed'])
            ops.r.record.count = 100
            progress(ops.r, dict(joint_coordinate=.1801))
            reverse = dict(status='TARGET_REACHED', gain_native=-.05,
                           physical_transmission_valid=True, requested_target_reached=True)
            ops.r.record.segments = [dict(first_step=50, role='VALIDATION_EARLY', result=reverse)]
            self.assertFalse(task_recovery.finish_check(ops, {})['allowed'])
            ops.r.record.segments.append(dict(first_step=110, role='VALIDATION_AFTER_GOAL', result=reverse))
            self.assertTrue(task_recovery.finish_check(ops, {})['task_complete'])


if __name__ == '__main__':
    unittest.main()
