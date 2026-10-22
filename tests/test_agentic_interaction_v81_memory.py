"""Same-episode measured direction support, without simulator/pose fixtures."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from agentic_interaction_v81.recovery_memory import conditioned, task_memory
from interaction_runtime.prismatic_memory_v75_r3 import recovery_memory


def action(target):
    return dict(target_q=target, coordinate_unit='m')


def fixture():
    attempts, records = [], []
    descriptions = [([0., 0., 0., 0.], 0., [(0.15, .05556, 'TRANSMISSION_FAILURE', None)]),
        ([0., 0., 0., 15.], .05551,
         [(.16, .16037, 'TARGET_REACHED', None), (.075, .15856, 'TRANSMISSION_FAILURE', None)]),
        ([0., 0., 180., 0.], .15826,
         [(.075, .07485, 'TARGET_REACHED', None),
          (.16, .08123, 'SAFE_PARTIAL_PROGRESS', 'PREDICTED_LOAD_MARGIN_REQUIRES_UNLOAD')])]
    for i, (family, start, movements) in enumerate(descriptions):
        attempt = dict(family=family, physical_grasp_started=True, failed_contact_candidate=True,
            first_record=i*1000, last_record=i*1000+999,
            acquisition=dict(acquired=True, joint_coordinate=start),
            safe_exit=dict(safe=True, park=dict(reached=True, unloaded=True)), segments=[])
        for j, (target, end, status, reason) in enumerate(movements):
            result = dict(status=status, reason=reason, joint_coordinate=end,
                requested_target_reached=status == 'TARGET_REACHED',
                physical_transmission_valid=status == 'TARGET_REACHED')
            attempt['segments'].append(result)
            records.append(dict(id=len(records)+1, role='FIT_MEASURED', completed=True,
                first_step=i*1000+100+j*100, declared_action=action(target), result=result))
        attempts.append(attempt)
    return attempts, records


class DirectionMemoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        (self.folder/'attempt_inputs').mkdir()
        self.attempts, self.records = fixture()
        for i in range(len(self.attempts)):
            (self.folder/'attempt_inputs'/('%02d.json'%i)).write_text(
                json.dumps(dict(requested_preload_n=1.0)))

    def evaluate(self, targets=(.125,), start=.08123, fresh=True):
        return conditioned(self.attempts, self.folder, 1.0, original=recovery_memory,
            actions=[action(t) for t in targets], start=start, fresh=fresh, recorded=self.records)

    def test_demonstrated_forward_family_survives_only_reverse_failure(self):
        blocked, memory = self.evaluate()
        self.assertNotIn((0., 0., 0., 15.), blocked)
        self.assertIn((0., 0., 0., 0.), blocked)
        self.assertIn((0., 0., 180., 0.), blocked)
        self.assertEqual(len(memory['task_conditioned_reconsideration']), 1)

    def test_known_failed_direction_cannot_be_repeated(self):
        blocked, _ = self.evaluate((.1,), start=.14)
        self.assertIn((0., 0., 0., 15.), blocked)
        self.assertNotIn((0., 0., 180., 0.), blocked)

    def test_entire_planned_chain_must_avoid_failed_direction(self):
        blocked, _ = self.evaluate((.125, .10))
        self.assertIn((0., 0., 0., 15.), blocked)

    def test_unobserved_interval_does_not_inherit_old_success(self):
        blocked, _ = self.evaluate((.20,))
        self.assertIn((0., 0., 0., 15.), blocked)
        blocked, _ = self.evaluate((.04,), start=.02)
        self.assertIn((0., 0., 0., 15.), blocked)

    def test_freshness_and_verified_exit_required(self):
        blocked, _ = self.evaluate(fresh=False)
        self.assertEqual(len(blocked), 3)
        self.attempts[1]['safe_exit']['park']['unloaded'] = False
        blocked, _ = self.evaluate()
        self.assertIn((0., 0., 0., 15.), blocked)

    def test_hard_fault_and_unknown_fault_remain_excluded(self):
        for status in ['SAFETY_STOP', 'BUDGET_EXHAUSTED']:
            with self.subTest(status=status):
                attempts, records = fixture()
                self.attempts, self.records = attempts, records
                self.records[2]['result']['status'] = status
                blocked, _ = self.evaluate()
                self.assertIn((0., 0., 0., 15.), blocked)

    def test_partial_progress_is_not_demonstrated_target_support(self):
        self.records[1]['result']['requested_target_reached'] = False
        blocked, _ = self.evaluate()
        self.assertIn((0., 0., 0., 15.), blocked)

    def test_missing_recorded_context_and_wrong_units_are_conservative(self):
        self.records = []
        blocked, _ = self.evaluate()
        self.assertEqual(len(blocked), 3)
        self.attempts, self.records = fixture()
        self.records[1]['declared_action']['coordinate_unit'] = 'rad'
        blocked, _ = self.evaluate()
        self.assertIn((0., 0., 0., 15.), blocked)

    def test_memory_is_immutable_and_fingerprint_tracks_task(self):
        before = deepcopy((self.attempts, self.records))
        _, a = self.evaluate()
        _, b = self.evaluate((.13,))
        self.assertEqual(before, (self.attempts, self.records))
        self.assertNotEqual(a['conditions_sha256'], b['conditions_sha256'])

    def test_scoped_owners_restore_even_on_rejected_plan(self):
        import interaction_runtime
        old = lambda *args: None
        operations = SimpleNamespace(recovery_memory=old)
        owner = SimpleNamespace(recovery_memory=old)
        modules = {'interaction_runtime.prismatic_operations_v75_r3': operations,
                   'interaction_runtime.prismatic_memory_v75_r3': owner}
        ops = SimpleNamespace(r=SimpleNamespace(actual=lambda:.08, record=SimpleNamespace(segments=[])),
                              g=dict(agent_observations=[]))
        with patch.dict('sys.modules', modules), \
             patch.object(interaction_runtime, 'prismatic_operations_v75_r3', operations, create=True), \
             patch.object(interaction_runtime, 'prismatic_memory_v75_r3', owner):
            with self.assertRaisesRegex(ValueError, 'rejected'):
                with task_memory(ops, dict(planned_segments=[action(.12)])):
                    self.assertIs(operations.recovery_memory, owner.recovery_memory)
                    self.assertIsNot(owner.recovery_memory, old)
                    raise ValueError('rejected')
        self.assertIs(operations.recovery_memory, old)
        self.assertIs(owner.recovery_memory, old)


if __name__ == '__main__':
    unittest.main()
