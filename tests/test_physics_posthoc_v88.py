import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from agentic_interaction_v88.study_posthoc import closed_episodes, native_moving_mass, parameter_errors


class PosthocReferenceTest(unittest.TestCase):
    def test_mass_excludes_unrelated_frame_and_other_drawers(self):
        truth = {'bodies': [{'body': 'drawer', 'mass_kg': 2.}, {'body': 'handle', 'mass_kg': .5}]}
        tensors = {'link_names': ['frame', 'drawer', 'other_drawer', 'handle'],
                   'get_masses': {'available': True, 'values': [[100., 2., 10., .5]]}}
        total, bodies = native_moving_mass(truth, tensors)
        self.assertEqual(total, 2.5)
        self.assertEqual(set(bodies), {'drawer', 'handle'})

    def test_native_mass_treatment_mismatch_is_not_silently_reported(self):
        with self.assertRaisesRegex(ValueError, 'MASS_MISMATCH'):
            native_moving_mass({'bodies': [{'body': 'drawer', 'mass_kg': 3.}]},
                {'link_names': ['drawer'], 'get_masses': {'available': True, 'values': [[1.]]}})

    def test_missing_model_stays_unknown(self):
        self.assertFalse(parameter_errors({}, [2., 1., .4])['available'])

    def test_online_owner_blocks_before_truth_access(self):
        with tempfile.TemporaryDirectory() as folder:
            calls = []
            def fake_read(path):
                calls.append(Path(path).name)
                if Path(path).name == 'FORMAL_GATE_SEALED.json':
                    return {'no_further_physical_actions_in_this_study': True}
                return {'actor_exited': False}
            with patch('agentic_interaction_v88.study_posthoc.read', side_effect=fake_read):
                with self.assertRaisesRegex(ValueError, 'ACTOR_NOT_CLOSED'):
                    closed_episodes(Path(folder), {'M0': 'ongoing'})
            self.assertEqual(calls, ['FORMAL_GATE_SEALED.json', 'PROCESS_COMPLETION.json'])


if __name__ == '__main__':
    unittest.main()
