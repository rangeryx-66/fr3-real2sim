"""Read-only fixed-schema recording contracts; no simulator or control calls."""
from copy import deepcopy
from types import SimpleNamespace
import unittest

import numpy as np

from agentic_interaction_v81.load_telemetry import instrument


def fixture(enabled=True, count=0):
    base = {'joint_q': np.float64(.3), 'untouched': np.array([4., 5.])}
    record = SimpleNamespace(count=count, metadata={}, coordinate_fields=lambda row: base)
    runtime = SimpleNamespace(job={'v83_load_telemetry': enabled}, record=record,
                              names=['base_x', 'arm_a', 'arm_b'], arm=np.array([1, 2]),
                              sensor=SimpleNamespace(mapping=dict(
                                  joint_names=['base_x', 'arm_a', 'arm_b', 'fixed_cut'],
                                  joint_indices={'base_x': 0, 'arm_a': 1, 'arm_b': 2, 'fixed_cut': 3})))
    return runtime, base


def sample():
    return dict(projected_joint_efforts=np.array([1., 2., 3.]),
                explicit_applied_efforts=np.array([0., 0., 0.]),
                joint_reactions=np.arange(30.).reshape(5, 6),
                actuator_position_target_arm=np.array([.2, .4]),
                actuator_gravity_bias_nm=np.array([2., 3.]),
                native_G1_master_effort_cap_nm=.7)


class LoadTelemetryTests(unittest.TestCase):
    def test_default_is_exact_noop_and_opt_in_is_boolean(self):
        r, _ = fixture(False); previous = r.record.coordinate_fields
        self.assertEqual(instrument(r), {'enabled': False})
        self.assertIs(r.record.coordinate_fields, previous)
        self.assertEqual(r.record.metadata, {})
        r.job.pop('v83_load_telemetry')
        self.assertEqual(instrument(r), {'enabled': False})
        for invalid in (1, None, 'true'):
            r.job['v83_load_telemetry'] = invalid
            with self.assertRaisesRegex(ValueError, 'BOOLEAN'):
                instrument(r)

    def test_fixed_shapes_missing_flags_and_original_fields(self):
        r, base = fixture(); metadata = instrument(r)
        good = r.record.coordinate_fields(sample())
        missing = r.record.coordinate_fields({})
        self.assertEqual(good['joint_q'], base['joint_q'])
        self.assertIs(good['untouched'], base['untouched'])
        self.assertEqual(set(base), {'joint_q', 'untouched'})
        for name, details in metadata['columns'].items():
            self.assertEqual(good[name].shape, tuple(details['shape']))
            self.assertEqual(missing[name].shape, good[name].shape)
            self.assertEqual(good[name+'_valid'], 1)
            self.assertEqual(missing[name+'_valid'], 0)
            self.assertTrue(np.isnan(missing[name]).all())
            self.assertNotEqual(good[name].dtype, object)
        self.assertEqual(good['incoming_joint_reactions_native'].shape, (5, 6))

    def test_none_ragged_wrong_shape_and_nonfinite_are_unavailable(self):
        r, _ = fixture(); meta = instrument(r)
        invalids = [None, np.asarray(None), np.array(2.), [1., 2.],
                    [[1.], [2., 3.]], np.array([1., np.inf, 3.]), 'invalid']
        for invalid in invalids:
            with self.subTest(invalid=repr(invalid)):
                row = sample(); row['projected_joint_efforts'] = invalid
                result = r.record.coordinate_fields(row)
                self.assertEqual(result['projected_joint_efforts_native'].shape, (3,))
                self.assertEqual(result['projected_joint_efforts_native_valid'], 0)
                self.assertTrue(np.isnan(result['projected_joint_efforts_native']).all())
                self.assertEqual(result['incoming_joint_reactions_native_valid'], 1)
        row = sample(); row['joint_reactions'] = np.zeros((3, 6))
        self.assertEqual(r.record.coordinate_fields(row)['incoming_joint_reactions_native_valid'], 0)
        row = sample(); row['native_G1_master_effort_cap_nm'] = np.array([.7])
        self.assertEqual(r.record.coordinate_fields(row)['native_G1_master_effort_cap_nm_valid'], 0)

    def test_copies_cannot_alias_live_row_or_previous_outputs(self):
        r, _ = fixture(); instrument(r); row = sample(); before = deepcopy(row)
        first = r.record.coordinate_fields(row)
        first['projected_joint_efforts_native'][1] = 1000.
        first['incoming_joint_reactions_native'][0, 0] = 1000.
        for key in row:
            np.testing.assert_array_equal(row[key], before[key])
        second = r.record.coordinate_fields(row)
        self.assertEqual(second['projected_joint_efforts_native'][1], 2.)
        row['projected_joint_efforts'][1] = -50.
        self.assertEqual(second['projected_joint_efforts_native'][1], 2.)

    def test_installation_is_ctor_only_and_idempotent(self):
        r, _ = fixture(count=1)
        with self.assertRaisesRegex(ValueError, 'BEFORE_FIRST_RECORD'):
            instrument(r)
        self.assertEqual(r.record.metadata, {})
        r, _ = fixture(); meta = instrument(r); wrapper = r.record.coordinate_fields
        r.record.count = 99
        self.assertEqual(instrument(r), meta)
        self.assertIs(r.record.coordinate_fields, wrapper)
        meta['native_dof_order'].append('mutated')
        self.assertEqual(len(r.record.metadata['v83_load_telemetry']['native_dof_order']), 3)

    def test_mapping_must_include_fixed_incoming_joint_rows(self):
        r, _ = fixture(); r.sensor.mapping['joint_indices']['fixed_cut'] = 7
        with self.assertRaisesRegex(ValueError, 'MAPPING'):
            instrument(r)
        self.assertEqual(r.record.metadata, {})

    def test_same_shape_rows_stack_numerically_with_no_control_runtime(self):
        r, _ = fixture(); meta = instrument(r)
        outputs = [r.record.coordinate_fields(x) for x in (sample(), {}, sample())]
        for name in meta['columns']:
            stacked = np.asarray([x[name] for x in outputs])
            self.assertEqual(stacked.shape, (3,)+outputs[0][name].shape)
            self.assertEqual(stacked.dtype, np.float64)
        self.assertFalse(meta['additional_sensor_queries'])
        self.assertFalse(meta['feeds_protection_controller_or_estimator'])
        self.assertTrue(meta['original_sampling_unchanged'])
        self.assertIn('not', meta['projected_efforts'])


if __name__ == '__main__':
    unittest.main()
