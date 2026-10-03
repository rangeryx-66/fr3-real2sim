"""Coordinate re-zero and effective MassAPI invariance, not physical trial results."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from articulated_demo.kinematics import URDFChain
from interactive_twin.initial_state import bake_initial_articulation, baked_mass_restore_spec
from interactive_twin.prepare import refresh_source_metadata
from interactive_twin.twin import write_twins
from test_interactive_twin_artifacts import URDF


class InitialState(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.asset = self.root / 'reference'
        (self.asset / 'urdf').mkdir(parents=True)
        self.urdf = self.asset / 'urdf/test.urdf'
        self.urdf.write_text(URDF)
        self.meta = {'asset_id': 'test', 'joint_name': 'hinge', 'moving_link': 'door',
                     'door_link': 'handle', 'moving_links': ['door', 'handle', 'handle_tip'],
                     'scale_source_to_meters': 1., 'moving_source_bounds': [[-.1, -.1, -.2], [.7, .1, .6]],
                     'static_source_bounds': [[-.2, 0., -.2], [.2, 1., .2]],
                     'source_joint_axis': '0 0 1', 'source_joint_limits_rad': {'lower': 0., 'upper': 1.5},
                     'category': 'wrong', 'object_name': 'wrong'}
        (self.asset / 'manifest.json').write_text(json.dumps(self.meta))

    def tearDown(self):
        self.tmp.cleanup()

    def test_nonzero_coordinates_preserve_solids_and_limits(self):
        original = self.urdf.read_bytes()
        q = np.deg2rad(2.)
        result = bake_initial_articulation(self.asset, self.root / 'zero', q)
        self.assertEqual(result['effective_initial_articulation_rad'], 0.)
        self.assertLess(result['audit']['max_translation_error_m'], 1e-12)
        self.assertLess(result['audit']['max_rotation_error_rad'], 1e-12)
        self.assertLess(result['mass_state_audit']['maximum_com_error_m'], 1e-12)
        self.assertLess(result['mass_state_audit']['maximum_inertia_tensor_error_kg_m2'], 1e-12)
        self.assertEqual(self.urdf.read_bytes(), original)
        prior = ET.parse(Path(result['asset_root']) / 'urdf/test.urdf').getroot()
        limit = prior.find("joint[@name='hinge']/limit")
        self.assertAlmostEqual(float(limit.get('lower')), -q)
        self.assertAlmostEqual(float(limit.get('upper')), 1.5 - q)
        self.assertEqual(limit.get('effort'), '3')
        self.assertEqual(result, bake_initial_articulation(self.asset, self.root / 'zero', q))
        with self.assertRaisesRegex(ValueError, 'PRESERVE_PREVIOUS_RESULTS'):
            bake_initial_articulation(self.asset, self.root / 'zero', q + .01)

    def test_all_twins_restore_same_reference_mass_com_inertia(self):
        q = np.deg2rad(1.)
        result = bake_initial_articulation(self.asset, self.root / 'zero', q)
        baked_root = Path(result['asset_root'])
        expected = json.loads((baked_root / 'manifest.json').read_text())['initial_state_bake']['mass_state_root']
        estimate = {'joint_type': 'revolute', 'confidence': .95,
                    'revolute': {'axis': [0., 0., 1.], 'point_on_axis': [.2, -.15, .13]}}
        twins = write_twins(baked_root, self.root / 'twins', estimate, np.eye(4))
        for version, paths in twins['versions'].items():
            metadata = json.loads((Path(paths['asset_root']) / 'manifest.json').read_text())
            records = baked_mass_restore_spec(paths['urdf'], metadata)
            chain = URDFChain(paths['urdf'])
            for state, target in zip(records, expected):
                T = chain.root_to_link(state['link'], {})
                com_root = (T @ np.r_[state['center_of_mass_local_m'], 1.])[:3]
                A = Rotation.from_quat(state['principal_axes_xyzw']).as_matrix()
                R = T[:3, :3] @ A
                inertia_root = R @ np.diag(state['diagonal_inertia_kg_m2']) @ R.T
                np.testing.assert_allclose(com_root, target['center_of_mass_root_m'], atol=1e-12)
                np.testing.assert_allclose(inertia_root, target['inertia_tensor_root_kg_m2'], atol=1e-12)
                self.assertEqual(state['mass_kg'], target['mass_kg'])
            self.assertTrue(metadata['initial_state_bake']['restore_mass_properties_before_world_reset'])

    def test_legacy_recompute_would_be_wrong_and_restore_is_required(self):
        q = .03
        result = bake_initial_articulation(self.asset, self.root / 'zero', q)
        root = Path(result['asset_root'])
        metadata = json.loads((root / 'manifest.json').read_text())
        chain = URDFChain(root / 'urdf/test.urdf')
        old_center = np.asarray(self.meta['moving_source_bounds']).mean(0)
        corrected = baked_mass_restore_spec(root / 'urdf/test.urdf', metadata)
        wrong = (np.linalg.inv(chain.root_to_link('door', {})) @ np.r_[old_center, 1.])[:3]
        self.assertGreater(np.linalg.norm(wrong - corrected[0]['center_of_mass_local_m']), 1e-3)

    def test_metadata_refresh_changes_only_names_not_geometry(self):
        source = self.root / 'source'
        (source / 'finaljson').mkdir(parents=True)
        (source / 'finaljson/test.json').write_text(json.dumps({'category': 'Storage Furniture', 'object_name': 'Cabinet', 'dimension': '60*40*90'}))
        urdf_digest = hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        result = refresh_source_metadata(source, self.asset, self.root / 'refresh.json')
        actual = json.loads((self.asset / 'manifest.json').read_text())
        expected = copy.deepcopy(self.meta)
        expected.update(category='Storage Furniture', object_name='Cabinet')
        self.assertEqual(actual, expected)
        self.assertEqual(hashlib.sha256(self.urdf.read_bytes()).hexdigest(), urdf_digest)
        self.assertFalse(result['geometry_urdf_physics_files_touched'])
        again = refresh_source_metadata(source, self.asset, self.root / 'refresh2.json')
        self.assertEqual(again['assets'][0]['status'], 'ALREADY_CORRECT')


if __name__ == '__main__':
    unittest.main()
