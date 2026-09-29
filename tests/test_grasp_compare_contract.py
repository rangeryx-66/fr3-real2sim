"""Frame and provider-format regression checks for the shared comparison."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from grasp_compare.adapters import read_candidates
from grasp_compare.candidate import GraspCandidate, graspnet_to_tcp
from grasp_compare.scene import Scene
from grasp_compare.dex1_geometry import Dex1Geometry
from grasp_compare.collision import Dex1SceneCollision


class CandidateContract(unittest.TestCase):
    def setUp(self):
        self.T_B_C = np.eye(4)
        self.T_B_C[:3, 3] = [.3, -.2, .5]
        self.scene = Scene(Path('/tmp/example.npz'), np.array([[0., 0., .5]]),
                           np.array([True]), self.T_B_C, None, 'test')

    def test_graspnet_tcp_chain_matches_existing_r1_convention(self):
        grasp = np.eye(4)
        grasp[:3, 3] = [.1, .2, .6]
        candidate = GraspCandidate('anygrasp', 0, 0.8, grasp,
                                   graspnet_to_tcp(.025), self.T_B_C)
        self.assertTrue(np.allclose(candidate.T_B_TCP,
                                    self.T_B_C @ grasp @ graspnet_to_tcp(.025)))
        self.assertTrue(np.allclose(candidate.T_B_TCP[:3, 3], [.425, 0., 1.1]))

    def test_graspgroup_npy_contract(self):
        row = np.zeros((1, 17))
        row[0, 0:4] = [.9, .055, .02, .025]
        row[0, 4:13] = np.eye(3).reshape(-1)
        row[0, 13:16] = [.1, .2, .6]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pred.npy'
            np.save(path, row)
            c, = read_candidates(path, 'zerograsp', self.scene)
        self.assertEqual(c.model, 'zerograsp')
        self.assertAlmostEqual(c.width_m, .055)
        self.assertAlmostEqual(c.depth_m, .025)

    def test_calibration_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pred.json'
            path.write_text(json.dumps({'frame': 'camera_optical',
                                        'T_B_C': np.eye(4).tolist(), 'grasps': []}))
            with self.assertRaisesRegex(ValueError, 'calibration differs'):
                read_candidates(path, 'graspgenx', self.scene)

    def test_dex1_opening_limit_rejects_wider_grasp(self):
        scene = Scene(Path('/tmp/example.npz'), np.array([[0., 0., .5], [.1, 0., .5]]),
                      np.array([True, False]), np.eye(4), None, 'test')
        geometry = Dex1Geometry()
        checker = Dex1SceneCollision(scene, geometry)
        verdict = checker.check(np.eye(4), geometry.open_width + .01)
        self.assertEqual(verdict['status'], 'WIDTH_UNREACHABLE')

    def test_target_mask_does_not_hide_panel_at_finger(self):
        # A handle/target mask can include nearby door-panel pixels. A point
        # inside the official finger mesh must still be reported as collision.
        scene = Scene(Path('/tmp/panel.npz'), np.array([[-.015, .047, .02], [1., 1., 1.]]),
                      np.array([True, False]), np.eye(4), None, 'test')
        checker = Dex1SceneCollision(scene, Dex1Geometry())
        verdict = checker.check(np.eye(4), checker.geometry.open_width)
        self.assertEqual(verdict['status'], 'COLLISION')
        self.assertEqual(verdict['allowed_target_contact_points'], 0)


if __name__ == '__main__':
    unittest.main()
