"""Unit/virtual-work regressions; not substitutes for native-contact validation."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from interaction_runtime.articulation_v7 import Articulation


class ArticulationTests(unittest.TestCase):
    def test_virtual_work_matches_finite_difference(self):
        rng = np.random.default_rng(7)
        for kind, limits in [('revolute', (-.1, 1.)), ('prismatic', (0., .2))]:
            for _ in range(12):
                a = Articulation(kind, tuple(rng.normal(size=3)), tuple(rng.normal(size=3)), limits)
                T = np.eye(4); T[:3, :3] = Rotation.random(random_state=rng).as_matrix(); T[:3, 3] = rng.normal(size=3)
                w = rng.normal(size=6); h = 1e-6; moved = a.delta(h) @ T
                dx = (moved[:3, 3] - T[:3, 3])/h
                dr = Rotation.from_matrix(moved[:3, :3] @ T[:3, :3].T).as_rotvec()/h
                expected = np.r_[dx, dr] @ np.r_[T[:3, :3] @ w[:3], T[:3, :3] @ w[3:]]
                self.assertAlmostEqual(a.effort(w, T), expected, places=5)

    def test_drawer_force_not_torque(self):
        a = Articulation('prismatic', (1, 0, 0), (3, 4, 5), (0, .2))
        T = np.eye(4); T[:3, 3] = [4, 5, 6]
        self.assertEqual(a.effort([2, 7, 8, 300, 400, 500], T), 2)
        np.testing.assert_allclose(a.delta(.1)[:3, 3], [.1, 0, 0])
        self.assertEqual(a.path_scale([7, 8, 9]), 1)
        self.assertEqual(a.validate_target(dict(target_q=.12, coordinate_unit='m')), .12)
        with self.assertRaises(ValueError):
            a.validate_target(dict(target_q=12, coordinate_unit='deg'))

    def test_hinge_shift_and_perpendicular_radius(self):
        a = Articulation('revolute', (0, 0, 1), (1, 2, 0), (0, 1))
        T = np.eye(4); T[:3, 3] = [2, 2, 9]
        self.assertEqual(a.path_scale(T[:3, 3]), 1)
        self.assertEqual(a.effort([0, 2, 0, 0, 0, 3], T), 5)
        np.testing.assert_allclose(a.delta(.3) @ a.delta(-.3), np.eye(4), atol=1e-14)

    def test_invalid_inputs_rejected(self):
        for kind, axis, limits in [('fixed', (1,0,0), (0,1)), ('prismatic',(0,0,0),(0,1)), ('revolute',(1,0,0),(1,0))]:
            with self.assertRaises(ValueError):
                Articulation(kind, axis, (0,0,0), limits)


if __name__ == '__main__':
    unittest.main()
