import unittest
import numpy as np
from native_g1_radial_families_v751 import radial_transforms


class RadialTest(unittest.TestCase):
    def test_compact_center_depth_and_orthonormality_are_preserved(self):
        T=np.eye(4);T[:3,3]=[.1,-.3,.2];anchor=np.array([.1,-.2,.2]);normal=np.array([0,-1.,0])
        poses=list(radial_transforms(T,anchor,normal));self.assertEqual(len(poses),8)
        for angle,P in poses:
            np.testing.assert_allclose(P[:3,:3].T@P[:3,:3],np.eye(3),atol=1e-14)
            self.assertAlmostEqual(np.linalg.det(P[:3,:3]),1.)
            self.assertAlmostEqual((P[:3,3]-anchor)@normal,(T[:3,3]-anchor)@normal)
            self.assertAlmostEqual(np.linalg.norm(P[:3,3]-anchor),np.linalg.norm(T[:3,3]-anchor))


if __name__=='__main__':unittest.main()
