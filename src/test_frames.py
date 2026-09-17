import unittest
import numpy as np
from frames import grasp_to_tcp, rigid
class FrameTests(unittest.TestCase):
    def test_optical_topdown_and_tip_alignment(self):
        C=np.diag([1.,-1.,-1.,1.]);C[:3,3]=[.5,0,.8]
        # +x_G = +z_C and +y_G = +x_C; right-handed orientation.
        R=np.array([[0,1,0],[0,0,1],[1,0,0.]])
        t=np.array([0,0,.75]);depth=.02
        pre,T,lift=grasp_to_tcp(C,R,t,depth)
        np.testing.assert_allclose(T[:3,2],[0,0,-1])
        tip=T[:3,3]+.0095*T[:3,2]
        expected=C[:3,:3]@(t+R[:,0]*depth)+C[:3,3]
        np.testing.assert_allclose(tip,expected)
        np.testing.assert_allclose(pre[:3,3]-T[:3,3],[0,0,.08])
        np.testing.assert_allclose(lift[:3,3]-T[:3,3],[0,0,.1])
    def test_reflections_rejected(self):
        T=np.eye(4);T[2,2]=-1
        with self.assertRaises(ValueError):rigid(T)
    def test_nan_rejected(self):
        T=np.eye(4);T[0,3]=float('nan')
        with self.assertRaises(ValueError):rigid(T)
if __name__=='__main__':unittest.main()
