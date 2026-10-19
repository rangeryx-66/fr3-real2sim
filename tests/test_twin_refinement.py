import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from interactive_twin_refinement.fitting import fit_se3,predict_action

class RefineTests(unittest.TestCase):
 def test_noisy_first_frame_and_axis_gauge(self):
  rng=np.random.default_rng(13);axis=np.array([.2,-.1,.97]);axis/=np.linalg.norm(axis);c=np.array([.1,.2,.3]);p=np.array([.6,.25,.4]);R0=Rotation.from_euler('xyz',[.2,.4,.1]).as_matrix();Ts=[]
  for q in np.linspace(0,.2,100):
   A=Rotation.from_rotvec(q*axis).as_matrix();T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(rng.normal(0,2e-5,3)).as_matrix()@A@R0;T[:3,3]=c+A@(p-c)+rng.normal(0,2e-5,3);Ts.append(T)
  Ts=np.asarray(Ts);Ts[0,:3,3]+=[.002,-.001,.001]
  f=fit_se3(Ts);a=np.asarray(f['revolute']['axis']);point=np.asarray(f['revolute']['point_on_axis'])
  self.assertLess(np.rad2deg(np.arccos(abs(a@axis))),.2)
  self.assertLess(np.linalg.norm(np.cross(axis,point-c)),.002)
  self.assertGreater(np.linalg.norm(np.asarray(f['reference_pose'])[:3,3]-Ts[0,:3,3]),.001)
  self.assertAlmostEqual(float(a@(point-Ts[:,:3,3].mean(0))),0,places=8)
 def test_validation_position_cannot_select_phase(self):
  axis=np.array([0.,0.,1.]);Ts=[]
  for q in np.linspace(0,.1,40):
   T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(q*axis).as_matrix();T[:3,3]=T[:3,:3]@np.array([.4,0.,0.]);Ts.append(T)
  fit={'revolute':{'axis':axis.tolist(),'point_on_axis':[0,0,0]}}
  clean=predict_action(Ts,fit);Ts=np.array(Ts);Ts[20,:3,3]+=.005
  corrupt=predict_action(Ts,fit)
  self.assertGreater(corrupt['position_rmse_m'],clean['position_rmse_m']+.0005)

class PhysicsSplitTests(unittest.TestCase):
 def test_test_action_rejected_by_fit_interface(self):
  from interactive_twin_refinement.physics import fit
  with self.assertRaisesRegex(ValueError,'EXPECTED_TRAIN_AND_VALIDATION_ONLY'):
   fit({'P1':{},'P2':{},'P3':{},'P4':{}},[],None,{})

if __name__=='__main__':unittest.main()
