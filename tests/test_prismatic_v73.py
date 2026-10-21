import unittest
from types import SimpleNamespace
import numpy as np
from interaction_runtime.prismatic_planning_v73 import closing_envelope,reachable_interval,representable_joint_bounds
from interaction_runtime.articulation_v7 import Articulation
from interaction_runtime.policy import Policy
from pathlib import Path


class PreflightTest(unittest.TestCase):
    def test_native_precision_cannot_cross_strict_original_joint_guard(self):
        limits=np.array([[-2.059,2.0944],[-.19198,3.927]])
        old=-2.0089999995097636
        self.assertGreater(old-limits[0,0],.05)
        self.assertLess(float(np.float32(old))-limits[0,0],.05)
        low,high=representable_joint_bounds(limits,.05)
        self.assertTrue(np.all(low-limits[:,0]>.05));self.assertTrue(np.all(limits[:,1]-high>.05))
        self.assertTrue(np.all(low==low.astype(np.float32)));self.assertTrue(np.all(high==high.astype(np.float32)))
        self.assertTrue(np.all(low-(limits[:,0]+.05)<3e-7))

    def setUp(self):
        self.policy=Policy.load(Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json')

    def model(self):
        def poses(q,base,width=.084,**kw):
            T=np.eye(4);T[0,3]=q[0];left=T.copy();right=T.copy();left[1,3]=-.5*width;right[1,3]=.5*width
            return dict(tcp_link=T,L=left,R=right)
        return SimpleNamespace(poses=poses,pads={'L':np.zeros((2,3)),'R':np.zeros((2,3))},margin=lambda q:.11-q[0])

    def solver(self):
        return SimpleNamespace(solve=lambda T,base,q:np.r_[T[0,3],np.zeros(6)])

    def test_closing_sweep_keeps_authorized_contact_and_original_bound(self):
        calls=[]
        collision=SimpleNamespace(check=lambda P,M,allow:(calls.append((P['R'][1,3]-P['L'][1,3],allow)) or (True,'OK')))
        g=dict(model=self.model(),base=np.zeros(4),FINGERS=['L','R'],solver=self.solver(),collision=collision,moving=lambda:np.eye(4),runtime_policy=self.policy)
        r=closing_envelope(g,np.zeros(7),.02,{'max_centering_displacement_m':.002})
        self.assertTrue(r['valid']);self.assertEqual(len(calls),123);self.assertTrue(all(a[1] for a in calls))
        self.assertAlmostEqual(min(x[0] for x in calls),.02);self.assertEqual(r['centering_bound_m'],.002)

    def test_genuine_closing_collision_rejected_without_disabling_checker(self):
        collision=SimpleNamespace(check=lambda P,M,allow:(False,'gripper_link2/original15') if P['R'][1,3]-.5*(P['R'][1,3]-P['L'][1,3])<0 else (False,'original15'))
        g=dict(model=self.model(),base=np.zeros(4),FINGERS=['L','R'],solver=self.solver(),collision=collision,moving=lambda:np.eye(4),runtime_policy=self.policy)
        r=closing_envelope(g,np.zeros(7),.02,{'max_centering_displacement_m':.002})
        self.assertFalse(r['valid']);self.assertIn('original15',r['reason'])

    def runtime(self,bias=0):
        return SimpleNamespace(actual=lambda:0.,tcp=lambda:np.eye(4),moving=lambda:np.eye(4),robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(13)),
            arm=np.arange(7),fingers=np.arange(7,13),qtarget=np.r_[bias,np.zeros(12)],solver=self.solver(),model=self.model(),base=np.zeros(4),dt=1/240,
            policy=self.policy,collision=SimpleNamespace(check=lambda *args:(True,'OK')))

    def test_current_state_command_bias_reduces_feasible_loaded_range(self):
        a=Articulation('prismatic',(1,0,0),(0,0,0),(0,.3))
        action=dict(reference_speed_m_s=.003,reference_acceleration_m_s2=.008)
        normal=reachable_interval(self.runtime(),a,.08,action);biased=reachable_interval(self.runtime(.02),a,.08,action)
        self.assertFalse(normal['valid']);self.assertFalse(biased['valid'])
        self.assertAlmostEqual(normal['feasible_target_q']-biased['feasible_target_q'],.02,places=3)
        self.assertGreater(normal['stopping_distance_m'],0);self.assertIn('JOINT_MARGIN',biased['reason'])

    def test_short_full_validation_endpoint_is_not_truncated(self):
        a=Articulation('prismatic',(1,0,0),(0,0,0),(0,.3))
        r=reachable_interval(self.runtime(.02),a,.02,dict(reference_speed_m_s=.003,reference_acceleration_m_s2=.008))
        self.assertTrue(r['valid']);self.assertEqual(r['requested_target_q'],r['feasible_target_q'])


if __name__=='__main__':unittest.main()
