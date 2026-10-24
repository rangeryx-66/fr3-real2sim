"""Geometric contract tests; native physical parking is validated separately."""
import math
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from agentic_interaction_v81.revolute_certificate import evaluate


class CertificateTests(unittest.TestCase):
    def fixture(self):
        records=[]
        native=SimpleNamespace(angle=lambda w:(.07-w)/.1,
            width=lambda a:.07-.1*a,width_derivative=lambda a:-.1,
            values=lambda a:dict.fromkeys(['a','b','c','d','e','f'],a))
        def poses(q,base,width):
            left=np.eye(4);right=np.eye(4);right[1,3]=width
            return dict(left=left,right=right,tcp_link=np.eye(4))
        model=SimpleNamespace(native=native,poses=poses,margin=lambda q:1.,
            pads=dict(left=np.zeros((3,3)),right=np.zeros((3,3))))
        def check(P,M,allow):records.append((M.copy(),allow));return True,'SAFE'
        solver=SimpleNamespace(solve=lambda T,base,q,**kw:np.r_[math.atan2(T[1,0],T[0,0]),np.zeros(6)])
        g=dict(model=model,base=np.zeros(4),moving=lambda:np.eye(4),actual=lambda:0.,dt=1/240,
            robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(13)),arm=np.arange(7),
            fingers=np.arange(7,13),qtarget=np.zeros(13),FINGERS=['left','right'],
            normal=[0,-1,0],collision=SimpleNamespace(check=check),policy=dict(preload_n=.5,max_centering_displacement_m=.001),
            job=dict(articulation_v7=dict(limits_native=[-.5,3.])))
        T=np.eye(4);T[:3,3]=[1.,0.,.5]
        candidate=dict(T=T,approach=[np.zeros(7)])
        actions=[dict(role=role,target_q=q,coordinate_unit='rad') for role,q in
            [('FIT_OPEN',.1),('FIT_REVERSE',.04),('VALIDATION_DISTINCT',.08)]]
        policy=SimpleNamespace(data=dict(joint_margin_rad=.05),
            motion=lambda p:dict(speed_m_s=.012,acceleration_m_s2=.04))
        close=Mock(return_value=dict(valid=True));exit_check=Mock(return_value=dict(valid=True,parking_region='native_rest'))
        modules={
            'native_g1_ft_identification':SimpleNamespace(kinematic_prior=lambda g:dict(
                joint_type='revolute',axis_world=[0,0,1],origin_world_m=[0,0,0])),
            'interaction_runtime.prismatic_planning_v73':SimpleNamespace(
                closing_envelope=close,predicted_aperture=lambda c:.02),
            'native_g1_safe_park':SimpleNamespace(plan_exit=exit_check),
            'xarm7_native_gripper':SimpleNamespace(GRIPPER_JOINTS=['a','b','c','d','e','f'])}
        return g,candidate,actions,policy,solver,modules,records,close,exit_check

    def test_whole_bidirectional_chain_closing_and_each_exit(self):
        g,c,a,p,s,m,records,close,exit_check=self.fixture()
        with patch.dict(sys.modules,m):r=evaluate(g,c,a,p,dict(gripper_kp_nm_rad=10.),s)
        self.assertTrue(r['valid']);close.assert_called_once();self.assertEqual(exit_check.call_count,3)
        self.assertEqual([e['target_rad']for e in r['endpoints']],[.1,.04,.08])
        self.assertLess(max(e['target_rad']for e in r['endpoints']),math.pi/2)
        self.assertGreater(r['checked_native_configurations'],100)
        self.assertEqual(r['perpendicular_radius_m'],1.)
        self.assertTrue(all(allow for _,allow in records))
        self.assertEqual(g['actual'](),0.)  # planning never commands object state

    def test_native_collision_and_joint_rejection_remain_final_for_candidate(self):
        for kind in ('collision','margin','exit'):
            g,c,a,p,s,m,_,_,exit_check=self.fixture()
            if kind=='collision':g['collision'].check=lambda *args:(False,'original31')
            if kind=='margin':g['model'].margin=lambda q:.049
            if kind=='exit':exit_check.return_value=dict(valid=False,reason='no route')
            with self.subTest(kind=kind),patch.dict(sys.modules,m):
                r=evaluate(g,c,a,p,dict(gripper_kp_nm_rad=10.),s)
                self.assertFalse(r['valid']);self.assertNotIn('cleared_latch',r)

    def test_reference_lead_is_not_erased_by_moving_object_with_command(self):
        g,c,a,p,s,m,records,_,_=self.fixture();seen=[]
        old=s.solve
        def solve(T,*args,**kw):seen.append(math.atan2(T[1,0],T[0,0]));return old(T,*args,**kw)
        s.solve=solve
        with patch.dict(sys.modules,m):r=evaluate(g,c,a[:1],p,dict(gripper_kp_nm_rad=10.),s)
        self.assertTrue(r['valid']);self.assertIn(-math.radians(.5),seen)
        self.assertEqual(records[0][0][0,0],1.)
        self.assertEqual(records[4][0][0,0],1.)


if __name__=='__main__':unittest.main()
