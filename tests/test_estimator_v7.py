import copy
import unittest
import numpy as np
from xarm_ft_v31_estimator import process, common_filter
from xarm_ft_v7_estimator import samples, analyze, prediction, VERSION


def fixture(kind='prismatic',role='FIT',phase=.0):
    t=np.arange(4800)/240
    q=.08+.025*np.sin(1.1*t+phase)+.01*np.sin(2.7*t+phase)
    v=.0275*np.cos(1.1*t+phase)+.027*np.cos(2.7*t+phase)
    acc=np.gradient(v,1/240);beta=np.array([3.,4.,.35]) if kind=='prismatic' else np.array([.13,.25,.03])
    effort=beta[0]*acc+beta[1]*v+beta[2]*np.sign(v)
    prior=dict(joint_type=kind,axis_world=[1,0,0],origin_world_m=[0,0,0])
    rows=[]
    for i,x in enumerate(t):
        w=np.zeros(6);w[0 if kind=='prismatic' else 3]=effort[i]
        rows.append(dict(t=x,door_angle_deg=np.rad2deg(q[i]),identification_segment=role,forces_n={},wrist_ft=dict(valid=True,T_world_sensor=np.eye(4).tolist(),raw_wrench_sensor=w.tolist(),tool_gravity_wrench_world=[0]*6,body_states=[])))
    return rows,prior,q,v,beta


def config(kind):
    return dict(joint_type=kind,domain_native=[0,.2],units=dict(coordinate='m' if kind=='prismatic' else 'rad',effort='N' if kind=='prismatic' else 'N m'),parameter_scales=[5.,10.,1.] if kind=='prismatic' else [1.,.25,.045],intervals=[2],smoothness=[.001],ridge=.001)


class EstimatorTests(unittest.TestCase):
    def test_SI_recovery_and_legacy_equivalence(self):
        for kind in ['prismatic','revolute']:
            rows,p,q,v,beta=fixture(kind)
            d=process(rows,p,[0]*6,v,native_coordinate=q);d['block_id']='FIT#0'
            ss=samples(d,[0,.2],[2]);r=analyze(ss,config(kind))
            np.testing.assert_allclose(r['constant_model']['parameters'],beta,rtol=2e-3,atol=1e-5)
            if kind=='revolute':
                old=process(rows,p,[0]*6,v)
                for key in ['q','qd','qdd','torque','direction','valid']:
                    if key=='q':np.testing.assert_allclose(d[key],old[key],rtol=0,atol=2e-17)
                    else:np.testing.assert_array_equal(d[key],old[key])
            else:
                with self.assertRaises(ValueError):process(rows,p,[0]*6,v)

    def test_validation_cannot_change_fit_and_free_prediction(self):
        rows,p,q,v,beta=fixture();d=process(rows,p,[0]*6,v,native_coordinate=q);d['block_id']='FIT#0'
        train=samples(d,[0,.2],[2]);a=analyze(train,config('prismatic'))
        test,p,q,v,_=fixture(role='VALIDATION',phase=.7)
        held=process(test,p,[0]*6,v,native_coordinate=q);held['block_id']='VALIDATION#0';samples(held,[0,.2],[2])
        result=prediction(held,a['constant_model'])
        self.assertEqual(result['status'],'predicted')
        # A filtered Coulomb reversal is not a native instantaneous reversal;
        # report that sensitivity rather than force an unrealistic zero error.
        self.assertTrue(np.isfinite(result['coordinate_rmse']))
        exact=copy.deepcopy(held);exact['torque']=exact['unfiltered_torque']
        raw=prediction(exact,a['constant_model'])
        self.assertLess(raw['coordinate_rmse'],.001)
        held['torque']*=99
        b=analyze(train,config('prismatic'))
        np.testing.assert_array_equal(a['selected']['parameters'],b['selected']['parameters'])
        np.testing.assert_array_equal(a['selected']['conditional_covariance'],b['selected']['conditional_covariance'])

    def test_segment_boundaries_and_invalid_units(self):
        rows,p,q,v,_=fixture();rows[1000]['identification_segment']='VALIDATION'
        with self.assertRaises(ValueError):process(rows,p,[0]*6,v,native_coordinate=q)
        rows,p,q,v,_=fixture()
        with self.assertRaises(ValueError):process(rows,p,[0]*6,v,native_coordinate=q[:-1])


if __name__=='__main__':unittest.main()
