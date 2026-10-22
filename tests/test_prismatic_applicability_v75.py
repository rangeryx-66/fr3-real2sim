import copy,unittest
import numpy as np
from prismatic_applicability_v75 import diagnose
from prismatic_estimator_audit_v75 import applicability
from xarm_ft_v7_estimator import fit


class ApplicabilityTest(unittest.TestCase):
    def rows(self,asymmetric=False):
        result=[]
        for segment,direction in [('FIT_A',1),('FIT_B',-1),('FIT_C',1)]:
            for t in np.arange(0,16,.1):
                velocity=direction*.002*(1+.4*np.sin(1.7*t))
                acceleration=direction*.00136*np.cos(1.7*t)
                force=2*acceleration+3*velocity+(.42 if asymmetric and direction<0 else .3)*direction
                result.append(dict(segment=segment,correlation_group=segment,t=float(t),q=.04+direction*.002*t,qdd=acceleration,
                    qd=velocity,direction=direction,tau=force))
        return result

    def model(self,rows):
        return fit(rows,dict(domain_native=[0,.15],parameter_scales=[5,10,1],ridge=.001,
            joint_type='prismatic',units=dict(coordinate='m',effort='N',inertial='kg',damping='N s/m')))

    def test_known_global_ridge_effect_is_not_a_new_physical_residual(self):
        rows=self.rows();model=self.model(rows);saved=copy.deepcopy(model)
        self.assertIn('untrusted',applicability(rows,model)['status'])
        audit=diagnose(rows,model)
        self.assertEqual(audit['status'],'CONDITIONAL_ONLY_NO_LACK_OF_FIT_DETECTED')
        self.assertEqual(model,saved);self.assertFalse(audit['production_coefficients_modified'])
        self.assertFalse(audit['validation_used'])

    def test_real_direction_mismatch_remains_visible(self):
        rows=self.rows(True);audit=diagnose(rows,self.model(rows))
        self.assertEqual(audit['status'],'CHECK_SAME_FAMILY_RESIDUAL_STRUCTURE')
        self.assertIn('UNKNOWN',audit['physical_materiality'])
        self.assertFalse(audit['execution_gate'])

    def test_rank_deficient_input_is_unknown(self):
        rows=self.rows()
        for r in rows:r['qdd']=0.
        self.assertEqual(diagnose(rows,self.model(rows))['status'],'UNKNOWN_RANK_DEFICIENT')


if __name__=='__main__':unittest.main()
