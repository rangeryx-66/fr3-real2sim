import unittest
import numpy as np
from agentic_interaction_v88.physics_aware import DemandModel,Capability,choose


class SelectionTests(unittest.TestCase):
    def select(self,beta,capacity=(.3,.6,1.2),cov=None):
        m=DemandModel(tuple(beta),tuple(map(tuple,np.zeros((3,3)) if cov is None else cov)),.01,'FIT only')
        c=[Capability(p,f,(p*.8,p*.8),(p*1.2,p*1.2),'measured calibration')
            for p,f in zip((.5,1.,2.),capacity)]
        return choose(m,c,distance=.04,speeds=(.002,.006,.012),accelerations=(.004,.012,.04),
            motion=dict(speed_m_s=.012,acceleration_m_s2=.04),sustained_pad_n=4.,emergency_wrist_n=10.)

    def test_model_changes_real_action_fields(self):
        low=self.select((2.,1.,.2))['action'];high=self.select((6.,70.,.4))['action']
        self.assertNotEqual((low['requested_preload_n'],low['reference_speed_m_s'],low['reference_acceleration_m_s2']),
            (high['requested_preload_n'],high['reference_speed_m_s'],high['reference_acceleration_m_s2']))
        self.assertLessEqual(high['predicted_force_upper_n'],high['contact_capacity_lower_n'])

    def test_uncertainty_changes_admission_without_fabricating_slip(self):
        ordinary=self.select((2.,1.,.2));uncertain=self.select((2.,1.,.2),cov=np.diag([400.,100.,.01]))
        self.assertNotEqual(ordinary['action'],uncertain['action'])
        self.assertFalse(uncertain['physical_failure_inferred'])
        self.assertFalse(uncertain['episode_termination_required'])

    def test_request_is_not_used_as_measured_capacity(self):
        result=self.select((2.,1.,.2),capacity=(.01,.01,.01))
        self.assertIsNone(result['action']);self.assertFalse(result['physical_failure_inferred'])

    def test_high_preload_has_no_invented_damage_penalty(self):
        r=self.select((6.,70.,.4));self.assertFalse(r['high_preload_damage_or_energy_cost_assumed'])
        self.assertTrue(r['pareto'])

    def test_invalid_model_is_rejected(self):
        with self.assertRaises(ValueError):self.select((-1.,1.,.1))

    def test_observed_breakaway_is_separate_from_sliding_friction(self):
        model=DemandModel((2.,1.,.2),tuple(map(tuple,np.zeros((3,3)))),.01,'FIT',.8)
        self.assertGreaterEqual(model.upper(.002,.004),.8)


if __name__=='__main__':unittest.main()
