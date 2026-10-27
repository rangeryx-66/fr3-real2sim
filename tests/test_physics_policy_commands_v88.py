import unittest
from agentic_interaction_v88.policy_commands import decision,prepare_command,acquire_command


class CommandTests(unittest.TestCase):
    def inputs(self,b):
        return dict(schema='physics-aware-measured-input/v1',hidden_dynamics_used=False,
            validation_used_for_fit_or_selection=False,model_usable=True,
            demand_model=dict(parameters=[2.,b,.2],covariance=[[0.]*3 for _ in range(3)],
                residual_bound_n=.01,source='original FIT-only measured estimate'),
            contact_capabilities=[dict(request_n=p,transmitted_axis_force_lower_n=f,
                measured_pad_normal_lower_n=[p*.8]*2,measured_pad_normal_upper_n=[p*1.2]*2,
                source='actual contact calibration') for p,f in [(1.,.5),(2.,1.)]])

    def select(self,method,measurement):
        return decision(method,measurement,dict(qualified_requested_preloads_n=[1.,2.],
            reference_speeds_m_s=[.002,.006],reference_accelerations_m_s2=[.004,.008]),
            distance=.04,motion=dict(speed_m_s=.012,acceleration_m_s2=.04),
            sustained_pad_n=4.,emergency_wrist_n=10.)

    def test_measured_model_changes_the_commands_really_submitted_to_abc(self):
        low=self.select('stale',self.inputs(2.));high=self.select('updated',self.inputs(150.))
        self.assertNotEqual(low['action'],high['action'])
        prepared=prepare_command(high,[dict(role='VALIDATION_HELDOUT',target_q=.04)],
            inspected_image_sha256='image',reason='measured model decision')
        command=acquire_command(prepared,inspected_plan_sha256='current checked plan',reason='finite B')
        self.assertEqual(command['native_g1_preload_request_n'],high['action']['requested_preload_n'])
        for k in ['reference_speed_m_s','reference_acceleration_m_s2']:
            self.assertEqual(command['segments'][0][k],high['action'][k])
        self.assertFalse(command['retain_contact'])

    def test_reactive_ignores_regression_and_has_the_same_full_control_menu(self):
        a=self.select('reactive',self.inputs(2.));b=self.select('reactive',self.inputs(150.))
        self.assertEqual(a['action'],b['action'])
        self.assertEqual(a['action'],dict(requested_preload_n=2.,reference_speed_m_s=.006,
            reference_acceleration_m_s2=.008))

    def test_unsupported_model_continues_with_normal_feedback(self):
        m=self.inputs(150.);m['model_usable']=False
        result=self.select('updated',m)
        self.assertEqual(result['action'],self.select('reactive',m)['action'])
        self.assertFalse(result['ordinary_failure_ends_episode'])
        self.assertTrue(result['fallback'])

    def test_zero_empirical_axis_budget_does_not_disable_qualified_reactive_fallback(self):
        m=self.inputs(150.)
        for c in m['contact_capabilities']:c['transmitted_axis_force_lower_n']=0.
        result=self.select('updated',m)
        self.assertEqual(result['action'],self.select('reactive',m)['action'])
        self.assertFalse(result['ordinary_failure_ends_episode'])
        self.assertEqual(result['fallback'],'NO_PREDICTED_FEASIBLE_ACTION_USE_REACTIVE_AND_OBSERVE')

    def test_oracle_or_leaked_truth_is_not_a_formal_method(self):
        with self.assertRaises(ValueError):self.select('oracle',self.inputs(2.))
        m=self.inputs(2.);m['hidden_dynamics_used']=True
        with self.assertRaises(ValueError):self.select('updated',m)

    def test_heldout_manipulation_cannot_silently_fit_the_model(self):
        result=self.select('updated',self.inputs(2.))
        with self.assertRaises(ValueError):prepare_command(result,
            [dict(role='FIT',target_q=.04)],inspected_image_sha256='image',reason='no')


if __name__=='__main__':unittest.main()
