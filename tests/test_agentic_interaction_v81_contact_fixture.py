import unittest
from agentic_interaction_v81.contact_fixture import check_declaration


class EmptyTelemetryFixtureTests(unittest.TestCase):
    def test_declaration_does_not_need_first_sensor_row(self):
        config=dict(segments=[dict(role='VALIDATION',coordinate_unit='m',
                                   target_q=.02,hold=True,profile='constant')])
        job=dict(articulation_v7=dict(joint_type='prismatic',limits_native=[0,.3]))
        self.assertTrue(check_declaration(config,job)['valid'])
        config['segments'][0]['coordinate_unit']='rad'
        with self.assertRaises(ValueError):check_declaration(config,job)


if __name__=='__main__':unittest.main()
