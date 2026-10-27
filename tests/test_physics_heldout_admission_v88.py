import tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from agentic_interaction_v81.operations import operations_class


class Parent:
    def interact(self,*args,**kwargs):
        self.executed=True
        return dict(status='TARGET_REACHED',physical_transmission_valid=True,gain_native=.02)


class HeldoutTests(unittest.TestCase):
    def operations(self,folder,role):
        cls=operations_class(Parent);ops=cls.__new__(cls);ops.executed=False
        ops.actions=[dict(role='VALIDATION_INDEPENDENT',target_q=.02,coordinate_unit='m')]
        ops._v81_action_index=0;ops.check_successor=Mock(return_value=dict(valid=True))
        ops.r=NS(job=dict(v81_integrated_repair=True,v84_abc=True,v88_coordination_policy='registered',
            v88_physics_study=dict(schema='physics-aware-contact-v1',role=role)),
            output=Path(folder),record=NS(segments=[]))
        return ops

    def test_formal_reactive_stale_updated_do_not_silently_fit_before_motion(self):
        for role in ('reactive','stale','updated'):
            with tempfile.TemporaryDirectory() as folder:
                ops=self.operations(folder,role)
                with patch('agentic_interaction_v81.validation_queue.admit',
                        side_effect=AssertionError('no same-episode model precondition')),\
                     patch('agentic_interaction_v81.validation_queue.mark'),\
                     patch('agentic_interaction_v81.costs.record'):
                    result=ops.interact(.02)
                self.assertTrue(ops.executed);self.assertEqual(result['status'],'TARGET_REACHED')
                ops.check_successor.assert_called_once()
                self.assertTrue((Path(folder)/'INDEPENDENT_HELDOUT_01.json').exists())

    def test_identification_qualification_keeps_original_validation_model_seal(self):
        with tempfile.TemporaryDirectory() as folder:
            ops=self.operations(folder,'identify_m0')
            with patch('agentic_interaction_v81.validation_queue.admit',
                    return_value=dict(ready=False,reason='DATA_INSUFFICIENT')) as admit:
                result=ops.interact(.02)
            admit.assert_called_once();self.assertFalse(ops.executed)
            self.assertEqual(result['status'],'VALIDATION_DEFERRED')


if __name__=='__main__':unittest.main()
