import tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from agentic_interaction_v81.operations import operations_class


class DecisionPacketTests(unittest.TestCase):
    def test_revolute_parent_without_packet_gets_read_only_native_units(self):
        class Owner:
            def feedback(self):return dict(contact_feedback={'available':False},model={'status':'pending'})
            def articulation(self):return SimpleNamespace(units={'coordinate':'rad','effort':'N m'})
        with tempfile.TemporaryDirectory()as d:
            o=operations_class(Owner)();o.g=dict(agent_observations=[{'folder':d}],observation_generations=[],contact_attempts=[])
            o.r=SimpleNamespace(job={'articulation_v7':{'joint_type':'revolute'}},output=Path(d),
                actual=lambda:.2,tick=0,dt=1/240,supervisor=SimpleNamespace(latched=None),
                episode_budget=SimpleNamespace(remaining=lambda t:{'wall_s':4800}))
            o.acquisitions=0;o.current_plan_ready=False
            packet=o.decision_packet(0)
            self.assertEqual(packet['units']['coordinate'],'rad');self.assertEqual(packet['joint_coordinate'],.2)
            self.assertEqual(packet['RGBD_board'],str(Path(d)/'agent_inspection.png'))
            self.assertIsNone(packet['plan_board'])

    def test_existing_parent_packet_is_forwarded_unchanged_for_revolute(self):
        expected={'sequence':2,'original':'untouched'}
        class Owner:
            def decision_packet(self,*args):return expected
        o=operations_class(Owner)();o.r=SimpleNamespace(job={'articulation_v7':{'joint_type':'revolute'}})
        self.assertIs(o.decision_packet(2),expected)


if __name__=='__main__':unittest.main()
