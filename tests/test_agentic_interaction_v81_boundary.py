import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from agentic_interaction_v81.boundary_operations import observe_view,wrist_evidence,prepare_grasp


class BoundaryTests(unittest.TestCase):
    def test_current_image_validation_before_view_execution(self):
        with tempfile.TemporaryDirectory()as d:
            p=Path(d);(p/'rgb.png').write_bytes(b'current');moves=[]
            ops=SimpleNamespace(g=dict(agent_observations=[dict(folder=d,world_sim_s=10)]),
                _last_contact_finished_world_s=9,evaluate_view=lambda t:dict(feasible=False),
                move_and_observe=lambda route:moves.append(route))
            c=dict(reason='look around',target={},inspected_image_sha256=hashlib.sha256(b'current').hexdigest())
            self.assertTrue(wrist_evidence(ops,c));self.assertFalse(observe_view(ops,c)['executed'])
            self.assertEqual(moves,[])
            ops._last_contact_finished_world_s=11
            self.assertFalse(observe_view(ops,c)['executed']);self.assertEqual(moves,[])

    def test_failed_planning_never_validates_or_approves_contact(self):
        ops=SimpleNamespace(request_plan=lambda c:dict(plan=False,reason='native collision'))
        self.assertFalse(prepare_grasp(ops,{})['plan'])


if __name__=='__main__':unittest.main()
