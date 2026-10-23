import hashlib,json,tempfile,unittest
import numpy as np
from pathlib import Path
from types import SimpleNamespace
from agentic_interaction_v81.target_association import start_latest,accept_latest


class AssociationTests(unittest.TestCase):
    def fixture(self,d):
        p=Path(d);(p/'rgb.png').write_bytes(b'fresh wrist')
        np.save(p/'candidate_00.npy',np.array([[True,False],[False,True]]))
        (p/'candidate_cues.json').write_text(json.dumps([dict(name='candidate_00',partial=False,eligible_for_grasp=True)]))
        old=dict(folder='old other part',world_sim_s=1)
        latest=dict(folder=d,world_sim_s=2)
        ops=SimpleNamespace(r=SimpleNamespace(job=dict(articulation_v7=dict(joint_type='prismatic')),
            output=p,supervisor=SimpleNamespace(latched=None),world=SimpleNamespace(current_time=3)),
            g=dict(recovery_state='UNLOADED_OBSERVATION',agent_observations=[old,latest],
                observation_generations=[],agent_moves=3,wrist_routes=['actual route']),
            cache={1:'stale'},current_plan_ready=True,_v81_plan_ticket={})
        c=dict(reason='Fresh view shows a different moving-part handle',
            inspected_image_sha256=hashlib.sha256(b'fresh wrist').hexdigest(),
            observed_handle_hypothesis=dict(source='SAM3',id='candidate_00',
                sha256=hashlib.sha256((p/'candidate_00.npy').read_bytes()).hexdigest(),reason='lower front bar'))
        return ops,c

    def test_explicit_switch_keeps_history_and_actual_routes_not_extra_views(self):
        with tempfile.TemporaryDirectory()as d:
            ops,c=self.fixture(d);r=start_latest(ops,c)
            self.assertEqual(len(ops.g['agent_observations']),1)
            self.assertEqual(len(ops.g['observation_generations'][0]),2)
            self.assertEqual(ops.g['agent_moves'],3)
            self.assertEqual(ops.g['wrist_routes'],['actual route'])
            self.assertFalse(ops.current_plan_ready);self.assertFalse(r['GT_geometry_used'])
            self.assertEqual(ops.cache,{})

    def test_stale_choice_or_contact_cannot_mutate_association(self):
        with tempfile.TemporaryDirectory()as d:
            ops,c=self.fixture(d);c['observed_handle_hypothesis']['sha256']='0'*64
            with self.assertRaisesRegex(ValueError,'STALE'):start_latest(ops,c)
            self.assertEqual(len(ops.g['agent_observations']),2)
            ops.g['recovery_state']='FINITE_CONTACT_OPERATION'
            with self.assertRaises(ValueError):start_latest(ops,c)
            self.assertEqual(ops.g['observation_generations'],[])

    def test_revolute_and_after_contact_old_image_rejected(self):
        with tempfile.TemporaryDirectory()as d:
            ops,c=self.fixture(d);ops.r.job['articulation_v7']['joint_type']='revolute'
            with self.assertRaises(ValueError):start_latest(ops,c)
            ops.r.job['articulation_v7']['joint_type']='prismatic'
            ops._last_contact_finished_world_s=4
            with self.assertRaises(ValueError):start_latest(ops,c)

    def test_accept_before_another_view_does_not_search_or_move(self):
        with tempfile.TemporaryDirectory()as d:
            ops,c=self.fixture(d);calls=[]
            ops.accept_geometry=lambda folder,reason:(calls.append((folder,reason))or dict(geometry=True))
            result=accept_latest(ops,c)
            self.assertTrue(result['accepted']);self.assertEqual(len(calls),1)
            self.assertFalse(result['grasp_search_started'])
            self.assertFalse(result['robot_motion_executed'])
            np.testing.assert_array_equal(np.load(Path(d)/'handle_mask.npy'),np.load(Path(d)/'candidate_00.npy'))


if __name__=='__main__':unittest.main()
