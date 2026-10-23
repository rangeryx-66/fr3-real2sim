import hashlib,json,tempfile,unittest
from pathlib import Path
from agentic_interaction_v81.blind_integration import prepare


class BlindIntegrationTests(unittest.TestCase):
    def test_two_fresh_opaque_commitments_and_finite_specs(self):
        with tempfile.TemporaryDirectory()as d:
            root=Path(d)/'new';public=prepare(root)
            self.assertEqual([e['joint_type']for e in public['episodes']],['revolute','prismatic'])
            for row in public['episodes']:
                self.assertNotIn('seed',row);self.assertNotIn('mass_scale',row)
                spec=root/(row['joint_type']+'_SPEC.json')
                self.assertEqual(hashlib.sha256(spec.read_bytes()).hexdigest(),row['spec_sha256'])
                data=json.loads(spec.read_text());plant=Path(data['physics_file'])
                self.assertEqual(hashlib.sha256(plant.read_bytes()).hexdigest(),row['physics_sha256'])
                self.assertEqual(data['wall_budget_s'],5400);self.assertEqual(data['sim_budget_s'],2700)
                self.assertEqual(data['agent_wait_s'],600)
            with self.assertRaises(FileExistsError):prepare(root)


if __name__=='__main__':unittest.main()
