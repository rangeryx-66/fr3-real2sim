import json
import tempfile
import unittest
from pathlib import Path
from scripts.register_prismatic_v74 import register


class RegistrationTest(unittest.TestCase):
    def fixture(self, root, ready):
        engineering=root/'engineering';(engineering/'jobs').mkdir(parents=True)
        ids=['19179','45290','27044','45746']
        cases={a:dict(acquired=True,reverse=True,complete_validation=True,safe_park=True) for a in ids}
        (engineering/'READINESS_REVIEW.json').write_text(json.dumps(dict(ready_for_independent_benchmark=ready,cases=cases)))
        import hashlib
        from scripts.register_prismatic_v74 import ROOT
        source='scripts/register_prismatic_v74.py'
        (engineering/'SOURCE_FREEZE.json').write_text(json.dumps(dict(sources={source:hashlib.sha256((ROOT/source).read_bytes()).hexdigest()})))
        reference=root/'reference.json';reference.write_text(json.dumps(dict(objects=[dict(asset_id=a,joint_type='prismatic') for a in ids])))
        for a in ids:(engineering/'jobs'/(a+'_fresh_00.json')).write_text(json.dumps(dict(benchmark_goal_native=.1)))
        return engineering,reference,root/'new_registration'

    def test_unready_never_generates_hidden_configuration(self):
        with tempfile.TemporaryDirectory() as t:
            e,r,o=self.fixture(Path(t),False)
            with self.assertRaises(ValueError):register(e,r,o,'0'*40)
            self.assertFalse(o.exists())

    def test_missing_physical_readiness_fails_closed(self):
        with tempfile.TemporaryDirectory() as t:
            e,r,o=self.fixture(Path(t),True);p=e/'READINESS_REVIEW.json';d=json.loads(p.read_text());d['cases']['27044']['safe_park']=False;p.write_text(json.dumps(d))
            with self.assertRaises(ValueError):register(e,r,o,'0'*40)
            self.assertFalse(o.exists())

    def test_data_only_registration_is_committed_private_and_non_overwriting(self):
        import contextlib,hashlib,io
        with tempfile.TemporaryDirectory() as t:
            e,r,o=self.fixture(Path(t),True)
            with contextlib.redirect_stdout(io.StringIO()) as stdout:d=register(e,r,o,'0'*40)
            self.assertEqual(len(d['episodes']),8);self.assertFalse(d['execution']['launched'])
            self.assertFalse(d['execution']['registered_launcher_activated'])
            self.assertTrue(all(not x['execution_authorized'] for x in d['episodes']))
            hidden=o/'scene_private/CONFIGURATIONS.json'
            self.assertEqual(d['hidden_configuration_sha256'],hashlib.sha256(hidden.read_bytes()).hexdigest())
            self.assertEqual(hidden.stat().st_mode&0o777,0o400)
            self.assertNotIn('seed',stdout.getvalue());self.assertNotIn('tau_c',stdout.getvalue())
            original=hidden.read_bytes()
            with self.assertRaises(ValueError):register(e,r,o,'0'*40)
            self.assertEqual(original,hidden.read_bytes())


if __name__=='__main__':unittest.main()
