import ast,hashlib,json,tempfile,unittest
from pathlib import Path
from scripts.prismatic_registration_v74 import verify,verify_job

ROOT=Path(__file__).resolve().parents[1]

class ActivationTest(unittest.TestCase):
    def test_registered_constructor_preserves_frozen_initialization_body(self):
        def body(path,name):
            tree=ast.parse((ROOT/path).read_text());c=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
            f=next(n for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
            return f.body
        old=body('interaction_runtime/prismatic_v73.py','PrismaticRuntimeV73')
        new=body('interaction_runtime/registered_prismatic_v74.py','RegisteredPrismaticRuntimeV74')
        self.assertEqual([ast.dump(n) for n in old[2:]],[ast.dump(n) for n in new[2:]])
        self.assertIn('verify_job',ast.dump(new[0]));self.assertIn('PrismaticPreloadRuntime',ast.dump(new[1]))

    def test_exact_original_owners_are_inherited(self):
        from interaction_runtime.prismatic_v73 import PrismaticRuntimeV73,serve_v73
        from interaction_runtime.registered_prismatic_v74 import RegisteredPrismaticRuntimeV74,serve_v73 as registered_serve
        self.assertIs(PrismaticRuntimeV73.step,RegisteredPrismaticRuntimeV74.step)
        self.assertIs(serve_v73,registered_serve)

    def fixture(self,t):
        r=Path(t);(r/'source.py').write_text('unchanged');private=r/'private.json';private.write_text('[]')
        m=dict(schema='prismatic-v74-preregistration',source_sha256={'source.py':hashlib.sha256((r/'source.py').read_bytes()).hexdigest()},hidden_configuration_sha256=hashlib.sha256(private.read_bytes()).hexdigest(),episodes=[dict(episode_id='slot'+str(i),configuration_index=i) for i in range(8)])
        p=r/'registration.json';p.write_text(json.dumps(m));return r,p,private,m

    def test_frozen_source_and_hidden_commitments_fail_closed(self):
        with tempfile.TemporaryDirectory() as t:
            r,p,h,m=self.fixture(t);verify(r,p,h)
            (r/'source.py').write_text('changed')
            with self.assertRaisesRegex(ValueError,'source mismatch'):verify(r,p,h)
            (r/'source.py').write_text('unchanged');h.write_text('[1]')
            with self.assertRaisesRegex(ValueError,'Hidden commitment'):verify(r,p,h)

    def test_order_and_engineering_relabelling_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            r,p,h,m=self.fixture(t);m['episodes'].reverse();p.write_text(json.dumps(m))
            with self.assertRaisesRegex(ValueError,'order'):verify(r,p,h)
            with self.assertRaisesRegex(ValueError,'registered V7.4'):verify_job(r,dict(experiment='V7',run_classification='PRISMATIC_ENGINEERING'))

    def test_atomic_publication_ack_and_bad_command_are_recoverable(self):
        from concurrent.futures import ThreadPoolExecutor
        from scripts.agent_command_protocol_v12 import publish,receive,complete
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);inbox=p/'commands';responses=p/'responses';responses.mkdir()
            def send(_):
                try:publish(inbox,0,dict(operation='inspect'));return True
                except FileExistsError:return False
            with ThreadPoolExecutor(4) as pool:sent=list(pool.map(send,range(4)))
            self.assertEqual(sum(sent),1)
            c,a=receive(inbox/'000.json',0,responses);self.assertEqual(a['status'],'ACCEPTED')
            complete(responses,0,dict(result=dict(unloaded=True)))
            self.assertEqual(json.loads((responses/'000.json').read_text())['status'],'COMPLETED')
            (inbox/'001.json').write_text('{bad');c,a=receive(inbox/'001.json',1,responses)
            self.assertIsNone(c);self.assertTrue(a['recoverable']);self.assertFalse(a['robot_action_executed'])

if __name__=='__main__':unittest.main()
