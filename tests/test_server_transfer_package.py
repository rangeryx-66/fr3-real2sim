"""Migration must preserve safety bindings and never become a launch path."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from server_transfer import prepare as transfer


class TransferTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.repo = self.base/'different checkout'
        (self.repo/'agentic_interaction_v81').mkdir(parents=True)
        self.domain = dict(execution='isaac_sim_only', actor_lock='/canonical/shared.lock', python='/canonical/python',
            output_parent=str(self.base/'private'))
        self.save(self.repo/'agentic_interaction_v81/ISOLATION_DOMAIN.json', self.domain)
        self.env = dict(python='/canonical/python', actor_lock='/canonical/shared.lock',
            output_root='/original/episodes', physics_gpu=5, perception_gpu=6,
            simulation_isolation=dict(schema='isaac-instance-isolation/v1',
                destroyed_instances=[dict(hold_path='/immutable/old_hold', hold_sha256='a'*64)]))
        self.spec = dict(schema='agentic-interaction/v81', phase='engineering',
            integration_profile='v87_abc_hybrid_recovery_v1', episode_id='historical',
            physics_file='/private/plant.json', physics_sha256='b'*64, staged_exit=True,
            graspgenx=dict(source='/pinned/gx', python='/pinned/gx/python', gpu=0))
        self.envpath = self.base/'environment.json'; self.specpath = self.base/'spec.json'
        self.save(self.envpath, self.env); self.save(self.specpath, self.spec)
        self.index = self.base/'catalog.json'
        self.save(self.index, dict(schema=transfer.SCHEMA,
            environment=self.pin(self.envpath), cases={'46427': self.pin(self.specpath)}))
        self.patch = patch.object(transfer, 'ROOT', self.repo)
        self.patch.start(); self.addCleanup(self.patch.stop)

    def save(self, path, value):
        path.write_text(json.dumps(value))

    def pin(self, path):
        return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    def run_prepare(self, destination=None, **kwargs):
        return transfer.prepare(self.index, '46427', destination or self.base/'private/new',
            kwargs.pop('episode_id', 'fresh'), **kwargs)

    def test_new_checkout_copies_only_metadata_and_preserves_all_guards(self):
        old_env = self.envpath.read_bytes(); old_spec = self.specpath.read_bytes()
        out = self.run_prepare(physics_gpu=3, perception_gpu=4, grasp_gpu=2)
        env = json.loads(Path(out['environment']).read_text())
        spec = json.loads(Path(out['spec']).read_text())
        self.assertEqual(env['actor_lock'], self.env['actor_lock'])
        self.assertEqual(env['simulation_isolation'], self.env['simulation_isolation'])
        self.assertEqual(spec['physics_sha256'], self.spec['physics_sha256'])
        self.assertEqual(spec['graspgenx']['gpu'], 2)
        self.assertFalse(out['launch_admission_granted']); self.assertTrue(out['no_physics_started'])
        self.assertEqual(self.envpath.read_bytes(), old_env); self.assertEqual(self.specpath.read_bytes(), old_spec)
        self.assertEqual(Path(out['directory']).stat().st_mode & 0o777, 0o700)
        self.assertEqual(Path(out['spec']).stat().st_mode & 0o777, 0o600)
        with self.assertRaises(ValueError): self.run_prepare()

    def test_read_only_check_creates_nothing(self):
        before = sorted(str(p) for p in self.base.rglob('*'))
        transfer.catalog(self.index)
        self.assertEqual(before, sorted(str(p) for p in self.base.rglob('*')))

    def test_changed_input_is_rejected_before_writes(self):
        self.envpath.write_text('{}')
        with self.assertRaisesRegex(ValueError, 'INPUT_CHANGED'): self.run_prepare()
        self.assertFalse((self.base/'private/new').exists())

    def test_lock_cannot_be_replaced_with_a_new_resource_domain(self):
        env = dict(self.env, actor_lock='/new/bypass.lock'); self.save(self.envpath, env)
        self.save(self.index, dict(schema=transfer.SCHEMA, environment=self.pin(self.envpath),
            cases={'46427': self.pin(self.specpath)}))
        with self.assertRaisesRegex(ValueError, 'SAFETY_DOMAIN'): self.run_prepare()

    def test_existing_episode_and_outside_private_destination_are_rejected(self):
        for case in [dict(episode_id='historical'), dict(destination=self.base/'outside')]:
            with self.subTest(case=case), self.assertRaises(ValueError): self.run_prepare(**case)

    def test_cannot_write_private_configuration_into_checkout(self):
        self.domain['output_parent'] = str(self.base)
        self.save(self.repo/'agentic_interaction_v81/ISOLATION_DOMAIN.json', self.domain)
        with self.assertRaises(ValueError): self.run_prepare(destination=self.repo/'private')


if __name__ == '__main__': unittest.main()
