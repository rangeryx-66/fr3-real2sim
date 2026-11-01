"""Real temporary Git index checks, before an Isaac actor can be created."""
import subprocess
import tempfile
import unittest
from pathlib import Path
from agentic_interaction_v89.provenance import frozen_controller_commit


class FrozenControllerTests(unittest.TestCase):
    def test_dirty_staged_and_untracked_code_rejected_caches_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def git(*args):
                return subprocess.check_output(['git', *args], cwd=root,
                    stderr=subprocess.DEVNULL).decode().strip()
            git('init', '-q')
            source = root / 'controller.py'
            source.write_text('version = 1\n')
            git('add', 'controller.py')
            git('-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                'commit', '-qm', 'initial')
            expected = git('rev-parse', 'HEAD')
            (root / 'assets').mkdir()
            (root / 'assets/cache_pointer.txt').write_text('/native/cache')
            (root / 'HANDOFF.md').write_text('untracked notes')
            self.assertEqual(frozen_controller_commit(root), expected)
            source.write_text('version = 2\n')
            with self.assertRaisesRegex(ValueError, 'SOURCE_NOT_COMMITTED'):
                frozen_controller_commit(root)
            git('add', 'controller.py')
            with self.assertRaisesRegex(ValueError, 'SOURCE_NOT_COMMITTED'):
                frozen_controller_commit(root)
            git('reset', '--hard', 'HEAD')
            (root / 'shadow.py').write_text('untracked = True')
            with self.assertRaisesRegex(ValueError, 'UNTRACKED_CONTROLLER_SOURCE'):
                frozen_controller_commit(root)


if __name__ == '__main__':
    unittest.main()
