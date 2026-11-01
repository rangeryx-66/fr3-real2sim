"""Explicit engineering-source admission, distinct from frozen V8 reproduction.

Only named controller replacement seams may differ. All other original
physical, sensing, asset, closure and policy commitments remain byte checked.
Replacement hashes are reported; they are never claimed to be frozen bytes.
"""
import json
from pathlib import Path
from agentic_interaction_v8.resources import digest, verify_bundle
from . import CONTROLLER

REPLACEMENTS = frozenset({
    'agentic_interaction_v8/cli.py',
    'interaction_runtime/runtime.py',
    'interaction_runtime/compact_v2.py',
    'scripts/xarm7_native_gripper.py',
})


def audit(bundle=None, *, controller=CONTROLLER, root=None):
    if controller != CONTROLLER:
        raise ValueError('UNKNOWN_CONTROLLER_REPLACEMENT')
    root = Path(root) if root else Path(__file__).resolve().parents[1]
    failures, replacements = [], {}
    def check(name, expected):
        path = root / name
        if not path.is_file():
            failures.append(name)
            return
        actual = digest(path)
        if actual != expected:
            if name in REPLACEMENTS:
                replacements[name] = dict(frozen_sha256=expected, replacement_sha256=actual)
            else:
                failures.append(name)
    freeze = json.loads((root / 'agentic_interaction_v8/BEHAVIOR.json').read_text())
    release = json.loads((root / 'agentic_interaction_v8/RELEASE_SOURCES.json').read_text())['sha256']
    revolute = json.loads((root / 'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_FREEZE.json').read_text())['source_sha256']
    for sources in (freeze['source_sha256'], release, revolute):
        for name, expected in sources.items():
            check(name, expected)
    for row in freeze['execution_directory_snapshot'].values():
        check(row['snapshot'], row['sha256'])
    for row in freeze['historical_execution_profiles'].values():
        check(row['registration'], row['registration_sha256'])
        for source in row['differing_sources'].values():
            check(source['snapshot'], source['sha256'])
    if bundle:
        data = verify_bundle(bundle)
        external = json.loads((root / 'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_EXTERNAL_FREEZE.json').read_text())['external_robot_camera_closure_uncertainty_sha256']
        for name, expected in external.items():
            if digest(Path(bundle) / data['frozen_external_to_bundle'][name]) != expected:
                failures.append(name)
    if failures:
        raise ValueError('UNDECLARED_PHYSICAL_OR_BASELINE_DRIFT:' + repr(failures))
    return dict(controller=controller, baseline_commit='2f021639aea65e6ae859bac9a3385d49db10212f',
        development_parent='22e652417f5afc9683fdfcca15fc0690d58e34dc',
        frozen_execution_reference='23c73e6e7b05893162f11865ea036a9f86935839',
        physical_source_files=len(freeze['source_sha256']), frozen_revolute_source_files=len(revolute),
        release_source_files=len(release), failures=[], declared_controller_replacements=replacements,
        original_controller_sources_unchanged=False, old_snapshots_unchanged=True)
