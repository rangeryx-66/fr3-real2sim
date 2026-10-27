"""Verified URI-to-local migration of the SAME stock Isaac ground asset.

The SDK's remote directory stat can fail although its actual files exist.
No USD is rewritten. The SDK still authors its original ground/materials.
"""
import hashlib
import json
from pathlib import Path
from contextlib import contextmanager
from unittest.mock import patch


def verify(declaration):
    if not isinstance(declaration, dict) or set(declaration) != {'path', 'sha256'}:
        raise ValueError('EXPLICIT_NATIVE_ASSET_MIRROR_REQUIRED')
    path = Path(declaration['path'])
    if not path.is_absolute() or hashlib.sha256(path.read_bytes()).hexdigest() != declaration['sha256']:
        raise ValueError('NATIVE_ASSET_MANIFEST_COMMITMENT_MISMATCH')
    manifest = json.loads(path.read_text()); root = path.parent
    if (manifest.get('schema') != 'native-asset-path-migration/v1'
            or not manifest.get('original_sdk_root_uri')
            or 'Isaac/Environments/Grid/default_environment.usd' not in manifest['files']):
        raise ValueError('NATIVE_GROUND_ASSET_REQUIRED')
    for name, expected in manifest['files'].items():
        file = (root / name).resolve()
        if not file.is_relative_to(root.resolve()) or hashlib.sha256(file.read_bytes()).hexdigest() != expected:
            raise ValueError('NATIVE_ASSET_BYTES_CHANGED:' + name)
    return root, manifest


@contextmanager
def constructor(job):
    declaration = job.get('v88_native_assets')
    if declaration is None:
        yield; return
    root, manifest = verify(declaration)
    import isaacsim
    original = isaacsim.SimulationApp
    def app(*args, **kwargs):
        value = original(*args, **kwargs)
        import carb
        settings = carb.settings.get_settings()
        key = '/persistent/isaac/asset_root/default'
        actual = settings.get(key)
        if actual != manifest['original_sdk_root_uri']:
            value.close()
            raise ValueError('SDK_NATIVE_ASSET_VERSION_MISMATCH')
        settings.set_string(key, str(root))
        from agentic_interaction_v8.resources import dump
        dump(Path(job['output']) / 'V88_NATIVE_ASSET_PATH_MIGRATION.json', dict(
            manifest=declaration, original_root_uri=actual, relocated_root=str(root),
            every_registered_file_byte_verified=True, USD_and_physics_fields_rewritten=False,
            SDK_ground_constructor_unchanged=True))
        return value
    with patch.object(isaacsim, 'SimulationApp', app): yield
