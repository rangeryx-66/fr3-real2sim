"""Read-only preflight for the external runtimes and licensed inputs."""
from __future__ import annotations
import argparse
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULTS = {
    'FR3_ASSET_DIRECTORY': ROOT / 'assets/arena_complex',
    'ROS_ENV': ROOT / 'ros_env',
    'MV_SAM3D_ROOT': ROOT.parent / 'MV-SAM3D',
    'ANYGRASP_SDK_ROOT': ROOT.parent / 'anygrasp_sdk/grasp_detection',
    'OFFICIAL_PAYLOAD_ROOT': ROOT / 'third_party/scalable_real2sim_robot_payload_id_upstream_c52e31c',
}

def value(name: str) -> Path:
    raw = os.environ.get(name)
    return Path(raw).expanduser() if raw else Path(DEFAULTS.get(name, '/MISSING/' + name))

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('--stage', choices=['grasp', 'scan', 'mv', 'payload', 'all'], default='all')
    a = p.parse_args()
    requested = set(['grasp', 'scan', 'mv', 'payload'] if a.stage == 'all' else [a.stage])
    checks: list[tuple[str, Path, bool]] = []
    def add(label, path, condition=None):
        path = Path(path)
        checks.append((label, path, path.exists() if condition is None else bool(condition)))
    if 'grasp' in requested or 'payload' in requested or 'scan' in requested:
        add('FR3 description checkout', ROOT/'franka_description/robots/fr3/fr3.urdf.xacro')
        add('FR3 collision mesh', ROOT/'franka_description/meshes/robots/fr3/collision/link0.stl')
        add('expanded URDF', ROOT/'config/fr3.urdf')
        add('Arena asset metadata', value('FR3_ASSET_DIRECTORY')/'inventory.json')
        add('Arena target mesh', value('FR3_ASSET_DIRECTORY')/'soup_mesh.npz')
        add('Isaac Python', value('ISAAC_PYTHON'))
        add('ROS Python', value('ROS_ENV')/'bin/python')
    if 'grasp' in requested or 'payload' in requested:
        sdk = value('ANYGRASP_SDK_ROOT')
        add('AnyGrasp SDK gsnet module', sdk,
            (sdk/'gsnet.py').exists() or any(sdk.glob('gsnet*.so')))
        add('AnyGrasp checkpoint', sdk/'log/checkpoint_detection.tar')
        license_dir = sdk/'license'
        add('AnyGrasp issued license directory', license_dir,
            license_dir.is_dir() and any(license_dir.glob('*.lic')))
        add('AnyGrasp Python', value('ANYGRASP_PYTHON'))
    if 'scan' in requested:
        add('Arena metadata', value('ARENA_METADATA_JSON'))
    if 'mv' in requested:
        add('MV-SAM3D checkout', value('MV_SAM3D_ROOT')/'run_inference_weighted.py')
        add('MV-SAM3D Python', value('MV_SAM3D_PYTHON'))
    if 'payload' in requested:
        add('official robot_payload_id', value('OFFICIAL_PAYLOAD_ROOT')/'robot_payload_id')
        add('official payload Python', value('OFFICIAL_PAYLOAD_PYTHON'))
    for label, path, ok in checks:
        print(f"{'OK' if ok else 'MISSING':7} {label}: {path}")
    return 0 if all(ok for _, _, ok in checks) else 1

if __name__ == '__main__':
    raise SystemExit(main())
