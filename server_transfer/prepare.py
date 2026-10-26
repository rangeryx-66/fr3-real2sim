"""Copy pinned private job inputs into a NEW directory; never launch physics.

Code/Skill paths are relative to this checkout. Licensed resources and the
canonical shared safety domain stay explicitly configured outside Git.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'agentic-real2sim/server-transfer-v1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pinned(row):
    path = Path(row['path'])
    if not path.is_absolute() or not path.is_file():
        raise ValueError('EXPLICIT_EXISTING_PRIVATE_FILE_REQUIRED')
    if digest(path) != row['sha256']:
        raise ValueError('PRIVATE_TRANSFER_INPUT_CHANGED:' + str(path))
    return json.loads(path.read_text())


def catalog(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema') != SCHEMA or not data.get('cases'):
        raise ValueError('PINNED_PRIVATE_TRANSFER_CATALOG_REQUIRED')
    environment = pinned(data['environment'])
    domain = json.loads((ROOT/'agentic_interaction_v81/ISOLATION_DOMAIN.json').read_text())
    # A checkout relocation must not create a different global resource mutex
    # or pretend an unqualified runtime is the same simulator/hardware domain.
    if (domain.get('execution') != 'isaac_sim_only'
            or environment.get('actor_lock') != domain['actor_lock']
            or environment.get('python') != domain['python']
            or environment.get('simulation_isolation', {}).get('schema') != 'isaac-instance-isolation/v1'):
        raise ValueError('ORIGINAL_SIMULATION_SAFETY_DOMAIN_REQUIRED')
    cases = {name: pinned(row) for name, row in data['cases'].items()}
    for spec in cases.values():
        if spec.get('schema') != 'agentic-interaction/v81' or spec.get('phase') != 'engineering':
            raise ValueError('REGISTERED_ENGINEERING_SPEC_REQUIRED')
        if spec.get('integration_profile') != 'v87_abc_hybrid_recovery_v1':
            raise ValueError('CURRENT_V87_PROFILE_REQUIRED')
    return data, environment, cases, domain


def write_new(path, value):
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    with os.fdopen(os.open(path, flags, 0o600), 'w') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def prepare(catalog_path, case, destination, episode_id,
            physics_gpu=None, perception_gpu=None, grasp_gpu=None):
    data, environment, cases, domain = catalog(catalog_path)
    if case not in cases:
        raise ValueError('UNKNOWN_TRANSFER_CASE')
    spec = cases[case]
    if (not isinstance(episode_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', episode_id)
            or episode_id == spec['episode_id']):
        raise ValueError('NEW_UNIQUE_EPISODE_ID_REQUIRED')
    target = Path(destination)
    if not target.is_absolute():
        raise ValueError('ABSOLUTE_PRIVATE_DESTINATION_REQUIRED')
    target = target.resolve()
    parent = Path(domain['output_parent']).resolve()
    if (target == parent or not target.is_relative_to(parent)
            or target.is_relative_to(ROOT.resolve()) or target.exists()):
        raise ValueError('NEW_DIRECTORY_IN_ORIGINAL_PRIVATE_OUTPUT_DOMAIN_REQUIRED')
    for value in (physics_gpu, perception_gpu, grasp_gpu):
        if value is not None and (type(value) is not int or value < 0):
            raise ValueError('NONNEGATIVE_EXPLICIT_GPU_REQUIRED')
    if grasp_gpu is not None and 'graspgenx' not in spec:
        raise ValueError('CASE_HAS_NO_REGISTERED_GRASPGENX_RESOURCE')
    environment['output_root'] = str(target/'episodes')
    spec['episode_id'] = episode_id
    if physics_gpu is not None:
        environment['physics_gpu'] = physics_gpu
    if perception_gpu is not None:
        environment['perception_gpu'] = perception_gpu
    if grasp_gpu is not None:
        spec['graspgenx']['gpu'] = grasp_gpu
    # Input checking precedes writes. No lock, incident, calibration, plant,
    # controller, runtime, or external resource is modified by this helper.
    target.mkdir(mode=0o700, parents=True, exist_ok=False)
    (target/'episodes').mkdir(mode=0o700)
    write_new(target/'environment.json', environment)
    write_new(target/'spec.json', spec)
    receipt = dict(schema=SCHEMA, case=case, episode_id=episode_id,
        repository=str(ROOT), catalog_sha256=digest(catalog_path),
        original_environment=data['environment'], original_spec=data['cases'][case],
        environment_sha256=digest(target/'environment.json'),
        spec_sha256=digest(target/'spec.json'),
        unchanged_safety_domain=domain, no_physics_started=True,
        launch_admission_granted=False, old_episode_resume_authorized=False,
        no_safety_latch_or_shared_resource_modified=True)
    write_new(target/'PREPARED_TRANSFER.json', receipt)
    return dict(directory=str(target), environment=str(target/'environment.json'),
        spec=str(target/'spec.json'), no_physics_started=True,
        launch_admission_granted=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    check = sub.add_parser('check', help='read-only pinned input check; no admission or physics')
    check.add_argument('--catalog', type=Path, required=True)
    build = sub.add_parser('prepare', help='write private copies only; original launcher still required')
    build.add_argument('--catalog', type=Path, required=True)
    build.add_argument('--case', required=True)
    build.add_argument('--destination', type=Path, required=True)
    build.add_argument('--episode-id', required=True)
    build.add_argument('--physics-gpu', type=int)
    build.add_argument('--perception-gpu', type=int)
    build.add_argument('--grasp-gpu', type=int)
    args = parser.parse_args()
    if args.operation == 'check':
        data, environment, cases, domain = catalog(args.catalog)
        result = dict(repository=str(ROOT), cases=sorted(cases),
            shared_actor_lock=environment['actor_lock'], output_parent=domain['output_parent'],
            no_physics_started=True, launch_admission_granted=False)
    else:
        result = prepare(args.catalog, args.case, args.destination, args.episode_id,
            args.physics_gpu, args.perception_gpu, args.grasp_gpu)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
