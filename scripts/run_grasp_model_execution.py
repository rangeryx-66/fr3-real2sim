"""Paired Isaac/MoveIt R1-Dex1 grasp execution for model top-20 candidates."""

import argparse
from datetime import datetime, timedelta
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

from run_r1a7_generalization import ROOT, SIM, start, stop, ready

PROVIDERS = ('graspgenx', 'zerograsp', 'economicgrasp', 'graspness', 'anygrasp')
ASSETS = '/data1/home/rangeryx/fr3_moveit_grasp/assets/arena_complex'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inference-dir', type=Path,
                        default=ROOT / 'results/grasp_model_execution')
    parser.add_argument('--output-dir', type=Path,
                        default=ROOT / 'results/grasp_model_execution_trials')
    parser.add_argument('--objects', nargs='*')
    parser.add_argument('--pose-index', type=int, choices=range(1, 6),
                        help='1..5: smoke test one matching pose per object')
    parser.add_argument('--providers', nargs='+', choices=PROVIDERS, default=list(PROVIDERS))
    parser.add_argument('--gpu', type=int, default=5)
    parser.add_argument('--port', type=int, default=18789)
    parser.add_argument('--ros-domain', type=int, default=229)
    parser.add_argument('--manifold-limit', type=int, default=8)
    args = parser.parse_args()
    protocol = json.loads((ROOT / 'ARENA_COMPLEX_PROTOCOL.json').read_text())
    objects = args.objects or protocol['classes']
    if any(obj not in protocol['classes'] for obj in objects):
        parser.error('unknown Arena asset')
    if args.manifold_limit < 1:
        parser.error('manifold-limit must be positive')
    now = datetime.now(ZoneInfo('Asia/Shanghai'))
    deadline = now.replace(hour=5, minute=0, second=0, microsecond=0)
    if deadline <= now:
        deadline += timedelta(days=1)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {'config': {'objects': objects, 'pose_index': args.pose_index,
                         'providers': args.providers, 'gpu': args.gpu,
                         'manifold_limit': args.manifold_limit,
                         'deadline': deadline.isoformat()}, 'objects': {}}
    for obj in objects:
        if (deadline - datetime.now(ZoneInfo('Asia/Shanghai'))).total_seconds() < 240:
            report['objects'][obj] = {'status': 'CUTOFF_05_00'}
            break
        obj_dir = args.output_dir / obj
        obj_dir.mkdir(parents=True, exist_ok=True)
        full_refs = json.loads((ROOT / 'results/r1a7_arena_ab/inputs' / obj /
                                'reference_manifest.json').read_text())
        full_scenarios = json.loads((ROOT / 'results/r1a7_arena_ab/inputs' / obj /
                                     'scenarios.json').read_text())
        indices = [args.pose_index - 1] if args.pose_index else list(range(5))
        refs = [full_refs[i] for i in indices]
        scenarios = [full_scenarios[i] for i in indices]
        refs_file = obj_dir / 'reference_manifest.json'
        scenarios_file = obj_dir / 'scenarios.json'
        refs_file.write_text(json.dumps(refs, indent=2))
        scenarios_file.write_text(json.dumps(scenarios, indent=2))
        env = {**os.environ, 'R1A7_BASE_POSE': '0.329,-0.175,0.237,56.295',
               'R1A7_PEDESTAL_SIZE': '0.10,0.10,0.20',
               'R1A7_PLANT_PORT': str(args.port), 'R1A7_IK_RANDOM_SEEDS': '6',
               'R1A7_MANIFOLD_LIMIT': str(args.manifold_limit),
               'R1A7_ARENA_ASSET_DIR': ASSETS,
               'ROS_DOMAIN_ID': str(args.ros_domain), 'ROS_LOCALHOST_ONLY': '1',
               'NO_PROXY': '127.0.0.1,localhost', 'no_proxy': '127.0.0.1,localhost',
               'OMNI_KIT_ACCEPT_EULA': 'YES', 'ACCEPT_EULA': 'Y'}
        procs = []
        report['objects'][obj] = {}
        try:
            print('OBJECT_START', obj, flush=True)
            procs.append(start([SIM, '-u', str(ROOT / 'src/r1a7_sim_server.py'),
                                '--gpu', str(args.gpu), '--port', str(args.port),
                                '--arena-target', obj], obj_dir / 'sim.log', env))
            procs.append(start(['ros2', 'launch', str(ROOT / 'src/r1a7_moveit.launch.py')],
                               obj_dir / 'moveit.log', env))
            procs.append(start([sys.executable, '-u', str(ROOT / 'src/r1a7_ros_bridge.py')],
                               obj_dir / 'bridge.log', env))
            ready(args.port, obj, procs)
            for provider in args.providers:
                dest = obj_dir / provider
                dest.mkdir(parents=True, exist_ok=True)
                if (dest / 'summary.json').is_file() and all(
                        (dest / f'trial_{i:02d}.json').is_file() for i in range(1, len(refs) + 1)):
                    report['objects'][obj][provider] = json.loads((dest / 'summary.json').read_text())
                    continue
                grasp_dir = dest / 'inputs'
                grasp_dir.mkdir(exist_ok=True)
                for index, ref in enumerate(refs, 1):
                    original = (Path(ref['grasps']) if provider == 'anygrasp' else
                                args.inference_dir / f"seed_{ref['seed']}" / f'{provider}_grasps.json')
                    if not original.is_file():
                        raise FileNotFoundError(original)
                    (grasp_dir / f'trial_{index:02d}_grasps.json').write_bytes(original.read_bytes())
                left = (deadline - datetime.now(ZoneInfo('Asia/Shanghai'))).total_seconds()
                if left < 120:
                    report['objects'][obj][provider] = {'status': 'CUTOFF_05_00'}
                    break
                command = [sys.executable, '-u', str(ROOT / 'scripts/replay_r1a7_model_candidates.py'),
                           '--provider', provider, '--grasps-dir', str(grasp_dir),
                           '--scenarios-json', str(scenarios_file),
                           '--reference-manifest', str(refs_file)]
                with (dest / 'run.log').open('w') as stream:
                    finished = subprocess.run(command, cwd=ROOT,
                                              env={**env, 'R1A7_RUN_DIR': str(dest)},
                                              stdout=stream, stderr=subprocess.STDOUT,
                                              timeout=min(3600, left - 10))
                summary = (json.loads((dest / 'summary.json').read_text())
                           if (dest / 'summary.json').exists() else {})
                summary['exit_code'] = finished.returncode
                report['objects'][obj][provider] = summary
                print('PROVIDER_DONE', obj, provider, json.dumps(summary), flush=True)
                if finished.returncode:
                    raise RuntimeError(f'{obj} {provider} failed; see {dest / "run.log"}')
                (args.output_dir / 'execution_summary.json').write_text(json.dumps(report, indent=2))
        except Exception as error:
            report['objects'][obj]['error'] = repr(error)
            print('OBJECT_ERROR', obj, repr(error), flush=True)
        finally:
            stop(procs)
            time.sleep(3)
            (args.output_dir / 'execution_summary.json').write_text(json.dumps(report, indent=2))
    print('EXECUTION_DONE', json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
