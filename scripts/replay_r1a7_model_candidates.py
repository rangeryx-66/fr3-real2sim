"""Replay precomputed grasp providers through the unchanged R1/Dex1 executor."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import rclpy
import r1a7_backend as backend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--provider', required=True,
                        choices=('graspgenx', 'zerograsp', 'economicgrasp', 'graspness'))
    parser.add_argument('--grasps-dir', required=True, type=Path)
    parser.add_argument('--scenarios-json', required=True, type=Path)
    parser.add_argument('--reference-manifest', required=True, type=Path)
    args = parser.parse_args()
    scenarios = json.loads(args.scenarios_json.read_text())
    references = json.loads(args.reference_manifest.read_text())
    if len(scenarios) != len(references):
        parser.error('scenario/reference count mismatch')
    # Provider is only a reporting/inference selector in the existing
    # backend. Every grasp JSON is supplied explicitly, so no inference or
    # AnyGrasp model code is changed here.
    backend.GRASP_PROVIDER = args.provider
    rclpy.init()
    node = backend.R1A7Backend()
    results = []
    try:
        for index, (scenario, reference) in enumerate(zip(scenarios, references), 1):
            path = args.grasps_dir / f'trial_{index:02d}_grasps.json'
            data = json.loads(path.read_text())
            if data.get('provider') != args.provider:
                raise ValueError(f'{path}: provider mismatch')
            if data.get('source_scene_sha256') != reference['cloud_sha256']:
                raise ValueError(f'{path}: scene hash mismatch')
            results.append(node.trial(index, path, 'adapted', scenario, reference))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    summary = {'provider': args.provider, 'trials': len(results),
               'successes': sum(bool(r.get('success')) for r in results),
               'categories': dict(Counter(r.get('category', 'MISSING') for r in results)),
               'path_valid': sum(r.get('candidate_counts', {}).get('path_valid', 0) for r in results),
               'planning_successes': sum(bool(r.get('planning_succeeded')) for r in results),
               'contact_loss_events': sum(a.get('status') == 'CONTACT_LOSS' or
                                          a.get('initial_contact_failure') == 'CONTACT_LOSS'
                                          for r in results for a in r.get('candidate_attempts', []))}
    out = Path(os.environ['R1A7_RUN_DIR']) / 'summary.json'
    out.write_text(json.dumps(summary, indent=2))
    print('MODEL_REPLAY_SUMMARY', json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
