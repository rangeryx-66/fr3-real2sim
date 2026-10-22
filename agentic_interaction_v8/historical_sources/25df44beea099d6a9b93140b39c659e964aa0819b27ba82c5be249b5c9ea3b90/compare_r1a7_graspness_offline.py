"""Replay sealed Arena clouds through graspness; preserve original AnyGrasp files."""
import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/r1a7_graspness_comparison/inference'
PYTHON = '/data1/home/rangeryx/.conda/envs/anygrasp/bin/python'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--limit', type=int, default=40)
    parser.add_argument('--gpu', default='2')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'results/r1a7_arena_ab/input_manifest.json').read_text())
    OUT.mkdir(parents=True, exist_ok=True)
    report = {'source': 'same sealed FR3 Arena clouds and target masks', 'rows': []}
    for row in manifest['rows'][:args.limit]:
        seed = row['seed']
        cloud = Path(row['cloud'])
        if hashlib.sha256(cloud.read_bytes()).hexdigest() != row['cloud_sha256']:
            raise RuntimeError(f'cloud hash changed: {seed}')
        original_path = Path(row['grasps'])
        original_bytes = original_path.read_bytes()
        if hashlib.sha256(original_bytes).hexdigest() != row['grasps_sha256']:
            raise RuntimeError(f'original AnyGrasp output hash changed: {seed}')
        original = json.loads(original_bytes)
        result_file = OUT / f'seed_{seed}_grasps.json'
        log = OUT / f'seed_{seed}.log'
        result = {'seed': seed, 'target': row['target'],
                  'anygrasp_count': len(original['grasps'])}
        if not result_file.exists():
            command = [PYTHON, '-u', str(ROOT / 'src/infer_graspness.py'),
                       '--input', str(cloud), '--output', str(result_file)]
            env = {**os.environ, 'PYTHONPATH': '', 'LD_LIBRARY_PATH': '',
                   'CUDA_VISIBLE_DEVICES': args.gpu, 'OMP_NUM_THREADS': '8'}
            with log.open('w') as stream:
                try:
                    finished = subprocess.run(command, cwd=ROOT, env=env, stdout=stream,
                                              stderr=subprocess.STDOUT, timeout=180)
                    result['inference_exit_code'] = finished.returncode
                except subprocess.TimeoutExpired:
                    result['inference_exit_code'] = 'TIMEOUT'
        if result_file.exists():
            new = json.loads(result_file.read_text())
            if new['T_B_C'] != original['T_B_C']:
                raise RuntimeError(f'camera transform mismatch: {seed}')
            result['graspness_count'] = len(new['grasps'])
        else:
            result['graspness_count'] = None
        report['rows'].append(result)
        (OUT / 'report.json').write_text(json.dumps(report, indent=2))
        print('INFERENCE_COMPARE', json.dumps(result), flush=True)
    valid = [r for r in report['rows'] if r['graspness_count'] is not None]
    report['summary'] = {'scenes': len(report['rows']), 'valid': len(valid),
                         'anygrasp_nonzero': sum(r['anygrasp_count'] > 0 for r in valid),
                         'graspness_nonzero': sum(r['graspness_count'] > 0 for r in valid),
                         'anygrasp_candidates': sum(r['anygrasp_count'] for r in valid),
                         'graspness_candidates': sum(r['graspness_count'] for r in valid)}
    (OUT / 'report.json').write_text(json.dumps(report, indent=2))
    print('INFERENCE_SUMMARY', json.dumps(report['summary']), flush=True)


if __name__ == '__main__':
    main()
