"""Prepare matched Arena inference and equivalent TCP poses for R1 replay.

This only runs grasp inference and diagnostics. Isaac execution consumes the
per-provider grasp JSONs separately; original AnyGrasp files stay untouched.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from grasp_compare.candidate import graspnet_to_tcp
from grasp_compare.dex1_geometry import Dex1Geometry

PROVIDERS = ('graspgenx', 'zerograsp', 'economicgrasp', 'graspness')


def provider_grasps(report, name, limit, open_width):
    """Preserve each provider's TCP exactly; change representation if needed."""
    result = []
    standard_tcp = graspnet_to_tcp(0)
    eligible = [candidate for candidate in report['raw_candidates'][name][:100]
                if candidate['checks'].get('target') == 'PASS' and
                candidate['checks'].get('approach') == 'PASS']
    priority = {'FREE': 2, 'LOW_CLEARANCE': 1}
    eligible.sort(key=lambda c: (priority.get(
        c['checks']['dex1_scene_collision']['status'], 0), c['score']), reverse=True)
    for candidate in eligible:
        T_C_TCP = np.asarray(candidate['T_C_TCP'], dtype=float)
        # GraspGenX uses a gripper-base origin rather than GraspNet's grasp
        # centre. Express the identical TCP as a zero-depth GraspNet pose so
        # the unchanged R1 backend can consume it without moving the grasp.
        T_C_G = T_C_TCP @ np.linalg.inv(standard_tcp)
        reconstructed = T_C_G @ standard_tcp
        if not np.allclose(reconstructed, T_C_TCP, atol=1e-8):
            raise AssertionError('provider TCP changed during conversion')
        result.append({'rank': len(result), 'score': candidate['score'],
                       'width': candidate['width_m'] if candidate['width_m'] is not None else open_width,
                       'depth': 0.0, 'rotation': T_C_G[:3, :3].tolist(),
                       'translation': T_C_G[:3, 3].tolist(),
                       'source_rank': candidate['rank'],
                       'source_collision': candidate['checks']['dex1_scene_collision']['status']})
        if len(result) == limit:
            break
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path,
                        default=ROOT / 'results/r1a7_arena_ab/input_manifest.json')
    parser.add_argument('--output-dir', type=Path,
                        default=ROOT / 'results/grasp_model_execution')
    parser.add_argument('--pose-index', type=int, help='1..5: one matched pose per asset')
    parser.add_argument('--limit-scenes', type=int)
    parser.add_argument('--top-k', type=int, default=20)
    parser.add_argument('--gpu', default='3')
    parser.add_argument('--graspgenx-python', required=True)
    parser.add_argument('--zerograsp-python', required=True)
    parser.add_argument('--economicgrasp-python', required=True)
    parser.add_argument('--graspness-python', required=True)
    args = parser.parse_args()
    if args.top_k < 1 or args.top_k > 100:
        parser.error('top-k must be between 1 and 100')
    manifest = json.loads(args.manifest.read_text())
    rows = [r for r in manifest['rows'] if args.pose_index is None or
            (r['seed'] - 1000) % 5 + 1 == args.pose_index]
    if args.limit_scenes is not None:
        rows = rows[:args.limit_scenes]
    open_width = Dex1Geometry().open_width
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = []
    for row in rows:
        seed = row['seed']
        cloud = Path(row['cloud'])
        frozen = Path(row['grasps'])
        if hashlib.sha256(cloud.read_bytes()).hexdigest() != row['cloud_sha256']:
            raise RuntimeError(f'scene hash mismatch: {seed}')
        if hashlib.sha256(frozen.read_bytes()).hexdigest() != row['grasps_sha256']:
            raise RuntimeError(f'AnyGrasp hash mismatch: {seed}')
        scene_dir = args.output_dir / f'seed_{seed}'
        diagnostics = scene_dir / 'diagnostics'
        scene_dir.mkdir(parents=True, exist_ok=True)
        command = [sys.executable, str(ROOT / 'compare_models.py'), '--scene', str(cloud),
                   '--models', *PROVIDERS, 'anygrasp', '--output-dir', str(diagnostics),
                   '--predictions', f'anygrasp={frozen}',
                   '--graspgenx-python', args.graspgenx_python,
                   '--zerograsp-python', args.zerograsp_python,
                   '--economicgrasp-python', args.economicgrasp_python,
                   '--graspness-python', args.graspness_python,
                   '--provider-top-k', '200', '--top-k', '100']
        report_path = diagnostics / 'candidates.json'
        report = json.loads(report_path.read_text()) if report_path.is_file() else None
        reusable = (report is not None and report.get('scene_sha256') == row['cloud_sha256']
                    and report.get('models', {}).get('anygrasp', {}).get('native_file_sha256')
                    == row['grasps_sha256'] and all(
                        report.get('models', {}).get(name, {}).get('status') == 'OK'
                        for name in PROVIDERS))
        if not reusable:
            with (scene_dir / 'inference.log').open('w') as stream:
                completed = subprocess.run(command, cwd=ROOT,
                                           env={**os.environ, 'CUDA_VISIBLE_DEVICES': args.gpu,
                                                'PYTHONPATH': ''},
                                           stdout=stream, stderr=subprocess.STDOUT, timeout=1800)
            if completed.returncode:
                raise RuntimeError(f'inference failed for seed {seed}; see {scene_dir / "inference.log"}')
            report = json.loads(report_path.read_text())
        outputs = {}
        for name in PROVIDERS:
            if report['models'][name]['status'] != 'OK':
                # An inference crash on a valid frozen scene is an end-to-end
                # failure, not a missing trial. Keep it distinct from a model
                # that ran correctly and predicted zero candidates.
                output = scene_dir / f'{name}_grasps.json'
                reason = report['models'][name].get('reason', '')
                output.write_text(json.dumps({'frame': 'camera_optical',
                    'T_B_C': json.loads(frozen.read_text())['T_B_C'],
                    'provider': name, 'source_scene_sha256': row['cloud_sha256'],
                    'inference_status': report['models'][name]['status'],
                    'inference_error': reason[-1000:], 'grasps': []}, indent=2))
                outputs[name] = {'status': report['models'][name]['status'],
                                 'top_k': 0, 'file': str(output), 'reason': reason}
                continue
            grasps = provider_grasps(report, name, args.top_k, open_width)
            output = scene_dir / f'{name}_grasps.json'
            output.write_text(json.dumps({'frame': 'camera_optical',
                'T_B_C': json.loads(frozen.read_text())['T_B_C'],
                'provider': name, 'source_scene_sha256': row['cloud_sha256'],
                'selection_policy': 'native_top100_then_free_low_clearance_collision_then_native_score',
                'grasps': grasps}, indent=2))
            outputs[name] = {'status': 'OK', 'top_k': len(grasps),
                             'file': str(output), 'diagnostic_counts': report['models'][name]['counts']}
        outputs['anygrasp'] = {'status': 'FROZEN', 'top_k': len(json.loads(frozen.read_text())['grasps']),
                              'file': str(frozen),
                              'diagnostic_counts': report['models']['anygrasp']['counts']}
        summary.append({'seed': seed, 'target': row['target'], 'cloud_sha256': row['cloud_sha256'],
                        'anygrasp_sha256': row['grasps_sha256'], 'providers': outputs})
        (args.output_dir / 'inference_summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary[-1]), flush=True)


if __name__ == '__main__':
    main()
