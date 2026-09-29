"""One RGB-D frame, multiple 6-DoF predictors, one R1/Dex1 filter stack.

Inference and geometry checks only; no robot motion is commanded.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from grasp_compare.adapters import read_candidates
from grasp_compare.collision import Dex1SceneCollision
from grasp_compare.dex1_geometry import Dex1Geometry
from grasp_compare.filters import filter_candidates
from grasp_compare.scene import load_scene

ROOT = Path(__file__).resolve().parent
MODELS = ('graspgenx', 'zerograsp', 'rngnet', 'economicgrasp', 'graspness', 'anygrasp')


def provider_command(name, input_path, output_path, args):
    if name == 'graspgenx':
        python = args.graspgenx_python or os.environ.get('GRASPGENX_PYTHON')
        if not python:
            raise FileNotFoundError('set --graspgenx-python to official GraspGenX .venv/bin/python')
        return [python, str(ROOT / 'src/infer_graspgenx.py'), '--input', str(input_path),
                '--output', str(output_path), '--top-k', str(args.provider_top_k)]
    if name == 'graspness':
        python = args.graspness_python or os.environ.get('GRASPNESS_PYTHON', sys.executable)
        return [python, str(ROOT / 'src/infer_graspness.py'), '--input', str(input_path),
                '--output', str(output_path), '--no-native-collision', '--top-k', str(args.provider_top_k)]
    if name == 'anygrasp':
        python = args.anygrasp_python or os.environ.get('ANYGRASP_PYTHON', sys.executable)
        return [python, str(ROOT / 'src/infer.py'), '--input', str(input_path),
                '--output', str(output_path)]
    if name == 'economicgrasp':
        python = args.economicgrasp_python or os.environ.get('ECONOMICGRASP_PYTHON', sys.executable)
        return [python, str(ROOT / 'src/infer_economicgrasp.py'), '--input', str(input_path),
                '--output', str(output_path), '--top-k', str(args.provider_top_k)]
    if name == 'zerograsp':
        python = args.zerograsp_python or os.environ.get('ZEROGRASP_PYTHON')
        if not python:
            raise FileNotFoundError('set --zerograsp-python to ZeroGrasp .venv/bin/python')
        return [python, str(ROOT / 'src/infer_zerograsp.py'), '--input', str(input_path),
                '--output', str(output_path), '--top-k', str(args.provider_top_k)]
    raise FileNotFoundError(f'{name} native inference adapter/weights are not installed; provide --predictions {name}=FILE')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True, type=Path)
    parser.add_argument('--models', nargs='+', choices=MODELS, default=list(MODELS))
    parser.add_argument('--output-dir', type=Path, default=Path('results/grasp_compare'))
    parser.add_argument('--predictions', action='append', default=[], metavar='MODEL=JSON',
                        help='Precomputed provider output; camera calibration must match scene.')
    parser.add_argument('--graspgenx-python')
    parser.add_argument('--graspness-python')
    parser.add_argument('--anygrasp-python')
    parser.add_argument('--economicgrasp-python')
    parser.add_argument('--zerograsp-python')
    parser.add_argument('--provider-top-k', type=int, default=200)
    parser.add_argument('--top-k', type=int, default=100)
    parser.add_argument('--target-radius-m', type=float, default=.04)
    parser.add_argument('--approach-camera', type=float, nargs=3)
    parser.add_argument('--approach-max-angle-deg', type=float, default=180)
    parser.add_argument('--skip-collision', action='store_true', help='Geometry dependency troubleshooting only')
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--run-ik', action='store_true',
                        help='Use an already running R1 MoveIt /compute_ik service; no execution.')
    parser.add_argument('--ik-python', help='ROS 2 Python interpreter')
    args = parser.parse_args()
    if args.top_k < 1 or args.provider_top_k < args.top_k:
        parser.error('provider-top-k must be >= top-k >= 1')
    supplied = {}
    for item in args.predictions:
        name, sep, path = item.partition('=')
        if not sep or name not in MODELS:
            parser.error(f'invalid --predictions {item}')
        supplied[name] = Path(path)
    scene = load_scene(args.scene)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    input_path = output / 'shared_input.npz'
    np.savez_compressed(input_path, points=scene.points_C.astype(np.float32),
                        mask=(scene.target_mask if scene.target_mask is not None
                              else np.ones(len(scene.points_C), dtype=bool)),
                        T_B_C=scene.T_B_C,
                        rgb=scene.rgb if scene.rgb is not None else np.empty(0),
                        depth_m=scene.depth_m if scene.depth_m is not None else np.empty(0),
                        K=scene.K if scene.K is not None else np.empty(0),
                        mask_image=scene.mask_image if scene.mask_image is not None else np.empty(0))
    geometry = Dex1Geometry()
    checker = None if args.skip_collision else Dex1SceneCollision(scene, geometry)
    report = {'scene': str(scene.path), 'scene_sha256': scene.sha256,
              'shared_input': str(input_path), 'target_points': len(scene.target_C),
              'scene_points': len(scene.points_C), 'dex1': geometry.provenance(),
              'frames': {'points': 'camera_optical', 'candidate': 'T_C_G',
                         'tcp': 'T_B_C @ T_C_G @ T_G_TCP', 'base': 'r1a7_world'},
              'models': {}, 'raw_candidates': {}, 'candidates': {}}
    display = {}
    for name in args.models:
        native = supplied.get(name, output / f'{name}_native.json')
        began = time.monotonic()
        inference_elapsed = None
        if name not in supplied:
            try:
                command = provider_command(name, input_path, native, args)
                completed = subprocess.run(command, capture_output=True, text=True,
                                           timeout=900, check=True)
                (output / f'{name}_inference.log').write_text(completed.stdout + '\n' + completed.stderr)
                inference_elapsed = time.monotonic() - began
            except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
                message = str(error)
                if isinstance(error, subprocess.CalledProcessError):
                    message += '\n' + (error.stderr or '')[-2000:]
                report['models'][name] = {'status': 'UNAVAILABLE', 'reason': message}
                report['raw_candidates'][name] = []
                report['candidates'][name] = []
                print(f'{name}: UNAVAILABLE: {message}', flush=True)
                continue
        try:
            all_native = read_candidates(native, name, scene)
            raw = sorted(all_native, key=lambda c: c.score, reverse=True)[:args.provider_top_k]
            native_info = {}
            if native.suffix == '.json':
                document = json.loads(native.read_text())
                native_info = {k: v for k, v in document.items()
                               if k not in ('grasps', 'T_B_C')}
            filter_started = time.monotonic()
            if checker is None:
                for c in raw:
                    c.checks['dex1_scene_collision'] = {'status': 'NOT_RUN'}
                selected = raw[:args.top_k]
                counts = {'raw': len(raw), 'collision_free': None}
            else:
                selected, counts = filter_candidates(raw, scene, checker,
                    target_radius_m=args.target_radius_m,
                    approach_C=args.approach_camera,
                    approach_max_angle_deg=args.approach_max_angle_deg,
                    max_candidates=args.top_k)
            report['models'][name] = {'status': 'OK', 'native_file': str(native),
                'native_file_sha256': hashlib.sha256(native.read_bytes()).hexdigest(),
                'provider_metadata': native_info,
                'inference_elapsed_s': inference_elapsed,
                'filter_elapsed_s': time.monotonic() - filter_started,
                'input_mode': 'precomputed' if name in supplied else 'fresh_inference',
                'counts': counts, 'saved_candidates': len(selected),
                'native_candidates_total': len(all_native),
                'score_note': 'Scores are native to each model; do not compare numeric scales.'}
            report['candidates'][name] = [c.to_dict() for c in selected]
            report['raw_candidates'][name] = [c.to_dict() for c in raw]
            display[name] = selected
            print(f'{name}: {len(raw)} raw, {len(selected)} saved', flush=True)
        except Exception as error:
            report['models'][name] = {'status': 'ERROR', 'reason': str(error)}
            report['raw_candidates'][name] = []
            report['candidates'][name] = []
            print(f'{name}: ERROR: {error}', flush=True)
    result = output / 'candidates.json'
    result.write_text(json.dumps(report, indent=2))
    if args.run_ik:
        python = args.ik_python or sys.executable
        ik_output = output / 'candidates_with_ik.json'
        subprocess.run([python, str(ROOT / 'src/check_grasp_ik.py'), '--input', str(result),
                        '--output', str(ik_output)], check=True)
        report = json.loads(ik_output.read_text())
        for name, candidates in report['candidates'].items():
            ik_by_rank = {c['rank']: c['checks'].get('ik') for c in candidates}
            for raw in report['raw_candidates'][name]:
                if raw['rank'] in ik_by_rank:
                    raw['checks']['ik'] = ik_by_rank[raw['rank']]
            if name in report['models'] and report['models'][name]['status'] == 'OK':
                report['models'][name]['ik_feasible'] = sum(
                    c['checks'].get('ik', {}).get('kinematic_ik', False) and
                    c['checks'].get('ik', {}).get('self_collision_free', False)
                    for c in candidates)
        ik_output.write_text(json.dumps(report, indent=2))
    if args.visualize:
        from grasp_compare.visualize import save_html
        save_html(output / 'candidates.html', scene, display)
    print(json.dumps({'result': str(result),
                      'models': {k: v.get('status') for k, v in report['models'].items()}}))


if __name__ == '__main__':
    main()
