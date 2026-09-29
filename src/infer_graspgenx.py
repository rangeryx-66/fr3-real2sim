"""GraspGenX diffusion candidates conditioned on official Unitree Dex1-1 geometry.

Run this in GraspGenX's own Python environment. The model source and released
checkpoints live outside the project repository. No robot commands are sent.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from grasp_compare.dex1_geometry import Dex1Geometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--source', type=Path, default=Path(os.environ.get(
        'GRASPGENX_ROOT', '/data1/home/rangeryx/GraspGenX')))
    parser.add_argument('--checkpoint-root', type=Path, default=None)
    parser.add_argument('--planner', choices=('diffusion', 'graspmoe'), default='diffusion')
    parser.add_argument('--num-grasps', type=int, default=200)
    parser.add_argument('--top-k', type=int, default=200)
    parser.add_argument('--seed', type=int, default=20260929)
    parser.add_argument('--max-input-points', type=int, default=32768,
                        help='Deterministic cap before upstream quadratic KNN preprocessing')
    args = parser.parse_args()
    source = args.source.resolve()
    checkpoint = args.checkpoint_root or source / 'ext/graspgenx_checkpoints/release'
    if not source.is_dir() or not (checkpoint / 'gen/config.yaml').is_file() or not (checkpoint / 'dis/config.yaml').is_file():
        raise FileNotFoundError('GraspGenX source and official generator/discriminator checkpoints are required')
    os.environ.setdefault('GRASPGENX_CHECKPOINT_DIR', str(checkpoint.parent))
    os.environ.setdefault('GRASPGENX_GRIPPER_CFG_DIR', str(source / 'assets'))
    sys.path.insert(0, str(source))

    import torch
    from graspgenx.grasp_server import GraspGenXSampler
    from graspgenx.samplers import run_planner_on_object
    from graspgenx.utils.checkpoint_io import load_model_cfg

    started = time.monotonic()
    with np.load(args.input) as data:
        points = np.asarray(data['points'], dtype=np.float32)
        mask = np.asarray(data['mask'], dtype=bool) if 'mask' in data else np.ones(len(points), dtype=bool)
        T_B_C = np.asarray(data['T_B_C']).tolist() if 'T_B_C' in data else np.eye(4).tolist()
    valid = np.all(np.isfinite(points), axis=1) & (points[:, 2] > 0)
    target = np.ascontiguousarray(points[valid & mask])
    if len(target) < 30:
        raise RuntimeError('target region has fewer than 30 usable points')
    original_target_count = len(target)
    if len(target) > args.max_input_points:
        selection = np.random.default_rng(args.seed).choice(
            len(target), args.max_input_points, replace=False)
        target = np.ascontiguousarray(target[selection])
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    dex1 = Dex1Geometry()
    params = dex1.sweep_params()
    cfg = load_model_cfg(str(checkpoint / 'gen'), str(checkpoint / 'dis'), None, None)
    sampler = GraspGenXSampler.from_sweep_volume(cfg, params)
    # The public GraspGenX release conditions on sweep_volume_v2. Collision
    # geometry in the shared post-filter is taken directly from Dex1's URDF.
    grasps, scores, branches, _ = run_planner_on_object(
        target, sampler, planner=args.planner, grasp_threshold=-1.0,
        num_grasps=args.num_grasps, topk_num_grasps=args.top_k)
    order = np.argsort(-np.asarray(scores))[:args.top_k]
    out = [{'rank': i, 'score': float(scores[j]),
            'T_C_G': np.asarray(grasps[j]).tolist(),
            'T_G_TCP': dex1.T_GX_TCP.tolist(), 'width': None, 'depth': None,
            'branch': branches[j]} for i, j in enumerate(order)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True, check=False).stdout.strip()
    args.output.write_text(json.dumps({'provider': 'graspgenx', 'source_commit': commit,
                                       'checkpoint_gen': str(checkpoint / 'gen/epoch_736.pth'),
                                       'checkpoint_dis': str(checkpoint / 'dis/epoch_1056.pth'),
                                       'frame': 'camera_optical', 'T_B_C': T_B_C,
                                       'grasps': out, 'input_target_points': original_target_count,
                                       'sampled_target_points': len(target),
                                       'input_sampling_seed': args.seed,
                                       'max_input_points': args.max_input_points,
                                       'dex1_sweep_params': params,
                                       'dex1_provenance': dex1.provenance()}, indent=2))
    print(json.dumps({'model': 'graspgenx', 'candidates': len(out),
                      'planner': args.planner, 'target_points': len(target),
                      'elapsed_s': round(time.monotonic() - started, 3)}), flush=True)


if __name__ == '__main__':
    main()
