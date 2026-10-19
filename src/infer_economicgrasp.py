"""EconomicGrasp RealSense checkpoint on the shared camera point cloud."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--source', type=Path, default=Path(os.environ.get(
        'ECONOMICGRASP_ROOT', '/data1/home/rangeryx/EconomicGrasp')))
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--num-point', type=int, default=20000)
    parser.add_argument('--top-k', type=int, default=200)
    parser.add_argument('--seed', type=int, default=20260929)
    args = parser.parse_args()
    source = args.source.resolve()
    checkpoint = args.checkpoint or source / 'checkpoints/economicgrasp_realsense.tar'
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    sys.path.insert(0, str(source))
    sys.path.insert(0, str(source / 'libs/pointnet2'))
    # Upstream utils.arguments parses argv at import time. Supply only its
    # required dataset/camera values; the dataset is never opened in this run.
    original_argv = sys.argv
    sys.argv = ['economicgrasp_inference', '--dataset_root', '/unused', '--camera', 'realsense']
    try:
        from models.economicgrasp import economicgrasp, pred_decode
    finally:
        sys.argv = original_argv
    import torch
    from scipy.spatial import cKDTree
    from graspnetAPI import GraspGroup

    started = time.monotonic()
    with np.load(args.input) as data:
        points = np.asarray(data['points'], dtype=np.float32)
        target_mask = np.asarray(data['mask'], dtype=bool)
        T_B_C = np.asarray(data['T_B_C']).tolist()
    valid = np.all(np.isfinite(points), axis=1) & (points[:, 2] > 0)
    cloud = points[valid]
    target = points[valid & target_mask]
    if len(target) < 30:
        raise RuntimeError('target has fewer than 30 points')
    rng = np.random.default_rng(args.seed)
    sample = np.ascontiguousarray(cloud[rng.choice(
        len(cloud), args.num_point, replace=len(cloud) < args.num_point)])
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    torch.manual_seed(args.seed)
    net = economicgrasp(seed_feat_dim=512, is_training=False).to(device)
    saved = torch.load(checkpoint, map_location='cpu', weights_only=False)
    net.load_state_dict(saved['model_state_dict'], strict=True)
    net.eval()
    batch = {'point_clouds': torch.from_numpy(sample[None]).to(device),
             'coordinates_for_voxel': torch.from_numpy((sample / .005)[None]).float()}
    with torch.no_grad():
        predictions = pred_decode(net(batch))[0].detach().cpu().numpy()
    grasps = GraspGroup(predictions).nms().sort_by_score()
    tree = cKDTree(target)
    out = []
    for grasp in grasps:
        center = np.asarray(grasp.translation)
        if tree.query(center)[0] > .04 or not .001 <= grasp.width <= .09:
            continue
        out.append({'rank': len(out), 'score': float(grasp.score),
                    'width': float(grasp.width), 'depth': float(grasp.depth),
                    'rotation': grasp.rotation_matrix.tolist(),
                    'translation': center.tolist()})
        if len(out) >= args.top_k:
            break
    args.output.parent.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True, check=False).stdout.strip()
    args.output.write_text(json.dumps({'provider': 'economicgrasp', 'source_commit': commit,
                                       'checkpoint': str(checkpoint),
                                       'frame': 'camera_optical', 'T_B_C': T_B_C,
                                       'grasps': out}, indent=2))
    print(json.dumps({'model': 'economicgrasp', 'predicted': len(predictions),
                      'target_candidates': len(out), 'elapsed_s': time.monotonic() - started}))


if __name__ == '__main__':
    main()
