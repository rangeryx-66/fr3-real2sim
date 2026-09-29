"""Convert graspness_unofficial predictions to the existing frozen-grasp JSON schema.

The upstream model and checkpoint live outside this repository. The source
point cloud and target mask are the same inputs used by src/infer.py.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source', type=Path, default=Path(os.environ.get(
        'GRASPNESS_ROOT', '/data1/home/rangeryx/graspness_unofficial')))
    parser.add_argument('--checkpoint', type=Path, default=None)
    parser.add_argument('--top-k', type=int, default=20)
    parser.add_argument('--num-point', type=int, default=15000)
    parser.add_argument('--no-native-collision', action='store_true',
                        help='Keep proposals for the shared official Dex1 collision filter.')
    parser.add_argument('--target-context-m', type=float, default=0.0,
                        help='Sample the scene within this many metres of the target bbox; 0 uses the full scene.')
    parser.add_argument('--seed', type=int, default=20260929)
    args = parser.parse_args()
    source = args.source.resolve()
    checkpoint = args.checkpoint or source / 'minkuresunet_realsense.tar'
    sys.path.insert(0, str(source))
    sys.path.insert(0, str(source / 'utils'))

    import torch
    from scipy.spatial import cKDTree
    from graspnetAPI.graspnet_eval import GraspGroup
    from models.graspnet import GraspNet, pred_decode
    from dataset.graspnet_dataset import minkowski_collate_fn
    from collision_detector import ModelFreeCollisionDetector

    started = time.monotonic()
    with np.load(args.input) as data:
        all_points = np.asarray(data['points'], dtype=np.float32)
        target_mask = np.asarray(data['mask'], dtype=bool)
        T_B_C = np.asarray(data['T_B_C']).tolist()
    valid = np.all(np.isfinite(all_points), axis=1) & (all_points[:, 2] > 0)
    cloud = all_points[valid]
    target = all_points[valid & target_mask]
    if len(target) < 30 or not len(cloud):
        raise RuntimeError('NO_GRASP: empty scene or target mask')
    rng = np.random.default_rng(args.seed)
    network_cloud = cloud
    if args.target_context_m > 0:
        lower = target.min(axis=0) - args.target_context_m
        upper = target.max(axis=0) + args.target_context_m
        network_cloud = cloud[np.all((cloud >= lower) & (cloud <= upper), axis=1)]
    sample = network_cloud[rng.choice(len(network_cloud), args.num_point,
                                      replace=len(network_cloud) < args.num_point)]
    sample = np.ascontiguousarray(sample, dtype=np.float32)
    features = np.ones_like(sample)
    batch = minkowski_collate_fn([{'point_clouds': sample, 'coors': sample / .005,
                                   'feats': features}])
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    model = GraspNet(seed_feat_dim=512, is_training=False).to(device)
    saved = torch.load(checkpoint, map_location='cpu', weights_only=True)
    model.load_state_dict(saved['model_state_dict'], strict=True)
    model.eval()
    for key, value in batch.items():
        batch[key] = value.to(device) if hasattr(value, 'to') else value
    try:
        with torch.no_grad():
            predictions = pred_decode(model(batch))[0].detach().cpu().numpy()
    except RuntimeError as error:
        if not str(error).startswith('NO_GRASP:'):
            raise
        predictions = np.empty((0, 17), dtype=np.float32)
    count_predicted = len(predictions)
    if count_predicted:
        grasps = GraspGroup(predictions)
        if not args.no_native_collision:
            collision_cloud = cloud if args.target_context_m > 0 else sample
            collision = ModelFreeCollisionDetector(collision_cloud, voxel_size=.01)
            collision_mask = collision.detect(grasps, approach_dist=.05, collision_thresh=.01)
            grasps = grasps[~collision_mask]
        count_collision_free = len(grasps)
        grasps = grasps.nms().sort_by_score()
    else:
        grasps = []
        count_collision_free = 0
    target_tree = cKDTree(target)
    target_min = target.min(axis=0) - .03
    target_max = target.max(axis=0) + .03
    output = []
    for grasp in grasps:
        center = np.asarray(grasp.translation)
        if not np.all((center >= target_min) & (center <= target_max)):
            continue
        if target_tree.query(center)[0] > .04:
            continue
        if not .001 <= grasp.width <= .09:
            continue
        output.append({'rank': len(output), 'score': float(grasp.score),
                       'width': float(grasp.width), 'depth': float(grasp.depth),
                       'rotation': grasp.rotation_matrix.tolist(),
                       'translation': center.tolist()})
        if len(output) >= args.top_k:
            break
    args.output.parent.mkdir(parents=True, exist_ok=True)
    checkpoint_sha256 = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    args.output.write_text(json.dumps({'provider': 'graspness_unofficial',
                                       'source_commit': 'b5abf5aaaf3a797514f89161d8ccc8dfc2ec0eca',
                                       'checkpoint_sha256': checkpoint_sha256,
                                       'frame': 'camera_optical', 'T_B_C': T_B_C,
                                       'grasps': output}, indent=2))
    print(json.dumps({'provider': 'graspness_unofficial',
                      'source_commit': 'b5abf5aaaf3a797514f89161d8ccc8dfc2ec0eca',
                      'checkpoint_sha256': checkpoint_sha256,
                      'input': str(args.input), 'predicted': count_predicted,
                      'collision_free': count_collision_free,
                      'native_collision_enabled': not args.no_native_collision,
                      'target_top_k': len(output),
                      'target_context_m': args.target_context_m,
                      'elapsed_s': round(time.monotonic() - started, 2)}), flush=True)


if __name__ == '__main__':
    main()
