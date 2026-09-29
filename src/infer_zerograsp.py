"""Run the official ZeroGrasp checkpoint on one calibrated RGB-D frame.

Its prediction uses the target mask. All models then share the same Dex1
collision and R1 IK checks in compare_models.py. No robot is commanded.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import types

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--source', type=Path, default=Path(os.environ.get(
        'ZEROGRASP_ROOT', '/data1/home/rangeryx/ZeroGrasp')))
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--top-k', type=int, default=200)
    args = parser.parse_args()
    source = args.source.resolve()
    checkpoint = (args.checkpoint or source / 'checkpoints/mirage.ckpt').resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)

    import imageio.v3 as iio
    import torch
    import torch.nn.functional as F
    import yaml
    from graspnetAPI import GraspGroup

    started = time.monotonic()
    with np.load(args.input) as data:
        rgb = np.asarray(data['rgb'])
        depth_m = np.asarray(data['depth_m'])
        mask = np.asarray(data['mask_image'])
        K = np.asarray(data['K'], dtype=float)
        T_B_C = np.asarray(data['T_B_C']).tolist()
    if (rgb.ndim != 3 or rgb.shape[:2] != depth_m.shape or
            mask.shape != depth_m.shape or K.shape != (3, 3)):
        raise ValueError('ZeroGrasp requires aligned RGB, depth, target mask and 3x3 K')
    if not torch.cuda.is_available():
        raise RuntimeError('Official ZeroGrasp inference requires CUDA')

    # Upstream fetch_data expects integer depth in millimetres and a .yml
    # camera file. Its .json branch has an unmatched else and always raises.
    work = args.output.parent / 'zerograsp_input'
    work.mkdir(parents=True, exist_ok=True)
    rgb_path = work / 'rgb.png'
    depth_path = work / 'depth.png'
    mask_path = work / 'mask.png'
    camera_path = work / 'camera.yml'
    iio.imwrite(rgb_path, np.asarray(rgb[..., :3], dtype=np.uint8))
    iio.imwrite(depth_path, np.clip(np.nan_to_num(depth_m) * 1000, 0, 65535).astype(np.uint16))
    iio.imwrite(mask_path, (mask.astype(bool) & (depth_m > .01)).astype(np.uint8))
    P = np.column_stack((K, np.zeros(3)))
    camera_path.write_text(yaml.safe_dump({'left_p': P.reshape(-1).tolist(),
                                           'width': int(depth_m.shape[1]),
                                           'rectified_width': int(depth_m.shape[1])}))

    sys.path.insert(0, str(source))
    original_argv = sys.argv
    original_cwd = Path.cwd()
    try:
        os.chdir(source)
        sys.argv = ['zerograsp', '--config', str(source / 'configs/demo.yaml'),
                    '--checkpoint', str(checkpoint)]
        from main import BaseTrainer
        from zerograsp.utils.config import parse_config
        from zerograsp.utils.dataset import fetch_data
        from zerograsp.utils.math import unnormalize_pts, rotation_6d_to_matrix
        from zerograsp.nets.utils import get_xyz_from_octree
        config = parse_config()
        # parse_config retains demo.yaml's 1024x1280 if CLI values equal the
        # parser defaults, so set the actual image dimensions after parsing.
        config.img_height, config.img_width = depth_m.shape
        config.update_octree = True
        model = BaseTrainer.load_from_checkpoint(str(checkpoint), config=config, strict=False)
        model.cuda().eval()
        # ZeroGrasp pins an OFE submodule revision whose forward signature
        # gained batch_start_id/batch_end_id, while the released model still
        # calls the older six-argument form. Bridge that upstream mismatch
        # for this single-frame, single-target inference only.
        ofe = model.model.ofe
        original_ofe_forward = ofe.forward
        def single_target_ofe_forward(self, points, masks, depth, intrinsics, batch_id, grid_size):
            if depth.shape[0] != 1:
                raise ValueError('ZeroGrasp OFE adapter requires one target in one frame')
            object_mask = masks[..., 0].reshape(1, *depth.shape[-2:]).contiguous()
            start = torch.zeros(1, dtype=torch.int32, device=points.device)
            end = torch.ones(1, dtype=torch.int32, device=points.device)
            return original_ofe_forward(points, object_mask, depth, intrinsics,
                                        batch_id, start, end, grid_size)
        ofe.forward = types.MethodType(single_target_ofe_forward, ofe)
        batch = fetch_data(str(rgb_path), str(depth_path), str(mask_path),
                           str(camera_path), config, 1.0)
        with torch.no_grad():
            result = model.model(batch)
            octree = result['octrees_out']
            pcd, batch_id = get_xyz_from_octree(
                octree, config.max_lod, nempty=True, return_batch=True)
            pcd = unnormalize_pts(pcd, batch[-2][0], config.grid_size, 1 << config.min_lod)
            normals = F.normalize(octree.normals[config.max_lod], dim=-1)
            signal = octree.features[config.max_lod]
            pcd = (pcd - normals * signal[:, :1]).cpu().numpy()
            batch_id = batch_id.cpu().numpy()
            arrays = []
            for index, _ in enumerate(torch.unique(batch[3][0].labels, sorted=True)):
                selected = batch_id == index
                if not selected.any():
                    continue
                features = signal[selected, 1:]
                quality = features[:, :1].cpu().numpy()
                rotation = rotation_6d_to_matrix(
                    torch.cat([-features[:, 5:8], features[:, 2:5]], dim=-1)).cpu().numpy()
                width = np.clip(features[:, 9:10].cpu().numpy() * .1, 0, .1)
                depth = np.clip(features[:, 8:9].cpu().numpy() * .04, 0, .04)
                arrays.append(np.concatenate((quality, width, np.full_like(quality, .02),
                    depth, rotation.reshape(-1, 9), pcd[selected] / 1000,
                    -np.ones_like(quality)), axis=1))
    finally:
        sys.argv = original_argv
        os.chdir(original_cwd)
    predictions = np.concatenate(arrays) if arrays else np.empty((0, 17))
    finite = np.all(np.isfinite(predictions), axis=1)
    group = GraspGroup(predictions[finite]).nms(.03, np.deg2rad(30)).sort_by_score()
    grasps = [{'rank': index, 'score': float(grasp.score),
               'width': float(grasp.width), 'depth': float(grasp.depth),
               'rotation': grasp.rotation_matrix.tolist(),
               'translation': grasp.translation.tolist()}
              for index, grasp in enumerate(group[:args.top_k])]
    commit = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True, check=False).stdout.strip()
    args.output.write_text(json.dumps({'provider': 'zerograsp', 'source_commit': commit,
        'checkpoint': str(checkpoint), 'frame': 'camera_optical', 'T_B_C': T_B_C,
        'grasps': grasps}, indent=2))
    print(json.dumps({'model': 'zerograsp', 'predicted': len(predictions),
                      'nms_candidates': len(group), 'saved': len(grasps),
                      'elapsed_s': time.monotonic() - started}))


if __name__ == '__main__':
    main()
