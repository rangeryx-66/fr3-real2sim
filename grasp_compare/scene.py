"""Load one frozen point cloud or a calibrated RGB-D capture."""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
import numpy as np

from .candidate import rigid


@dataclass
class Scene:
    path: Path
    points_C: np.ndarray
    target_mask: np.ndarray | None
    T_B_C: np.ndarray
    rgb: np.ndarray | None
    sha256: str
    K: np.ndarray | None = None
    depth_m: np.ndarray | None = None
    mask_image: np.ndarray | None = None

    @property
    def points_B(self):
        return self.points_C @ self.T_B_C[:3, :3].T + self.T_B_C[:3, 3]

    @property
    def target_C(self):
        return self.points_C if self.target_mask is None else self.points_C[self.target_mask]


def _image(path):
    if path.suffix == '.npy':
        return np.load(path)
    from PIL import Image
    return np.asarray(Image.open(path))


def load_scene(path):
    path = Path(path).resolve()
    if path.is_file() and path.suffix == '.npz':
        with np.load(path) as data:
            points = np.asarray(data['points'], dtype=np.float64)
            mask = np.asarray(data['mask'], dtype=bool) if 'mask' in data else None
            T_B_C = rigid(data['T_B_C']) if 'T_B_C' in data else np.eye(4)
            rgb = np.asarray(data['rgb']) if 'rgb' in data else None
            K = np.asarray(data['K'], dtype=float).reshape(3, 3) if 'K' in data else None
            if rgb is not None and rgb.ndim == 3 and points.shape[0] == np.prod(rgb.shape[:2]):
                depth_m = points[:, 2].reshape(rgb.shape[:2]).copy()
                mask_image = mask.reshape(rgb.shape[:2]).copy() if mask is not None else None
            else:
                depth_m = mask_image = None
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    elif path.is_dir():
        depth_path = next((p for name in ('depth.npy', 'depth.png', 'depth.tiff')
                           if (p := path / name).exists()), None)
        if depth_path is None:
            raise FileNotFoundError('scene directory needs depth.npy/png/tiff')
        depth = np.asarray(_image(depth_path), dtype=np.float64)
        meta = json.loads((path / 'camera.json').read_text())
        K = np.asarray(meta['K'], dtype=np.float64).reshape(3, 3)
        scale = float(meta.get('depth_scale_to_m', 1.0))
        z = depth * scale
        v, u = np.indices(z.shape)
        points = np.stack(((u - K[0, 2]) * z / K[0, 0],
                           (v - K[1, 2]) * z / K[1, 1], z), axis=-1).reshape(-1, 3)
        mask_path = next((p for name in ('mask.npy', 'mask.png')
                          if (p := path / name).exists()), None)
        mask = np.asarray(_image(mask_path), dtype=bool).reshape(-1) if mask_path else None
        depth_m = z
        mask_image = mask.reshape(depth.shape) if mask is not None else None
        rgb_path = next((p for name in ('rgb.npy', 'rgb.png', 'rgb.jpg')
                         if (p := path / name).exists()), None)
        rgb = _image(rgb_path) if rgb_path else None
        T_B_C = rigid(meta.get('T_B_C', np.eye(4)))
        h = hashlib.sha256()
        for file in sorted(path.iterdir()):
            if file.is_file():
                h.update(file.name.encode()); h.update(file.read_bytes())
        digest = h.hexdigest()
    else:
        raise ValueError('scene must be a frozen .npz or calibrated RGB-D directory')
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N,3)')
    if mask is not None and mask.shape != (len(points),):
        raise ValueError('mask must align with point cloud')
    valid = np.all(np.isfinite(points), axis=1) & (points[:, 2] > 0)
    if mask is not None:
        mask = mask[valid]
    points = points[valid]
    if len(points) == 0 or (mask is not None and np.count_nonzero(mask) < 30):
        raise ValueError('scene has no usable target point cloud')
    return Scene(path, points, mask, T_B_C, rgb, digest, K, depth_m, mask_image)
