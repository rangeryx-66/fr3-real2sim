"""Read provider outputs without changing their native score or pose."""

import json
from pathlib import Path

import numpy as np

from .candidate import GraspCandidate, graspnet_to_tcp, rigid


def read_candidates(path, model, scene):
    path = Path(path)
    if path.suffix == '.npy':
        # Official GraspNetAPI GraspGroup order, used by ZeroGrasp, RNGNet,
        # EconomicGrasp and graspness: score,width,height,depth,R(9),t(3),id.
        array = np.load(path, allow_pickle=False)
        if array.ndim != 2 or array.shape[1] != 17:
            raise ValueError(f'{model}: expected GraspGroup N x 17 array')
        document = {'frame': 'camera_optical', 'T_B_C': scene.T_B_C.tolist(),
                    'grasps': [{'score': float(row[0]), 'width': float(row[1]),
                                'depth': float(row[3]),
                                'rotation': row[4:13].reshape(3, 3).tolist(),
                                'translation': row[13:16].tolist()}
                               for row in array]}
    else:
        document = json.loads(path.read_text())
    if document.get('frame') != 'camera_optical':
        raise ValueError(f'{model}: expected camera_optical poses')
    if 'T_B_C' in document and not np.allclose(rigid(document['T_B_C']), scene.T_B_C, atol=1e-4):
        raise ValueError(f'{model}: camera calibration differs from scene')
    result = []
    for rank, raw in enumerate(document.get('grasps', [])):
        if 'T_C_G' in raw:
            T_C_G = rigid(raw['T_C_G'])
            T_G_TCP = rigid(raw['T_G_TCP']) if 'T_G_TCP' in raw else graspnet_to_tcp(raw.get('depth') or 0)
        else:
            T_C_G = np.eye(4)
            T_C_G[:3, :3] = raw['rotation']
            T_C_G[:3, 3] = raw['translation']
            T_G_TCP = graspnet_to_tcp(raw.get('depth') or 0)
        result.append(GraspCandidate(model, rank, raw['score'], T_C_G, T_G_TCP,
                                     scene.T_B_C, raw.get('width'), raw.get('depth'),
                                     {k: v for k, v in raw.items() if k not in
                                      ('T_C_G', 'T_G_TCP', 'rotation', 'translation', 'score', 'width', 'depth')}))
    return result
