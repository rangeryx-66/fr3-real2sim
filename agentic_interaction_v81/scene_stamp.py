"""Measured collision-state identity, separate from trajectory/grasp utility."""
import hashlib
import json
import numpy as np


def capture(r):
    value=dict(joint_coordinate=float(r.actual()),moving_body_pose=np.asarray(r.moving()).tolist(),
        base=np.asarray(r.base).tolist(),station_generation=r.base_generation,
        grasp_attached=r.grasp_reference is not None)
    raw=json.dumps(value,sort_keys=True,allow_nan=False).encode()
    return dict(sha256=hashlib.sha256(raw).hexdigest(),state=value,
        source='fresh measured articulated body/base/attachment; never closed-state replay')


def same(a,b):return a['sha256']==b['sha256']
