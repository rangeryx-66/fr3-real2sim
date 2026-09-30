"""Explicit conservative proxies equivalent to Isaac's imported convex hulls.

Only collision mesh filenames change. Unitree source meshes, origins, scale,
kinematics, visuals, and contact permissions are preserved.
"""
import os
import xml.etree.ElementTree as ET
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
PROFILE = 'isaac_convex_hull'

def enabled():
    value = os.environ.get('DEX1_COLLISION_PROFILE', 'official_stl')
    if value not in ('official_stl', PROFILE):
        raise ValueError(value)
    return value == PROFILE

def proxy_path(link):
    return ROOT / 'assets/dex1_collision_hulls' / (link.removeprefix('dex1_') + '.stl')

def apply_profile(model):
    if enabled():
        for link in model.findall('link'):
            if not link.get('name', '').startswith('dex1_'):
                continue
            for mesh in link.findall('collision/geometry/mesh'):
                path = proxy_path(link.get('name'))
                if not path.is_file():
                    raise FileNotFoundError(f'{path}: run scripts/prepare_dex1_collision_hulls.py')
                mesh.set('filename', str(path))
    return model
