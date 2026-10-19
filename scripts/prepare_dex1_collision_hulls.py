"""Derive full-size collision hulls; never modify the official assets."""
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from dex1_collision_profile import proxy_path
source=ROOT/'third_party/unitree_ros/robots/dexterous_hand_description/dex1_1/dex1_1.urdf'
rows=[]
for link in ET.parse(source).getroot().findall('link'):
    for collision in link.findall('collision'):
        node=collision.find('geometry/mesh')
        if node is None: continue
        path=(source.parent/node.get('filename')).resolve()
        mesh=trimesh.load(path,force='mesh',process=True); hull=mesh.convex_hull
        if not np.allclose(mesh.bounds,hull.bounds,atol=1e-9):raise RuntimeError('hull bounds changed')
        dest=proxy_path(link.get('name'));dest.parent.mkdir(parents=True,exist_ok=True);hull.export(dest)
        rows.append(dict(link=link.get('name'),source=str(path.relative_to(ROOT)),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                         proxy_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),origin=collision.find('origin').attrib if collision.find('origin') is not None else {},
                         scale=node.get('scale','1 1 1'),bounds=mesh.bounds.tolist(),source_volume_m3=float(mesh.volume),hull_volume_m3=float(hull.volume),
                         source_watertight=bool(mesh.is_watertight),hull_vertices=len(hull.vertices)))
(ROOT/'assets/dex1_collision_hulls/manifest.json').write_text(json.dumps({'profile':'isaac_convex_hull','links':rows},indent=2))
print(json.dumps(rows,indent=2))
