"""Numerical checks of collision parity and asymmetric closure replay geometry."""
import json
import os
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
import trimesh
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from grasp_compare.dex1_geometry import Dex1Geometry
os.environ['DEX1_COLLISION_PROFILE']='isaac_convex_hull'
g=Dex1Geometry();rng=np.random.default_rng(731);rows=[]
reference=dict(g.meshes_in_tcp(g.open_width))
for q in ((.012,.0107),(.018,.007)):
    direct=dict(g.meshes_in_tcp(g.open_width,q));shifts=g.finger_translations_in_tcp(q)
    for name in ('Link1_2','Link1_3','Link2_2','Link2_3'):
        mesh=reference[name];actual=direct[name]
        error=float(np.max(np.abs(mesh.vertices+shifts[name]-actual.vertices)))
        if error>1e-9:raise RuntimeError('prismatic geometry translation disagrees with direct FK')
        def raycast(m,points):
            r=o3d.t.geometry.RaycastingScene();r.add_triangles(o3d.t.geometry.TriangleMesh(
                o3d.core.Tensor(m.vertices.astype('float32')),o3d.core.Tensor(m.faces.astype('uint32'))))
            return r.compute_signed_distance(o3d.core.Tensor(points.astype('float32'))).numpy()
        points=rng.uniform(actual.bounds[0]-.003,actual.bounds[1]+.003,(1000,3))
        distance_error=float(np.max(np.abs(raycast(actual,points)-raycast(mesh,points-shifts[name]))))
        if distance_error>2e-7:raise RuntimeError('reused collider distances differ')
        rows.append({'finger_q':q,'link':name,'vertex_error_m':error,'signed_distance_error_m':distance_error})
print(json.dumps({'checks':rows,'status':'PASS'},indent=2))
