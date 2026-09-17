"""Build a GT-geometry smoke asset; never used for reconstruction metrics."""
import argparse,json
from pathlib import Path
import cv2,numpy as np,trimesh
from real2sim.collision_mesh import decompose
from real2sim.asset_builder import build

p=argparse.ArgumentParser();p.add_argument('--mesh-npz',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--name',default='smoke_object');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
d=np.load(a.mesh_npz);mesh=trimesh.Trimesh(d['vertices'],d['triangles'],process=True);visual=a.output/'visual.obj';mesh.export(visual)
texture=a.output/'material_0.png';cv2.imwrite(str(texture),np.full((8,8,3),[40,80,220],np.uint8))
inertial=a.output/'inertial.json';inertial.write_text(json.dumps({'mass':.45,'center_of_mass':[0,0,0],'inertia_matrix':np.diag([.0005,.0005,.0002]).tolist(),'estimator':'smoke-only'}))
collision=decompose(visual,a.output/'collision',.05,24)
print(json.dumps(build(a.name,visual,texture,a.output/'collision/collision_manifest.json',inertial,a.output/'asset'),indent=2))
