"""Inventory existing Arena assets; export GT meshes without changing their scale."""
import json
import os
from pathlib import Path
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':1,'physics_gpu':1,'multi_gpu':False})
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd
ROOT=Path(__file__).resolve().parents[1]
meta=json.loads(Path(os.environ.get('ARENA_METADATA_JSON','/data1/home/rangeryx/isaaclab-arena/active_real2sim/outputs/maple/metadata.json')).read_text())
selected=['mustard','raisin','hidden_tuna','bowl','banana','sugar','soup','mug']
entries={o['id']:o for o in meta['objects']}
stage=omni.usd.get_context().get_stage()
out=ROOT/'assets/arena_complex';out.mkdir(exist_ok=True,parents=True)
inventory={}
for name in selected+['table']:
    spec=entries[name] if name!='table' else dict(usd_path=meta['background_usd'],registry_name='maple_table_robolab')
    prim=add_reference_to_stage(spec['usd_path'],'/World/'+name)
    for p in list(Usd.PrimRange(prim)):
        if p.IsInstance():p.SetInstanceable(False)
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
    bounds=cache.ComputeWorldBound(prim).ComputeAlignedRange()
    verts=[];faces=[];colliders=[];bodies=[]
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):bodies.append(str(p.GetPath()))
        if p.HasAPI(UsdPhysics.CollisionAPI):colliders.append(dict(path=str(p.GetPath()),approx=UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()))
        if p.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible':
            mesh=UsdGeom.Mesh(p);points=mesh.GetPointsAttr().Get()
            if points is None:continue
            M=UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            v=np.array([M.Transform(Gf.Vec3d(*map(float,x))) for x in points]);offset=len(verts);verts.extend(v.tolist())
            counts=mesh.GetFaceVertexCountsAttr().Get();indices=mesh.GetFaceVertexIndicesAttr().Get();k=0
            for n in counts:
                f=indices[k:k+n];k+=n
                for j in range(1,n-1):faces.append([offset+f[0],offset+f[j],offset+f[j+1]])
    np.savez_compressed(out/(name+'_mesh.npz'),vertices=verts,triangles=faces)
    inventory[name]=dict(registry_name=spec['registry_name'],usd_path=spec['usd_path'],bounds=[list(bounds.GetMin()),list(bounds.GetMax())],bodies=bodies,colliders=colliders,vertices=len(verts),triangles=len(faces))
    print('ASSET',name,json.dumps(inventory[name]),flush=True)
    (out/'inventory.json').write_text(json.dumps(inventory,indent=2))
app.close()
