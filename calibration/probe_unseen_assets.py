"""Export a preregistered Arena asset list, without grasp-dependent selection."""
import argparse, json, hashlib
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--gpu',type=int,default=3);a=p.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':a.gpu,'physics_gpu':a.gpu,'multi_gpu':False})
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.usd

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'assets/unseen_v1';OUT.mkdir(parents=True,exist_ok=True)
PREFIX='https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/'
# Data manifest only: none of these identifiers is used by execution decisions.
SELECTED=['hope/ketchup_bottle','hope/mayonnaise_bottle','hope/milk_carton',
          'hope/yogurt_cup','hot3d/wooden_bowl','hot3d/clay_plates',
          'hot3d/ceramic_mug','fruits_veggies/avocado01',
          'objaverse/red_bell_pepper','ycb/spam_can','fruits_veggies/red_onion']
old=json.loads((ROOT/'assets/arena_complex/inventory.json').read_text())
inventory={'table':old['table']}
stage=omni.usd.get_context().get_stage()
(OUT/'selection.json').write_text(json.dumps({'assets':SELECTED,'excluded_development':list(old),'scale':1.0},indent=2))
for rel in SELECTED:
    name=rel.replace('/','__');path=PREFIX+rel+'.usd'
    prim=add_reference_to_stage(path,'/World/'+name)
    for item in list(Usd.PrimRange(prim)):
        if item.IsInstance():item.SetInstanceable(False)
    vertices=[];triangles=[];bodies=[];colliders=[]
    for item in Usd.PrimRange(prim):
        if item.HasAPI(UsdPhysics.RigidBodyAPI):
            mass=UsdPhysics.MassAPI(item)
            bodies.append(dict(path=str(item.GetPath()),mass_kg=mass.GetMassAttr().Get(),
                               center_of_mass=None if mass.GetCenterOfMassAttr().Get() is None else list(mass.GetCenterOfMassAttr().Get())))
        if item.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append(dict(path=str(item.GetPath()),approximation=UsdPhysics.MeshCollisionAPI(item).GetApproximationAttr().Get()))
        if not item.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(item).ComputeVisibility()=='invisible':continue
        mesh=UsdGeom.Mesh(item);points=mesh.GetPointsAttr().Get()
        if points is None:continue
        T=UsdGeom.Xformable(item).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        offset=len(vertices);vertices.extend([list(T.Transform(Gf.Vec3d(*map(float,x)))) for x in points])
        indices=mesh.GetFaceVertexIndicesAttr().Get();start=0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face=indices[start:start+count];start+=count
            triangles.extend([[offset+face[0],offset+face[j],offset+face[j+1]] for j in range(1,count-1)])
    assert len(vertices)>0 and colliders and len(bodies)==1,(name,len(vertices),bodies)
    if bodies[0]['path']!='/World/'+name:
        (OUT/(name+'_excluded.json')).write_text(json.dumps(dict(reason='NESTED_RIGID_ROOT_UNSUPPORTED_BY_EXISTING_CONTACT_BRIDGE',usd_path=path,bodies=bodies),indent=2))
        print('ASSET_EXCLUDED_BEFORE_GRASP',name,'nested rigid body root',flush=True)
        continue
    v=np.asarray(vertices);meshpath=OUT/(name+'_mesh.npz')
    np.savez_compressed(meshpath,vertices=v,triangles=triangles)
    inventory[name]=dict(usd_path=path,registry_relative=rel,scale=[1,1,1],
                         bounds=[v.min(0).tolist(),v.max(0).tolist()],bodies=bodies,
                         colliders=colliders,vertices=len(v),triangles=len(triangles),
                         mesh_sha256=hashlib.sha256(meshpath.read_bytes()).hexdigest())
    (OUT/'inventory.json').write_text(json.dumps(inventory,indent=2))
    print('ASSET_READY',name,'dimensions',v.ptp(0) if hasattr(v,'ptp') else np.ptp(v,axis=0),flush=True)
app.close()
