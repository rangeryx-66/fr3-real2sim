"""Read authored/normalized physics and source hashes from saved scene snapshots."""
import json,hashlib
from pathlib import Path
from pxr import Usd,UsdPhysics,UsdShade
ROOT=Path(__file__).resolve().parents[1];assets=ROOT/'assets/arena_complex'
protocol=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text());inventory=json.loads((assets/'inventory.json').read_text());rows={}
for name in protocol['classes']:
    file=assets/f'{name}_scene.usdc'
    if not file.exists():continue
    stage=Usd.Stage.Open(str(file));episode=next(e for e in protocol['episodes'] if e['target']==name)
    for i,obj in enumerate(episode['objects']):
        path='/World/box' if i==0 else f'/World/clutter_{i-1}';prim=stage.GetPrimAtPath(path)
        colliders=[]
        for p in Usd.PrimRange(prim):
            if not p.HasAPI(UsdPhysics.CollisionAPI):continue
            material,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial(materialPurpose='physics');m=UsdPhysics.MaterialAPI(material.GetPrim())
            assert abs(m.GetStaticFrictionAttr().Get()-.8)<1e-6 and abs(m.GetDynamicFrictionAttr().Get()-.7)<1e-6
            colliders.append(dict(path=str(p.GetPath()),approximation=UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get(),static_friction=m.GetStaticFrictionAttr().Get(),dynamic_friction=m.GetDynamicFrictionAttr().Get(),contact_offset=p.GetAttribute('physxCollision:contactOffset').Get(),rest_offset=p.GetAttribute('physxCollision:restOffset').Get()))
        rows[obj['asset']]=dict(inventory=inventory[obj['asset']],mass_kg=UsdPhysics.MassAPI(prim).GetMassAttr().Get(),colliders=colliders)
    table=stage.GetPrimAtPath('/World/table/table');rb=UsdPhysics.RigidBodyAPI(table)
    assert all(v==0 for v in rb.GetVelocityAttr().Get()) and all(v==0 for v in rb.GetAngularVelocityAttr().Get())
out=ROOT/'results/arena_complex_summary';out.mkdir(exist_ok=True,parents=True)
(out/'asset_physics.json').write_text(json.dumps(dict(assets=rows,hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in assets.iterdir() if p.is_file()}),indent=2))
print(json.dumps({n:r['mass_kg'] for n,r in rows.items()}))
