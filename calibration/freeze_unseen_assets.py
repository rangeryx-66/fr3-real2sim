"""Flatten USD composition locally before formal execution; preserve physics."""
import json
from pathlib import Path
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':0,'physics_gpu':0,'multi_gpu':False})
from pxr import Usd,UsdGeom
ROOT=Path(__file__).resolve().parents[1];folder=ROOT/'assets/unseen_v1'
inventory=json.loads((folder/'inventory.json').read_text())
for name,spec in inventory.items():
    source=spec.get('source_usd_path',spec['usd_path'])
    stage=Usd.Stage.CreateInMemory();prim=UsdGeom.Xform.Define(stage,'/Asset').GetPrim()
    prim.GetReferences().AddReference(source);stage.SetDefaultPrim(prim)
    path=folder/(name+'_asset.usdc');stage.Flatten().Export(str(path))
    spec.update(source_usd_path=source,usd_path=str(path))
    print('FLATTENED',name,flush=True)
(folder/'inventory.json').write_text(json.dumps(inventory,indent=2))
app.close()
