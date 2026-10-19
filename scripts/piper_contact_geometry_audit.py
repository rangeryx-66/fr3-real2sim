"""Export cooked collision data at a previously measured closure state, without motion."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_execute import bootstrap
from piper_mobile_demo.cooked_geometry import export_cooked
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source',type=Path,required=True);p.add_argument('--asset-root',type=Path,required=True);p.add_argument('--measured-report',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--hull-vertex-limit',type=int,help='Optional cooking-only fidelity ablation; never changes source mesh dimensions');p.add_argument('--gpu',type=int,default=6)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
r=json.loads(a.measured_report.read_text());s=bootstrap(a,r['base_final'])
try:
 if a.hull_vertex_limit is not None:
  from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema
  # De-instance only the collider owner in this in-memory diagnostic stage.
  paths=[str(p.GetPath()) for p in Usd.PrimRange.Stage(s['stage'],Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]
  s['world'].stop()
  for path in paths:
   prim=s['stage'].GetPrimAtPath(path);owner=prim
   while owner and not owner.IsInstance():owner=owner.GetParent()
   if owner:owner.SetInstanceable(False)
   prim=s['stage'].GetPrimAtPath(path)
   PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr().Set(a.hull_vertex_limit)
  s['world'].reset()
 q=np.asarray(r['actual_closure']['robot_q']);s['robot'].set_joint_positions(q);s['robot'].set_joint_velocities(np.zeros_like(q))
 # Update USD transforms through one unchanged physics step; report actual q.
 from isaacsim.core.utils.types import ArticulationAction
 s['controller'].apply_action(ArticulationAction(joint_positions=q));s['world'].step(render=True)
 actual=np.asarray(s['robot'].get_joint_positions())
 export_cooked(s['stage'],a.output/'cooked_shapes.json')
 (a.output/'state.json').write_text(json.dumps({'requested_q':q.tolist(),'actual_q':actual.tolist(),'base':r['base_final'],'purpose':'cooking audit; teleport to recorded state, not an execution result','hull_vertex_limit_ablation':a.hull_vertex_limit}))
except BaseException:
 import traceback
 (a.output/'error.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
finally:s['app'].close()
