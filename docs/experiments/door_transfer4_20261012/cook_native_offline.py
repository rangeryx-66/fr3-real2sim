"""Read cached USD and request native hulls. No World, robot or physics steps."""
import sys,json,copy,hashlib,os
from pathlib import Path
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import numpy as np
from piper_mobile_demo.model import Model
from interactive_twin_recovery.mobile import at_base
from articulated_demo.kinematics import transform
out=ROOT/'results/door_transfer4_20261012';prior=ROOT/'results/benchmarkA_30deg_20261011';template=ROOT/'results/45415_precontact_continuity_20261011/physical_01/cooked_initial.json';data=json.loads(template.read_text());template_base=json.loads((ROOT/'results/45415_precontact_continuity_20261011/whole_plan.json').read_text())['base']
os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':1,'physics_gpu':1,'multi_gpu':False})
from pxr import Usd,UsdGeom,Gf,Sdf
from piper_mobile_demo.cooked_geometry import export_cooked
from omni.usd import get_context
for i in ['45134','45443']:
 asset=prior/'discovery_assets'/i;source=json.loads((prior/'visual_inspection'/i/'source/report.json').read_text());model=Model(ROOT/'config/piper.urdf',asset,source);usd=next((asset/'usd').glob('*/'+i+'/'+i+'.usda'));folder=out/i;folder.mkdir(exist_ok=True);get_context().new_stage();stage=get_context().get_stage();root=UsdGeom.Xform.Define(stage,'/World/articulated_pose/asset');root.GetPrim().GetReferences().AddReference(str(usd));root.AddTransformOp().Set(Gf.Matrix4d(model.asset_T.T.tolist()));get_context().get_stage();
 # app.update loads referenced schema/cooking services; there is no timeline
 # play, World or simulator step and no robot in this temporary stage.
 for _ in range(4):app.update()
 export_cooked(stage,folder/'object_native_export.json');obj=json.loads((folder/'object_native_export.json').read_text());combined=copy.deepcopy(at_base(data,template_base,source['robot_base_pose']));combined['shapes']=[x for x in combined['shapes'] if '/articulated_pose/' not in x['path']];fixture=json.loads((prior/'visual_inspection'/i/'initial_scene_private.json').read_text())['fixture']
 for shape in combined['shapes']:
  if 'fixture' in shape['path'].lower() and shape.get('analytic',{}).get('shape')=='box':
   from scipy.spatial.transform import Rotation
   R=Rotation.from_quat(np.roll(fixture['quaternion_wxyz'],-1)).as_matrix();T=np.eye(4);T[:3,:3]=R@np.diag(np.array(fixture['size'])/shape['analytic']['size']);T[:3,3]=fixture['center'];shape['world_transform']=T.tolist();shape['rigid_body_world_transform']=T.tolist()
 combined['shapes']+=obj['shapes'];combined['planning_scene_provenance']={'native_object_usd':str(usd),'cached_usd_sha256':hashlib.sha256(usd.read_bytes()).hexdigest(),'robot_template':str(template),'fixture':'unchanged generic fixture from archived preview metadata','physics_steps_executed':0,'robot_control_executed':False,'collision_editing':False};(folder/'cooked_initial.json').write_text(json.dumps(combined,indent=2));print('OFFLINE_NATIVE',i,len(obj['shapes']),flush=True)
(out/'NATIVE_COOKING_ONLY.json').write_text(json.dumps({'physics_steps':0,'manipulation_episodes':0,'objects':['45134','45443'],'method':'native synchronous hull cooking of unchanged cached USD; planning scene uses frozen robot and archived generic fixture'},indent=2));app.close()
