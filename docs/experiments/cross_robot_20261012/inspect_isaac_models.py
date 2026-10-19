"""Isaac URDF/USD structural audit only. No object, World or physics step.
Drive defaults are inspected, never accepted as validated actuator behavior.
"""
import os,json,time
from pathlib import Path
os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');OUT=ROOT/'results/cross_robot_20261012';models=json.loads((OUT/'ROBOT_MODELS.json').read_text());results={}
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':6,'physics_gpu':6,'multi_gpu':False})
try:
 from isaacsim.asset.importer.urdf import URDFImporter,URDFImporterConfig
 from pxr import Usd,UsdPhysics,UsdGeom,PhysxSchema
 for name,c in models.items():
  p=OUT/'models'/name/'isaac_model_only';p.mkdir(exist_ok=True)
  cfg=URDFImporterConfig(urdf_path=c['urdf'],usd_path=str(p),fix_base=True,allow_self_collision=True,merge_fixed_joints=False,joint_drive_type='force',joint_target_type='position')
  usd=URDFImporter(cfg).import_urdf();stage=Usd.Stage.Open(usd);joints=[];bodies=[];cache=UsdGeom.XformCache()
  for prim in stage.Traverse():
   if prim.HasAPI(UsdPhysics.RigidBodyAPI):bodies.append(str(prim.GetPath()))
   if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
    joint=UsdPhysics.Joint(prim);angular=prim.IsA(UsdPhysics.RevoluteJoint);schema=UsdPhysics.RevoluteJoint(prim) if angular else UsdPhysics.PrismaticJoint(prim);drive=UsdPhysics.DriveAPI(prim,'angular' if angular else 'linear');joints.append({'name':prim.GetName(),'path':str(prim.GetPath()),'body0':[str(x) for x in joint.GetBody0Rel().GetTargets()],'body1':[str(x) for x in joint.GetBody1Rel().GetTargets()],'axis':schema.GetAxisAttr().Get(),'lower_native':schema.GetLowerLimitAttr().Get(),'upper_native':schema.GetUpperLimitAttr().Get(),'limit_unit':'degrees' if angular else 'm','stiffness':drive.GetStiffnessAttr().Get(),'damping':drive.GetDampingAttr().Get(),'max_force':drive.GetMaxForceAttr().Get()})
  names=[j['name'] for j in joints];entry={'usd':usd,'arm_joint_name_to_usd_path':{n:next((j['path'] for j in joints if j['name']==n),None) for n in c['arm_joint_names']},'joints':joints,'rigid_bodies':bodies,'all_arm_joints_found':all(n in names for n in c['arm_joint_names']),'dynamic_tensor_order':'not asserted without runtime physics model','physics_steps_executed':0,'physical_episodes_consumed':0,'no_object_scene':True,'import_defaults_not_actuator_validation':True};results[name]=entry;print(name,len(joints),entry['all_arm_joints_found'],flush=True)
 (OUT/'ISAAC_STRUCTURAL_MAPPING.json').write_text(json.dumps(results,indent=2))
finally:app.close()
