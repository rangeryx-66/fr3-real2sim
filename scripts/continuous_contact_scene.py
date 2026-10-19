"""Experiment-only actuated planar carriage; frozen robot/object untouched.
Two driven PhysX prismatic joints replace the robot's world mounting constraint.
This is a dynamic XY stage, not a wheeled-base or hardware claim.
"""
def author_mobile_stage(stage, base, output):
 import numpy as np,json
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,Sdf
 from scipy.spatial.transform import Rotation
 root='/World/Piper'; cache=UsdGeom.XformCache()
 bodies=[p for p in stage.Traverse() if str(p.GetPath()).startswith(root) and p.GetName()=='base_link' and p.HasAPI(UsdPhysics.RigidBodyAPI)]
 link=bodies[0]; W=np.asarray(cache.GetLocalToWorldTransform(link)).T
 fixed=[p for p in stage.Traverse() if str(p.GetPath()).startswith(root) and p.IsA(UsdPhysics.FixedJoint) and str(link.GetPath()) in [str(x) for x in UsdPhysics.Joint(p).GetBody1Rel().GetTargets()]]
 old=fixed[0]; old.SetActive(False)
 for p in list(stage.Traverse()):
  if str(p.GetPath()).startswith(root) and p.HasAPI(UsdPhysics.ArticulationRootAPI):p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
 rot=Rotation.from_matrix(W[:3,:3]); qw=np.roll(rot.as_quat(),1); iq=np.roll(rot.inv().as_quat(),1)
 def body(name):
  prim=UsdGeom.Xform.Define(stage,root+'/'+name);prim.AddTranslateOp().Set(Gf.Vec3d(*W[:3,3]));prim.AddOrientOp().Set(Gf.Quatf(float(qw[0]),Gf.Vec3f(*qw[1:])))
  rb=UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim());rb.CreateRigidBodyEnabledAttr(True)
  mass=UsdPhysics.MassAPI.Apply(prim.GetPrim());mass.CreateMassAttr(1.);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.01,.01,.01));mass.CreateCenterOfMassAttr(Gf.Vec3f(0,0,0))
  return prim.GetPrim()
 anchor=body('mobile_anchor');carriage=body('mobile_x_carriage')
 j=UsdPhysics.FixedJoint.Define(stage,root+'/mobile_world_mount');j.CreateBody1Rel().SetTargets([anchor.GetPath()]);j.CreateLocalPos0Attr().Set(Gf.Vec3f(*W[:3,3]));j.CreateLocalRot0Attr().Set(Gf.Quatf(float(qw[0]),Gf.Vec3f(*qw[1:])));UsdPhysics.ArticulationRootAPI.Apply(j.GetPrim())
 for name,axis,a,b in [('mobile_x','X',anchor,carriage),('mobile_y','Y',carriage,link)]:
  j=UsdPhysics.PrismaticJoint.Define(stage,root+'/'+name);j.CreateBody0Rel().SetTargets([a.GetPath()]);j.CreateBody1Rel().SetTargets([b.GetPath()]);j.CreateAxisAttr(axis)
  for k in (0,1):getattr(j,'CreateLocalRot%dAttr'%k)().Set(Gf.Quatf(float(iq[0]),Gf.Vec3f(*iq[1:])))
  j.CreateLowerLimitAttr(-1.);j.CreateUpperLimitAttr(1.)
  drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(1000000.);drive.CreateDampingAttr(20000.);drive.CreateMaxForceAttr(10000.);drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
 # The existing pedestal retains its collision shape/material and follows the
 # physical robot base through a fixed joint. Only this previously static mount
 # needs a mass for the newly requested base dynamics.
 pedestal=stage.GetPrimAtPath('/World/r1a7_pedestal')
 if pedestal:
  P=np.asarray(cache.GetLocalToWorldTransform(pedestal)).T
  UsdPhysics.RigidBodyAPI.Apply(pedestal).CreateRigidBodyEnabledAttr(True)
  mass=UsdPhysics.MassAPI.Apply(pedestal);mass.CreateMassAttr(100.);mass.CreateCenterOfMassAttr(Gf.Vec3f(0,0,0))
  mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1.84,1.84,.167))
  j=UsdPhysics.FixedJoint.Define(stage,root+'/mobile_pedestal_mount');j.CreateBody0Rel().SetTargets([link.GetPath()]);j.CreateBody1Rel().SetTargets([pedestal.GetPath()])
  L=np.linalg.inv(W)@P;j.CreateLocalPos0Attr().Set(Gf.Vec3f(*L[:3,3]));q=np.roll(Rotation.from_matrix(L[:3,:3]).as_quat(),1);j.CreateLocalRot0Attr().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*q[1:])))
  # Keep pedestal geometry out of the robot-monitor link labels. Its rigid
  # attachment introduces no new contact with the object.
 from pathlib import Path
 Path(output,'mobile_stage_setup.json').write_text(json.dumps({'classification':'KNOWN_MODEL_DIAGNOSTIC','base_actuation':'PhysX driven XY prismatic carriage, no pose writes during execution','old_mount_deactivated':str(old.GetPath()),'base_body':str(link.GetPath()),'mobile_joint_names':['mobile_x','mobile_y'],'new_carriage_mass_kg':1.,'new_mobile_pedestal_mass_kg':100.,'base_gains':{'kp':1e6,'kd':2e4,'max_effort_n':1e4},'existing_arm_object_mass_material_geometry_unchanged':True,'wheeled_base_dynamics':False},indent=2))
 return str(link.GetPath())

def install_mobile_scene_hook():
 import interactive_twin.plant as plant
 original=plant.adapt_loader_source
 def adapt(source,job):
  source=original(source,job)
  marker='world.reset(); camera.initialize()'
  source=source.replace(marker,"from continuous_contact_scene import author_mobile_stage\nmobile_base_body_path=author_mobile_stage(stage,BASE_POSE,os.environ['R1A7_RUN_DIR'])\nworld.scene.remove_object('piper',registry_only=True)\nrobot=world.scene.add(SingleArticulation('/World/Piper',name='piper'))\n"+marker)
  source=source.replace('kp[fingers] = 1000.; kd[fingers] = 40.',"kp[fingers] = 1000.; kd[fingers] = 40.\nkp[[names.index('mobile_x'),names.index('mobile_y')]]=1e6;kd[[names.index('mobile_x'),names.index('mobile_y')]]=2e4")
  return source
 plant.adapt_loader_source=adapt
