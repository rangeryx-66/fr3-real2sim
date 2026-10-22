"""Versioned simulated XY mount, adapted from continuous_contact_scene.

Only the world mount/support becomes mobile. Original arm/G1 bodies, their
drives, contact surfaces and object physics are not authored here. This is a
driven carriage, not wheel dynamics or a verified physical xArm mobile platform.
"""
import json
from pathlib import Path


# Identical authority to the repository's existing diagnostic XY carriage.
BASE_KP=1e6
BASE_KD=2e4
BASE_MAX_EFFORT_N=1e4


def author(stage, output, root='/World/Piper', base_body='link_base'):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import UsdGeom,UsdPhysics,Gf
    cache=UsdGeom.XformCache()
    bodies=[p for p in stage.Traverse() if str(p.GetPath()).startswith(root+'/')
            and p.GetName()==base_body and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies)!=1:raise RuntimeError('UNIQUE_NATIVE_BASE_BODY_REQUIRED')
    link=bodies[0]
    fixed=[p for p in stage.Traverse() if str(p.GetPath()).startswith(root+'/')
           and p.IsA(UsdPhysics.FixedJoint)
           and list(UsdPhysics.Joint(p).GetBody1Rel().GetTargets())==[link.GetPath()]
           and all(str(t)==root and not stage.GetPrimAtPath(t).HasAPI(UsdPhysics.RigidBodyAPI)
                   for t in UsdPhysics.Joint(p).GetBody0Rel().GetTargets())]
    if len(fixed)!=1:raise RuntimeError('UNIQUE_WORLD_BASE_MOUNT_REQUIRED')
    W=np.asarray(cache.GetLocalToWorldTransform(link)).T
    parent=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(root))).T
    local=np.linalg.inv(parent)@W
    def quat(R):
        q=np.roll(Rotation.from_matrix(R).as_quat(),1)
        return Gf.Quatf(float(q[0]),Gf.Vec3f(*q[1:]))
    fixed[0].SetActive(False)
    for p in list(stage.Traverse()):
        if str(p.GetPath()).startswith(root) and p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    def body(name):
        x=UsdGeom.Xform.Define(stage,root+'/'+name)
        x.AddTranslateOp().Set(Gf.Vec3d(*local[:3,3]));x.AddOrientOp().Set(quat(local[:3,:3]))
        b=x.GetPrim();UsdPhysics.RigidBodyAPI.Apply(b)
        m=UsdPhysics.MassAPI.Apply(b);m.CreateMassAttr(1.)
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(.01,.01,.01));m.CreateCenterOfMassAttr(Gf.Vec3f(0,0,0))
        return b
    anchor=body('mobile_anchor');carriage=body('mobile_x_carriage')
    mount=UsdPhysics.FixedJoint.Define(stage,root+'/mobile_world_mount')
    mount.CreateBody1Rel().SetTargets([anchor.GetPath()])
    mount.CreateLocalPos0Attr().Set(Gf.Vec3f(*W[:3,3]));mount.CreateLocalRot0Attr().Set(quat(W[:3,:3]))
    UsdPhysics.ArticulationRootAPI.Apply(mount.GetPrim())
    for name,axis,a,b in [('mobile_x','X',anchor,carriage),('mobile_y','Y',carriage,link)]:
        j=UsdPhysics.PrismaticJoint.Define(stage,root+'/'+name)
        j.CreateBody0Rel().SetTargets([a.GetPath()]);j.CreateBody1Rel().SetTargets([b.GetPath()]);j.CreateAxisAttr(axis)
        for i in (0,1):getattr(j,'CreateLocalRot%dAttr'%i)().Set(quat(W[:3,:3].T))
        j.CreateLowerLimitAttr(-1.);j.CreateUpperLimitAttr(1.)
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force')
        d.CreateStiffnessAttr(BASE_KP);d.CreateDampingAttr(BASE_KD);d.CreateMaxForceAttr(BASE_MAX_EFFORT_N)
        d.CreateTargetPositionAttr(0.);d.CreateTargetVelocityAttr(0.)
    pedestal=stage.GetPrimAtPath('/World/r1a7_pedestal')
    if not pedestal:raise RuntimeError('ORIGINAL_SUPPORT_GEOMETRY_REQUIRED')
    P=np.asarray(cache.GetLocalToWorldTransform(pedestal)).T
    UsdPhysics.RigidBodyAPI.Apply(pedestal)
    m=UsdPhysics.MassAPI.Apply(pedestal);m.CreateMassAttr(100.)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0,0,0));m.CreateDiagonalInertiaAttr(Gf.Vec3f(1.84,1.84,.167))
    j=UsdPhysics.FixedJoint.Define(stage,root+'/mobile_pedestal_mount')
    j.CreateBody0Rel().SetTargets([link.GetPath()]);j.CreateBody1Rel().SetTargets([pedestal.GetPath()])
    L=np.linalg.inv(W)@P;j.CreateLocalPos0Attr().Set(Gf.Vec3f(*L[:3,3]));j.CreateLocalRot0Attr().Set(quat(L[:3,:3]))
    result=dict(base_body=str(link.GetPath()),original_world_mount=str(fixed[0].GetPath()),
        joint_names=['mobile_x','mobile_y'],initial_T_world_base=W.tolist(),
        initial_T_world_pedestal=P.tolist(),native_limits_m=[-1.,1.],
        carriage_mass_kg=1.,pedestal_mass_kg=100.,kp=BASE_KP,kd=BASE_KD,max_effort_n=BASE_MAX_EFFORT_N,
        limitation='simulated driven XY carriage; not wheeled mobile dynamics or hardware validation',
        original_robot_object_physics_modified=False,original_mount_replaced=True)
    Path(output,'V81_MOBILE_MOUNT.json').write_text(json.dumps(result,indent=2))
    return result


def drive_class(original):
    """Preserve all original arm/G1 values; retain separate carriage drives.

    Native G1 command_width rewrites the full max-effort vector every step.
    Therefore base authority must live in that same vector, not a stale USD cap.
    """
    class MobileNativeDrive(original):
        def __init__(self,robot,c,output):
            import numpy as np
            super().__init__(robot,c,output)
            self.base_indices=np.array([self.names.index(n) for n in ['mobile_x','mobile_y']])
            self.caps[self.base_indices]=BASE_MAX_EFFORT_N
            self.robot._articulation_view.set_max_efforts(self.caps.reshape(1,-1))
            speeds=self.robot._articulation_view.get_max_joint_velocities().copy()
            speeds[...,self.base_indices]=.08  # unchanged FREE travel policy
            self.robot._articulation_view.set_max_joint_velocities(speeds)
            self._base_gains()
            Path(output,'V81_DRIVE_INTERFACE.json').write_text(json.dumps(dict(
                native_joint_order=self.names,base_indices=self.base_indices.tolist(),
                actual_max_efforts=self.caps.tolist(),base_max_velocity_m_s=.08,
                arm_G1_caps_unchanged=True,base_authority_source='continuous_contact_scene.py existing XY stage')))
        def _base_gains(self):
            kp,kd=self.raw.get_gains();kp=kp.copy();kd=kd.copy()
            kp[self.base_indices]=BASE_KP;kd[self.base_indices]=BASE_KD
            self.raw.set_gains(kps=kp,kds=kd,save_to_usd=True)
        def set_gains(self,*args,**kwargs):
            super().set_gains(*args,**kwargs)
            self._base_gains()
    return MobileNativeDrive
