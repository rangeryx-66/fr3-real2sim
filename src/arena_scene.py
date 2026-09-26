"""Arena USD scene/GT interfaces; no planning, scoring, or grasp controls."""
import os,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdShade
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleRigidPrim,RigidPrim
from workspace_mount import load_mount,transform_pose
ROOT=Path(__file__).resolve().parents[1]
MOUNT=load_mount()
INVENTORY=json.loads((ROOT/'assets/arena_complex/inventory.json').read_text())
PROTOCOL=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())
TARGET=os.environ.get('FR3_ARENA_TARGET','mustard')
EPISODES={e['seed']:e for e in PROTOCOL['episodes']}
DEFAULT=next(e for e in EPISODES.values() if e['target']==TARGET)
def spawn(world,stage,material,name,spec):
    asset=INVENTORY[spec['asset']];prim=add_reference_to_stage(asset['usd_path'],'/World/'+name)
    for p in list(Usd.PrimRange(prim)):
        if p.IsInstance():p.SetInstanceable(False)
    bodies=[p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert len(bodies)==1 and bodies[0]==prim,(name,[str(p.GetPath()) for p in bodies])
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
    mass=UsdPhysics.MassAPI(prim).GetMassAttr().Get()
    if not mass or mass<=0:UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.2)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    position,orientation=transform_pose(spec['position'],spec['quaternion_wxyz'],MOUNT)
    b=world.scene.add(SingleRigidPrim('/World/'+name,name=name,position=position,orientation=orientation))
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(p).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/grasp_material')),bindingStrength='strongerThanDescendants',materialPurpose='physics')
    print('ARENA_BODY',name,spec['asset'],'mass',UsdPhysics.MassAPI(prim).GetMassAttr().Get(),flush=True)
    return b
def table(world,stage,material):
    prim=add_reference_to_stage(INVENTORY['table']['usd_path'],'/World/table')
    for p in list(Usd.PrimRange(prim)):
        if p.IsInstance():p.SetInstanceable(False)
    bounds=np.array(INVENTORY['table']['bounds']);shift=np.array([.5,0,0])-np.r_[bounds.mean(0)[:2],bounds[1,2]]
    position,orientation=transform_pose(shift,[1,0,0,0],MOUNT)
    xf=UsdGeom.Xformable(prim);xf.AddTranslateOp().Set(Gf.Vec3d(*position.tolist()))
    xf.AddOrientOp().Set(Gf.Quatf(float(orientation[0]),Gf.Vec3f(*map(float,orientation[1:]))))
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rb=UsdPhysics.RigidBodyAPI(p);rb.CreateKinematicEnabledAttr(True)
            # The reused table USD contains nonzero recorded velocities. A fixed
            # support must not act as a moving conveyor belt through contact.
            rb.CreateVelocityAttr(Gf.Vec3f(0,0,0));rb.CreateAngularVelocityAttr(Gf.Vec3f(0,0,0))
        if p.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(p).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/grasp_material')),bindingStrength='strongerThanDescendants',materialPurpose='physics')
    return spawn(world,stage,material,'box',DEFAULT['objects'][0])
def reset_target(box,seed):
    spec=EPISODES[seed]['objects'][0];assert spec['asset']==TARGET
    position,orientation=transform_pose(spec['position'],spec['quaternion_wxyz'],MOUNT)
    box.set_world_pose(position,orientation);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])
def target_mask(camera,valid):
    data=camera.get_current_frame()['instance_id_segmentation']
    raw=data['data'].reshape(-1)
    ids=[int(k) for k,v in data['info']['idToLabels'].items() if str(v)=='/World/box' or str(v).startswith('/World/box/')]
    if not ids:raise RuntimeError('GT target instance ID missing: '+str(data['info']))
    return np.isin(raw[valid],ids)

# Preserve the exact measurement logic; parameterize only object count, dimensions,
# center offsets, and USD construction. Original source remains untouched.
source=Path(__file__).with_name('clutter_scene.py').read_text()
source=source.replace('range(3)','range(5)').replace('forces.shape==(3,len(self.filters))','forces.shape==(5,len(self.filters))')
ns={};exec(compile(source,'clutter_scene.py','exec'),ns)
class ArenaMonitor(ns['ClutterMonitor']):
    def __init__(self,world,stage,material):
        self.bodies=[spawn(world,stage,material,f'clutter_{i}',s) for i,s in enumerate(DEFAULT['objects'][1:])]
        self.target=world.scene.get_object('box')
        robot_root='/World/Piper/' if os.environ.get('GRASP_ROBOT','fr3')=='piper' else '/World/FR3/'
        self.filters=[str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()).startswith(robot_root) and p.HasAPI(UsdPhysics.RigidBodyAPI)]+['/World/box']
        self.contacts=world.scene.add(RigidPrim(prim_paths_expr='/World/clutter_.*',name='clutter_contacts',contact_filter_prim_paths_expr=self.filters,track_contact_forces=True,prepare_contact_sensors=True))
        ns['SIZES']=np.array([np.diff(np.array(INVENTORY[s['asset']]['bounds']),axis=0)[0] for s in DEFAULT['objects'][1:]])
        assert all(np.max(np.abs(np.mean(INVENTORY[s['asset']]['bounds'],axis=0)))<1e-6 for s in DEFAULT['objects'][1:])
        def mounted_layout(seed):
            poses=[transform_pose(s['position'],s['quaternion_wxyz'],MOUNT) for s in EPISODES[seed]['objects'][1:]]
            return np.array([p for p,_ in poses]),np.array([q for _,q in poses])
        ns['layout']=mounted_layout
        self.armed=False;self.seed=DEFAULT['seed'];self.phase='IDLE';self.metrics={};self.events=[]
