"""Independent Coulomb sanity check; no changes to the formal grasp experiment."""
import json
from pathlib import Path
import numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':7,'physics_gpu':7,'multi_gpu':False})
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid,FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import RigidPrim
from pxr import PhysxSchema
DT=1/240;world=World(stage_units_in_meters=1.,physics_dt=DT,rendering_dt=1/30,backend='numpy',device='cpu')
pad=PhysicsMaterial('/World/pad_material',static_friction=.3,dynamic_friction=.3,restitution=0.)
target=PhysicsMaterial('/World/target_material',static_friction=1.,dynamic_friction=1.,restitution=0.)
for m in [pad,target]:
 api=PhysxSchema.PhysxMaterialAPI.Apply(m.prim);api.CreateFrictionCombineModeAttr('multiply');api.CreateRestitutionCombineModeAttr('multiply')
world.scene.add(FixedCuboid('/World/pad',name='pad',position=[0,0,-.05],scale=[2.,2.,.1],physics_material=pad))
box=world.scene.add(DynamicCuboid('/World/target',name='target',position=[0,0,.051],scale=[.1,.1,.1],mass=1.,physics_material=target))
contacts=world.scene.add(RigidPrim(prim_paths_expr='/World/target',name='contact',contact_filter_prim_paths_expr=['/World/pad'],track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=128))
world.reset()
rows=[]
for force in [1.,2.,3.,4.,5.,6.]:
 for rep in range(2):
  box.set_world_pose([0,0,.051],[1,0,0,0]);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])
  for _ in range(240):world.step(render=False)
  start=box.get_world_pose()[0].copy();samples=[]
  for t in range(120):
   contacts.apply_forces(forces=np.array([[force,0,0]]),is_global=True);world.step(render=False)
   ff,_,cc,ii=contacts.get_friction_data(dt=DT);fn,_,nn,_,nc,ni=contacts.get_contact_force_data(dt=DT)
   friction=np.asarray(ff)[int(ii[0,0]):int(ii[0,0])+int(cc[0,0])].sum(axis=0)
   normal=(np.asarray(fn)[int(ni[0,0]):int(ni[0,0])+int(nc[0,0])]*np.asarray(nn)[int(ni[0,0]):int(ni[0,0])+int(nc[0,0])]).sum(axis=0)
   samples.append(dict(t=t*DT,position=box.get_world_pose()[0].tolist(),velocity=box.get_linear_velocity().tolist(),friction_N=friction.tolist(),normal_N=normal.tolist()))
  row=dict(force_N=force,repeat=rep,displacement_m=(box.get_world_pose()[0]-start).tolist(),velocity_m_s=box.get_linear_velocity().tolist(),samples=samples);rows.append(row);print(force,rep,row['displacement_m'],flush=True)
root=Path(__file__).resolve().parents[1]/'results/hand_calibration/physics_diagnostic';root.mkdir(exist_ok=True,parents=True)
(root/'friction_coupon.json').write_text(json.dumps(dict(mu_pad=.3,mu_target=1.,combine='multiply',mass_kg=1.,runtime_target_materials=np.asarray(contacts._physics_view.get_material_properties()).tolist(),rows=rows),indent=2))
app.close()
