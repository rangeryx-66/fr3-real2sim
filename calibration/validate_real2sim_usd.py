"""Reload a generated asset in a fresh Isaac process and run a render/drop test."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from isaacsim import SimulationApp

p=argparse.ArgumentParser();p.add_argument('--asset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--gpu',type=int,default=3);p.add_argument('--untextured-debug',action='store_true');a=p.parse_args()
app=SimulationApp({'headless':True,'active_gpu':a.gpu,'physics_gpu':a.gpu,'multi_gpu':False})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from pxr import UsdGeom,UsdLux,UsdShade,UsdPhysics

world=World(stage_units_in_meters=1.,physics_dt=1/240,rendering_dt=1/30,backend='numpy',device='cpu');world.scene.add_default_ground_plane()
light=UsdLux.DomeLight.Define(world.stage,'/World/ValidationLight');light.CreateIntensityAttr(1200.0)
add_reference_to_stage(str(a.asset.resolve()),'/World/TestObject')
if a.untextured_debug:
    visual_prim=world.stage.GetPrimAtPath('/World/TestObject/Visual')
    UsdShade.MaterialBindingAPI(visual_prim).UnbindAllBindings()
    UsdGeom.Mesh(visual_prim).CreateDisplayColorAttr([(1.0,.05,.05)])
body=world.scene.add(SingleRigidPrim('/World/TestObject',name='test_object',position=[.5,0,.08]))
camera_position=np.array([.88,-.58,.48]);look_at=np.array([.5,0.,.05])
camera=Camera('/World/Camera',position=camera_position,resolution=(640,480),frequency=30)
camera.set_clipping_range(.02,3.0)
camera_z=look_at-camera_position;camera_z/=np.linalg.norm(camera_z)
camera_x=np.cross(camera_z,np.array([0.,0.,1.]));camera_x/=np.linalg.norm(camera_x);camera_y=np.cross(camera_z,camera_x)
camera_R=np.column_stack([camera_x,camera_y,camera_z]);camera_q=np.roll(Rotation.from_matrix(camera_R).as_quat(),1)
camera.set_world_pose(position=camera_position,orientation=camera_q,camera_axes='ros')
world.reset();camera.initialize()
# Read the physical attributes from the newly loaded USD before stepping.  Isaac
# stores inertia as diagonal principal moments plus a principal-axis quaternion;
# keeping both values in the report catches packaging/readback errors that a mass
# or COM-only check would miss.
mass_api = UsdPhysics.MassAPI(world.stage.GetPrimAtPath('/World/TestObject'))
mass_readback = float(mass_api.GetMassAttr().Get())
com_value = mass_api.GetCenterOfMassAttr().Get()
com_readback = np.asarray(com_value, dtype=float)
diag_value = mass_api.GetDiagonalInertiaAttr().Get()
diag_readback = np.asarray(diag_value, dtype=float)
axes_value = mass_api.GetPrincipalAxesAttr().Get()
axes_quat = [float(axes_value.GetReal()), *[float(x) for x in axes_value.GetImaginary()]]
inertia_readback = bool(
    np.isfinite(mass_readback)
    and np.isfinite(com_readback).all()
    and np.isfinite(diag_readback).all()
    and np.all(diag_readback > 0)
    and np.isfinite(axes_quat).all()
)
poses=[]
for i in range(2400):
    world.step(render=i%8==0)
    if i%24==0:poses.append(body.get_world_pose()[0].tolist())
a.output.parent.mkdir(parents=True,exist_ok=True)
for _ in range(8):world.step(render=True)
rgba=np.asarray(camera.get_rgba());cv2.imwrite(str(a.output),rgba[...,:3][...,[2,1,0]])
velocity=np.linalg.norm(body.get_linear_velocity());angular_velocity=np.linalg.norm(body.get_angular_velocity());finite=np.isfinite(poses).all();settled=float(np.max(np.ptp(np.asarray(poses[-10:]),axis=0)))<.002
result = {
    'loaded': True,
    'render': str(a.output),
    'finite': bool(finite),
    'settled': bool(settled),
    'linear_speed_m_s': float(velocity),
    'angular_speed_rad_s': float(angular_velocity),
    'final_position_m': poses[-1],
    'physics_passed': bool(
        finite and settled and inertia_readback and velocity < .05 and angular_velocity < .1
    ),
    'mass_readback_kg': mass_readback,
    'com_readback_m': com_readback.tolist(),
    'diagonal_inertia_readback_kg_m2': diag_readback.tolist(),
    'principal_axes_readback_quat_wxyz': axes_quat,
    'inertia_readback': inertia_readback,
}
(a.output.parent/'reload_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));app.close()
