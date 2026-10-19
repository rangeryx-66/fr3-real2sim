"""Isaac R1/Dex1 articulation plant; physical execution never scripts the door.

An explicit planning-preview mode renders prescribed saved IK waypoints.
Its watermarked output cannot count as a manipulation experiment.
"""
import argparse
import hashlib
import json
import os
import queue
import shutil
import subprocess
import threading
import time
import itertools
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser()
p.add_argument('--gpu', type=int, default=1)
p.add_argument('--port', type=int, default=18795)
p.add_argument('--asset-root', type=Path, default=Path('/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1'))
p.add_argument('--asset-x', type=float, default=.45)
p.add_argument('--asset-y', type=float, default=.05)
p.add_argument('--asset-yaw-deg', type=float, default=-90.)
p.add_argument('--fixture-height-m', type=float, default=.18)
p.add_argument('--home-q',type=float,nargs=7,
               default=(0.,1.15,-1.35,0.,.2,0.),metavar='Q')
p.add_argument('--door-mass-model',choices=('geometry','legacy'),default='geometry')
p.add_argument('--overview-dir',type=Path,help='Render two whole-scene views and exit without robot execution')
p.add_argument('--planning-preview-report',type=Path,help='Render prescribed planned IK waypoints without execution or contact claims')
p.add_argument('--planning-preview-video',type=Path)
p.add_argument('--record-overview',action='store_true',help='Record two whole-scene views during real physics execution')
p.add_argument('--enable-isolation-diagnostics',action='store_true',help='Allow explicit passive door torque and temporary locked-door tests')
p.add_argument('--door-sanity-only',type=Path,help='No robot loaded: save isolated passive-door torque sweep here')
p.add_argument('--camera-offset', type=float, nargs=3, default=(.20, -.75, .36),
               metavar=('DX', 'DY', 'DZ'))
a = p.parse_args()
ROOT = Path(__file__).resolve().parents[1]
# Keep the imported robot and MoveIt world mount consistent for every layout.
pass # xArm model prepared in isolated output
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'active_gpu': a.gpu, 'physics_gpu': a.gpu,
                     'multi_gpu': False})
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, RigidPrim, SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
import omni.usd

DT = 1 / 240
HOME = np.array([0, -0.5, 0, 1.0, 0, 0.5, 0],dtype=float)
BASE_POSE = np.array([float(v) for v in os.environ.get('R1A7_BASE_POSE',
                           '0.329,-0.175,0.237,56.3').split(',')])
PEDESTAL_SIZE = np.array([float(v) for v in os.environ.get('R1A7_PEDESTAL_SIZE',
                               '0.10,0.10,0.20').split(',')])
manifest = json.loads((a.asset_root / 'manifest.json').read_text())
asset_id=manifest['asset_id']
asset_urdf=a.asset_root/'urdf'/f'{asset_id}.urdf'
from interactive_twin.visual_import import compatible_urdf
asset_import_urdf,visual_import_audit=compatible_urdf(asset_urdf,a.asset_root/'visual_compatibility')
moving_link=manifest.get('moving_link','abstract_2_1')
door_link_name=manifest.get('door_link','l_1')
asset_usd = a.asset_root/'visual_compatibility'/'no_legacy_visual_cache.usd'
if not asset_usd.is_file():
    version=hashlib.sha256(asset_import_urdf.read_bytes()).hexdigest()[:12]
    cache=a.asset_root/f'usd/imported_path_{version}.txt'
    if cache.exists() and Path(cache.read_text().strip()).is_file():asset_usd=Path(cache.read_text().strip())
    else:
        directory=a.asset_root/'usd'/version;directory.mkdir(parents=True,exist_ok=True)
        imported=URDFImporter(URDFImporterConfig(urdf_path=str(asset_import_urdf),usd_path=str(directory),
            fix_base=True,allow_self_collision=False,merge_fixed_joints=False,
            joint_drive_type='force',joint_target_type='position')).import_urdf()
        asset_usd=Path(imported);cache.write_text(str(asset_usd))
world = World(stage_units_in_meters=1., physics_dt=DT, rendering_dt=1/30,
              backend='numpy', device='cpu')
world.scene.add_default_ground_plane(z_position=-.76)
mat = PhysicsMaterial('/World/grasp_material', static_friction=.8,
                      dynamic_friction=.7, restitution=0.)
world.scene.add(FixedCuboid('/World/table', name='table', position=[.5, 0, -.025],
                            scale=[.9, .9, .05], physics_material=mat))
support_bottom=float(os.environ.get('R1A7_SUPPORT_BOTTOM_Z','0'))
if BASE_POSE[2] > support_bottom:
    world.scene.add(FixedCuboid('/World/r1a7_pedestal', name='r1a7_pedestal',
        position=[BASE_POSE[0], BASE_POSE[1], (BASE_POSE[2]+support_bottom)/2],
        scale=[PEDESTAL_SIZE[0],PEDESTAL_SIZE[1],BASE_POSE[2]-support_bottom], physics_material=mat))
stage = omni.usd.get_context().get_stage()
asset_parent = UsdGeom.Xform.Define(stage, '/World/articulated_pose')
xyz_op = asset_parent.AddTranslateOp()
quat_op = asset_parent.AddOrientOp()
scale = float(manifest['scale_source_to_meters'])
static_low_y = float(manifest['static_source_bounds'][0][1])
asset_xyz = np.array([a.asset_x, a.asset_y, -scale * static_low_y])
asset_xyz[2] += a.fixture_height_m
source_to_world = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
asset_rotation = Rotation.from_euler('z', a.asset_yaw_deg, degrees=True).as_matrix() @ source_to_world
handle_bounds = np.asarray(manifest['grasp_mesh_bounds_source'], dtype=float)
handle_center = asset_xyz + asset_rotation @ (handle_bounds.mean(axis=0) * scale)
sys.path.insert(0,str(ROOT))
from articulated_demo.kinematics import URDFChain
from articulated_demo.fixture_geometry import fixture_box
asset_chain=URDFChain(asset_urdf)
fixture=fixture_box(manifest,asset_chain,asset_urdf,asset_rotation,asset_xyz,a.fixture_height_m)
fixture_size=np.array(fixture['size']);fixture_quat=np.array(fixture['quaternion_wxyz'])
if a.fixture_height_m>0.:
    world.scene.add(FixedCuboid('/World/cabinet_fixture', name='cabinet_fixture',
        position=fixture['center'],
        scale=fixture_size,orientation=fixture_quat,physics_material=mat))
asset_quat = np.roll(Rotation.from_matrix(asset_rotation).as_quat(), 1)
xyz_op.Set(Gf.Vec3d(*asset_xyz))
quat_op.Set(Gf.Quatf(*asset_quat))
asset_path = '/World/articulated_pose/asset'
add_reference_to_stage(str(asset_usd), asset_path)
# Passive asset joint: the robot must supply the opening force.
for prim in stage.Traverse():
    if str(prim.GetPath()).startswith(asset_path) and prim.IsA(UsdPhysics.RevoluteJoint):
        drive=UsdPhysics.DriveAPI.Get(prim,'angular')
        if drive:
            drive.CreateStiffnessAttr().Set(0.);drive.CreateDampingAttr().Set(0.)
            drive.CreateMaxForceAttr().Set(0.)
articulation = world.scene.add(SingleArticulation(asset_path, name='cabinet'))
asset_joint_name = manifest['joint_name']

asset_cache = ROOT / 'assets'
asset_cache.mkdir(exist_ok=True)
model_hash = hashlib.sha256((Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_shared_doors_20261012/xarm7_piper_sim.urdf')).read_bytes()).hexdigest()[:12]
asset_file = asset_cache / f'piper_mobile_asset_path_{model_hash}.txt'
if asset_file.exists() and Path(asset_file.read_text().strip()).exists():
    robot_usd = asset_file.read_text().strip()
else:
    robot_usd = URDFImporter(URDFImporterConfig(
        urdf_path=str(Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_shared_doors_20261012/xarm7_piper_sim.urdf')), usd_path=str(asset_cache),
        fix_base=True, allow_self_collision=True, merge_fixed_joints=False,
        joint_drive_type='force', joint_target_type='position')).import_urdf()
    asset_file.write_text(robot_usd)
if not a.door_sanity_only:
    add_reference_to_stage(robot_usd, '/World/Piper')
    robot = world.scene.add(SingleArticulation('/World/Piper', name='piper'))
    robot.set_world_pose(BASE_POSE[:3], np.roll(Rotation.from_euler('z',BASE_POSE[3],degrees=True).as_quat(),1))

def one_prim(name, under):
    matches = [prim for prim in stage.Traverse()
               if prim.GetName() == name and str(prim.GetPath()).startswith(under)
               and (prim.HasAPI(UsdPhysics.RigidBodyAPI) or name == 'tcp_link')]
    if len(matches) != 1:
        raise RuntimeError(f'expected one {name} under {under}, got {len(matches)}')
    return str(matches[0].GetPath())

moving_path = one_prim(moving_link, asset_path)
moving = SingleXFormPrim(moving_path)
contact_target_path = one_prim(door_link_name, moving_path)
door_link = SingleXFormPrim(contact_target_path)
sys.path.insert(0,str(ROOT))
from articulated_demo.kinematics import URDFChain
asset_chain=URDFChain(asset_urdf)
# Prepared inertias placed both moving-body COMs at their link origins. l_1's
# origin is the asset origin, far outside the door. Preserve source masses,
# but put COMs at the moving geometry center and use a uniform-box inertia.
# This is a documented simulation approximation, not a measured door inertia.
mass_audit=[]
geometry_bounds=np.asarray(manifest['moving_source_bounds'])*scale
geometry_center=geometry_bounds.mean(axis=0)
geometry_size=geometry_bounds[1]-geometry_bounds[0]
urdf_links={l.get('name'):l for l in ET.parse(asset_urdf).findall('link')}
for link_name,link_path in ((moving_link,moving_path),(door_link_name,contact_target_path)):
    api=UsdPhysics.MassAPI(stage.GetPrimAtPath(link_path))
    previous={'center_of_mass':list(api.GetCenterOfMassAttr().Get()),
              'diagonal_inertia':list(api.GetDiagonalInertiaAttr().Get()),
              'mass_kg':float(api.GetMassAttr().Get())}
    mass=float(urdf_links[link_name].find('inertial/mass').get('value'))
    local=np.linalg.inv(asset_chain.root_to_link(link_name,{}))@np.r_[geometry_center,1.]
    inertia=mass/12*np.array([geometry_size[1]**2+geometry_size[2]**2,
        geometry_size[0]**2+geometry_size[2]**2,geometry_size[0]**2+geometry_size[1]**2])
    if a.door_mass_model=='geometry':
        api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*local[:3]))
        api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*inertia))
        api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.))
    mass_audit.append({'link':link_name,'before':previous,
        'geometry_com_local_m':local[:3].tolist(),'geometry_inertia_kg_m2':inertia.tolist()})
static_bounds = np.asarray(manifest['static_source_bounds'], dtype=float) * scale
def oriented_box(bounds, world_matrix):
    """Convert a USD box and its live transform to a MoveIt oriented box."""
    low = np.asarray(bounds.GetMin(), dtype=float)
    high = np.asarray(bounds.GetMax(), dtype=float)
    center_local = Gf.Vec3d(*((low + high) / 2))
    center = np.asarray(world_matrix.Transform(center_local), dtype=float)
    basis = np.column_stack([
        np.asarray(world_matrix.TransformDir(Gf.Vec3d(*axis)), dtype=float)
        for axis in np.eye(3)])
    lengths = np.linalg.norm(basis, axis=0)
    rotation = basis / lengths
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4):
        raise RuntimeError('cabinet USD collision transform contains shear')
    quat = np.roll(Rotation.from_matrix(rotation).as_quat(), 1)
    return {'center':center.tolist(), 'size':((high-low)*lengths).tolist(),
            'quaternion_wxyz':quat.tolist()}

def collision_boxes():
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    door_bound = cache.ComputeWorldBound(stage.GetPrimAtPath(moving_path))
    static = {'id':'cabinet_static',
              'center':(asset_xyz + asset_rotation @ static_bounds.mean(axis=0)).tolist(),
              'size':(static_bounds[1]-static_bounds[0]).tolist(),
              'quaternion_wxyz':asset_quat.tolist()}
    door = {'id':'cabinet_door', **oriented_box(door_bound.GetBox(), door_bound.GetMatrix())}
    return [static, door,
            {'id':'cabinet_fixture','center':fixture['center'],
             'size':fixture_size.tolist(),
             'quaternion_wxyz':fixture_quat.tolist()}]
if a.door_sanity_only:
    from articulated_demo.isolation_diagnostics import run_door_without_robot
    run_door_without_robot(a.door_sanity_only,world,stage,articulation,asset_chain,
        asset_path,moving_path,contact_target_path,asset_rotation,asset_xyz,handle_center,mass_audit,manifest,DT,fixture['audit'])
    app.close();sys.exit(0)
contact_paths = [one_prim(name, '/World/Piper') for name in ('gripper_link1', 'gripper_link2')]
for path in contact_paths:
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
finger_views = [world.scene.add(RigidPrim(prim_paths_expr=path, name=f'finger_contact_{i}',
    contact_filter_prim_paths_expr=[contact_target_path], track_contact_forces=True,
    prepare_contact_sensors=True, max_contact_count=256)) for i, path in enumerate(contact_paths)]
body_views=[]
if a.enable_isolation_diagnostics:
    body_views=[world.scene.add(RigidPrim(prim_paths_expr=one_prim(name,'/World/Piper'),name=f'finger_body_contact_{i}',
        contact_filter_prim_paths_expr=[contact_target_path],track_contact_forces=True,
        prepare_contact_sensors=True,max_contact_count=256))
        for i,name in enumerate(('gripper_base','link6'))]
door_contacts=None
if a.enable_isolation_diagnostics:
    diagnostic_contact_paths=[str(prim.GetPath()) for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(asset_path) and not str(prim.GetPath()).startswith(moving_path)
        and prim.HasAPI(UsdPhysics.RigidBodyAPI)] + [path for path in ['/World/table','/World/cabinet_fixture'] if stage.GetPrimAtPath(path).IsValid()]
    door_contacts=world.scene.add(RigidPrim(prim_paths_expr=contact_target_path,name='door_diagnostic_contacts',
        contact_filter_prim_paths_expr=diagnostic_contact_paths,track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=256))
scene_monitor_views=[]
for monitor_name in ['link1','link2','link3','link4','link5','link6','link7','gripper_base','gripper_link1','gripper_link2']:
    filters=list(diagnostic_contact_paths)
    if monitor_name not in ['gripper_link1','gripper_link2']:filters.append(contact_target_path)
    scene_monitor_views.append((monitor_name,world.scene.add(RigidPrim(
        prim_paths_expr=one_prim(monitor_name,'/World/Piper'),name='scene_contact_'+monitor_name,
        contact_filter_prim_paths_expr=filters,track_contact_forces=True,
        prepare_contact_sensors=True,max_contact_count=256))))
tcp = SingleXFormPrim(one_prim('tcp_link', '/World/Piper'))
# Camera-optical rotation: columns are image right, image down, viewing forward.
eye = handle_center + np.array(a.camera_offset); focus = handle_center + [0., 0., .02]
forward = (focus - eye) / np.linalg.norm(focus - eye)
right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
down = np.cross(forward, right)
T_B_C = np.eye(4); T_B_C[:3, :3] = np.column_stack((right, down, forward)); T_B_C[:3, 3] = eye
camera = Camera('/World/camera', position=eye, resolution=(640, 480), frequency=30)
camera.set_world_pose(position=eye,
    orientation=np.roll(Rotation.from_matrix(T_B_C[:3, :3]).as_quat(), 1), camera_axes='ros')
camera.set_clipping_range(.05, 3.)

from interactive_twin.plant import author_passive_resistance
plant_resistance_audit=author_passive_resistance(stage,asset_path,{})
from interactive_twin.initial_state import restore_baked_mass_properties
baked_mass_audit=restore_baked_mass_properties(stage,asset_path,asset_urdf,manifest)

from interactive_twin_refinement.assembly import capture_assembly
capture_assembly(globals(),Path(os.environ['R1A7_RUN_DIR'])/'native_authored_assembly_private.json')
from xarm7_runtime import configure_stage,configuration,XArm7Drive
xarm_config=configuration()
configure_stage(stage,'/World/Piper',xarm_config,Path(os.environ['R1A7_RUN_DIR']))
world.reset(); camera.initialize(); camera.add_distance_to_image_plane_to_frame()
camera.add_instance_id_segmentation_to_frame()
overview=[]
if a.overview_dir or a.planning_preview_report or a.record_overview:
    focus=np.array([a.asset_x+.05,a.asset_y-.05,.32])
    for index,offset in enumerate(([1.1,-1.4,.9],[-1.1,-1.1,.8])):
        position=focus+offset;direction=(focus-position)/np.linalg.norm(focus-position)
        xaxis=np.cross(direction,[0.,0.,1.]);xaxis/=np.linalg.norm(xaxis)
        rotation=np.column_stack((xaxis,np.cross(direction,xaxis),direction))
        view=Camera(f'/World/overview_{index}',position=position,resolution=(1280,960),frequency=30)
        view.set_focal_length(2.)
        view.set_world_pose(position=position,orientation=np.roll(Rotation.from_matrix(rotation).as_quat(),1),camera_axes='ros')
        view.initialize();overview.append(view)
names = robot.dof_names
arm = [names.index(f'joint{i}') for i in range(1, 8)]
fingers = [names.index('gripper_joint1'), names.index('gripper_joint2')]
asset_names = articulation.dof_names
if asset_names != [asset_joint_name]:
    raise RuntimeError(f'asset DOF mismatch: {asset_names}, expected {asset_joint_name}')
controller = robot.get_articulation_controller()
kp = np.zeros(len(names)); kd = np.zeros(len(names))
kp[arm] = [1200.0, 1400.0, 1200.0, 850.0, 500.0, 500.0, 300.0]
kd[arm] = [10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0]
kp[fingers] = 1000.; kd[fingers] = 40.
controller.set_gains(kps=kp, kds=kd, save_to_usd=True)
q = np.zeros(len(names)); q[arm] = HOME; q[fingers] = [.05,-.05]
robot.set_joint_positions(q); robot.set_joint_velocities(np.zeros(len(q)))
controller=XArm7Drive(robot,xarm_config,Path(os.environ['R1A7_RUN_DIR']))
controller.apply_action(ArticulationAction(joint_positions=q[arm],joint_velocities=np.zeros(7),joint_indices=arm))
controller.apply_action(ArticulationAction(joint_positions=q[fingers],joint_efforts=np.zeros(2),joint_indices=fingers))

articulation.set_joint_positions(np.array([0.0]))
articulation.set_joint_velocities(np.array([0.]))
startup_angles=[];xarm_startup_records=[]
for i in range(240):
    controller.apply_action(ArticulationAction(joint_positions=q[arm],joint_velocities=np.zeros(7),joint_indices=arm))
    world.step(render=i % 8 == 0)
    try:xarm_startup_records.append({'step':i,'t':float(world.current_time),'q':robot.get_joint_positions().tolist(),'dq':robot.get_joint_velocities().tolist(),'projected_efforts':robot.get_measured_joint_efforts().tolist(),'joint_reactions':robot.get_measured_joint_forces().tolist(),'actuator':controller.last.copy()})
    except Exception:pass
    startup_angles.append(float(articulation.get_joint_positions()[0]))
if a.planning_preview_report:
    if not a.planning_preview_video:raise ValueError('preview video output is required')
    sys.path.insert(0,str(ROOT/'scripts'))
    from render_articulated_plan_preview import render_preview
    render_preview(a.planning_preview_report,a.planning_preview_video,
        world,robot,articulation,overview,q,arm,fingers,BASE_POSE,a)
    app.close();sys.exit(0)
if a.overview_dir:
    import cv2
    a.overview_dir.mkdir(parents=True,exist_ok=True)
    for _ in range(32):world.step(render=True)
    for index,view in enumerate(overview):
        image=np.asarray(view.get_rgba())[:,:,:3]
        cv2.imwrite(str(a.overview_dir/f'overview_{index}.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
    (a.overview_dir/'scene.json').write_text(json.dumps({'asset_id':asset_id,
        'asset_installation':{'x_m':a.asset_x,'y_m':a.asset_y,'yaw_deg':a.asset_yaw_deg},
        'robot_base_pose':BASE_POSE.tolist(),
        'table_top_z_m':0.,'table_front_edge_y_m':-.45,
        'robot_base_below_table_m':float(-BASE_POSE[2]),
        'robot_base_distance_outside_table_m':float(-.45-BASE_POSE[1]),
        'joint_axis_world':(asset_rotation@asset_chain.joints[moving_link].axis).tolist(),
        'actual_joint_q_rad':float(articulation.get_joint_positions()[0]),
        'note':'whole-scene render only; no grasp or object-joint actuation'},indent=2))
    app.close();sys.exit(0)

import gzip,pickle
with gzip.open(Path(os.environ['R1A7_RUN_DIR'])/'startup_240hz.pkl.gz','wb',compresslevel=1) as f:pickle.dump(xarm_startup_records,f,protocol=5)

try:
 native_tensor={}
 for method in ['get_masses','get_inertias','get_coms']:
  native_tensor[method]=np.asarray(getattr(robot._articulation_view._physics_view,method)()).tolist()
 (Path(os.environ['R1A7_RUN_DIR'])/'NATIVE_MASS_TENSOR_READBACK.json').write_text(json.dumps(native_tensor,indent=2))
except Exception:pass
