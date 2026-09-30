"""Isaac R1/Dex1 + PhysX-Mobility 47686 plant; no scripted object-joint motion."""
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
               default=(0.,1.3,1.,-1.3,0.,0.,0.),metavar='Q')
p.add_argument('--camera-offset', type=float, nargs=3, default=(.20, -.75, .36),
               metavar=('DX', 'DY', 'DZ'))
a = p.parse_args()
ROOT = Path(__file__).resolve().parents[1]
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
HOME = np.array(a.home_q,dtype=float)
BASE_POSE = np.array([float(v) for v in os.environ.get('R1A7_BASE_POSE',
                           '0.329,-0.175,0.237,56.3').split(',')])
PEDESTAL_SIZE = np.array([float(v) for v in os.environ.get('R1A7_PEDESTAL_SIZE',
                               '0.10,0.10,0.20').split(',')])
manifest = json.loads((a.asset_root / 'manifest.json').read_text())
if manifest['asset_id'] != '47686':
    raise ValueError('this minimal demo requires PhysX-Mobility 47686')
asset_usd = a.asset_root / 'usd/47686/47686.usda/47686/47686.usda'
if not asset_usd.is_file():
    raise FileNotFoundError(asset_usd)
world = World(stage_units_in_meters=1., physics_dt=DT, rendering_dt=1/30,
              backend='numpy', device='cpu')
world.scene.add_default_ground_plane(z_position=-.76)
mat = PhysicsMaterial('/World/grasp_material', static_friction=.8,
                      dynamic_friction=.7, restitution=0.)
world.scene.add(FixedCuboid('/World/table', name='table', position=[.5, 0, -.025],
                            scale=[.9, .9, .05], physics_material=mat))
if BASE_POSE[2] > 0:
    world.scene.add(FixedCuboid('/World/r1a7_pedestal', name='r1a7_pedestal',
        position=[BASE_POSE[0], BASE_POSE[1], PEDESTAL_SIZE[2]/2],
        scale=PEDESTAL_SIZE, physics_material=mat))
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
world.scene.add(FixedCuboid('/World/cabinet_fixture', name='cabinet_fixture',
    position=[a.asset_x, a.asset_y, a.fixture_height_m/2],
    scale=[.70, .25, a.fixture_height_m], physics_material=mat))
asset_quat = np.roll(Rotation.from_matrix(asset_rotation).as_quat(), 1)
xyz_op.Set(Gf.Vec3d(*asset_xyz))
quat_op.Set(Gf.Quatf(*asset_quat))
asset_path = '/World/articulated_pose/asset'
add_reference_to_stage(str(asset_usd), asset_path)
articulation = world.scene.add(SingleArticulation(asset_path, name='cabinet'))
asset_joint_name = manifest['joint_name']

asset_cache = ROOT / 'assets'
asset_cache.mkdir(exist_ok=True)
model_hash = hashlib.sha256((ROOT / 'config/r1a7_dex1.urdf').read_bytes()).hexdigest()[:12]
asset_file = asset_cache / f'r1a7_dex1_filtered_asset_path_{model_hash}.txt'
if asset_file.exists() and Path(asset_file.read_text().strip()).exists():
    robot_usd = asset_file.read_text().strip()
else:
    robot_usd = URDFImporter(URDFImporterConfig(
        urdf_path=str(ROOT / 'config/r1a7_dex1.urdf'), usd_path=str(asset_cache),
        fix_base=True, allow_self_collision=False, merge_fixed_joints=False,
        joint_drive_type='force', joint_target_type='position')).import_urdf()
    asset_file.write_text(robot_usd)
add_reference_to_stage(robot_usd, '/World/R1A7')
robot = world.scene.add(SingleArticulation('/World/R1A7', name='r1a7'))

def one_prim(name, under):
    matches = [prim for prim in stage.Traverse()
               if prim.GetName() == name and str(prim.GetPath()).startswith(under)]
    if len(matches) != 1:
        raise RuntimeError(f'expected one {name} under {under}, got {len(matches)}')
    return str(matches[0].GetPath())

moving_path = one_prim('abstract_2_1', asset_path)
moving = SingleXFormPrim(moving_path)
contact_target_path = one_prim('l_1', moving_path)
door_link = SingleXFormPrim(contact_target_path)
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
            {'id':'cabinet_fixture','center':[a.asset_x,a.asset_y,a.fixture_height_m/2],
             'size':[.70,.25,a.fixture_height_m],
             'quaternion_wxyz':[1.,0.,0.,0.]}]
contact_paths = [one_prim(name, '/World/R1A7') for name in ('dex1_Link1_3', 'dex1_Link2_3')]
for path in contact_paths:
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
finger_views = [world.scene.add(RigidPrim(prim_paths_expr=path, name=f'finger_contact_{i}',
    contact_filter_prim_paths_expr=[contact_target_path], track_contact_forces=True,
    prepare_contact_sensors=True, max_contact_count=256)) for i, path in enumerate(contact_paths)]
tcp = SingleXFormPrim(one_prim('r1a7_tcp', '/World/R1A7'))
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
world.reset(); camera.initialize(); camera.add_distance_to_image_plane_to_frame()
camera.add_instance_id_segmentation_to_frame()
names = robot.dof_names
arm = [names.index(f'J{i}') for i in range(1, 8)]
fingers = [names.index('dex1_Joint1_1'), names.index('dex1_Joint2_1')]
asset_names = articulation.dof_names
if asset_names != [asset_joint_name]:
    raise RuntimeError(f'asset DOF mismatch: {asset_names}, expected {asset_joint_name}')
controller = robot.get_articulation_controller()
kp = np.zeros(len(names)); kd = np.zeros(len(names))
kp[arm] = [500., 500., 400., 400., 250., 150., 150.]
kd[arm] = [40., 40., 32., 32., 22., 15., 15.]
kp[fingers] = 800.; kd[fingers] = 30.
controller.set_gains(kps=kp, kds=kd, save_to_usd=False)
q = np.zeros(len(names)); q[arm] = HOME; q[fingers] = -.02
robot.set_joint_positions(q); robot.apply_action(ArticulationAction(joint_positions=q))
for i in range(240): world.step(render=i % 8 == 0)
# Reset the freely swinging door once after the arm has settled. This is the
# trial's initial condition; opening is never scripted through this joint.
articulation.set_joint_positions(np.array([0.]))
articulation.set_joint_velocities(np.array([0.]))
world.step(render=True)
commands = queue.Queue(); state = {}; lock = threading.Lock()
tick = 240; active = None; target = q.copy(); results = {}; history = []
paused = False
recorder = None; recorder_frames = 0; recorder_path = None
boxes = collision_boxes()

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def do_GET(self):
        with lock:
            payload = state.copy()
            if self.path == '/joints': payload = {k:v for k,v in payload.items() if k in ('t','names','q')}
            elif self.path == '/control': payload = {k:v for k,v in payload.items() if k in ('t','names','q','results','busy')}
        self.send_response(200); self.end_headers(); self.wfile.write(json.dumps(payload).encode())
    def do_POST(self):
        try:
            data = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            token = str(time.time_ns()); commands.put((token, data))
            self.send_response(200); self.end_headers(); self.wfile.write(json.dumps({'id':token}).encode())
        except Exception as error:
            self.send_response(400); self.end_headers(); self.wfile.write(str(error).encode())
server = ThreadingHTTPServer(('127.0.0.1', a.port), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
print('ARTICULATED_SIM_READY', json.dumps({'robot_dofs':names, 'asset_dofs':asset_names,
    'moving_path':moving_path, 'contact_target_path':contact_target_path,
    'joint_name':asset_joint_name, 'asset_xyz':asset_xyz.tolist(),
    'asset_quat_wxyz':asset_quat.tolist(), 'camera_T_B_C':T_B_C.tolist(),
    'handle_center_from_source':handle_center.tolist(),
    'camera_offset_m':a.camera_offset,
    'fixture_height_m':a.fixture_height_m}), flush=True)
try:
    while app.is_running():
        while not commands.empty():
            token, cmd = commands.get_nowait()
            try:
                op = cmd['op']
                if op == 'reset':
                    if active: raise RuntimeError('trajectory active')
                    target[arm] = HOME; target[fingers] = -.02
                    robot.set_joint_positions(target)
                    robot.set_joint_velocities(np.zeros(len(names)))
                    articulation.set_joint_positions(np.array([0.]))
                    history = []; results[token] = {'ok':True}
                elif op == 'capture':
                    depth = np.asarray(camera.get_depth(), dtype=float)
                    if depth.shape != (480, 640): raise RuntimeError('camera depth unavailable')
                    K = camera.get_intrinsics_matrix()
                    v, u = np.indices(depth.shape)
                    points = np.stack(((u-K[0,2])*depth/K[0,0],
                        (v-K[1,2])*depth/K[1,1], depth), axis=-1)
                    rgb = np.asarray(camera.get_rgba())[:,:,:3]
                    path = Path(cmd['path']).resolve(); path.parent.mkdir(parents=True,exist_ok=True)
                    np.savez_compressed(path, points=points.reshape(-1,3), rgb=rgb,
                        depth_m=depth, K=K, T_B_C=T_B_C)
                    results[token] = {'ok':True,'path':str(path),'valid_depth_pixels':int(np.isfinite(depth).sum())}
                    if cmd.get('evaluation_mask_path'):
                        segment = camera.get_current_frame()['instance_id_segmentation']
                        instance_ids = np.asarray(segment['data']).reshape(depth.shape)
                        labels = {int(key):str(value) for key,value in segment['info']['idToLabels'].items()}
                        # Asset-derived oracle is saved separately and is never an input to the full demo.
                        selected = [key for key,value in labels.items()
                                    if 'original29' in value.replace('-','').lower()]
                        evaluator_mask = np.isin(instance_ids,selected)
                        eval_path = Path(cmd['evaluation_mask_path']).resolve()
                        eval_path.parent.mkdir(parents=True,exist_ok=True)
                        np.save(eval_path,evaluator_mask)
                        results[token]['evaluation_only']={'path':str(eval_path),
                            'pixels':int(evaluator_mask.sum()),'matching_ids':selected,
                            'label_examples':dict(list(labels.items())[:5])}
                elif op == 'video_start':
                    if recorder is not None: raise RuntimeError('video already recording')
                    ffmpeg = shutil.which('ffmpeg')
                    if ffmpeg is None: raise RuntimeError('ffmpeg unavailable')
                    recorder_path = Path(cmd['path']).resolve()
                    recorder_path.parent.mkdir(parents=True, exist_ok=True)
                    recorder = subprocess.Popen([ffmpeg,'-hide_banner','-loglevel','error','-y',
                        '-f','rawvideo','-pixel_format','rgb24','-video_size','640x480',
                        '-framerate','30','-i','pipe:0','-c:v','libx264','-preset','veryfast',
                        '-crf','18','-pix_fmt','yuv420p',str(recorder_path)],
                        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                        stderr=(recorder_path.with_suffix('.ffmpeg.log')).open('w'))
                    recorder_frames = 0; results[token] = {'ok':True,'path':str(recorder_path)}
                elif op == 'video_stop':
                    if recorder is None: raise RuntimeError('video not recording')
                    recorder.stdin.close(); code = recorder.wait(timeout=60)
                    results[token] = {'ok':code==0,'frames':recorder_frames,'exit_code':code}
                    recorder = None
                elif op == 'trajectory':
                    if paused:raise RuntimeError('physics paused; resume before trajectory')
                    if active: raise RuntimeError('trajectory active')
                    idx = [names.index(n) for n in cmd['names']]
                    ts = np.array([point['t'] for point in cmd['points']])
                    ps = np.array([point['q'] for point in cmd['points']])
                    if not np.isfinite(ps).all() or not len(ts) or np.any(np.diff(ts)<=0):
                        raise ValueError('invalid trajectory')
                    if ts[0] > 0:
                        ts = np.r_[0,ts]; ps = np.vstack((robot.get_joint_positions()[idx],ps))
                    active = dict(id=token,start=tick,idx=idx,ts=ts,ps=ps,gripper=cmd.get('gripper',False))
                elif op == 'stop':
                    if active: results[active['id']] = {'ok':False,'reason':'canceled'}
                    active = None; target = robot.get_joint_positions().copy(); results[token] = {'ok':True}
                elif op == 'pause':
                    if active:raise RuntimeError('cannot pause during robot motion')
                    paused=True;results[token]={'ok':True}
                elif op == 'resume':
                    paused=False;results[token]={'ok':True}
                else: raise ValueError('unknown operation')
            except Exception as error:
                results[token] = {'ok':False,'reason':str(error)}
        if paused:
            with lock:
                state={**state,'paused':True,'results':results.copy(),'busy':False}
            time.sleep(.01)
            continue
        if active:
            elapsed = (tick - active['start']) * DT
            for i,j in enumerate(active['idx']):
                target[j] = np.interp(elapsed,active['ts'],active['ps'][:,i])
            if elapsed >= active['ts'][-1] + .3:
                error = float(np.max(np.abs(robot.get_joint_positions()[active['idx']]-active['ps'][-1])))
                if active['gripper'] or error < .025 or elapsed > active['ts'][-1]+3:
                    results[active['id']] = {'ok':active['gripper'] or error < .025,
                                             'joint_error':error}; active = None
        robot.apply_action(ArticulationAction(joint_positions=target))
        rendered = tick % 8 == 0
        world.step(render=rendered); tick += 1
        if recorder is not None and rendered:
            frame = np.asarray(camera.get_rgba())[:,:,:3]
            recorder.stdin.write(np.ascontiguousarray(frame,dtype=np.uint8).tobytes())
            recorder_frames += 1
        force = [float(np.linalg.norm(np.asarray(view.get_contact_force_matrix(dt=DT)).reshape(-1,3), axis=1).sum())
                 for view in finger_views]
        if rendered:
            boxes = collision_boxes()
        mp,mq = moving.get_world_pose(); lp,lq = door_link.get_world_pose()
        tp,tq = tcp.get_world_pose()
        aq = float(articulation.get_joint_positions()[0])
        sample = {'t':tick*DT,'joint_q':aq,'forces':force,'tcp':tp.tolist()}
        history.append(sample)
        if len(history)>2400: history=history[-2400:]
        with lock:
            state = {'t':tick*DT,'names':names,'q':robot.get_joint_positions().tolist(),
                'joint_name':asset_joint_name,'joint_q':aq,'joint_limits':manifest['source_joint_limits_rad'],
                'moving_link':'abstract_2_1','moving_pose':{'position':mp.tolist(),
                    'quaternion_wxyz':mq.tolist()},
                'door_link_pose':{'position':lp.tolist(),'quaternion_wxyz':lq.tolist()},
                'tcp':tp.tolist(),'tcp_quat':tq.tolist(),
                'forces':force,'results':results.copy(),'history':history[::8],
                'busy':active is not None,'camera_T_B_C':T_B_C.tolist(),
                'paused':False,
                'asset_root_pose':{'position':asset_xyz.tolist(),'quaternion_wxyz':asset_quat.tolist()},
                'collision_boxes':boxes,'handle_center_from_source':handle_center.tolist(),
                'fixture_height_m':a.fixture_height_m}
finally:
    if recorder is not None:
        recorder.stdin.close(); recorder.wait(timeout=60)
    server.shutdown(); app.close()
