"""Isaac Sim physics plant. No IK, grasp heuristics, pose attachment or planning."""
import argparse
import hashlib
import json
import os
import queue
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
p=argparse.ArgumentParser()
p.add_argument('--gpu',type=int,default=1)
p.add_argument('--port',type=int,default=18765)
p.add_argument('--clutter',action='store_true')
p.add_argument('--object-id',default='benchmark_box')
p.add_argument('--arena-target')
a=p.parse_args()
ROOT=Path(__file__).resolve().parents[1]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':a.gpu,'physics_gpu':a.gpu,'multi_gpu':False})
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder, FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, RigidPrim, SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from pxr import UsdGeom, UsdPhysics, PhysxSchema
import omni.usd
DT=1/240
HOME=np.array([0.,1.3,1.0,-1.3,0.,0.,0.])
OBJECTS={'benchmark_box':dict(id='benchmark_box',shape='box',size=[.045,.045,.05],mass=.06)}
OBJECTS.update({o['id']:o for o in json.loads((ROOT/'config/r1a7_generalization_objects.json').read_text())})
ARENA_TARGET=a.arena_target
if ARENA_TARGET:
    ARENA_INVENTORY=json.loads((ROOT/'assets/arena_complex/inventory.json').read_text())
    ARENA_PROTOCOL=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())
    ARENA_EPISODES={episode['seed']:episode for episode in ARENA_PROTOCOL['episodes']}
    if ARENA_TARGET not in ARENA_PROTOCOL['classes']:raise ValueError('unknown Arena target '+ARENA_TARGET)
    os.environ['FR3_ARENA_TARGET']=ARENA_TARGET
    bounds=np.array(ARENA_INVENTORY[ARENA_TARGET]['bounds'])
    OBJECT=dict(id=ARENA_TARGET,shape='arena_mesh',size=(bounds[1]-bounds[0]).tolist(),
                registry_name=ARENA_INVENTORY[ARENA_TARGET]['registry_name'],
                usd_path=ARENA_INVENTORY[ARENA_TARGET]['usd_path'])
    initial=next(ep for ep in ARENA_EPISODES.values() if ep['target']==ARENA_TARGET)
else:
    if a.object_id not in OBJECTS:raise ValueError('unknown object '+a.object_id)
    OBJECT=OBJECTS[a.object_id]
SIZE=np.array(OBJECT.get('size',[2*OBJECT['radius'],2*OBJECT['radius'],OBJECT['height']]) if OBJECT['shape']=='cylinder' else OBJECT['size'],dtype=float)
BOX=np.array(initial['objects'][0]['position'] if ARENA_TARGET else [.5,0,SIZE[2]/2])
OBJECT_YAW=0.
def yaw_quat(yaw):return np.array([np.cos(yaw/2),0.,0.,np.sin(yaw/2)])
BASE_POSE=np.array([float(v) for v in os.environ.get('R1A7_BASE_POSE','0,0,0,90').split(',')])
PEDESTAL_SIZE=np.array([float(v) for v in os.environ.get('R1A7_PEDESTAL_SIZE','0.10,0.10,0.20').split(',')])
assert BASE_POSE.shape==(4,) and PEDESTAL_SIZE.shape==(3,) and np.all(PEDESTAL_SIZE>0)
T_B_C=np.diag([1.,-1.,-1.,1.]); T_B_C[:3,3]=[.5,0,.8]
world=World(stage_units_in_meters=1.,physics_dt=DT,rendering_dt=1/30,backend='numpy',device='cpu')
world.scene.add_default_ground_plane(z_position=-.76 if ARENA_TARGET else -.06)
mat=PhysicsMaterial('/World/grasp_material',static_friction=0.8,dynamic_friction=0.7,restitution=0.0)
if ARENA_TARGET:
    import arena_scene
    scene_stage=omni.usd.get_context().get_stage()
    box=arena_scene.table(world,scene_stage,mat)
    arena_clutter=[arena_scene.spawn(world,scene_stage,mat,f'clutter_{i}',spec)
                   for i,spec in enumerate(initial['objects'][1:])]
else:
    world.scene.add(FixedCuboid('/World/table',name='table',position=[.5,0,-.025],scale=[.7,.7,.05],physics_material=mat))
    arena_clutter=[]
if BASE_POSE[2]>0:
    world.scene.add(FixedCuboid('/World/r1a7_pedestal',name='r1a7_pedestal',
        position=[BASE_POSE[0],BASE_POSE[1],PEDESTAL_SIZE[2]/2],
        scale=PEDESTAL_SIZE,physics_material=mat))
    print('PEDESTAL',json.dumps(dict(base=BASE_POSE.tolist(),size=PEDESTAL_SIZE.tolist())),flush=True)
if ARENA_TARGET:
    pass
elif OBJECT['shape']=='cylinder':
    box=world.scene.add(DynamicCylinder('/World/box',name='box',position=BOX,radius=float(OBJECT['radius']),height=float(OBJECT['height']),mass=float(OBJECT['mass']),color=np.array([.8,.12,.08]),physics_material=mat))
else:
    box=world.scene.add(DynamicCuboid('/World/box',name='box',position=BOX,scale=SIZE,mass=float(OBJECT['mass']),color=np.array([.8,.12,.08]),physics_material=mat))
print('OBJECT',json.dumps(OBJECT),flush=True)
asset=ROOT/'assets'
asset.mkdir(exist_ok=True)
model_hash=hashlib.sha256((ROOT/'config/r1a7_dex1.urdf').read_bytes()).hexdigest()[:12]
asset_file=asset/f'r1a7_dex1_filtered_asset_path_{model_hash}.txt'
if asset_file.exists() and Path(asset_file.read_text().strip()).exists():
    usd=asset_file.read_text().strip()
else:
    # The official Dex1 STL parts overlap at fixed-joint interfaces. MoveIt
    # checks self collision using the full collision meshes and SRDF adjacency;
    # PhysX internal pair contacts are disabled to avoid wedging the fingers.
    usd=URDFImporter(URDFImporterConfig(urdf_path=str(ROOT/'config/r1a7_dex1.urdf'),usd_path=str(asset),fix_base=True,allow_self_collision=False,merge_fixed_joints=False,joint_drive_type='force',joint_target_type='position')).import_urdf()
    asset_file.write_text(usd)
add_reference_to_stage(usd,'/World/R1A7')
stage=omni.usd.get_context().get_stage()
robot=world.scene.add(SingleArticulation('/World/R1A7',name='r1a7'))
finger_paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName() in ['dex1_Link1_3','dex1_Link2_3'] and p.HasAPI(UsdPhysics.RigidBodyAPI)]
print('FINGER_PATHS',finger_paths,flush=True)
assert len(finger_paths)==2, finger_paths
for path in finger_paths:
    prim=stage.GetPrimAtPath(path)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
_finger_views=[world.scene.add(RigidPrim(prim_paths_expr=path,name=f'r1a7_contact_{i}',contact_filter_prim_paths_expr=['/World/box'],track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=256)) for i,path in enumerate(finger_paths)]
palm_paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='dex1_base_link']
assert len(palm_paths)==1,palm_paths
palm=SingleXFormPrim(palm_paths[0])
clutter_enabled=True
tcp_paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='r1a7_tcp']
assert len(tcp_paths)==1,tcp_paths
tcp=SingleXFormPrim(tcp_paths[0])
camera=Camera('/World/camera',position=T_B_C[:3,3],resolution=(640,480),frequency=30)
camera.set_world_pose(position=T_B_C[:3,3],orientation=np.array([0,1,0,0]),camera_axes='ros')
camera.set_clipping_range(.05,2.)
from clutter_scene import ClutterMonitor
clutter=ClutterMonitor(world,stage,mat) if a.clutter else None
world.reset()
camera.initialize()
camera.add_distance_to_image_plane_to_frame()
if ARENA_TARGET:camera.add_instance_id_segmentation_to_frame()
names=robot.dof_names
arm=[names.index('J'+str(i)) for i in range(1,8)]
fingers=[names.index('dex1_Joint1_1'),names.index('dex1_Joint2_1')]
# Unitree's URDF supplies inertia/effort/velocity but no Isaac drive gains.
# Select force-mode position gains for this model, then log the actual values.
# These are independent of the FR3 hand and remain subject to the URDF effort limits.
controller=robot.get_articulation_controller()
kp=np.zeros(len(names));kd=np.zeros(len(names))
kp[arm]=[500.,500.,400.,400.,250.,150.,150.]
kd[arm]=[40.,40.,32.,32.,22.,15.,15.]
kp[fingers]=800.;kd[fingers]=30.
controller.set_gains(kps=kp,kds=kd,save_to_usd=False)
baseline_kp,baseline_kd=robot.get_articulation_controller().get_gains()
baseline_kp=np.array(baseline_kp,copy=True);baseline_kd=np.array(baseline_kd,copy=True)
print('BASELINE_GAINS',json.dumps(dict(names=names,kp=baseline_kp.tolist(),kd=baseline_kd.tolist())),flush=True)
q=np.zeros(len(names));q[arm]=HOME;q[fingers]=-.02
robot.set_joint_positions(q)
robot.apply_action(ArticulationAction(joint_positions=q))
for _ in range(240): world.step(render=(_%8==0))
commands=queue.Queue(); state={}; lock=threading.Lock(); tick=240; active=None; target=q.copy(); results={}; history=[]
seed=initial['seed'] if ARENA_TARGET else None
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def do_GET(self):
        with lock:
            payload=state.copy()
            if self.path=='/joints':payload={k:v for k,v in payload.items() if k in ['t','names','q']}
            elif self.path=='/control':payload={k:v for k,v in payload.items() if k in ['t','names','q','results','busy']}
        self.send_response(200);self.end_headers();self.wfile.write(json.dumps(payload).encode())
    def do_POST(self):
        try:
            data=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            token=str(time.time_ns()); commands.put((token,data))
            self.send_response(200);self.end_headers();self.wfile.write(json.dumps({'id':token}).encode())
        except Exception as e:
            self.send_response(400);self.end_headers();self.wfile.write(str(e).encode())
server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler)
threading.Thread(target=server.serve_forever,daemon=True).start()
print('SIM_READY',names,flush=True)
try:
    while app.is_running():
        while not commands.empty():
            token,cmd=commands.get_nowait()
            try:
                op=cmd['op']
                if op=='reset':
                    if active: raise RuntimeError('trajectory active')
                    if ARENA_TARGET:
                        seed=int(cmd['seed']);episode=ARENA_EPISODES[seed]
                        if episode['target']!=ARENA_TARGET:raise ValueError('Arena target/seed mismatch')
                        spec=episode['objects'][0];BOX[:]=spec['position']
                        OBJECT_YAW=float(2*np.arctan2(spec['quaternion_wxyz'][3],spec['quaternion_wxyz'][0]))
                        box.set_world_pose(spec['position'],spec['quaternion_wxyz'])
                        box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])
                        for body,other in zip(arena_clutter,episode['objects'][1:]):
                            body.set_world_pose(other['position'],other['quaternion_wxyz'])
                            body.set_linear_velocity([0,0,0]);body.set_angular_velocity([0,0,0])
                    else:
                        xy=np.asarray(cmd.get('xy',[.5,0.]),dtype=float)
                        if xy.shape!=(2,) or not (.37<=xy[0]<=.62 and -.14<=xy[1]<=.14):raise ValueError('target placement outside benchmark workspace')
                        OBJECT_YAW=float(cmd.get('yaw',0.))
                        if not np.isfinite(OBJECT_YAW):raise ValueError('invalid object yaw')
                        BOX[:]=[xy[0],xy[1],SIZE[2]/2]
                        box.set_world_pose(BOX,yaw_quat(OBJECT_YAW));box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])
                    target[arm]=HOME;target[fingers]=-.02
                    robot.set_joint_positions(target);robot.set_joint_velocities(np.zeros(len(names)))
                    history=[]
                    if clutter:
                        clutter.reset(cmd.get('seed',0))
                        if not clutter_enabled:
                            for i,body in enumerate(clutter.bodies):body.set_world_pose([3.+i,3.,float(clutter.bodies[i].get_world_pose()[0][2])],[1,0,0,0])
                    results[token]={'ok':True}
                elif op=='clutter_enabled':
                    clutter_enabled=bool(cmd['enabled']);results[token]={'ok':True}
                elif op=='arm_gains':
                    if active:raise RuntimeError('cannot change gains during trajectory')
                    controller=robot.get_articulation_controller();kp,kd=controller.get_gains()
                    kp=np.array(kp,copy=True);kd=np.array(kd,copy=True)
                    if 'kp' in cmd:
                        if not 0<float(cmd['kp'])<=40000 or not 0<float(cmd['kd'])<=1000:raise ValueError('invalid arm gains')
                        kp[arm]=float(cmd['kp']);kd[arm]=float(cmd['kd']);controller.set_gains(kps=kp,kds=kd,save_to_usd=False)
                    actual_kp,actual_kd=controller.get_gains()
                    results[token]=dict(ok=True,names=names,kp=np.asarray(actual_kp).tolist(),kd=np.asarray(actual_kd).tolist())
                elif op=='arm_gain_scale':
                    if active:raise RuntimeError('cannot change gains during trajectory')
                    scale=float(cmd['scale'])
                    if not .5<=scale<=4:raise ValueError('invalid arm gain scale')
                    kp=baseline_kp.copy();kd=baseline_kd.copy();kp[arm]*=scale;kd[arm]*=np.sqrt(scale)
                    controller=robot.get_articulation_controller();controller.set_gains(kps=kp,kds=kd,save_to_usd=False)
                    actual_kp,actual_kd=controller.get_gains()
                    results[token]=dict(ok=True,names=names,scale=scale,kp=np.asarray(actual_kp).tolist(),kd=np.asarray(actual_kd).tolist())
                elif op=='arm_metrics':
                    if not clutter:raise RuntimeError('clutter mode required')
                    clutter.arm(tick*DT);results[token]={'ok':True}
                elif op=='phase':
                    if clutter:clutter.phase=cmd['phase']
                    results[token]={'ok':True}
                elif op=='capture':
                    depth=camera.get_depth()
                    if depth is None: raise RuntimeError('camera depth unavailable')
                    K=camera.get_intrinsics_matrix()
                    v,u=np.indices(depth.shape)
                    pts=np.stack(((u-K[0,2])*depth/K[0,0],(v-K[1,2])*depth/K[1,1],depth),axis=-1).reshape(-1,3)
                    valid=np.isfinite(pts).all(axis=1)&(pts[:,2]>.05)&(pts[:,2]<1.5)
                    pts=pts[valid]
                    xyz=pts@T_B_C[:3,:3].T+T_B_C[:3,3]
                    bp,_=box.get_world_pose()
                    rel=xyz-bp
                    cy,sy=np.cos(OBJECT_YAW),np.sin(OBJECT_YAW)
                    local=np.stack((cy*rel[:,0]+sy*rel[:,1],-sy*rel[:,0]+cy*rel[:,1],rel[:,2]),axis=-1)
                    if ARENA_TARGET:mask=arena_scene.target_mask(camera,valid)
                    elif OBJECT['shape']=='cylinder':mask=(np.linalg.norm(local[:,:2],axis=1)<=float(OBJECT['radius'])+.002)&(np.abs(local[:,2])<=SIZE[2]/2+.002)
                    else:mask=(np.abs(local)<=SIZE/2+.002).all(axis=1)
                    path=ROOT/'results'/f'{token}_cloud.npz'
                    if ARENA_TARGET:np.savez_compressed(path,points=pts,mask=mask,T_B_C=T_B_C,K=K,rgb=camera.get_rgba())
                    else:np.savez_compressed(path,points=pts,mask=mask,T_B_C=T_B_C,K=K)
                    results[token]={'ok':True,'path':str(path),'object_points':int(mask.sum())}
                elif op=='trajectory':
                    if active: raise RuntimeError('trajectory active')
                    idx=[names.index(n) for n in cmd['names']]
                    ts=np.array([p['t'] for p in cmd['points']]);ps=np.array([p['q'] for p in cmd['points']])
                    if not np.isfinite(ps).all() or len(ts)<1 or np.any(np.diff(ts)<=0): raise ValueError('invalid trajectory')
                    if ts[0]>0: ts=np.r_[0,ts];ps=np.vstack((robot.get_joint_positions()[idx],ps))
                    active=dict(id=token,start=tick,idx=idx,ts=ts,ps=ps,gripper=cmd.get('gripper',False))
                elif op=='stop':
                    if active: results[active['id']]={'ok':False,'reason':'canceled'}
                    active=None;target=robot.get_joint_positions().copy();results[token]={'ok':True}
                else: raise ValueError('unknown operation')
            except Exception as e: results[token]={'ok':False,'reason':str(e)}
        if active:
            elapsed=(tick-active['start'])*DT
            for i,j in enumerate(active['idx']): target[j]=np.interp(elapsed,active['ts'],active['ps'][:,i])
            if elapsed>=active['ts'][-1]+.3:
                error=float(np.max(np.abs(robot.get_joint_positions()[active['idx']]-active['ps'][-1])))
                ok=active['gripper'] or error<.025
                if ok or elapsed>active['ts'][-1]+3:
                    results[active['id']]={'ok':bool(ok),'joint_error':error}
                    active=None
        robot.apply_action(ArticulationAction(joint_positions=target))
        world.step(render=tick%(24 if clutter else 8)==0);tick+=1
        f=np.array([float(np.linalg.norm(np.asarray(view.get_contact_force_matrix(dt=DT)).reshape(-1,3).sum(axis=0))) for view in _finger_views])
        bp,bq=box.get_world_pose();tp,tq=tcp.get_world_pose()
        measured_q=robot.get_joint_positions()
        sample=dict(t=tick*DT,z=float(bp[2]),forces=f.tolist(),box=bp.tolist(),tcp=tp.tolist(),
                    arm_tracking_error_rad=float(np.max(np.abs(target[arm]-measured_q[arm]))))
        history.append(sample)
        if len(history)>2400: history=history[-2400:]
        if ARENA_TARGET:
            clutter_state={'obstacles':[dict(id=f'clutter_{i}',position=body.get_world_pose()[0].tolist(),
                                      quaternion_wxyz=body.get_world_pose()[1].tolist(),
                                      aabb_size=(np.array(ARENA_INVENTORY[spec['asset']]['bounds'])[1]-np.array(ARENA_INVENTORY[spec['asset']]['bounds'])[0]).tolist())
                                        for i,(body,spec) in enumerate(zip(arena_clutter,ARENA_EPISODES[seed]['objects'][1:]))]}
        else:clutter_state=clutter.sample(tick*DT,bp) if clutter else None
        with lock:
            state=dict(t=tick*DT,names=names,q=measured_q.tolist(),box=bp.tolist(),box_quat=bq.tolist(),object={**OBJECT,'size':SIZE.tolist(),'yaw':OBJECT_YAW},tcp=tp.tolist(),tcp_quat=tq.tolist(),forces=f.tolist(),results=results.copy(),history=history[::8],busy=active is not None,clutter=clutter_state)
finally:
    server.shutdown();app.close()
