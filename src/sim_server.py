"""Isaac Sim physics plant. No IK, grasp heuristics, pose attachment or planning."""
import argparse
import os
import json
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
a=p.parse_args()
ROOT=Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0,str(ROOT/'src'))
from robot_profile import get_profile,gripper_positions
PROFILE=get_profile()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':a.gpu,'physics_gpu':a.gpu,'multi_gpu':False})
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, RigidPrim, SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from pxr import UsdGeom, UsdPhysics, PhysxSchema
import omni.usd
DT=1/240
HOME=np.asarray(PROFILE.home,dtype=float)
BOX=np.array([float(os.environ.get('GRASP_SCENE_X','.5')),0,.025])
SIZE=np.array([.045,.045,.05])
T_B_C=np.diag([1.,-1.,-1.,1.]); T_B_C[:3,3]=[.5,0,.8]
world=World(stage_units_in_meters=1.,physics_dt=DT,rendering_dt=1/30,backend='numpy',device='cpu')
world.scene.add_default_ground_plane(z_position=-.06)
mat=PhysicsMaterial('/World/grasp_material',static_friction=0.8,dynamic_friction=0.7,restitution=0.0)
world.scene.add(FixedCuboid('/World/table',name='table',position=[.5,0,-.025],scale=[.7,.7,.05],physics_material=mat))
box=world.scene.add(DynamicCuboid('/World/box',name='box',position=BOX,scale=SIZE,mass=.06,color=np.array([.8,.12,.08]),physics_material=mat))
asset=ROOT/'assets'
asset.mkdir(exist_ok=True)
asset_file=asset/f'{PROFILE.name}_asset_path.txt'
drive_kp={'.*joint[1-7]':10000.,'.*finger_joint.*':1000.} if PROFILE.name=='fr3' else {'.*joint[1-6]':10000.,'.*gripper.*':1000.}
drive_kd={'.*joint[1-7]':400.,'.*finger_joint.*':40.} if PROFILE.name=='fr3' else {'.*joint[1-6]':400.,'.*gripper.*':40.}
if asset_file.exists() and Path(asset_file.read_text().strip()).exists():
    usd=asset_file.read_text().strip()
else:
    usd=URDFImporter(URDFImporterConfig(urdf_path=str(PROFILE.urdf),usd_path=str(asset),fix_base=True,allow_self_collision=True,merge_fixed_joints=False,joint_drive_type='force',joint_target_type='position',override_joint_stiffness=drive_kp,override_joint_damping=drive_kd)).import_urdf()
    asset_file.write_text(usd)
add_reference_to_stage(usd,PROFILE.usd_prim)
stage=omni.usd.get_context().get_stage()
robot=world.scene.add(SingleArticulation(PROFILE.usd_prim,name=PROFILE.name))
finger_paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName() in PROFILE.touch_links and p.HasAPI(UsdPhysics.RigidBodyAPI)]
print('FINGER_PATHS',finger_paths,flush=True)
assert len(finger_paths)==2, finger_paths
for path in finger_paths:
    prim=stage.GetPrimAtPath(path)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
if PROFILE.name=='fr3':
    contacts=world.scene.add(RigidPrim(prim_paths_expr=finger_paths[0].rsplit('/',1)[0]+'/fr3_.*finger',name='contacts',contact_filter_prim_paths_expr=['/World/box'],track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=512))
else:
    _finger_views=[world.scene.add(RigidPrim(prim_paths_expr=path,name=f'piper_contact_{i}',contact_filter_prim_paths_expr=['/World/box'],track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=256)) for i,path in enumerate(finger_paths)]
    class _PairContacts:
        prim_paths=finger_paths
        def get_contact_force_matrix(self,dt):
            return np.concatenate([np.asarray(v.get_contact_force_matrix(dt=dt)).reshape(1,-1,3) for v in _finger_views],axis=0)
        def get_contact_force_data(self,dt):
            rows=[];offset=0
            for v in _finger_views:
                force,point,normal,separation,count,index=[np.asarray(x) for x in v.get_contact_force_data(dt=dt)]
                rows.append((force,point,normal,separation,count,index+offset));offset+=len(force)
            return tuple(np.concatenate([r[i] for r in rows],axis=0) for i in range(6))
    contacts=_PairContacts()
from contact_trace import ContactTrace
trace=ContactTrace(contacts)
palm_paths=([finger_paths[0].rsplit('/',1)[0]] if PROFILE.name=='piper' else [str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='fr3_hand'])
assert len(palm_paths)==1,palm_paths
palm=SingleXFormPrim(palm_paths[0])
clutter_enabled=True
tcp_paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()==PROFILE.tcp_link]
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
names=robot.dof_names
arm=[names.index(n) for n in PROFILE.arm_joints]
fingers=[names.index(n) for n in PROFILE.physical_finger_joints if n in names]
gripper_dofs={n:names.index(n) for n in gripper_positions(PROFILE,PROFILE.open_width_m) if n in names}
baseline_kp,baseline_kd=robot.get_articulation_controller().get_gains()
baseline_kp=np.array(baseline_kp,copy=True);baseline_kd=np.array(baseline_kd,copy=True)
print('BASELINE_GAINS',json.dumps(dict(names=names,kp=baseline_kp.tolist(),kd=baseline_kd.tolist())),flush=True)
q=np.zeros(len(names));q[arm]=HOME
for n,v in gripper_positions(PROFILE,PROFILE.open_width_m).items():
    if n in gripper_dofs:q[gripper_dofs[n]]=v
robot.set_joint_positions(q)
robot.apply_action(ArticulationAction(joint_positions=q))
for _ in range(240): world.step(render=(_%8==0))
commands=queue.Queue(); state={}; lock=threading.Lock(); tick=240; active=None; target=q.copy(); results={}; history=[]
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
                    box.set_world_pose(BOX,[1,0,0,0]);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])
                    target[arm]=HOME
                    for n,v in gripper_positions(PROFILE,PROFILE.open_width_m).items():
                        if n in gripper_dofs:target[gripper_dofs[n]]=v
                    robot.set_joint_positions(target);robot.set_joint_velocities(np.zeros(len(names)))
                    history=[]
                    if clutter:
                        clutter.reset(cmd.get('seed',0))
                        if not clutter_enabled:
                            for i,body in enumerate(clutter.bodies):body.set_world_pose([3.+i,3.,float(clutter.bodies[i].get_world_pose()[0][2])],[1,0,0,0])
                    results[token]={'ok':True}
                elif op=='clutter_enabled':
                    clutter_enabled=bool(cmd['enabled']);results[token]={'ok':True}
                elif op=='trace_start':
                    trace.start(cmd['path']);results[token]={'ok':True}
                elif op=='trace_stop':results[token]=trace.stop(names)
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
                    mask=(np.abs(xyz-bp)<=SIZE/2+.002).all(axis=1)
                    path=ROOT/'results'/f'{token}_cloud.npz'
                    np.savez_compressed(path,points=pts,mask=mask,T_B_C=T_B_C,K=K)
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
        forces=contacts.get_contact_force_matrix(dt=DT)
        f=np.linalg.norm(np.asarray(forces).reshape(2,-1,3).sum(axis=1),axis=1)
        bp,bq=box.get_world_pose();tp,tq=tcp.get_world_pose()
        if trace.path is not None:
            pp,pq=palm.get_world_pose()
            trace.sample(tick*DT,clutter.phase if clutter else 'IDLE',robot.get_joint_positions(),target,bp,bq,tp,tq,pp,pq,f)
        sample=dict(t=tick*DT,z=float(bp[2]),forces=f.tolist(),box=bp.tolist(),tcp=tp.tolist())
        history.append(sample)
        if len(history)>2400: history=history[-2400:]
        clutter_state=clutter.sample(tick*DT,bp) if clutter else None
        with lock:
            state=dict(t=tick*DT,names=names,q=robot.get_joint_positions().tolist(),box=bp.tolist(),box_quat=bq.tolist(),tcp=tp.tolist(),tcp_quat=tq.tolist(),forces=f.tolist(),results=results.copy(),history=history[::8],busy=active is not None,clutter=clutter_state)
finally:
    server.shutdown();app.close()
