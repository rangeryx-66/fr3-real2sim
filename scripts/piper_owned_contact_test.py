"""Native collider-ownership closure/pull diagnostic, isolated from articulated demo."""
import argparse,json,os,sys,subprocess,shutil
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.cooked_geometry import export_cooked
from piper_mobile_demo.contact_ownership import install_owned_colliders,NativeOwnershipReports


def matrix(p,q):
 T=np.eye(4);T[:3,3]=p;T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();return T


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--measured-report',type=Path,required=True);p.add_argument('--pull-plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--gpu',type=int,default=7);p.add_argument('--deadline-shanghai',default='2026-10-02T05:00:00+08:00');p.add_argument('--geometry-python',default='/data1/home/rangeryx/.conda/envs/anygrasp/bin/python');p.add_argument('--ownership',type=Path,default=ROOT/'config/piper_contact_ownership.json');p.add_argument('--width-m',type=float,default=.024);p.add_argument('--length-m',type=float,default=.016);p.add_argument('--thickness-m',type=float,default=.006)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);r=json.loads(a.measured_report.read_text());pull=json.loads(a.pull_plan.read_text());base=r['base_final'];zone=ZoneInfo('Asia/Shanghai');deadline=datetime.fromisoformat(a.deadline_shanghai)
 if datetime.now(zone)>=deadline:raise RuntimeError('CUTOFF_05_00')
 os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
 from isaacsim import SimulationApp
 app=SimulationApp({'headless':True,'active_gpu':a.gpu,'physics_gpu':a.gpu,'multi_gpu':False})
 result={'kind':'canonical analytic locked handle, no articulated-object replacement; diagnostic only','base':base,'geometry_m':{'length':a.length_m,'width':a.width_m,'thickness':a.thickness_m},'friction':[.5,.5],'finger_effort_limit_n':10,'changes_to_formal_asset':False,'legal_pad_only_closure':False}
 observations=[];video=None;native=None
 try:
  from isaacsim.core.api import World
  from isaacsim.core.api.objects import FixedCuboid
  from isaacsim.core.api.materials import PhysicsMaterial
  from isaacsim.core.prims import SingleArticulation,SingleXFormPrim,RigidPrim
  from isaacsim.core.utils.stage import add_reference_to_stage
  from isaacsim.core.utils.types import ArticulationAction
  from isaacsim.sensors.camera import Camera
  from pxr import UsdGeom,UsdPhysics,Usd
  import omni.usd,trimesh,cv2
  from articulated_demo.kinematics import URDFChain
  world=World(stage_units_in_meters=1.,physics_dt=1/240,rendering_dt=1/30,backend='numpy',device='cpu');world.scene.add_default_ground_plane(z_position=-.76);stage=omni.usd.get_context().get_stage()
  tablemat=PhysicsMaterial('/World/table_material',static_friction=.8,dynamic_friction=.7,restitution=0)
  world.scene.add(FixedCuboid('/World/table',name='table',position=[.5,0,-.025],scale=[.9,.9,.05],physics_material=tablemat))
  import hashlib
  model_hash=hashlib.sha256((ROOT/'config/piper_sim.urdf').read_bytes()).hexdigest()[:12]
  cache=ROOT/'assets'/f'piper_mobile_asset_path_{model_hash}.txt'
  if not cache.is_file():raise RuntimeError('official coupled PiPER USD cache missing; run contact geometry audit first')
  add_reference_to_stage(cache.read_text().strip(),'/World/Piper');robot=world.scene.add(SingleArticulation('/World/Piper',name='piper'))
  robot.set_world_pose(base[:3],np.roll(Rotation.from_euler('z',base[3],degrees=True).as_quat(),1))
  manifest=json.loads(a.ownership.read_text());owned=install_owned_colliders(stage,manifest);result['owned_colliders']=owned;result['ownership_manifest']=str(a.ownership);result['official_surface_rule']='independent official DAE distal surface, four STL triangle IDs; NOT material color or a point tolerance'
  def link(name):
   ps=[p for p in stage.Traverse() if p.GetName()==name and str(p.GetPath()).startswith('/World/Piper') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
   assert len(ps)==1,(name,len(ps));return str(ps[0].GetPath())
  chain=URDFChain(ROOT/'config/piper.urdf');qclosed=np.array(r['actual_closure']['robot_q']);joint_names=r['joint_names'];values=dict(zip(joint_names,qclosed));values.update(gripper_joint1=.05,gripper_joint2=-.05)
  B=np.eye(4);B[:3,:3]=Rotation.from_euler('z',base[3],degrees=True).as_matrix();B[:3,3]=base[:3];T=B@chain.root_to_link('tcp_link',values)
  pads={};fingerframes={};centers=[]
  for name in ['gripper_link1','gripper_link2']:
   mesh=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{name}.stl',force='mesh');tri=mesh.triangles[manifest['fingers'][name]['official_pad_face_ids']];pads[name]=np.array([tri.reshape(-1,3).min(0),tri.reshape(-1,3).max(0)])
   P=B@chain.root_to_link(name,values);centers.append((P@np.r_[pads[name].mean(0),1])[:3]);fingerframes[name]=SingleXFormPrim(link(name))
  center=np.mean(centers,axis=0);orientation=np.roll(Rotation.from_matrix(T[:3,:3]).as_quat(),1)
  # Exact analytical box, same dimensions in USD and FCL. Center comes solely
  # from official flat pad centroids, not from object-specific offsets.
  handlemat=PhysicsMaterial('/World/handle_material',static_friction=.5,dynamic_friction=.5,restitution=0)
  handle=world.scene.add(FixedCuboid('/World/canonical_handle',name='canonical_handle',position=center,orientation=orientation,scale=[a.length_m,a.width_m,a.thickness_m],physics_material=handlemat))
  from pxr import PhysxSchema
  box_contact_offset=.02*min(a.length_m,a.width_m,a.thickness_m)
  PhysxSchema.PhysxCollisionAPI.Apply(stage.GetPrimAtPath('/World/canonical_handle')).CreateContactOffsetAttr(box_contact_offset)
  result['canonical_contact_offset_m']=box_contact_offset;result['canonical_geometry_scope']='diagnostic box fits official distal surface with 2 mm tangential exploration allowance; formal handle dimensions untouched'
  result['handle_world_transform']=matrix(center,orientation).tolist();result['center_rule']='mean of official planar finger-pad centroids at open state';result['pull_kind']='locked analytic handle; 2 mm tangential load, not freely moving object success'
  views=[];bodyviews=[]
  for name in ['gripper_link1','gripper_link2']:
   views.append((name,world.scene.add(RigidPrim(link(name),name='contact_'+name,contact_filter_prim_paths_expr=['/World/canonical_handle'],track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=4096))))
  for name in ['gripper_base','link6']:
   bodyviews.append(world.scene.add(RigidPrim(link(name),name='body_'+name,contact_filter_prim_paths_expr=['/World/canonical_handle'],track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=4096)))
  from isaacsim.core.utils.viewports import set_camera_view
  camera=world.scene.add(Camera('/World/camera',name='overview',frequency=30,resolution=(1280,960)))
  world.reset();camera.initialize();set_camera_view(eye=center+np.array([.45,-.55,.28]),target=center,camera_prim_path='/World/camera')
  names=robot.dof_names;assert names==joint_names,(names,joint_names);arm=[names.index(f'joint{i}') for i in range(1,7)];fingers=[names.index('gripper_joint1'),names.index('gripper_joint2')]
  controller=robot.get_articulation_controller();kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000;kd[arm]=400;kp[fingers]=1000;kd[fingers]=40;controller.set_gains(kps=kp,kds=kd,save_to_usd=True)
  qtarget=qclosed.copy();qtarget[fingers]=[.05,-.05];robot.set_joint_positions(qtarget);robot.set_joint_velocities(np.zeros_like(qtarget))
  for prim in stage.Traverse():
   if prim.GetName() in ['gripper_joint1','gripper_joint2'] and prim.IsA(UsdPhysics.PrismaticJoint):
    cap=UsdPhysics.DriveAPI.Get(prim,'linear').GetMaxForceAttr().Get()
    if cap is None or abs(float(cap)-10.)>1e-6:raise RuntimeError('official effort cap mismatch')
  result['start_mode']='direct initialize to previously measured arm posture; closure/pull physically simulated';result['robot_q_initial']=qtarget.tolist()
  import xml.etree.ElementTree as ET
  root=ET.parse(ROOT/'config/piper.urdf').getroot();limits=np.array([[float(root.find(f"joint[@name='joint{i}']/limit").get(k)) for k in ['lower','upper']] for i in range(1,7)])
  video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','16','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(a.output/'diagnostic.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
  phase='SETTLE';tick=0
  def sample_physics():
   q=np.asarray(robot.get_joint_positions());poses={}
   for n,view in views:
    p,r=view.get_world_poses();poses[n]=matrix(p[0],r[0]).tolist()
   return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':float(np.min(np.minimum(q[arm]-limits[:,0],limits[:,1]-q[arm]))),'finger_world_poses':poses}
  native=NativeOwnershipReports(stage, ['/World/canonical_handle'],1/240,world=world,sample_provider=sample_physics)
  def step():
   nonlocal tick
   if datetime.now(zone)>=deadline:raise RuntimeError('CUTOFF_05_00')
   pre_poses={n:matrix(*frame.get_world_pose()).tolist() for n,frame in fingerframes.items()};native.clear();controller.apply_action(ArticulationAction(joint_positions=qtarget));world.step(render=tick%8==0)
   q=np.asarray(robot.get_joint_positions());margin=float(np.min(np.minimum(q[arm]-limits[:,0],limits[:,1]-q[arm])));contacts=[];forces=[]
   ownership=native.state();contacts=ownership['contacts'];forces=[ownership['pad_forces_n'][name] for name,_ in views]
   row={'t':tick/240,'physics_time_s':ownership['physics_time_s'],'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':margin,'finger_forces_n':forces,'contacts':contacts,'ownership':ownership,'pre_step_finger_world_poses':pre_poses,'geometry_pose_time':'after physics step; native contact generation precedes integration','finger_world_poses':{name:matrix(*frame.get_world_pose()).tolist() for name,frame in fingerframes.items()},'body_force_n':sum(float(np.linalg.norm(v.get_contact_force_matrix(dt=1/240))) for v in bodyviews)};observations.append(row)
   if tick%8==0 and np.asarray(camera.get_rgba()).ndim==3:
    image=np.asarray(camera.get_rgba())[:,:,:3].copy();cv2.rectangle(image,(0,0),(1280,80),(10,10,10),-1);cv2.putText(image,'CANONICAL LOCKED HANDLE / DIAGNOSTIC ONLY',(12,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1);cv2.putText(image,f'{phase}: aperture={row["aperture_m"]*1000:.2f} mm; force={forces[0]:.2f}/{forces[1]:.2f} N',(12,63),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);video.stdin.write(np.ascontiguousarray(image).tobytes())
   tick+=1
   if margin<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
   if row['body_force_n']>.01:raise RuntimeError('CANONICAL_PALM_CONTACT')
   if ownership['metal_contacts']:raise RuntimeError('NATIVE_METAL_CONTACT')
   return row
  for _ in range(240):step()
  export_cooked(stage,a.output/'cooked_open.json')
  phase='CLOSE'
  for opening in np.linspace(.1,0,480):qtarget[fingers]=[opening/2,-opening/2];step()
  phase='CLOSURE_HOLD'
  for _ in range(240):closed=step()
  result['actual_closure']=closed;export_cooked(stage,a.output/'cooked_closed.json')
  subprocess.run(['env','-u','PYTHONPATH',a.geometry_python,str(ROOT/'scripts/validate_piper_owned_contact.py'),str(a.output/'cooked_closed.json'),str(a.output/'closure_validation.json'),'--ownership',str(a.ownership),'--target-body','canonical_handle'],check=True)
  validation=json.loads((a.output/'closure_validation.json').read_text())
  result['legal_pad_only_closure']=min(closed['ownership']['pad_forces_n'].values())>=.2 and closed['ownership']['metal_contacts']==0 and validation['geometry_safe']
  if not result['legal_pad_only_closure']:raise RuntimeError('OWNED_CLOSURE_NONPAD_OR_BAD_CONTACT')
  phase='PULL_DIAGNOSTIC'
  for item in pull['waypoints']:
   start=qtarget[arm].copy()
   for fraction in np.linspace(0,1,60):qtarget[arm]=start+fraction*(np.asarray(item['q'])-start);step()
   pass # Static cooked geometry is replayed with measured poses; avoid cooking queries during loaded motion.
  phase='HOLD'
  for _ in range(240):step()
  export_cooked(stage,a.output/'cooked_final.json')
  result['status']='NATIVE_PAD_ONLY_CLOSURE_AND_2MM_PULL_PENDING_FULL_REPLAY'
 except BaseException as error:
  import traceback
  result['status']=str(error);result['traceback']=traceback.format_exc();print(result['traceback'],flush=True)
 finally:
  result['native_metal_contact_samples']=sum(x['ownership']['metal_contacts']>0 for x in observations);result['native_contact_count']=sum(len(x['contacts']) for x in observations);result['physics_step_samples']=len(native.physics_steps) if native is not None else 0;(a.output/'physics_steps.json').write_text(json.dumps(native.physics_steps if native is not None else []));result['sample_count']=len(observations);result['minimum_joint_margin_rad']=min((x['margin_rad'] for x in observations),default=None)
  (a.output/'report.json').write_text(json.dumps(result,indent=2));(a.output/'observations.json').write_text(json.dumps(observations));print(json.dumps({k:v for k,v in result.items() if k!='actual_closure'},indent=2),flush=True)
  if video:video.stdin.close();video.wait(timeout=30)
  app.close()
if __name__=='__main__':main()
