"""One no-object integration session. No door/contact episode is consumed."""
from pathlib import Path
import os,json,time,hashlib,gzip,pickle,subprocess,signal
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/nero_feasibility_20261012/integration_probe';O.mkdir(exist_ok=True);start=time.perf_counter();os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y';stop=False
signal.signal(signal.SIGTERM,lambda *a:globals().update(stop=True));signal.signal(signal.SIGINT,lambda *a:globals().update(stop=True))
from nero_door_adapter import configuration,RobotPinkIK
c=configuration();c['urdf']=c['sim_urdf'];solver=RobotPinkIK(c)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':5,'physics_gpu':5,'multi_gpu':False});records=[];video=None;success=False;status='STARTING';dt=1/240;phase='INIT';tick=0
try:
 from isaacsim.core.api import World
 from isaacsim.core.prims import SingleArticulation,SingleXFormPrim
 from isaacsim.core.utils.stage import add_reference_to_stage
 from isaacsim.core.utils.types import ArticulationAction
 from isaacsim.asset.importer.urdf import URDFImporter,URDFImporterConfig
 from isaacsim.sensors.camera import Camera
 from pxr import UsdPhysics
 import omni.usd
 from nero_actuator import FiniteEffortNero,apply_verified_filters
 from articulated_demo.kinematics import URDFChain
 from precontact_trajectory import segment_samples
 world=World(physics_dt=dt,rendering_dt=1/30,backend='numpy',device='cpu');world.scene.add_default_ground_plane(z_position=-.76)
 usd=URDFImporter(URDFImporterConfig(urdf_path=c['sim_urdf'],usd_path=str(O/'native_asset'),fix_base=True,allow_self_collision=True,merge_fixed_joints=False,joint_drive_type='force',joint_target_type='position')).import_urdf();add_reference_to_stage(usd,'/World/Nero');stage=omni.usd.get_context().get_stage();filters=apply_verified_filters(stage,'/World/Nero',c);(O/'VERIFIED_NATIVE_FILTERS.json').write_text(json.dumps(filters,indent=2));robot=world.scene.add(SingleArticulation('/World/Nero',name='nero'))
 camera=Camera('/World/probe_camera',resolution=(960,720),frequency=30);camera.set_world_pose(position=np.array([1.1,-1.1,.9]),orientation=np.roll(Rotation.from_euler('xyz',[60,0,45],degrees=True).as_quat(),1));world.reset();camera.initialize();names=list(robot.dof_names);arm=np.array([names.index(n) for n in c['arm_joint_names']]);fingers=np.array([names.index('gripper_joint1'),names.index('gripper_joint2')]);q=np.zeros(len(names));q[arm]=c['home_q'];q[fingers]=[.04,-.04];robot.set_joint_positions(q);robot.set_joint_velocities(np.zeros(len(q)));driver=FiniteEffortNero(robot,world,c,O);driver.raw.apply_action(ArticulationAction(joint_positions=q[fingers],joint_efforts=np.zeros(2),joint_indices=fingers));chain=URDFChain(c['sim_urdf'])
 paths={p.GetName():str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/World/Nero')};views={n:SingleXFormPrim(p) for n,p in paths.items()};offset=chain.root_to_link('gripper_base',{})
 offset=np.linalg.inv(offset)@chain.root_to_link('tcp_link',{})
 ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg';video=subprocess.Popen([ff,'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23',str(O/'integration_continuous.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(O/'ffmpeg.log').open('w'))
 (O/'RUNTIME_MAPPING.json').write_text(json.dumps({'actual_native_joint_order':names,'arm_indices':arm.tolist(),'finger_indices':fingers.tolist(),'body_paths':paths,'physics_dt_s':dt,'URDF':c['sim_urdf'],'no_object_scene':True,'door_contact_episodes':0},indent=2))
 def step():
  global tick
  if stop:raise KeyboardInterrupt('OWNER_STOP')
  world.step(render=False,update_fabric=True);q=np.asarray(robot.get_joint_positions());dq=np.asarray(robot.get_joint_velocities());p,r=views['gripper_base'].get_world_pose();T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(r,-1)).as_matrix();T[:3,3]=p;T=T@offset;fk=chain.root_to_link('tcp_link',dict(zip(names,q)));finger_poses={}
  for n in ['gripper_link1','gripper_link2']:
   fp,fq=views[n].get_world_pose();P=np.eye(4);P[:3,:3]=Rotation.from_quat(np.roll(fq,-1)).as_matrix();P[:3,3]=fp;finger_poses[n]=P.tolist()
  records.append({'t':tick*dt,'step':tick,'phase':phase,'q':q.tolist(),'dq':dq.tolist(),'q_des_arm':driver.desired.tolist(),'dq_des_arm':driver.desired_velocity.tolist(),'tau_arm_nm':driver.last_torque.tolist(),'T_tcp':T.tolist(),'FK_position_error_m':float(np.linalg.norm(T[:3,3]-fk[:3,3])),'FK_orientation_error_rad':float(Rotation.from_matrix(T[:3,:3]@fk[:3,:3].T).magnitude()),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'finger_mimic_error_m':float(q[fingers[0]]+q[fingers[1]]),'finger_poses':finger_poses});tick+=1
  if tick%8==0:
   world.render()
   try:video.stdin.write(np.ascontiguousarray(camera.get_rgba()[:,:,:3]).astype(np.uint8).tobytes())
   except Exception:pass
  if tick%240==0:(O/'PROGRESS.json').write_text(json.dumps({'phase':phase,'sim_s':tick*dt,'pid':os.getpid(),'aperture_m':records[-1]['aperture_m'],'joint_error_max_rad':float(np.max(abs(q[arm]-driver.desired)))},indent=2))
 def hold(seconds):
  for _ in range(int(seconds/dt)):step()
 phase='GRAVITY_HOLD';hold(4.)
 phase='SMOOTH_JOINT_TRACKING';end=np.array(c['home_q']);end[0]+=.08;end[3]-=.06;end[5]+=.06
 for command,velocity in segment_samples(solver,np.array(c['home_q']),end,4.,dt,.02):driver.desired=command;driver.desired_velocity=velocity;step()
 driver.desired_velocity=np.zeros(7);hold(2.)
 phase='GRIPPER_CLOSE_OPEN'
 for startw,endw in [(.08,.02),(.02,.08)]:
  for w in np.linspace(startw,endw,960):driver.raw.apply_action(ArticulationAction(joint_positions=np.array([w/2,-w/2]),joint_efforts=np.zeros(2),joint_indices=fingers));step()
  hold(.5)
 phase='GRAVITY_HOLD_AFTER_MOTION';hold(2.);success=True;status='PROBE_COMPLETE'
except BaseException as e:
 import traceback;print(traceback.format_exc(),flush=True);status=repr(e)
finally:
 try:world.pause()
 except Exception:pass
 for name,data in [('observations',records),('actuator',driver.records if 'driver' in globals() else [])]:
  dest=O/(name+'_chunks');dest.mkdir(exist_ok=True);idx=[]
  for j in range(0,len(data),1024):
   p=dest/('%08d.pkl.gz'%j)
   with gzip.open(p,'wb',compresslevel=1) as f:pickle.dump(data[j:j+1024],f,protocol=5)
   idx.append({'file':p.name,'records':min(1024,len(data)-j)})
  (dest/'index.json').write_text(json.dumps({'records':len(data),'chunks':idx,'lossless':True},indent=2))
 try:video.stdin.close();video.wait(timeout=30)
 except Exception:pass
 result={'status':status,'completed':success,'sim_s':tick*dt,'wall_s':time.perf_counter()-start,'physics_steps':len(records),'door_contact_episodes':0,'max_FK_position_error_m':max((r['FK_position_error_m'] for r in records),default=None),'max_FK_orientation_error_rad':max((r['FK_orientation_error_rad'] for r in records),default=None),'max_finger_mimic_error_m':max((abs(r['finger_mimic_error_m']) for r in records),default=None),'max_joint_tracking_error_rad':max((float(np.max(abs(np.asarray(r['q'])[arm]-r['q_des_arm']))) for r in records),default=None),'hold_last_joint_error_rad':float(np.max(abs(np.asarray(records[-1]['q'])[arm]-records[-1]['q_des_arm']))) if records else None,'maximum_TCP_speed_m_s':max((float(np.linalg.norm(np.array(b['T_tcp'])[:3,3]-np.array(a['T_tcp'])[:3,3])/dt) for a,b in zip(records,records[1:])),default=None),'all_telemetry_diagnostic_only':True};(O/'PROBE_REPORT.json').write_text(json.dumps(result,indent=2));print(result,flush=True);app.close()
