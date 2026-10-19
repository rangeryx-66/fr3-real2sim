"""Bounded robot-only implicit-drive verification. Diagnostics never gate motion."""
from pathlib import Path
import os,json,time,hashlib,gzip,pickle,subprocess,signal
import numpy as np
from scipy.spatial.transform import Rotation
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--run',type=int,required=True,choices=[1,2,3]);args=parser.parse_args()
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/nero_actuator_stability_20261012'/('run_%02d'%args.run);O.mkdir(parents=True,exist_ok=False);start=time.perf_counter();os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y';stop=False
signal.signal(signal.SIGTERM,lambda *a:globals().update(stop=True));signal.signal(signal.SIGINT,lambda *a:globals().update(stop=True))
from nero_door_adapter import configuration,RobotPinkIK
c=configuration();c['urdf']=c['sim_urdf'];solver=RobotPinkIK(c)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':2,'physics_gpu':2,'multi_gpu':False});records=[];video=None;success=False;status='STARTING';dt=1/240;phase='INIT';tick=0
try:
 from isaacsim.core.api import World
 from isaacsim.core.prims import SingleArticulation,SingleXFormPrim
 from isaacsim.core.utils.stage import add_reference_to_stage
 from isaacsim.core.utils.types import ArticulationAction
 from isaacsim.asset.importer.urdf import URDFImporter,URDFImporterConfig
 from isaacsim.sensors.camera import Camera
 from pxr import UsdPhysics,PhysxSchema,PhysicsSchemaTools,Usd,Sdf
 import omni.usd
 from nero_actuator import apply_verified_filters
 from nero_implicit_actuator import ImplicitNero
 from articulated_demo.kinematics import URDFChain
 from precontact_trajectory import segment_samples
 world=World(physics_dt=dt,rendering_dt=1/30,backend='numpy',device='cpu');world.scene.add_default_ground_plane(z_position=-.76)
 usd=URDFImporter(URDFImporterConfig(urdf_path=c['sim_urdf'],usd_path=str(O/'native_asset'),fix_base=True,allow_self_collision=True,merge_fixed_joints=False,joint_drive_type='force',joint_target_type='position')).import_urdf();add_reference_to_stage(usd,'/World/Nero');stage=omni.usd.get_context().get_stage();filters=apply_verified_filters(stage,'/World/Nero',c);
 # TGS integrates external gravity at every substep, preventing the measured
 # final-substep velocity bias. No mass, force, gain, or collision changes.
 for scene in stage.Traverse():
  if scene.IsA(UsdPhysics.Scene):scene.CreateAttribute('physxScene:enableExternalForcesEveryIteration',Sdf.ValueTypeNames.Bool).Set(True)
 audit=json.loads((ROOT/'results/nero_actuator_stability_20261012/MEASURED_CONTACT_GEOMETRY_AUDIT.json').read_text());bodies={p.GetName():p for p in stage.Traverse() if str(p.GetPath()).startswith('/World/Nero') and p.HasAPI(UsdPhysics.RigidBodyAPI)}
 # Exact official meshes are clear over initial + all four saved paths. The
 # importer missed adjacency across link7's fixed flange when merge_fixed=False.
 if audit['all_raw_meshes_clear']:
  api=UsdPhysics.FilteredPairsAPI.Apply(bodies['link6']);api.CreateFilteredPairsRel().AddTarget(bodies['flange_link'].GetPath());filters.append({'pair':['link6','flange_link'],'reason':'joint7 adjacent rigid clusters; flange fixed to link7; independently audited raw-mesh gap','audit':'MEASURED_CONTACT_GEOMETRY_AUDIT.json'})
 (O/'VERIFIED_NATIVE_FILTERS.json').write_text(json.dumps(filters,indent=2));robot=world.scene.add(SingleArticulation('/World/Nero',name='nero'))
 camera=Camera('/World/probe_camera',resolution=(960,720),frequency=30);camera.set_focal_length(16.);eye=np.array([1.8,-2.,1.3]);forward=np.array([0.,0.,.65])-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);rot=np.column_stack([right,np.cross(forward,right),forward]);camera.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(rot).as_quat(),1),camera_axes='ros');
 for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
  if str(prim.GetPath()).startswith('/World/Nero') and prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
 world.reset();camera.initialize();names=list(robot.dof_names);arm=np.array([names.index(n) for n in c['arm_joint_names']]);fingers=np.array([names.index('gripper_joint1'),names.index('gripper_joint2')]);q=np.zeros(len(names));q[arm]=c['home_q'];q[fingers]=[.04,-.04];robot.set_joint_positions(q);robot.set_joint_velocities(np.zeros(len(q)));driver=ImplicitNero(robot,c,O);driver.raw.apply_action(ArticulationAction(joint_positions=q[fingers],joint_efforts=np.zeros(2),joint_indices=fingers));chain=URDFChain(c['sim_urdf'])
 paths={p.GetName():str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/World/Nero')};views={n:SingleXFormPrim(p) for n,p in paths.items()};offset=chain.root_to_link('gripper_base',{})
 offset=np.linalg.inv(offset)@chain.root_to_link('tcp_link',{})
 ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg';video=subprocess.Popen([ff,'-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23',str(O/'integration_continuous.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(O/'ffmpeg.log').open('w'))
 (O/'RUNTIME_MAPPING.json').write_text(json.dumps({'actual_native_joint_order':names,'arm_indices':arm.tolist(),'finger_indices':fingers.tolist(),'body_paths':paths,'physics_dt_s':dt,'URDF':c['sim_urdf'],'no_object_scene':True,'door_contact_episodes':0},indent=2))
 contact_rows=[];contact_window=[];contact_errors=[]
 for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
  if str(prim.GetPath()).startswith('/World/Nero') and prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
 import omni.physx
 def contact_callback(headers,data):
  for h in headers:
   try:
    a,b=str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))
    if not (a.startswith('/World/Nero') or b.startswith('/World/Nero')):continue
    for d in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
     row={'step':tick,'collider0':a,'collider1':b,'impulse_world_ns':[d.impulse.x,d.impulse.y,d.impulse.z],'point_world_m':[d.position.x,d.position.y,d.position.z],'normal_world':[d.normal.x,d.normal.y,d.normal.z],'separation_m':float(d.separation)}
     contact_rows.append(row);contact_window.append(row)
   except Exception as e:contact_errors.append(repr(e))
 contact_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_callback)
 optional_errors={}
 def optional(name):
  try:
   value=getattr(robot,name)();return None if value is None else np.asarray(value).tolist()
  except Exception as e:optional_errors[name]=repr(e);return None
 drives=[]
 for prim in stage.Traverse():
  if str(prim.GetPath()).startswith('/World/Nero') and prim.IsA(UsdPhysics.Joint):
   drives.append({'path':str(prim.GetPath()),'attributes':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if any(x in a.GetName() for x in ['drive:','lowerLimit','upperLimit','jointFriction','maxJointVelocity'])}})
 (O/'ACTIVE_DRIVES.json').write_text(json.dumps({'joints':drives,'gains': [np.asarray(x).tolist() for x in driver.raw.get_gains()],'max_efforts':robot._articulation_view.get_max_efforts().tolist(),'authored_limits':c['joint_limits'],'solver_type':world.get_physics_context().get_solver_type(),'solver_position_iterations':optional('get_solver_position_iteration_count'),'solver_velocity_iterations':optional('get_solver_velocity_iteration_count'),'external_forces_every_iteration':True},indent=2))
 files=[Path(__file__),ROOT/'scripts/nero_implicit_actuator.py',Path(c['sim_urdf']),Path(c['sim_urdf']).with_name('nero_dynamics_coordinates.urdf')]
 (O/'ACTIVE_SOURCE_HASHES.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
 def step():
  global tick
  if stop or (O/'STOP').exists():raise KeyboardInterrupt('OWNER_STOP')
  pre_q=np.asarray(robot.get_joint_positions()).copy();pre_dq=np.asarray(robot.get_joint_velocities()).copy();driver.command();contact_window.clear();world.step(render=False,update_fabric=True);q=np.asarray(robot.get_joint_positions());dq=np.asarray(robot.get_joint_velocities());p,r=views['gripper_base'].get_world_pose();T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(r,-1)).as_matrix();T[:3,3]=p;T=T@offset;fk=chain.root_to_link('tcp_link',dict(zip(names,q)));finger_poses={}
  for n in ['gripper_link1','gripper_link2']:
   fp,fq=views[n].get_world_pose();P=np.eye(4);P[:3,:3]=Rotation.from_quat(np.roll(fq,-1)).as_matrix();P[:3,3]=fp;finger_poses[n]=P.tolist()
  records.append({'t':tick*dt,'step':tick,'phase':phase,'q':q.tolist(),'dq':dq.tolist(),'q_des_arm':driver.desired.tolist(),'dq_des_arm':driver.desired_velocity.tolist(),'pre_q':pre_q.tolist(),'pre_dq':pre_dq.tolist(),'actuator':driver.last.copy(),'externally_applied_effort':optional('get_applied_joint_efforts'),'solver_projected_effort':optional('get_measured_joint_efforts'),'joint_reactions_local':optional('get_measured_joint_forces'),'contacts':contact_window.copy(),'physics_dt_s':dt,'T_tcp':T.tolist(),'FK_position_error_m':float(np.linalg.norm(T[:3,3]-fk[:3,3])),'FK_orientation_error_rad':float(Rotation.from_matrix(T[:3,:3]@fk[:3,:3].T).magnitude()),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'finger_mimic_error_m':float(q[fingers[0]]+q[fingers[1]]),'finger_poses':finger_poses});tick+=1
  if tick%8==0:
   world.render()
   try:video.stdin.write(np.ascontiguousarray(camera.get_rgba()[:,:,:3]).astype(np.uint8).tobytes())
   except Exception as e:optional_errors['video']=repr(e)
  if tick%8==0 and not (O/'FIRST_FRAME.png').exists():
   try:
    from PIL import Image
    frame=camera.get_rgba()
    if frame is not None:Image.fromarray(np.ascontiguousarray(frame[:,:,:3]).astype(np.uint8)).save(O/'FIRST_FRAME.png')
   except Exception as e:optional_errors['first_frame']=repr(e)
  if tick%240==0:(O/'PROGRESS.json').write_text(json.dumps({'phase':phase,'sim_s':tick*dt,'pid':os.getpid(),'aperture_m':records[-1]['aperture_m'],'joint_error_max_rad':float(np.max(abs(q[arm]-driver.desired)))},indent=2))
 def hold(seconds):
  for _ in range(int(seconds/dt)):step()
 phase='GRAVITY_HOLD';hold(3.)
 phase='SMOOTH_JOINT_TRACKING';end=np.array(c['home_q']);end[0]+=.08;end[3]-=.06;end[5]+=.06
 for command,velocity in segment_samples(solver,np.array(c['home_q']),end,4.,dt,.02):driver.desired=command;driver.desired_velocity=velocity;step()
 driver.desired_velocity=np.zeros(7);hold(2.)
 if args.run==3:
  # Existing full plans are replayed without object; no re-planning or new poses.
  for asset in ['45134','45443','45176','45415']:
   plan=json.loads((ROOT/'results/nero_feasibility_20261012/offline'/asset/'candidate.json').read_text())
   phase='RETURN_HOME_'+asset
   for command,velocity in segment_samples(solver,driver.desired.copy(),np.array(c['home_q']),2.,dt,.04):driver.desired=command;driver.desired_velocity=velocity;step()
   route=plan['preplan'][1:]+plan['approach'][1:]
   opening=plan['path']
   # Saved opening path rows may carry angle/q fields; keep exact joint targets.
   route += [x['q'] if isinstance(x,dict) else x for x in opening]
   phase='REPRESENTATIVE_PLAN_'+asset
   for target in route:
    for command,velocity in segment_samples(solver,driver.desired.copy(),np.array(target),.1,dt,.04):driver.desired=command;driver.desired_velocity=velocity;step()
   driver.desired_velocity=np.zeros(7);hold(1.)
   # Return along the same verified route rather than interpolating new branches.
   for target in route[::-1]+[c['home_q']]:
    for command,velocity in segment_samples(solver,driver.desired.copy(),np.array(target),.1,dt,.04):driver.desired=command;driver.desired_velocity=velocity;step()
 phase='GRIPPER_CLOSE_OPEN'
 for startw,endw in [(.08,.02),(.02,.08)]:
  for w in np.linspace(startw,endw,480):driver.aperture=float(w);step()
  hold(.5)
 phase='GRAVITY_HOLD_AFTER_MOTION';driver.desired_velocity=np.zeros(7);hold(2.);success=True;status='PROBE_COMPLETE'
 if args.run==3:
  # Actual cooperative owner request after all verification motions complete.
  (O/'READY_FOR_OWNER_STOP').write_text(str(tick))
  phase='FINAL_HOLD_AWAIT_OWNER_STOP'
  for _ in range(2400):step()
except BaseException as e:
 import traceback;print(traceback.format_exc(),flush=True);status=repr(e)
finally:
 try:world.pause()
 except Exception:pass
 for name,data in [('observations',records),('contacts',contact_rows if 'contact_rows' in globals() else [])]:
  dest=O/(name+'_chunks');dest.mkdir(exist_ok=True);idx=[]
  for j in range(0,len(data),1024):
   p=dest/('%08d.pkl.gz'%j)
   with gzip.open(p,'wb',compresslevel=1) as f:pickle.dump(data[j:j+1024],f,protocol=5)
   idx.append({'file':p.name,'records':min(1024,len(data)-j)})
  (dest/'index.json').write_text(json.dumps({'records':len(data),'chunks':idx,'lossless':True},indent=2))
 try:video.stdin.close();video.wait(timeout=30)
 except Exception:pass
 result={'status':status,'completed':success,'sim_s':tick*dt,'wall_s':time.perf_counter()-start,'physics_steps':len(records),'door_contact_episodes':0,'max_FK_position_error_m':max((r['FK_position_error_m'] for r in records),default=None),'max_FK_orientation_error_rad':max((r['FK_orientation_error_rad'] for r in records),default=None),'max_finger_mimic_error_m':max((abs(r['finger_mimic_error_m']) for r in records),default=None),'max_joint_tracking_error_rad':max((float(np.max(abs(np.asarray(r['q'])[arm]-r['q_des_arm']))) for r in records),default=None),'hold_last_joint_error_rad':float(np.max(abs(np.asarray(records[-1]['q'])[arm]-records[-1]['q_des_arm']))) if records else None,'maximum_TCP_speed_m_s':max((float(np.linalg.norm(np.array(b['T_tcp'])[:3,3]-np.array(a['T_tcp'])[:3,3])/dt) for a,b in zip(records,records[1:])),default=None),'all_telemetry_diagnostic_only':True,'optional_errors':optional_errors if 'optional_errors' in globals() else {},'contact_callback_errors':contact_errors if 'contact_errors' in globals() else [],'native_contact_records':len(contact_rows) if 'contact_rows' in globals() else 0,'shutdown':'world.pause, lossless chunks flush, ffmpeg EOF, app.close'};(O/'PROBE_REPORT.json').write_text(json.dumps(result,indent=2));print(result,flush=True);(O/'SHUTDOWN_FLUSHED.json').write_text(json.dumps({'pid':os.getpid(),'world_paused':True,'records':len(records),'app_close_requested':True}));app.close()
