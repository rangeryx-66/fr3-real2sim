"""45746 explicit physical regrasp milestone; existing contact control and telemetry.
Known-model diagnostic. Real closure/contact; no object state commands.
"""
import time as _profile_clock
_profile_entry = _profile_clock.perf_counter()
from piper_mobile_execute import *
from wrist_reconstruction.planner import camera_clearance
from wrist_reconstruction.geometry import calibration
import time
telemetry_issues={};telemetry_disabled=set();telemetry_events=[];telemetry_counts={};telemetry_previous={}
def diagnostic_issue(key,error):
 try:
  item=telemetry_issues.setdefault(key,{'count':0,'first_error':repr(error)});item['count']+=1
 except Exception:pass
def diagnostic_write(path,value):
 try:path.write_text(value)
 except Exception as error:diagnostic_issue('write:'+str(path),error)
# Passive timing only. These values are never consumed by manipulation.
_profile_phase='initialization';_profile_phase_start=_profile_entry
_profile_data={};_profile_prev_command=None;_profile_activity='auto';_profile_costs={}
def profile_mark(name):
 global _profile_phase,_profile_phase_start
 try:
  now=time.perf_counter();d=_profile_data.setdefault(_profile_phase,{})
  d['wall_s']=d.get('wall_s',0.)+now-_profile_phase_start
  _profile_phase=name;_profile_phase_start=now
  profile_save()
 except Exception as error:diagnostic_issue('profile_mark',error)
def profile_save():
 try:
  diagnostic_write(a.output/'timing_profile.json',json.dumps({'classification':'DIAGNOSTIC_ONLY','active_phase':_profile_phase,'wall_since_entry_s':time.perf_counter()-_profile_entry,'phases':_profile_data,'costs_wall_s':_profile_costs,'activity_definition':'explicit move/release/base interpolation versus explicit hold; closure command changes versus stationary command. Physics steps and control dt unchanged.'},indent=2))
 except Exception as error:diagnostic_issue('profile_save',error)
def profile_step_begin():
 global _profile_step_start,_profile_motion,_profile_prev_command
 try:
  _profile_step_start=time.perf_counter()
  command=np.r_[qtarget,np.asarray(base)]
  _profile_motion=(_profile_activity=='motion' or (_profile_activity=='auto' and (_profile_prev_command is None or np.any(np.abs(command-_profile_prev_command)>1e-12))))
  if _profile_activity=='dwell':_profile_motion=False
  _profile_prev_command=command.copy()
 except Exception as error:diagnostic_issue('profile_step_begin',error)
def profile_step_end():
 try:
  d=_profile_data.setdefault(_profile_phase,{})
  k='active_command' if _profile_motion else 'stationary_command'
  d[k+'_sim_s']=d.get(k+'_sim_s',0.)+dt
  d[k+'_step_wall_s']=d.get(k+'_step_wall_s',0.)+time.perf_counter()-_profile_step_start
  d['steps']=d.get('steps',0)+1
 except Exception as error:diagnostic_issue('profile_step_end',error)
def profile_cost(name,started):
 try:_profile_costs[name]=_profile_costs.get(name,0.)+time.perf_counter()-started
 except Exception as error:diagnostic_issue('profile_cost',error)
def refresh_same_base_collision(scene, export, initial, base):
 # Reuse only invariant solids. Rebuild the exact same platform solids with
 # the original transforms. PhysicalScene.check/contact_guard are unchanged.
 from interactive_twin_recovery.mobile import transform as base_transform
 from piper_mobile_demo.owned_scene import Shape,meshes
 import trimesh
 delta=base_transform(base)@np.linalg.inv(base_transform(initial));replacement={}
 for entry in export['shapes']:
  if 'pedestal' not in entry['path']:continue
  updated=dict(entry);updated['world_transform']=(delta@np.asarray(entry['world_transform'])).tolist()
  scene.entries[entry['path']]=updated
  replacement[entry['path']]=iter([Shape(mesh,np.asarray(updated['world_transform']),True,entry['path'],entry.get('rigid_body_path','').split('/')[-1]) for mesh in meshes(entry)])
 B=base_transform(base);B[:3,3]=[base[0],base[1],-.66]
 replacement['/World/mobile_chassis']=iter([Shape(trimesh.creation.box([.34,.30,.20]),B,True,'/World/mobile_chassis','mobile_chassis')])
 scene.scene=[next(replacement[item.path]) if item.path in replacement else item for item in scene.scene]
 return scene
def export_raw_records(name,records):
 try:
  if not chunked_export:
   diagnostic_write(a.output/(name+'.json'),json.dumps(records));return
  import gzip,pickle
  folder=a.output/(name+'_chunks');folder.mkdir(exist_ok=True)
  chunks=[]
  for start in range(0,len(records),1024):
   target=folder/('%08d.pkl.gz'%start)
   with gzip.open(target,'wb',compresslevel=1) as stream:pickle.dump(records[start:start+1024],stream,protocol=5)
   chunks.append({'file':target.name,'first_record':start,'records':min(1024,len(records)-start),'bytes':target.stat().st_size})
  diagnostic_write(folder/'index.json',json.dumps({'format':'gzip level 1 / Python pickle protocol 5','records':len(records),'lossless_raw_information':True,'chunks':chunks,'read_note':'Only unpickle these trusted locally generated experiment files.'},indent=2))
 except Exception as error:
  diagnostic_issue('raw_export:'+name,error)
  try:diagnostic_write(a.output/(name+'.json'),json.dumps(records))
  except Exception as fallback_error:diagnostic_issue('raw_export_fallback:'+name,fallback_error)
from types import SimpleNamespace
p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);args=p.parse_args();job=json.loads(args.job.read_text());whole=json.loads(Path(job['whole_task_plan']).read_text())
a=SimpleNamespace(source=Path(job['source']),asset_root=Path(job['asset_root']),plan=Path(job['plan']),output=Path(job['output']),gpu=job['gpu'],trial=True,candidate=0,no_operation=False,deadline_shanghai=job['deadline_shanghai'],ownership=None)
a.output.mkdir(parents=True,exist_ok=True)
# Job timing parameters only; frozen physics, geometry and confirmation remain unchanged.
speed_scale=float(job.get('timing_experiment',{}).get('speed_scale',1.));shorten_endpoint_holds=bool(job.get('timing_experiment',{}).get('shorten_endpoint_holds',False));chunked_export=bool(job.get('timing_experiment',{}).get('chunked_export',False))
kind=job['skill']['joint_type'];base_moves=0;regrasp_count=0;initial_base=list(whole['base']);max_state=0.;first_divergence=None;command_state=0.;planned_reference=None

source=json.loads((a.source/'report.json').read_text());base=source['robot_base_pose'];plan=json.loads(a.plan.read_text());chosen=plan['trial_candidates'][0]
if base!=chosen['base']:raise RuntimeError('BASE_CHANGED')
a.ownership=None;from articulated_interaction_skill.scene import install
install()
from interactive_twin_refinement.assembly import install_setup_capture
install_setup_capture()
from interactive_twin.plant import bootstrap_job
scene=bootstrap_job(a,base,job);world=scene['world'];app=scene['app'];robot=scene['robot'];stage=scene['stage'];arm=scene['arm'];fingers=scene['fingers'];names=scene['names'];controller=scene['controller'];dt=scene['DT']
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from piper_mobile_demo.model import Model
from piper_mobile_demo.cooked_geometry import export_cooked
from articulated_interaction.physical_baseline import PhysicalScene,WholeFingerReports,FINGERS
from articulated_interaction.control import JawCenteredClosure
from articulated_demo.kinematics import transform
import cv2
deadline=datetime.fromisoformat(a.deadline_shanghai)
model=Model(ROOT/'config/piper.urdf',a.asset_root,source);model.deadline_timestamp=deadline.timestamp()
export_cooked(stage,a.output/'cooked_initial.json');export=json.loads((a.output/'cooked_initial.json').read_text())
allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]
if len(allowed)!=len(model.manifest['interaction_geometry']['pieces']):raise RuntimeError('FROZEN_PROXY_ASSOCIATION_FAILED')
collision=PhysicalScene(export,model,allowed)
meta=model.manifest['interaction_geometry']['selection'];normal=model.asset_T[:3,:3]@np.asarray(meta['outward_normal_root']);axis=model.asset_T[:3,:3]@np.asarray(meta['axis_root'])
section=min(meta['sections'],key=lambda s:abs(s['fraction']));anchor_world=(model.asset_T@np.r_[section['anchor_root_m'],1])[:3]
moving_initial=collision.moving_reference;anchor_link=(np.linalg.inv(moving_initial)@np.r_[anchor_world,1])[:3]
# Freeze and record the existing approximation; no proxy regeneration.
fingerprint={'manifest_sha256':hashlib.sha256((a.asset_root/'manifest.json').read_bytes()).hexdigest(),'asset_urdf_sha256':hashlib.sha256(model.asset_urdf.read_bytes()).hexdigest(),'geometry':'existing bar box plus two support primitives; approximate, not calibrated real geometry','selection':meta}
try:(a.output/'frozen_proxy.json').write_text(json.dumps(fingerprint,indent=2))
except Exception as error:diagnostic_issue('optional_write',error)
# At most 12 local changes in this same bar-side-pinch family.
variants=[(0,0,0)]
slide,depth,roll=variants[a.candidate];T=np.asarray(chosen['T']);T[:3,3]+=slide*axis+depth*T[:3,2];T[:3,:3]=Rotation.from_rotvec(axis*np.deg2rad(roll)).as_matrix()@T[:3,:3]
actual_views=dict(scene['scene_monitor_views']);B=transform(base[:3],[0,0,np.deg2rad(base[3])]);chain=model.chain
positions=dict(zip(names,robot.get_joint_positions()));offset=np.linalg.inv(chain.root_to_link('gripper_base',positions))@chain.root_to_link('tcp_link',positions)
def moving():
 p,q=scene['door_contacts'].get_world_poses();return matrix(p[0],q[0])
def tcp():
 p,q=actual_views['gripper_base'].get_world_poses();return matrix(p[0],q[0])@offset
def poses(q):return model.poses(np.asarray(q)[arm],base,finger_q=np.asarray(q)[fingers])
qtarget=np.asarray(robot.get_joint_positions(),dtype=float).copy();qtarget[fingers]=[.05,-.05];qvelocity=np.zeros(6);phase='SETTLE';rows=[];tick=0;reference=None;legal=False;success=False;pull2=False;status='STARTED';loss_s=0.;slip_s=0.;preflight={}
urdf=ET.parse(ROOT/'config/piper.urdf').getroot();vel=np.array([float(urdf.find(f"joint[@name='joint{i}']/limit").get('velocity')) for i in range(1,7)]);limits=model.limits
from wrist_reconstruction.force_policy import TemporalForceGuard
force_guard=TemporalForceGuard(job['wrist_experiment']['force_policy'])
policy=json.loads((ROOT/'config/semantic_interaction.json').read_text());policy['slow_closure_m_s']=.001*speed_scale;mode='position';effort=0.
near=scene['overview'][1];eye=anchor_world+normal*.45+np.array([0,0,.20]);focus=anchor_world+np.array([0,0,.02]);forward=(focus-eye)/np.linalg.norm(focus-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward));near.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros');near.set_clipping_range(.02,3.)
try:video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/'contact_baseline.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/'ffmpeg.log','w'))
except Exception as error:video=None;diagnostic_issue('video_start',error)
def optional_read(label,call):
 if label in telemetry_disabled:return None
 try:
  value=call()
  if value is None:raise ValueError('API returned None')
  return np.asarray(value).tolist()
 except Exception as error:
  diagnostic_issue(label,error);telemetry_disabled.add(label);return None
def observe_step(s,contacts):
 try:
  h={'physics_step_index':tick-1,'step_start_s':s['t'],'post_step_simulation_time_s':float(native.physics_time),'physics_dt_s':contacts.get('physics_window_dt_s',dt),'physics_substeps':contacts.get('physics_substeps'),'sensor_period_s':dt,'sensor_rate_hz':1/dt,
     'command_q_all':qtarget.tolist(),'command_q_velocity_arm':qvelocity.tolist(),'command_aperture_m':float(qtarget[fingers[0]]-qtarget[fingers[1]]),'measured_aperture_m':s['aperture_m'],'command_finger_efforts':([0.,0.] if mode=='position' else [-effort,effort]),'gripper_control_mode':mode,
     'joint_velocities':optional_read('joint_velocities',robot.get_joint_velocities),'applied_joint_efforts':optional_read('applied_joint_efforts',lambda:robot.get_applied_joint_efforts()),'measured_joint_efforts':optional_read('measured_joint_efforts',lambda:robot.get_measured_joint_efforts()),'measured_joint_forces':optional_read('measured_joint_forces',lambda:robot.get_measured_joint_forces()),
     'drawer_velocity_m_s':optional_read('drawer_velocity_m_s',lambda:scene['articulation'].get_joint_velocities()[scene.get('selected_asset_dof',0)]),'base_pose':list(base),'command_base_pose':list(base),'base_motion_state':phase,'grasp_index':regrasp_count,'pad_load_raw_n':dict(s['forces_n']),'pad_load_filtered_n':filtered().tolist(),'raw_contact_records':s['contacts']}
  h['pad_contact']={n:float(s['forces_n'][n])>.05 for n in FINGERS}
  h['finger_net_impulse_world_ns']={n:np.sum([c['impulse_world_ns'] for c in s['contacts'] if c['finger']==n and c['allowed_pad_target']],axis=0).tolist() if any(c['finger']==n and c['allowed_pad_target'] for c in s['contacts']) else [0.,0.,0.] for n in FINGERS}
  h['finger_net_force_world_n']={n:(np.asarray(v)/h['physics_dt_s']).tolist() for n,v in h['finger_net_impulse_world_ns'].items()}
  # No telemetry field is read back by the control policy.
  s['telemetry']=h
  for k,v in h.items():
   if v is not None:telemetry_counts[k]=telemetry_counts.get(k,0)+1
  if telemetry_previous.get('phase')!=phase:
   telemetry_events.append({'event':'PHASE_CHANGE','from':telemetry_previous.get('phase'),'to':phase,'t':s['t'],'step':tick-1,'displacement_m':s['drawer_displacement_m'],'base_moves':base_moves,'regrasp_count':regrasp_count})
   if phase=='RELEASE':telemetry_events.append({'event':'RELEASE_START','t':s['t'],'step':tick-1,'displacement_m':s['drawer_displacement_m'],'pad_loads_n':dict(s['forces_n'])})
   if phase=='BASE_ROUTE':telemetry_events.append({'event':'BASE_MOTION_START','t':s['t'],'step':tick-1,'base':list(base)})
   if telemetry_previous.get('phase')=='BASE_ROUTE':telemetry_events.append({'event':'BASE_MOTION_END','t':s['t'],'step':tick-1,'base':list(base)})
  for n,present in h['pad_contact'].items():
   if telemetry_previous.get(n)!=present:telemetry_events.append({'event':'PAD_CONTACT_ACQUIRED' if present else 'PAD_CONTACT_LOST','finger':n,'t':s['t'],'step':tick-1,'phase':phase,'grasp_index':regrasp_count,'raw_load_n':s['forces_n'][n]})
   telemetry_previous[n]=present
  telemetry_previous['phase']=phase
 except Exception as error:diagnostic_issue('observe_step',error)
def observe_grasp():
 try:telemetry_events.append({'event':'GRASP_VERIFIED','t':rows[-1]['t'],'step':tick-1,'displacement_m':rows[-1]['drawer_displacement_m'],'grasp_index':base_moves,'pad_loads_n':filtered().tolist(),'base':list(base)})
 except Exception as error:diagnostic_issue('observe_grasp',error)
def sample():
 q=np.asarray(robot.get_joint_positions());P=poses(q);D=moving();E=tcp();rel=np.linalg.inv(D)@E
 return {'phase':phase,'q':q.tolist(),'aperture_m':float(q[fingers[0]]-q[fingers[1]]),'margin_rad':model.margin(q[arm]),'finger_world_poses':{n:P[n].tolist() for n in FINGERS},'T_tcp':E.tolist(),'T_moving_link':D.tolist(),'relative_translation_slip_m':0. if reference is None else float(np.linalg.norm(rel[:3,3]-reference[:3,3])),'relative_rotation_slip_deg':0. if reference is None else float(np.rad2deg(Rotation.from_matrix(rel[:3,:3]@reference[:3,:3].T).magnitude())),'drawer_displacement_m':float(np.rad2deg(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])) if kind=='revolute' else float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])}
native=WholeFingerReports(stage,export,allowed,world,dt,sample)
try:
 sensor_metadata={'classification':'DIAGNOSTIC_ONLY','sensor_type':'PhysX native contact report; whole official finger/handle reaction, not a calibrated tactile array','finger_labels':{'left':'gripper_link1','right':'gripper_link2'},'attachment_bodies':native.finger_body_paths,'allowed_handle_colliders':allowed,'sensor_frame':'world for impulses, points and normals; finger frame is each named rigid body','T_sensor_to_finger':'inverse of per-sample finger_world_poses for world-to-finger; no separate sensor-body offset','raw_quantity':'d.impulse in N*s, world vector','signed_impulse_rule':'native d.impulse is negated if the robot collider is collider1; collider0/collider1 are retained. Recover original native impulse by reversing that same sign.','contact_normal_rule':'native normal retained without sign change; collider ordering retained','per_contact_force_rule':'norm(native impulse)/nominal physics dt','reported_pad_load_rule':'norm(sum signed world contact impulses for each finger whose target is an allowed handle collider))/measured physics interval','filter':'arithmetic mean of reported per-step scalar pad loads over rows[-max(1,int(.08/dt)):]; equal weights; no extra scaling','filter_window_samples':max(1,int(.08/dt)),'filter_coefficients':[1/max(1,int(.08/dt))]*max(1,int(.08/dt)),'filter_nominal_window_s':.08,'software_gain':1.,'software_bias':0.,'telemetry_clipping':None,'physical_calibration':'not hardware calibrated','contact_report_threshold_n':0.,'contact_boolean_threshold_n':.05,'regrasp_confirmation':{'closure_state':'existing JawCenteredClosure FORCE_HOLD; no extra aperture, symmetry, center, or sample gates'},'physics_dt_s':dt,'sensor_period_s':dt,'sensor_hz':1/dt,'joint_names':names,'arm_indices':list(map(int,arm)),'finger_indices':list(map(int,fingers)),'units':{'revolute_position':'rad','revolute_velocity':'rad/s','revolute_effort':'N*m (simulator generalized effort)','prismatic_finger_position':'m','prismatic_finger_effort':'N (simulator generalized effort)','contact_force':'N','contact_impulse':'N*s','contact_position':'m','object_displacement':'m','object_reference':'m','object_velocity':'m/s'},'measured_effort_note':'optional Isaac getters; null/error means unavailable, never a zero substitute; actuator generalized effort is not pad load','measured_joint_force_note':'Isaac six-component joint reaction getter as returned; SDK source is archived where available for frame interpretation','gripper_position_gains':{'kp':1000.,'kd':40.,'effort_explicitly_zero':True},'gripper_effort_gains':{'kp':0.,'kd':40.},'arm_gains':{'kp':10000.,'kd':400.},'original_control_policy':policy,'original_temporal_force_policy':job['wrist_experiment']['force_policy'],'timing_note':'observations contain post-step poses and completed-step contacts; native physics_steps poses are sampled immediately before integration; interval boundaries retained','sensor_completeness_is_not_a_pass_gate':True}
 diagnostic_write(a.output/'sensor_metadata.json',json.dumps(sensor_metadata,indent=2))
 try:
  joint_metadata=robot._articulation_view._metadata
  diagnostic_write(a.output/'joint_sensor_index_metadata.json',json.dumps({'joint_names':list(joint_metadata.joint_names),'joint_indices':dict(joint_metadata.joint_indices),'reaction_row_rule':'joint_index + 1; row 0 is base incoming joint','reaction_components':['Fx','Fy','Fz','Tx','Ty','Tz'],'reaction_frame':'local body reference frame / child joint frame, per archived Isaac API source','dof_names':names},indent=2))
 except Exception as error:diagnostic_issue('joint_sensor_index_metadata',error)
 import inspect
 for label,obj in [('native_contact',type(native)),('native_contact_base',type(native).__bases__[0]),('robot_sensor_api',type(robot))]:
  try:diagnostic_write(a.output/(label+'_source.py'),inspect.getsource(obj))
  except Exception as error:diagnostic_issue('source:'+label,error)
except Exception as error:diagnostic_issue('sensor_metadata',error)
def window(seconds=.3):return rows[-max(1,int(seconds/dt)):]
def filtered():return np.mean([list(s['forces_n'].values()) for s in window(.08)],axis=0) if rows else np.zeros(2)
def step():
 global tick,loss_s,slip_s,max_state,first_divergence
 if datetime.now(ZoneInfo('Asia/Shanghai'))>=deadline:raise RuntimeError('CUTOFF_05_00')
 profile_step_begin()
 native.clear();controller.apply_action(ArticulationAction(joint_positions=qtarget[arm],joint_velocities=qvelocity,joint_indices=arm))
 controller.apply_action(ArticulationAction(joint_positions=qtarget[fingers],joint_efforts=np.zeros(2),joint_indices=fingers) if mode=='position' else ArticulationAction(joint_efforts=np.array([-effort,effort]),joint_indices=fingers))
 _profile_timer=time.perf_counter()
 world.step(render=False,update_fabric=True)
 profile_cost('physics_and_native_callback',_profile_timer)
 _profile_timer=time.perf_counter()
 if tick%8==0:world.render()
 profile_cost('world_render',_profile_timer)
 s=sample();s.update(base=list(base),base_moves=base_moves,regrasp_count=regrasp_count);contacts=native.state();s.update(t=tick*dt,forces_n=contacts['finger_handle_forces_n'],contacts=contacts['contacts'],mode=mode,closing_effort_n=effort,command_q_arm=qtarget[arm].tolist());s['force_event']=force_guard.update(s['forces_n'],dt,s['t']);s['planned_articulation_state']=command_state;s['planned_T_tcp']=None if planned_reference is None else planned_reference.tolist();rows.append(s);tick+=1
 _profile_timer=time.perf_counter()
 observe_step(s,contacts)
 profile_cost('passive_telemetry',_profile_timer)
 max_state=max(max_state,s['drawer_displacement_m'])
 if planned_reference is not None and first_divergence is None:
  error=float(np.linalg.norm(np.asarray(s['T_tcp'])[:3,3]-planned_reference[:3,3]));state_error=abs(s['drawer_displacement_m']-(np.rad2deg(command_state) if kind=='revolute' else command_state))
  if error>.003 or state_error>(2. if kind=='revolute' else .003):first_divergence={'t':s['t'],'phase':phase,'planned_state':command_state,'actual_state':s['drawer_displacement_m'],'TCP_position_error_m':error,'definition':'diagnostic first >3mm TCP or >2deg/3mm object-state discrepancy; never a manipulation veto'}
 if abs(contacts['physics_window_dt_s']-dt)>1e-6:diagnostic_issue('physics_clock_interval',contacts['physics_window_dt_s'])
 if s['margin_rad']<=.05:raise RuntimeError('LOW_JOINT_MARGIN')
 P=poses(np.asarray(s['q']));ok,why=collision.contact_guard(s['contacts'],P)
 if not ok:raise RuntimeError(why)
 if tick%8==0:
  ok,why=collision.check(P,np.asarray(s['T_moving_link']),allow_handle=phase not in ('SETTLE','PREGRASP','APPROACH','NO_OPERATION','CLEARANCE_RETREAT','BASE_ROUTE','SECOND_APPROACH'))
  if not ok:raise RuntimeError(why)
  _profile_timer=time.perf_counter()
  try:
   image=np.asarray(near.get_rgba())[:,:,:3].copy();overview=np.asarray(scene['overview'][0].get_rgba())[:,:,:3];image[-240:,-320:]=cv2.resize(overview,(320,240));cv2.rectangle(image,(0,0),(1280,80),(12,12,12),-1)
   cv2.putText(image,f'KNOWN_MODEL_DIAGNOSTIC / PHYSICAL CONTACT | {phase}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1);cv2.putText(image,f'drawer={s["drawer_displacement_m"]*1000:.2f} mm | slip={s["relative_translation_slip_m"]*1000:.2f} mm | margin={s["margin_rad"]:.3f}',(12,62),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1);video.stdin.write(np.ascontiguousarray(image).tobytes())
  except Exception as error:diagnostic_issue('video',error)
  profile_cost('video_and_camera_read',_profile_timer)
 try:
  if tick%240==0:
   with (a.output/'live.jsonl').open('a') as live:live.write(json.dumps({k:s[k] for k in ('t','phase','drawer_displacement_m','aperture_m','forces_n','base','base_moves','regrasp_count','margin_rad')})+'\n')
   try:(a.output/'progress.json').write_text(json.dumps({'pid':os.getpid(),'base_moves':base_moves,'regrasp_count':regrasp_count,'base':base,'phase':phase,'t':s['t'],'drawer_displacement_m':s['drawer_displacement_m'],'aperture_m':s['aperture_m'],'forces_n':s['forces_n'],'margin_rad':s['margin_rad'],'relative_translation_slip_m':s['relative_translation_slip_m']},indent=2))
   except Exception as error:diagnostic_issue('optional_write',error)
   cv2.imwrite(str(a.output/'latest_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR))
 except Exception as error:diagnostic_issue('progress',error)
 if s['force_event']['status'].startswith('HARD_FORCE_STOP'):raise RuntimeError(s['force_event']['status'])
 if reference is not None:
  if np.any(abs(np.asarray(robot.get_joint_velocities())[arm])>vel/3):raise RuntimeError('EXISTING_ARM_SPEED_LIMIT')
  if len(rows)>1 and np.linalg.norm(np.asarray(rows[-1]['T_tcp'])[:3,3]-np.asarray(rows[-2]['T_tcp'])[:3,3])/dt>.005:raise RuntimeError('PROBE_CARTESIAN_SPEED_LIMIT')
  loss_s=loss_s+dt if np.min(filtered())<.05 else 0.;slip_s=slip_s+dt if s['relative_translation_slip_m']>policy['max_slip_m'] else 0.
  if loss_s>.15:
   s['grasp_event']='GRASP_LOAD_ASYMMETRY';s['effort_valid']=False
   if np.max(filtered())<.05:raise RuntimeError('CONTACT_LOST_PAUSE_IN_SAME_SCENE')
  if slip_s>.1:s['relative_motion_event']='GRASP_RELATIVE_MOTION' # diagnostic only
 profile_step_end()
 return s
def move(q,duration):
 global qvelocity,_profile_activity
 _profile_activity='motion';old=qtarget[arm].copy();delta=np.asarray(q)-old
 duration=max(duration,float(np.max(1.5*np.abs(delta)/(vel/3))))
 for f in np.linspace(0,1,max(2,int(duration/dt))):
  qtarget[arm]=old+f*f*(3-2*f)*delta;qvelocity=6*f*(1-f)*delta/duration;step()
 qvelocity=np.zeros(6);_profile_activity='auto'

def ik(target,seed):return model.ik(target,base,seed,starts=1)
def hold(seconds):
 global _profile_activity
 if shorten_endpoint_holds and phase in ('SWITCH_SAFE_HOLD','FINAL_HOLD') and seconds==2.:seconds=.5
 previous_activity=_profile_activity;_profile_activity='dwell'
 for _ in range(int(seconds/dt)):step()
 _profile_activity=previous_activity
grasp_records=[];local_plans=[]
def actual_state():return float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
def body_at(s):return model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:float(s)})
def note(event,**data):
 telemetry_events.append(dict(event=event,t=tick*dt,phase=phase,displacement_m=actual_state(),**data))
 diagnostic_write(a.output/'milestone_events.json',json.dumps(telemetry_events,indent=2))
def close_contact():
 global phase,mode,effort,qvelocity,legal,reference,loss_s,slip_s
 closure=JawCenteredClosure(tcp(),policy);legal=False;loss_s=0.;slip_s=0.;phase='CLOSE_GRIPPER'
 for _ in range(int(100/dt)):
  state=rows[-1];P=poses(np.asarray(state['q']));centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]
  cmd=closure.update(tcp(),state['aperture_m'],filtered(),centers,dt)
  if cmd['mode']=='position':qtarget[fingers]=[cmd['opening_m']/2,-cmd['opening_m']/2]
  else:
   if mode!='effort':
    kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True);mode='effort'
   effort=cmd['closing_effort_n']
  q=ik(cmd['T'],np.asarray(state['q'])[arm])
  if q is not None:qvelocity=np.clip((q-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=q
  else:diagnostic_issue('closure_centering_ik','retain current physically reached command');qvelocity=np.zeros(6)
  step()
  if closure.state=='FORCE_HOLD':legal=True;break
 if not legal:
  if np.max(filtered())>.05:note('DEGRADED_CONTACT_CONTINUE',loads=filtered().tolist());legal=True
  else:raise RuntimeError('NO_PHYSICAL_CONTACT_LOCAL_RECOVERY_IN_SAME_SCENE')
 qvelocity=np.zeros(6);phase='VERIFY_PHYSICAL_CONTACT';reference=np.linalg.inv(moving())@tcp()
 record={'t':tick*dt,'displacement_m':actual_state(),'pad_loads_n':filtered().tolist(),'aperture_m':rows[-1]['aperture_m'],'base':list(base),'closure_state':closure.state}
 grasp_records.append(record);note('GRASP_VERIFIED',**{k:v for k,v in record.items() if k not in ('t','displacement_m')});hold(.5)
 return record

def pull_to(goal,label):
 global phase,command_state,planned_reference,qvelocity
 phase=label;profile_mark(label)
 reached=actual_state();E0=tcp();origin=body_at(reached);seed=np.asarray(robot.get_joint_positions())[arm];bias=qtarget[arm]-seed
 diagnostic_write(a.output/(label+'_contact_reference.json'),json.dumps({'actual_state_m':reached,'T_tcp':E0.tolist(),'body_frame':origin.tolist(),'loaded_command_bias':bias.tolist(),'goal_m':goal},indent=2))
 start_tick=tick;best=reached;last_progress_tick=tick
 while actual_state()<goal:
  current=actual_state();command_state=current+.002
  planned_reference=body_at(command_state)@np.linalg.inv(origin)@E0
  seed=np.asarray(robot.get_joint_positions())[arm];q=ik(planned_reference,seed)
  if q is None or model.margin(q+bias)<=.05:
   note('WORKSPACE_TRANSITION_TO_REGRASP',reference_m=command_state);break
  distance=np.linalg.norm(planned_reference[:3,3]-tcp()[:3,3]);duration=max(.25,1.5*distance/.003)
  if force_guard.pending:
   qvelocity=np.zeros(6);hold(.25);duration*=2
   if force_guard.settled():force_guard.pending=False
  try:move(q+bias,duration)
  except RuntimeError as error:
   if str(error)=='CONTACT_LOST_PAUSE_IN_SAME_SCENE':note('TRUE_CONTACT_LOSS_RECOVERY');qvelocity=np.zeros(6);break
   raise
  # Existing finite local attempt budget causes recovery, never scene teardown.
  if actual_state()>best+.0001:best=actual_state();last_progress_tick=tick
  if (tick-last_progress_tick)*dt>15.:
   note('STALL_REANCHOR_OR_REGRASP',best_m=best);break
 qvelocity=np.zeros(6);return actual_state()

def release_and_retreat():
 global phase,reference,planned_reference,effort,mode,qvelocity
 phase='HOLD';profile_mark('pre_release_hold');qvelocity=np.zeros(6);hold(.5)
 reference=None;planned_reference=None;phase='RELEASE';profile_mark('RELEASE');effort=0.;mode='position'
 kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kp[fingers]=1000.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True)
 present=rows[-1]['aperture_m'];opening=min(.1,max(present,grasp_records[0]['aperture_m'])+.02)
 for w in np.linspace(present,opening,max(2,int(abs(opening-present)/(.001*dt))+1)):
  qtarget[fingers]=[w/2,-w/2];step()
 hold(.3);note('RELEASE_MEASUREMENT',target_m=opening,actual_m=rows[-1]['aperture_m'],pad_loads_n=filtered().tolist(),aperture_error_classification='DIAGNOSTIC_ONLY',closing_effort_cleared=True)
 if np.any(filtered()>.05):hold(1.)
 if np.any(filtered()>.05):raise RuntimeError('RELEASE_CONTACT_REMAINS_LOCAL_RECOVERY_IN_SAME_SCENE')
 note('PHYSICAL_RELEASE');phase='CLEARANCE_RETREAT';profile_mark('RETREAT')
 E=tcp();q0=np.asarray(robot.get_joint_positions())[arm];back=-E[:3,2];up=np.array([0.,0,1.]);lateral=E[:3,0]
 directions=[back,back+.5*up,normal,normal+.5*up,back+.25*lateral,back-.25*lateral,up]
 for direction in directions:
  direction=direction/np.linalg.norm(direction);path=[];seed=q0.copy();reason=None
  # The successful same-scene continuation used this first 3 cm prefix.
  # A mandatory 6 cm retreat unnecessarily exhausted IK after safe clearance.
  for amount in np.linspace(0,.03,13):
   X=E.copy();X[:3,3]+=amount*direction;q=ik(X,seed)
   if q is None:reason='NO_IK';break
   if model.margin(q)<=.05:reason='LOW_JOINT_MARGIN';break
   ok,why=collision.check(model.poses(q,base,width=rows[-1]['aperture_m']),moving(),False)
   if not ok:reason=why;break
   path.append(q.tolist());seed=q
  local_plans.append({'phase':'RETREAT','direction':direction.tolist(),'q_path':path,'reason':reason})
  diagnostic_write(a.output/'local_plans.json',json.dumps(local_plans,indent=2))
  if reason is not None:continue
  for q in path[1:]:
   distance=np.linalg.norm(model.poses(q,base,width=opening)['tcp_link'][:3,3]-tcp()[:3,3]);move(q,max(.25,1.5*distance/.003))
  hold(.3);note('RETREAT_COMPLETE');return
 raise RuntimeError('LOCAL_RETREAT_CANDIDATES_EXHAUSTED_SCENE_PRESERVED')

def relocate_base():
 global base,collision,B,phase,base_moves,_profile_activity
 from interactive_twin_recovery.mobile import scene_at
 from isaacsim.core.prims import SingleXFormPrim
 phase='BASE_ROUTE';profile_mark('BASE_MOVE');target=list(whole['second_base']);initial=list(base);locked=np.asarray(robot.get_joint_positions())[arm];width=rows[-1]['aperture_m'];route=None
 yaw=(target[3]-initial[3]+180)%360-180
 for rotate_first in (False,True):
  xy=[[*(np.asarray(initial[:2])*(1-f)+np.asarray(target[:2])*f),initial[2],initial[3]+(yaw if rotate_first else 0)] for f in np.linspace(0,1,max(2,int(np.linalg.norm(np.subtract(target[:2],initial[:2]))/.025)+2))]
  yy=[[initial[0] if rotate_first else target[0],initial[1] if rotate_first else target[1],initial[2],initial[3]+f*yaw] for f in np.linspace(0,1,max(2,int(abs(yaw)/3)+2))]
  points=yy+xy[1:] if rotate_first else xy+yy[1:];sc=scene_at(ROOT,export,model,initial_base,initial);reason=None
  for pose in points:
   sc=refresh_same_base_collision(sc,export,initial_base,pose);ok,why=sc.check(model.poses(locked,pose,width=width),moving(),False)
   if not ok:reason=why;break
  local_plans.append({'phase':'BASE_ROUTE','rotate_first':rotate_first,'waypoints':points,'reason':reason});diagnostic_write(a.output/'local_plans.json',json.dumps(local_plans,indent=2))
  if reason is None:route=points;break
 if route is None:raise RuntimeError('SAVED_BASE_LOCAL_ROUTE_BLOCKED_SCENE_PRESERVED')
 support=SingleXFormPrim('/World/r1a7_pedestal');chassis=SingleXFormPrim('/World/mobile_chassis');collision=scene_at(ROOT,export,model,initial_base,base);_profile_activity='motion';note('BASE_MOTION_START',target=target)
 for target_base in route[1:]:
  old=np.asarray(base).copy();delta=np.asarray(target_base)-old;duration=max(np.linalg.norm(delta[:2])/.01,abs(delta[3])/5.,dt)
  for f in np.linspace(0,1,max(2,int(duration/dt)))[1:]:
   pose=old+f*delta;quat=np.roll(Rotation.from_euler('z',pose[3],degrees=True).as_quat(),1)
   robot.set_world_pose(pose[:3],quat);support.set_world_pose([pose[0],pose[1],-.33],quat);chassis.set_world_pose([pose[0],pose[1],-.66],quat)
   base[:]=pose.tolist();B=transform(base[:3],[0,0,np.deg2rad(base[3])])
   if tick%8==0:collision=refresh_same_base_collision(collision,export,initial_base,base)
   step()
 base_moves+=1;hold(.3);note('BASE_MOTION_END',base=list(base))

def approach_second():
 global phase
 import copy
 phase='SECOND_APPROACH';profile_mark('APPROACH_2');E=moving()@successful_body_TCP;pre=E.copy();pre[:3,3]-=.04*E[:3,2];start=np.asarray(robot.get_joint_positions())[arm];width=rows[-1]['aperture_m'];q=ik(pre,start)
 if q is None:q=model.ik(pre,base,seed=model.home,starts=3)
 if q is None:raise RuntimeError('SAVED_TEMPLATE_APPROACH_NO_IK_LOCAL_RECOVERY')
 def local_check(q,b):
  if model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN',None
  ok,why=collision.check(model.poses(q,b,width=width),moving(),False);return ok,why,None
 view=copy.copy(model);view.check=local_check;path=Model.joint_plan(view,start,q,base,iterations=1200)
 if path is None:raise RuntimeError('SAVED_TEMPLATE_APPROACH_NO_ARM_PATH_LOCAL_RECOVERY')
 approach=[]
 for f in np.linspace(0,1,21):
  X=pre.copy();X[:3,3]=(1-f)*pre[:3,3]+f*E[:3,3];nq=ik(X,q)
  if nq is None:raise RuntimeError('SAVED_TEMPLATE_LOCAL_APPROACH_NO_IK')
  ok,why,_=local_check(nq,base)
  if not ok:raise RuntimeError('SAVED_TEMPLATE_LOCAL_APPROACH_'+why)
  approach.append(nq);q=nq
 local_plans.append({'phase':'SECOND_APPROACH','T_grasp':E.tolist(),'preplan':np.asarray(path).tolist(),'approach':np.asarray(approach).tolist()});diagnostic_write(a.output/'local_plans.json',json.dumps(local_plans,indent=2))
 for q in path[1:]:move(q,1.5)
 for q in approach[1:]:move(q,.15)
 note('APPROACH_2_COMPLETE')

try:
 phase='SETTLE';hold(.5)
 profile_mark('initial_approach_grasp')
 phase='PREFLIGHT';P=poses(robot.get_joint_positions());pre=T.copy();pre[:3,3]-=.04*T[:3,2]
 approach=[];seed=np.asarray(chosen['approach'][0]);seed=ik(pre,seed)
 if seed is None:raise RuntimeError('NO_PREGRASP_IK')
 for f in np.linspace(0,1,21):
  target=pre.copy();target[:3,3]=pre[:3,3]*(1-f)+T[:3,3]*f;seed=ik(target,seed)
  if seed is None:raise RuntimeError('NO_APPROACH_IK')
  P=model.poses(seed,base,width=.1);ok,why=collision.check(P,moving(),False)
  if not ok:raise RuntimeError('APPROACH_PREFLIGHT_'+why)
  approach.append(seed.tolist())
 path=chosen['preplan'];last=np.asarray(path[-1]);qpre=np.asarray(approach[0])
 for x,y in zip(path+[qpre.tolist()],(path+[qpre.tolist()])[1:]):
  for q in np.linspace(x,y,max(2,int(np.ceil(np.max(np.abs(np.asarray(y)-x))/.025))+1)):
   ok,why=collision.check(model.poses(q,base,width=.1),moving(),False)
   if not ok:raise RuntimeError('HOME_PREFLIGHT_'+why)
 phase='PREGRASP'
 for q in path[1:]:move(q,1.5)
 move(qpre,.8);phase='APPROACH'
 for q in approach[1:]:move(q,.12)
 close_contact();successful_body_TCP=np.linalg.inv(moving())@tcp()
 diagnostic_write(a.output/'successful_grasp_template.json',json.dumps({'T_moving_body_TCP':successful_body_TCP.tolist(),'grasp_record':grasp_records[0],'prior_template_source':whole['template_source']},indent=2))
 pull_to(.04,'PULL_1');switch_actual=actual_state()
 release_and_retreat();relocate_base();approach_second();close_contact();regrasp_count+=1;regrasp_state=actual_state()
 pull_to(regrasp_state+.025,'PULL_2');phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(2.)
 success=bool(switch_actual>rows[0]['drawer_displacement_m'] and actual_state()>regrasp_state and base_moves>=1 and regrasp_count>=1)
 status='DRAWER_REGRASP_SUCCESS' if success else 'CONTINUE_LOCAL_RECOVERY_IN_SAME_SCENE'
 if not success:raise RuntimeError(status)
 note('DRAWER_REGRASP_SUCCESS',additional_displacement_m=actual_state()-regrasp_state)

except BaseException as error:
 import traceback
 status=str(error) if isinstance(error,RuntimeError) else 'IMPLEMENTATION_ERROR:'+str(error);print(traceback.format_exc(),flush=True)
 world.pause();world.render()
 try:(a.output/'paused.json').write_text(json.dumps({'status':status,'phase':phase,'pid':os.getpid(),'max_displacement_m':max_state,'base_moves':base_moves,'regrasp_count':regrasp_count,'scene_preserved':True},indent=2))
 except Exception as error:diagnostic_issue('optional_write',error)
 try:(a.output/'observations.json').write_text(json.dumps(rows))
 except Exception as error:diagnostic_issue('optional_write',error)
 while app.is_running() and not success:
  app.update();time.sleep(.1)
  continuation=a.output/'continue_same_scene.py'
  if continuation.exists():
   command=continuation.read_text();continuation.rename(a.output/f'continuation_{time.time_ns()}.py')
   try:
    world.play();exec(compile(command,str(continuation),'exec'),globals())
   except Exception:
    world.pause();print(traceback.format_exc(),flush=True)
    try:(a.output/'paused.json').write_text(json.dumps({'status':traceback.format_exc(),'phase':phase,'pid':os.getpid(),'max_displacement_m':max_state,'base_moves':base_moves,'regrasp_count':regrasp_count,'scene_preserved':True},indent=2))
    except Exception as error:diagnostic_issue('optional_write',error)
    try:(a.output/'observations.json').write_text(json.dumps(rows))
    except Exception as error:diagnostic_issue('optional_write',error)

finally:
 profile_mark('final_export')
 diagnostic_write(a.output/'telemetry_summary.json',json.dumps({'physics_control_steps':tick,'available_field_counts':telemetry_counts,'issues':telemetry_issues,'disabled_optional_apis':sorted(telemetry_disabled),'manipulation_success':success,'telemetry_never_used_as_gate':True},indent=2))
 diagnostic_write(a.output/'events.json',json.dumps(telemetry_events,indent=2))
 world.pause()
 try:
  world.render();cv2.imwrite(str(a.output/'final_close.png'),cv2.cvtColor(np.asarray(near.get_rgba())[:,:,:3],cv2.COLOR_RGB2BGR))
 except Exception as error:diagnostic_issue('final_image',error)
 try:video.stdin.close();video.wait(timeout=30)
 except Exception as error:diagnostic_issue('final_video',error)
 try:export_cooked(stage,a.output/'cooked_final.json')
 except Exception as error:diagnostic_issue('final_geometry_export',error)
 _profile_timer=time.perf_counter()
 try:export_raw_records('observations',rows)
 except Exception as error:diagnostic_issue('observations_serialization',error)
 profile_cost('serialize_observations',_profile_timer)
 _profile_timer=time.perf_counter()
 try:export_raw_records('physics_steps',native.physics_steps)
 except Exception as error:diagnostic_issue('physics_steps_serialization',error)
 profile_cost('serialize_physics_steps',_profile_timer)
 report={'status':status,'success':success,'mode':'KNOWN_MODEL_DIAGNOSTIC','GT_planning':True,'GT_progress_readout':True,'GT_collision_scene':True,'unknown_structure_identification_success':False,'autonomous_reconstruction_success':False,'maximum_actual_displacement_m':max_state,'initial_displacement_m':rows[0]['drawer_displacement_m'] if rows else None,'before_release_displacement_m':globals().get('switch_actual'),'second_contact_displacement_m':globals().get('regrasp_state'),'additional_displacement_after_regrasp_m':rows[-1]['drawer_displacement_m']-regrasp_state if 'regrasp_state' in globals() else None,'final_displacement_m':rows[-1]['drawer_displacement_m'] if rows else None,'base_moves':base_moves,'regrasp_count':regrasp_count,'initial_grasp':grasp_records[0] if grasp_records else None,'second_grasp':grasp_records[1] if len(grasp_records)>1 else None,'peak_pad_load_n':max((max(r['forces_n'].values()) for r in rows),default=0.),'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),'duration_sim_s':len(rows)*dt,'duration_wall_s':time.perf_counter()-_profile_entry,'first_planned_actual_divergence':first_divergence,'contact_loss_events':[e for e in telemetry_events if e['event']=='PAD_CONTACT_LOST'],'progress_reference':'actual reached displacement + 2 mm; grasp frame fixed in moving body; no time-driven object reference','physical_parameters_changed':False,'attachments':False,'object_actuation':'none during execution; normal closed loader initialization only','scene_preserved':True,'video':'contact_baseline.mp4','physics_dt_s':dt,'source_assets':whole}
 diagnostic_write(a.output/'report.json',json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('contact_loss_events','source_assets')},indent=2),flush=True)


 diagnostic_write(a.output/'timing_configuration.json',json.dumps({'requested_speed_scale':speed_scale,'loaded_spline_effective_scale':min(speed_scale,2.4 if job.get('timing_experiment',{}).get('continuous_hinge_time_law',False) else .005/.003),'hinge_time_law':'endpoint ramps and continuous interior' if job.get('timing_experiment',{}).get('continuous_hinge_time_law',False) else 'original per-segment smoothstep','phase_timing_options':job.get('timing_experiment',{}),'existing_tcp_speed_limit_m_s':.005,'shortened_endpoint_holds':shorten_endpoint_holds,'all_other_holds_and_confirmation_windows_unchanged':True,'reuse_invariant_collision_solids':job.get('timing_experiment',{}).get('reuse_invariant_collision_solids',False),'chunked_lossless_export':chunked_export,'physics_dt_s':dt},indent=2))
 profile_mark('completed_scene_pause')
 # Keep the endpoint scene alive after publishing success. No process signals
 # are used for pauses; further continuation, if requested, uses this scene.
 diagnostic_write(a.output/'scene_alive.json',json.dumps({'pid':os.getpid(),'status':status,'success':success,'world_paused':True,'scene_preserved':True,'displacement_m':rows[-1]['drawer_displacement_m'] if rows else None,'max_displacement_m':max_state,'base_moves':base_moves,'regrasp_count':regrasp_count},indent=2))
 while app.is_running():
  if success and (a.output/'close_completed_scene').exists():
   app.close();break
  extension=a.output/'extend_after_prefix_freeze.py'
  if extension.exists():
   extension_source=extension.read_text();extension.rename(a.output/f'extension_{time.time_ns()}.py')
   try:
    world.play();exec(compile(extension_source,str(extension),'exec'),globals())
   except Exception as extension_error:
    import traceback
    world.pause();print(traceback.format_exc(),flush=True)
    diagnostic_write(a.output/'extension_paused.json',json.dumps({'error':repr(extension_error),'phase':phase,'displacement_m':actual_state(),'pid':os.getpid(),'scene_preserved':True},indent=2))
  app.update();time.sleep(.1)
