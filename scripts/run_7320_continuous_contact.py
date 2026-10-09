"""KNOWN_MODEL_DIAGNOSTIC: physical XY carriage motion during one grasp.
Experiment-only orchestration; frozen arm/jaw controllers and protections reused.
"""
import sys,json,textwrap,time,hashlib
from pathlib import Path
from continuous_contact_scene import install_mobile_scene_hook
install_mobile_scene_hook()
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
baseline=Path(spec['baseline_runner']);baseline_text=baseline.read_text()
setup,sequence=baseline_text.split("try:\n phase='SETTLE';hold(.5)",1)
exec(compile(setup,str(baseline)+'::frozen_setup','exec'),globals())
from isaacsim.core.prims import RigidPrim
from interactive_twin_recovery.mobile import scene_at
base_indices=[names.index('mobile_x'),names.index('mobile_y')]
base_view=RigidPrim(prim_paths_expr=scene['mobile_base_body_path'],name='continuous_base_feedback');base_view.initialize()
base_goal=np.zeros(2);base_command_previous=np.zeros(2);base_velocity=np.zeros(2);base_motion=False;compensate=False;comp_target=None;comp_relative=None;comp_bias=None;comp_seed=None;base_records=[];plans=[];adjustments=[];grasp_relative=None;coord_speed=0.;coord_twist=np.zeros(6);coord_scale=1.;coord_blocked=False;coord_mode="HOLD";coord_goal=0.;desired_tcp=None;coord_events=[];opening_index=0;coord_angle_ref=0.;coord_station_boundary=False
original_gains=controller.set_gains
def stage_gains(*args,**kwargs):
 # Arm/jaw entries unchanged; supply gains solely for the new physical base DOFs.
 for k,v in [('kps',1e6),('kds',2e4)]:
  if kwargs.get(k) is not None:
   x=np.asarray(kwargs[k]).copy();x[base_indices]=v;kwargs[k]=x
 return original_gains(*args,**kwargs)
controller.set_gains=stage_gains
kp,kd=controller.get_gains();kp=np.asarray(kp).reshape(-1);kd=np.asarray(kd).reshape(-1);stage_gains(kps=kp,kds=kd,save_to_usd=True)
base_scene=collision
original_sample=sample;original_step=step;original_observe=observe_step

def feedback_base():
 p,q=base_view.get_world_poses();yaw=Rotation.from_quat(np.roll(q[0],-1)).as_euler('xyz',degrees=True)[2]
 return [*p[0].tolist(),float(yaw)]

def sample():
 global B,collision
 base[:]=feedback_base();B=transform(base[:3],[0,0,np.deg2rad(base[3])])
 if tick%8==0 and (base_motion or tick<10):collision=refresh_same_base_collision(base_scene,export,initial_base,base)
 return original_sample()

def observe_step(s,contacts):
 original_observe(s,contacts)
 try:
  s['telemetry']['command_base_joint_positions_m']=base_goal.tolist();s['telemetry']['command_base_joint_velocities_m_s']=base_velocity.tolist();s['telemetry']['actual_base_joint_positions_m']=np.asarray(robot.get_joint_positions())[base_indices].tolist();s['telemetry']['actual_base_joint_velocities_m_s']=np.asarray(robot.get_joint_velocities())[base_indices].tolist()
  s['telemetry']['base_pose_feedback']=feedback_base();s['telemetry']['base_actuation']='dynamic XY prismatic drive; no runtime root pose/state writes'
 except Exception as error:diagnostic_issue('mobile_telemetry',error)

def bounded_reference(desired, speed):
 global comp_target,coord_speed,coord_twist
 dp=desired[:3,3]-comp_target[:3,3]
 rotation=Rotation.from_matrix(desired[:3,:3]@comp_target[:3,:3].T).as_rotvec()
 joint=next(j for j in model.asset.joints.values() if j.name==model.manifest['joint_name'])
 hinge=model.asset_T@model.asset.root_to_link(joint.parent,{model.manifest['joint_name']:0.})@joint.origin
 radius=max(1e-12,float(np.linalg.norm(comp_target[:3,3]-hinge[:3,3])))
 error=np.r_[dp,rotation*radius];distance=float(np.linalg.norm(error));acceleration=.002
 wanted=error/max(distance,1e-12)*min(speed,np.sqrt(2*acceleration*distance))
 change=wanted-coord_twist;coord_twist+=change*min(1.,acceleration*dt/max(float(np.linalg.norm(change)),1e-12))
 increment=coord_twist*dt
 if np.dot(increment,error)>0 and np.linalg.norm(increment)>distance:
  increment=error.copy();coord_twist=increment/dt
 coord_speed=float(np.linalg.norm(coord_twist))
 result=comp_target.copy();result[:3,3]+=increment[:3]
 result[:3,:3]=Rotation.from_rotvec(increment[3:]/radius).as_matrix()@comp_target[:3,:3]
 return result

def step():
 global qvelocity,comp_seed,comp_target,planned_reference,base_goal,base_velocity,coord_scale,coord_blocked,desired_tcp,coord_angle_ref
 if (a.output/"pause_requested").exists():
  (a.output/"pause_requested").unlink();world.pause();raise RuntimeError("OPERATOR_PAUSE_SCENE_PRESERVED")
 coord_scale=1.;coord_blocked=False
 if compensate:
  theta=actual();body=moving()
  if coord_mode=='OPEN':
   lag=max(0.,coord_angle_ref-theta)
   radius=float(np.linalg.norm(tcp()[:3,3]-body_at(theta)[:3,3]))
   # Gentle forward drive survives static compliance; lag slows the clock
   # continuously rather than making progress a required execution gate.
   rate=.0015/max(radius,1e-12)/(1.+lag/np.deg2rad(.05))
   reference_end=coord_goal if coord_station_boundary else coord_goal+np.deg2rad(.5)
   coord_angle_ref=min(reference_end,max(coord_angle_ref+rate*dt,theta+np.deg2rad(.05)))
  else:coord_angle_ref=max(coord_angle_ref,theta)
  # Hold reference when physical progress lags or rebounds. Never chase a
  # backward pose update with the opening drive; no lag-triggered abort.
  desired_tcp=body_at(coord_angle_ref)@np.linalg.inv(body_at(theta))@body@grasp_relative
  previous_ref=comp_target.copy();next_ref=bounded_reference(desired_tcp,.0015 if coord_mode=='OPEN' else .001)
  current=feedback_base();measured=np.asarray(robot.get_joint_positions());seed=measured[arm] if comp_seed is None else comp_seed
  measured_v=np.asarray(robot.get_joint_velocities())[base_indices]
  lead=list(current);lead[0]+=measured_v[0]*dt;lead[1]+=measured_v[1]*dt
  q=model.ik(next_ref,lead,seed,starts=1)
  if q is None:
   # Stop advancement of both commands at the last safe configuration. The
   # caller selects another local base direction; the original guard still runs.
   coord_blocked=True;coord_scale=0.;base_goal=measured[base_indices].copy();base_velocity=np.zeros(2);qvelocity=np.zeros(6)
   coord_events.append({'event':'NO_IK_COUPLED_HOLD','t':tick*dt,'base':current,'scene_preserved':True})
  else:
   command=q+comp_bias;delta=command-qtarget[arm]
   tracking=command-(measured[arm]+comp_bias)
   required=np.maximum(np.abs(delta),np.abs(tracking))/dt
   coord_scale=min(1.,float(np.min((vel/3)/np.maximum(required,1e-12))))
   # The same scalar clock slows arm, Cartesian reference and base together.
   previous_base=base_command_previous.copy()
   base_goal=previous_base+coord_scale*(base_goal-previous_base);base_velocity=base_velocity*coord_scale
   comp_target=previous_ref.copy();comp_target[:3,3]+=coord_scale*(next_ref[:3,3]-previous_ref[:3,3])
   rv=Rotation.from_matrix(next_ref[:3,:3]@previous_ref[:3,:3].T).as_rotvec()
   comp_target[:3,:3]=Rotation.from_rotvec(coord_scale*rv).as_matrix()@previous_ref[:3,:3]
   qtarget[arm]+=coord_scale*delta;qvelocity=coord_scale*delta/dt;comp_seed=qtarget[arm]-comp_bias
  planned_reference=comp_target.copy()
 controller.apply_action(ArticulationAction(joint_positions=base_goal,joint_velocities=base_velocity,joint_indices=base_indices))
 state=original_step()
 try:
  state['coordination']={'mode':coord_mode,'clock_scale':coord_scale,'reference_speed_m_s':coord_speed,'desired_T_tcp':None if desired_tcp is None else desired_tcp.tolist(),'grasp_T_body_tcp':None if grasp_relative is None else grasp_relative.tolist(),'IK_hold':coord_blocked}
 except Exception as error:diagnostic_issue('coordination_telemetry',error)
 base_command_previous[:]=base_goal
 return state

def note(name,value):
 try:diagnostic_write(a.output/name,json.dumps(value,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
 except Exception as e:diagnostic_issue(name,e)

def body_at(theta):
 # Native imported rigid-body origin differs from the authored FK link origin.
 # The motion delta is unchanged; collision queries receive the native frame.
 authored=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:float(theta)})
 closed=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:0.})
 return authored@np.linalg.inv(closed)@base_scene.moving_reference
def actual():return float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])

def available(goal,station=None,E=None,theta=None,seed=None,bias=None):
 station=base if station is None else station;theta=actual() if theta is None else theta;E=tcp() if E is None else E
 seed=np.asarray(robot.get_joint_positions())[arm] if seed is None else seed;bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm] if bias is None else bias
 origin=body_at(theta);path=[];reason=None
 C=refresh_same_base_collision(base_scene,export,initial_base,station)
 for state in np.linspace(theta,goal,max(2,int(abs(goal-theta)/np.deg2rad(1.))+1)):
  target=body_at(state)@np.linalg.inv(origin)@E;q=model.ik(target,station,seed,starts=1)
  if q is None:reason='NO_IK';break
  if model.margin(q+bias)<=.05:reason='EXISTING_JOINT_MARGIN';break
  ok,why=C.check(model.poses(q+bias,station,finger_q=np.asarray(robot.get_joint_positions())[fingers]),body_at(state),True)
  if not ok:reason=why;break
  path.append(dict(state=float(state),q=q.tolist(),T=target.tolist(),margin=model.margin(q+bias)));seed=q
 return path,bias,reason

def open_to(goal,station_boundary=False):
 global phase,command_state,planned_reference,qvelocity,collision,compensate,comp_target,comp_bias,comp_seed,coord_mode,coord_goal,coord_speed,coord_twist,opening_index,coord_angle_ref,coord_station_boundary
 coord_station_boundary=station_boundary
 coord_angle_ref=actual();coord_goal=min(float(goal),np.deg2rad(90.));coord_mode='OPEN';coord_speed=0.;coord_twist=np.zeros(6);comp_target=tcp().copy();comp_bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm];comp_seed=np.asarray(robot.get_joint_positions())[arm].copy();compensate=True;opening_index+=1
 collision=refresh_same_base_collision(base_scene,export,initial_base,base)
 begin=tick;reason=None
 while actual()<coord_goal:
  phase='CONTINUOUS_OPEN';command_state=min(coord_goal+np.deg2rad(.05),actual()+np.deg2rad(.05));step()
  if coord_blocked:reason='NO_IK';break
  if coord_station_boundary and coord_angle_ref>=coord_goal:reason='PLANNED_STATION_REFERENCE_BOUNDARY';break
 coord_mode='HOLD';phase='GRASP_HOLD';compensate=False;qvelocity=np.zeros(6);hold(.3)
 note('opening_execution_%02d.json'%opening_index,{'start_s':begin*dt,'end_s':tick*dt,'goal_rad':coord_goal,'actual_rad':actual(),'blocker':reason,'reference':'monotonic lag-slowed measured-progress drive; fixed acquired body-relative grasp; smooth Cartesian clock'})
 return reason

def plan_base_motion():
 global collision
 now=np.asarray(base);E=moving()@grasp_relative;theta=actual();qmeas=np.asarray(robot.get_joint_positions())[arm];bias=qtarget[arm]-qmeas
 fixed,_,fixed_block=available(np.deg2rad(90),seed=qmeas,bias=bias);fixed_max=float(np.rad2deg(fixed[-1]['state'])) if fixed else float(np.rad2deg(theta))
 choices=[]
 for deg in (0,45,90,135,180,225,270,315):
  direction=np.array([np.cos(np.deg2rad(deg)),np.sin(np.deg2rad(deg))]);seed=qmeas.copy();path=[];reason=None
  for distance in np.linspace(0,.03,31):
   station=now.copy();station[:2]+=distance*direction;q=model.ik(E,station,seed,starts=1)
   if q is None:reason='NO_IK';break
   if model.margin(q+bias)<=.05:reason='EXISTING_JOINT_MARGIN';break
   C=refresh_same_base_collision(base_scene,export,initial_base,station);ok,why=C.check(model.poses(q+bias,station,finger_q=np.asarray(robot.get_joint_positions())[fingers]),moving(),True)
   if not ok:reason=why;break
   path.append({'base':station.tolist(),'q':q.tolist(),'margin_rad':model.margin(q+bias),'distance_m':float(distance)});seed=q
  entry={'direction_deg':deg,'path':path,'blocker':reason,'complete_3cm':len(path)==31}
  if len(path)==31:
   remaining,_,block=available(np.deg2rad(90),station,E,theta,seed,bias);entry.update(reachable_deg=float(np.rad2deg(remaining[-1]['state'])) if remaining else float(np.rad2deg(theta)),remaining_blocker=block,end_margin=model.margin(seed+bias),min_margin=min(w['margin_rad'] for w in path))
   choices.append(entry)
  plans.append(entry)
 choices.sort(key=lambda v:(v['reachable_deg'],v['end_margin']),reverse=True)
 note('base_search_%02d.json'%len(adjustments),{'fixed_base_reachable_deg':fixed_max,'fixed_base_blocker':fixed_block,'all_candidates':plans,'ranked':choices,'criterion':'existing continuous IK + unchanged .05 margin/collision; rank range then margin; no new gates'})
 collision=refresh_same_base_collision(base_scene,export,initial_base,base)
 return choices[0] if choices else None

def coordinated_motion(candidate):
 global phase,base_goal,base_velocity,base_motion,compensate,comp_target,comp_relative,comp_bias,comp_seed,qvelocity,planned_reference,coord_mode,coord_speed,coord_twist,coord_angle_ref
 coord_angle_ref=actual();start=np.asarray(robot.get_joint_positions())[base_indices];delta=np.asarray(candidate['path'][-1]['base'])[:2]-np.asarray(base)[:2]
 origin=np.asarray(base);theta=actual();comp_target=tcp().copy();comp_relative=grasp_relative.copy();comp_seed=np.asarray(robot.get_joint_positions())[arm].copy();coord_mode="HOLD";coord_speed=0.;coord_twist=np.zeros(6);comp_bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm]
 planned_reference=comp_target.copy();base_motion=True;compensate=True;begin=tick;previous=start.copy()
 # Start with 1 mm, then 5 mm, then the full 30 mm. Smooth starts/stops.
 for length in (.001,.005,.03):
  endpoint=start+delta*(length/.03);d=endpoint-previous;duration=max(4.,1.5*np.linalg.norm(d)/.0015)
  phase='BASE_COMPENSATED_%.0fMM'%(length*1000)
  elapsed=0.
  while elapsed<duration:
   f=min(1.,(elapsed+dt)/duration);scalar=f*f*(3-2*f);rate=6*f*(1-f)/duration
   base_goal=previous+scalar*d;base_velocity=rate*d;step();elapsed+=dt*coord_scale
   if coord_blocked:
    base_velocity=np.zeros(2);compensate=False;base_motion=False;return False
  base_velocity=np.zeros(2);phase='BASE_COMPENSATED_HOLD';hold(.5);previous=endpoint.copy()
 compen_end=feedback_base();compensate=False;base_motion=False;qvelocity=np.zeros(6)
 adjustments.append({'start_t_s':begin*dt,'end_t_s':tick*dt,'angle_before_deg':float(np.rad2deg(theta)),'angle_after_deg':float(np.rad2deg(actual())),'start_base':origin.tolist(),'end_base':compen_end,'command_delta_xy_m':delta.tolist(),'actual_delta_xy_m':(np.asarray(compen_end[:2])-origin[:2]).tolist(),'candidate':candidate,'release':False,'regrasp':False})
 note('coordinated_adjustments.json',adjustments)
 return True

milestone=False;initial_grasp=False;first_hold_angle=None;after_motion_angle=None;manual_continuations=0;continuation_stop_reason=None
try:
 # Frozen known-model closed approach and first physical closure, no regrasp.
 initial="if True:\n phase='SETTLE';hold(.5)"+sequence.split("  profile_mark('OPEN_1')",1)[0]
 exec(compile(initial,str(baseline)+'::unchanged_initial_grasp','exec'),globals());initial_grasp=legal
 grasp_relative=np.linalg.inv(moving())@tcp();reference=grasp_relative.copy();note('acquired_grasp_reference.json',{'T_body_tcp':grasp_relative,'source':'physically verified grasp; never replaced during episode','t':tick*dt})
 open_to(np.deg2rad(20.3));first_hold_angle=float(np.rad2deg(actual()));phase='GRASP_HOLD';hold(.5)
 note('held_20deg.json',{'angle_deg':first_hold_angle,'pad_loads_n':filtered(),'base_feedback':feedback_base(),'q':robot.get_joint_positions(),'T_tcp':tcp(),'relative':np.linalg.inv(moving())@tcp()})
 if not job.get('continuous_contact',{}).get('fixed_base_control',False):
  candidate=plan_base_motion()
  if candidate is not None:
   for retry in range(8):
    if coordinated_motion(candidate):base_moves+=1;break
    candidate=plan_base_motion()
    if candidate is None:break
  else:status='LOCAL_BASE_DIRECTIONS_INFEASIBLE_SCENE_PRESERVED'
 after_motion_angle=float(np.rad2deg(actual()));open_to(np.deg2rad(after_motion_angle+10.5));milestone=bool(adjustments and np.rad2deg(actual())-after_motion_angle>=10.)
 note('milestone.json',{'continuous_contact_PASS':milestone,'angle_before_base_deg':first_hold_angle,'angle_after_base_deg':after_motion_angle,'angle_after_continuation_deg':float(np.rad2deg(actual())),'regrasps':0,'release_count':0})
 if job.get('continuous_contact',{}).get('extend',True):
  while True:
   # The selected station already has a planned reachable interval. Use its
   # endpoint to relocate before the physical joint-margin stop, not as failure.
   station_goal=min(90.,adjustments[-1]['candidate']['reachable_deg']) if adjustments else 90.
   blocker=open_to(np.deg2rad(station_goal),station_boundary=station_goal<90.)
   if np.rad2deg(actual())>=90.:break
   if job.get('continuous_contact',{}).get('fixed_base_control',False):break
   candidate=plan_base_motion()
   if candidate is None:continuation_stop_reason='FINITE_LOCAL_BASE_CANDIDATES_EXHAUSTED';status='LOCAL_BASE_DIRECTIONS_INFEASIBLE_SCENE_PRESERVED';break
   if coordinated_motion(candidate):base_moves+=1
 phase='FINAL_HOLD';qvelocity=np.zeros(6);base_velocity=np.zeros(2);hold(.5)
 success=milestone;status='CONTINUOUS_CONTACT_90_SUCCESS' if np.rad2deg(actual())>=90. else ('CONTINUOUS_CONTACT_MILESTONE_SUCCESS' if milestone else 'FIXED_BASE_CONTROL_COMPLETE')
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True)
 # Stop commanded base/arm advance, preserve preload and scene. No release.
 base_goal=np.asarray(robot.get_joint_positions())[base_indices].copy();base_velocity=np.zeros(2);qvelocity=np.zeros(6);compensate=False;base_motion=False
world.pause()
try:
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps);note('events.json',telemetry_events);note('coordination_events.json',coord_events)
 result={'classification':'KNOWN_MODEL_DIAGNOSTIC_CONTINUOUS_CONTACT','status':status,'continuous_contact_milestone_PASS':milestone,'final_angle_deg':float(np.rad2deg(actual())),'maximum_angle_deg':max_state,'initial_grasp':initial_grasp,'release_count':0,'regrasps':0,'physical_base_moves':len(adjustments),'angle_before_base_deg':first_hold_angle,'angle_after_base_deg':after_motion_angle,'adjustments':adjustments,'peak_pad_load_n':max((max(s['forces_n'].values()) for s in rows),default=0),'minimum_joint_margin_rad':min((s['margin_rad'] for s in rows),default=None),'max_relative_slip_m':max((s['relative_translation_slip_m'] for s in rows),default=None),'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'physics_steps':len(native.physics_steps),'manual_continuations':manual_continuations,'continuation_stop_reason':continuation_stop_reason,'compensation_target':'smooth velocity-limited measured-body feedback with one fixed physical-grasp relative transform; synchronized base/arm clock','scene_preserved':True,'issues':telemetry_issues,'base_actuation':'physical driven XY carriage; not wheeled mobile dynamics','runtime_base_pose_teleport':False,'object_actuation':False,'attachments':False,'pid':os.getpid()}
 note('report.json',result);note('sensor_metadata_existing_mapping.json',sensor_metadata);print('CONTINUOUS_RESULT',json.dumps(result),flush=True)
except Exception as error:diagnostic_issue('export',error)
try:video.stdin.close();video.wait(timeout=30)
except Exception as error:diagnostic_issue('video_finalize',error)
while app.is_running():
 if (a.output/'close_completed_scene').exists():app.close();break
 continuation=a.output/'continue_same_scene.py'
 if continuation.exists():
  command=continuation.read_text();continuation.rename(a.output/f'continuation_{time.time_ns()}.py');manual_continuations+=1
  note('manual_continuation.json',{'count':manual_continuations,'scene_preserved':True,'t':tick*dt})
  try:world.play();exec(compile(command,str(continuation),'exec'),globals())
  except Exception as error:
   import traceback
   print(traceback.format_exc(),flush=True)
  world.pause()
 app.update();time.sleep(.1)
