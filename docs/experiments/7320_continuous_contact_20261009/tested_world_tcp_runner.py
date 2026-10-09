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
base_goal=np.zeros(2);base_velocity=np.zeros(2);base_motion=False;compensate=False;comp_target=None;comp_bias=None;comp_seed=None;base_records=[];plans=[];adjustments=[]
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

def step():
 global qvelocity,comp_seed
 if compensate:
  current=feedback_base();seed=np.asarray(robot.get_joint_positions())[arm]
  # Measured base pose, plus one physics-step motion lead, compensates the
  # PhysX carriage before the same step. No commanded pose replaces feedback.
  measured_v=np.asarray(robot.get_joint_velocities())[base_indices]
  lead=list(current);lead[0]+=measured_v[0]*dt;lead[1]+=measured_v[1]*dt
  q=model.ik(comp_target,lead,seed,starts=1)
  if q is None:raise RuntimeError('LOCAL_BASE_COMPENSATION_NO_IK_SCENE_PRESERVED')
  command=q+comp_bias;qvelocity=np.clip((command-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=command;comp_seed=q
 controller.apply_action(ArticulationAction(joint_positions=base_goal,joint_velocities=base_velocity,joint_indices=base_indices))
 return original_step()

def note(name,value):
 try:diagnostic_write(a.output/name,json.dumps(value,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
 except Exception as e:diagnostic_issue(name,e)

def body_at(theta):return model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:float(theta)})
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

def open_to(goal):
 global phase,command_state,planned_reference,qvelocity,arc,_arc_index,collision
 path,bias,reason=available(goal);arc=path
 note('opening_plan_%02d.json'%len(adjustments),{'base':base,'path':path,'blocker':reason})
 collision=refresh_same_base_collision(base_scene,export,initial_base,base)
 for _arc_index,w in enumerate(path):
  phase='CONTINUOUS_OPEN';command_state=w['state'];planned_reference=np.asarray(w['T']);distance=np.linalg.norm(planned_reference[:3,3]-tcp()[:3,3])
  move(np.asarray(w['q'])+bias,max(.5,1.5*distance/.002));qvelocity=np.zeros(6)
 phase='GRASP_HOLD';hold(.3)
 return reason

def plan_base_motion():
 global collision
 now=np.asarray(base);E=tcp();theta=actual();qmeas=np.asarray(robot.get_joint_positions())[arm];bias=qtarget[arm]-qmeas
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
 global phase,base_goal,base_velocity,base_motion,compensate,comp_target,comp_bias,qvelocity,planned_reference
 start=np.asarray(robot.get_joint_positions())[base_indices];delta=np.asarray(candidate['path'][-1]['base'])[:2]-np.asarray(base)[:2]
 origin=np.asarray(base);theta=actual();comp_target=tcp().copy();comp_bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm]
 planned_reference=comp_target.copy();base_motion=True;compensate=True;begin=tick;previous=start.copy()
 # Start with 1 mm, then 5 mm, then the full 30 mm. Smooth starts/stops.
 for length in (.001,.005,.03):
  endpoint=start+delta*(length/.03);d=endpoint-previous;duration=max(4.,1.5*np.linalg.norm(d)/.0003)
  phase='BASE_COMPENSATED_%.0fMM'%(length*1000)
  for f in np.linspace(0,1,max(2,int(duration/dt))):
   scalar=f*f*(3-2*f);rate=6*f*(1-f)/duration;base_goal=previous+scalar*d;base_velocity=rate*d;step()
  base_velocity=np.zeros(2);phase='BASE_COMPENSATED_HOLD';hold(.5);previous=endpoint.copy()
 compen_end=feedback_base();compensate=False;base_motion=False;qvelocity=np.zeros(6)
 adjustments.append({'start_t_s':begin*dt,'end_t_s':tick*dt,'angle_before_deg':float(np.rad2deg(theta)),'angle_after_deg':float(np.rad2deg(actual())),'start_base':origin.tolist(),'end_base':compen_end,'command_delta_xy_m':delta.tolist(),'actual_delta_xy_m':(np.asarray(compen_end[:2])-origin[:2]).tolist(),'candidate':candidate,'release':False,'regrasp':False})
 note('coordinated_adjustments.json',adjustments)

milestone=False;initial_grasp=False;first_hold_angle=None;after_motion_angle=None
try:
 # Frozen known-model closed approach and first physical closure, no regrasp.
 initial="if True:\n phase='SETTLE';hold(.5)"+sequence.split("  profile_mark('OPEN_1')",1)[0]
 exec(compile(initial,str(baseline)+'::unchanged_initial_grasp','exec'),globals());initial_grasp=legal
 open_to(np.deg2rad(20.3));first_hold_angle=float(np.rad2deg(actual()));phase='GRASP_HOLD';hold(.5)
 note('held_20deg.json',{'angle_deg':first_hold_angle,'pad_loads_n':filtered(),'base_feedback':feedback_base(),'q':robot.get_joint_positions(),'T_tcp':tcp(),'relative':np.linalg.inv(moving())@tcp()})
 if not job.get('continuous_contact',{}).get('fixed_base_control',False):
  candidate=plan_base_motion()
  if candidate is not None:coordinated_motion(candidate);base_moves+=1
  else:status='LOCAL_BASE_DIRECTIONS_INFEASIBLE_SCENE_PRESERVED'
 after_motion_angle=float(np.rad2deg(actual()));open_to(np.deg2rad(after_motion_angle+10.5));milestone=bool(adjustments and np.rad2deg(actual())-after_motion_angle>=10.)
 note('milestone.json',{'continuous_contact_PASS':milestone,'angle_before_base_deg':first_hold_angle,'angle_after_base_deg':after_motion_angle,'angle_after_continuation_deg':float(np.rad2deg(actual())),'regrasps':0,'release_count':0})
 if job.get('continuous_contact',{}).get('extend',True):
  for cycle in range(12):
   blocker=open_to(np.deg2rad(90.3))
   if np.rad2deg(actual())>=90.:break
   if job.get('continuous_contact',{}).get('fixed_base_control',False):break
   candidate=plan_base_motion()
   if candidate is None:status='LOCAL_BASE_DIRECTIONS_INFEASIBLE_SCENE_PRESERVED';break
   coordinated_motion(candidate);base_moves+=1
 phase='FINAL_HOLD';qvelocity=np.zeros(6);base_velocity=np.zeros(2);hold(.5)
 success=milestone;status='CONTINUOUS_CONTACT_90_SUCCESS' if np.rad2deg(actual())>=90. else ('CONTINUOUS_CONTACT_MILESTONE_SUCCESS' if milestone else 'FIXED_BASE_CONTROL_COMPLETE')
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True)
 # Stop commanded base/arm advance, preserve preload and scene. No release.
 base_goal=np.asarray(robot.get_joint_positions())[base_indices].copy();base_velocity=np.zeros(2);qvelocity=np.zeros(6);compensate=False;base_motion=False
world.pause()
try:
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps);note('events.json',telemetry_events)
 result={'classification':'KNOWN_MODEL_DIAGNOSTIC_CONTINUOUS_CONTACT','status':status,'continuous_contact_milestone_PASS':milestone,'final_angle_deg':float(np.rad2deg(actual())),'maximum_angle_deg':max_state,'initial_grasp':initial_grasp,'release_count':0,'regrasps':0,'physical_base_moves':len(adjustments),'angle_before_base_deg':first_hold_angle,'angle_after_base_deg':after_motion_angle,'adjustments':adjustments,'peak_pad_load_n':max((max(s['forces_n'].values()) for s in rows),default=0),'minimum_joint_margin_rad':min((s['margin_rad'] for s in rows),default=None),'max_relative_slip_m':max((s['relative_translation_slip_m'] for s in rows),default=None),'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'physics_steps':len(native.physics_steps),'manual_continuations':0,'scene_preserved':True,'issues':telemetry_issues,'base_actuation':'physical driven XY carriage; not wheeled mobile dynamics','runtime_base_pose_teleport':False,'object_actuation':False,'attachments':False,'pid':os.getpid()}
 note('report.json',result);note('sensor_metadata_existing_mapping.json',sensor_metadata);print('CONTINUOUS_RESULT',json.dumps(result),flush=True)
except Exception as error:diagnostic_issue('export',error)
try:video.stdin.close();video.wait(timeout=30)
except Exception as error:diagnostic_issue('video_finalize',error)
while app.is_running():
 if (a.output/'close_completed_scene').exists():app.close();break
 app.update();time.sleep(.1)
