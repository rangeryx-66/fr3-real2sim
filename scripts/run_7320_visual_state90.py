"""Extend the frozen 52-degree RGB-D state skill with repeated visual regrasps.
No runtime object GT state supplies motion, planning or endpoint references.
Physical GT outcomes are recorded and evaluated only after execution.
"""
import sys,json,textwrap
from pathlib import Path
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
frozen_52=Path(spec['frozen_52_source_dir']);sys.path.insert(0,str(frozen_52))
source52=(frozen_52/'run_7320_estimated_regrasp.py').read_text()
prefix52=source52.split("try:\n exec(compile(initial_visual_sequence",1)[0]
exec(compile(prefix52,str(frozen_52/'run_7320_estimated_regrasp.py')+'::frozen_estimated_helpers','exec'),globals())
# Preserve one observation/correction bundle per physical regrasp. Only output
# labels differ; frozen release/retreat/base/approach/closure functions are reused.
switch_source='def estimated_switch('+source52.split('def estimated_switch(',1)[1].split("\ntry:\n exec(compile(initial_visual_sequence",1)[0]
switch_source=switch_source.replace("rgbd_snapshot('regrasp_reference')", "rgbd_snapshot(f'regrasp_{regrasp_count+1:02d}_reference')")
switch_source=switch_source.replace("rgbd_snapshot('regrasp_pregrasp')", "rgbd_snapshot(f'regrasp_{regrasp_count+1:02d}_pregrasp')")
switch_source=switch_source.replace("save_note('regrasp_visual_target.json'", "save_note(f'regrasp_{regrasp_count+1:02d}_visual_target.json'")
exec(compile(switch_source,'frozen_estimated_switch::unique_observation_labels','exec'),globals())
endpoint_reference=None;reference_target_completed=False;recovery_events=[];cycle=0


def measured_endpoint():
 """Same 0.15-degree slow continuation and 90.5 reference cap, RGB-D state.
The reference advances through small compliance increments at the authored
endpoint. Its completion is not a claim that a noisy estimate is exact GT.
"""
 global phase,qvelocity,command_state,planned_reference,endpoint_reference,reference_target_completed
 profile_mark('VISUAL_ENDPOINT');phase='OPEN_2_ENDPOINT';qvelocity=np.zeros(6);hold(.5)
 theta0=estimated_state();E0=tcp();origin=body_at(theta0);bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm]
 if endpoint_reference is None:endpoint_reference=theta0
 for attempt in range(30):
  if endpoint_reference>=np.deg2rad(90.5):reference_target_completed=True;break
  command_state=min(np.deg2rad(90.5),max(estimated_state()+np.deg2rad(.15),endpoint_reference+np.deg2rad(.15)))
  planned_reference=body_at(command_state)@np.linalg.inv(origin)@E0
  q=ik(planned_reference,np.asarray(robot.get_joint_positions())[arm])
  if q is None:raise RuntimeError('NO_IK_TRANSITION_TO_REGRASP')
  if model.margin(q+bias)<=.05:raise RuntimeError('WORKSPACE_TRANSITION_TO_REGRASP')
  distance=np.linalg.norm(planned_reference[:3,3]-tcp()[:3,3]);move(q+bias,max(.5,1.5*distance/.001));qvelocity=np.zeros(6);hold(.1)
  endpoint_reference=command_state
  save_note('endpoint_reference.json',{'reference_rad':endpoint_reference,'estimated_angle_rad':estimated_state(),'reference_cap_deg':90.5,'state_source':'RGB-D; no GT fallback','t':tick*dt})
 if endpoint_reference>=np.deg2rad(90.5):reference_target_completed=True


def perform_visual_switch():
 global whole,cycle,first_regrasp_state,endpoint_reference,regrasp_count,initial_closure_attempts
 capture_tracking(True,'visual_recovery_reobserve_before_plan')
 whole=plan_switch()
 if whole is None:
  # One current-image local replanning attempt, using the same finite planner.
  hold(.3);capture_tracking(True,'visual_recovery_reobserve_retry');whole=plan_switch()
 if whole is None:return False
 cycle+=1
 entry={'cycle':cycle,'release_estimated_angle_deg':float(np.rad2deg(estimated_state())),'base_before':list(base),'source':'RGB-D dynamic state + acquired visual template'}
 try:estimated_switch(whole)
 except RuntimeError as error:
  if str(error) not in ('BILATERAL_HOLD_NOT_ESTABLISHED','CLOSURE_CENTERING_IK_FAILURE'):raise
  # Reuse the established local contact-centering retry at measured TCP. It
  # keeps the original limits and never substitutes a GT handle/body pose.
  recovery_events.append({'event':str(error),'cycle':cycle,'t':tick*dt,'action':'safe release/local retreat/reapproach acquired visual contact pose'})
  old_attempts=initial_closure_attempts;retry_initial_contact();initial_closure_attempts=old_attempts;regrasp_count+=1
 entry.update(regrasp_estimated_angle_deg=float(np.rad2deg(estimated_state())),base_after=list(base),pad_loads_n=filtered().tolist(),regrasp_count=regrasp_count)
 switches.append(entry);save_note('switch_events.json',switches)
 if first_regrasp_state is None:first_regrasp_state=estimated_state()
 capture_tracking(True,f'visual_regrasp_{regrasp_count:02d}_confirmed')
 save_note(f'regrasp_{regrasp_count:02d}_contact.json',{'estimated_angle_deg':float(np.rad2deg(estimated_state())),'pad_loads_n':filtered(),'base':base,'t':tick*dt,'physical_contact_confirmed':True})
 endpoint_reference=None
 return True

try:
 try:exec(compile(initial_visual_sequence,str(visual_runner)+'::estimated_state_initial_grasp','exec'),globals())
 except RuntimeError as error:
  if str(error)!='BILATERAL_HOLD_NOT_ESTABLISHED':raise
  for retry in range(12):
   try:retry_initial_contact();break
   except RuntimeError as retry_error:
    if str(retry_error)!='BILATERAL_HOLD_NOT_ESTABLISHED':raise
  if not grasp_success:raise RuntimeError('INITIAL_VISUAL_CONTACT_ATTEMPTS_EXHAUSTED_SCENE_PRESERVED')
 capture_tracking(True,'initial_visual_grasp_complete')
 visual_template_closed=collision_body_closed@np.linalg.inv(estimated_moving())@tcp()
 save_note('acquired_visual_template.json',{'T_tcp_closed':visual_template_closed,'state_source':'RGB-D estimated imported-body pose','GT_grasp_or_dynamic_pose_input':False})
 while not reference_target_completed:
  need_switch=False
  try:
   if regrasp_count>0 and estimated_state()>=np.deg2rad(86.):
    measured_endpoint()
    if reference_target_completed:break
    continue
   path,bias,blocker=available_arc(np.deg2rad(90.));need_switch=blocker is not None or regrasp_count==0
   if need_switch and len(path)>2:path=path[:max(2,int((len(path)-1)*.75)+1)]
   if not need_switch:path=[point for point in path if point['state']<=np.deg2rad(86.)]
   save_note(f'available_arc_cycle_{cycle:02d}.json',{'path':path,'blocker':blocker,'current_angle_source':'RGB-D','switch_selection':'existing 75 percent reachable interval heuristic'})
   execute_arc(path,bias)
   if not need_switch:
    measured_endpoint()
    if reference_target_completed:break
    continue
  except RuntimeError as error:
   if str(error) not in ('PROBE_CARTESIAN_SPEED_LIMIT','CONTACT_LOST_PAUSE_IN_SAME_SCENE','NO_IK_TRANSITION_TO_REGRASP','WORKSPACE_TRANSITION_TO_REGRASP'):raise
   recovery_events.append({'event':str(error),'t':tick*dt,'estimated_angle_deg':float(np.rad2deg(estimated_state())),'scene_preserved':True})
   qvelocity=np.zeros(6)
   if str(error)=='PROBE_CARTESIAN_SPEED_LIMIT':
    phase='SAFE_HOLD';hold(.5)
    try:measured_endpoint()
    except RuntimeError as local_error:
     if str(local_error) not in ('PROBE_CARTESIAN_SPEED_LIMIT','CONTACT_LOST_PAUSE_IN_SAME_SCENE','NO_IK_TRANSITION_TO_REGRASP','WORKSPACE_TRANSITION_TO_REGRASP'):raise
    if reference_target_completed:break
  if not perform_visual_switch():status='LOCAL_VISUAL_RECOVERY_CANDIDATES_EXHAUSTED_SCENE_PRESERVED';break
 if reference_target_completed:
  phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(.5);final_hold_completed=True;capture_tracking(True,'authored_reference_final_hold');success=True;status='VISUAL_STATE_AUTHORED_REFERENCE_COMPLETE'
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True)
world.pause()
try:world.render()
except Exception as error:diagnostic_issue('optional_final_render',error)
try:
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
 save_note('tracking/estimates.json',tracking_frames);save_note('tracking/GT_OFFLINE_EVALUATION_ONLY.json',tracking_gt);save_note('events.json',telemetry_events);save_note('switch_events.json',switches);save_note('recovery_events.json',recovery_events)
 actual_final=float(np.rad2deg(native_state_read()))
 physical_pass=bool(grasp_success and regrasp_count>0 and final_hold_completed and actual_final>=90.)
 result={'classification':'RGBD_DYNAMIC_STATE_VISUAL_MOBILE_REGRASP_KNOWN_HINGE','status':status,'physical_90_PASS_offline':physical_pass,'final_angle_GT_offline_deg':actual_final,'maximum_angle_GT_offline_deg':max_state,'final_estimated_angle_deg':float(np.rad2deg(estimated_state())),'reference_target_completed':reference_target_completed,'final_reference_deg':None if endpoint_reference is None else float(np.rad2deg(endpoint_reference)),'final_hold_completed':final_hold_completed,'initial_visual_grasp_success':bool(grasp_success),'initial_closure_attempts':initial_closure_attempts,'base_route_counter':base_moves,'physical_regrasps':regrasp_count,'switches':switches,'recoveries':recovery_events,'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'manual_continuations':manual_continuations,'GT_dynamic_state_fallback':False,'GT_initial_grasp_fallback':False,'GT_usage':'static known geometry/calibration; passive telemetry and post-execution evaluation. Not dynamic state/target inputs.','per_step_pose_field':'T_moving_link is the estimated imported-body pose','tracking_frames':len(tracking_frames),'telemetry_issues':telemetry_issues,'scene_preserved':True,'pid':os.getpid()}
 save_note('report.json',result);print('VISUAL_STATE90_RESULT',json.dumps(result),flush=True)
except Exception as error:diagnostic_issue('final_export',error)
try:video.stdin.close();video.wait(timeout=30)
except Exception as error:diagnostic_issue('video_finalize',error)
while app.is_running():
 if (a.output/'close_completed_scene').exists():app.close();break
 continuation=a.output/'continue_same_scene.py'
 if continuation.exists():
  command=continuation.read_text();continuation.rename(a.output/f'continuation_{time.time_ns()}.py');manual_continuations+=1
  try:world.play();exec(compile(command,str(continuation),'exec'),globals())
  except Exception as error:
   import traceback
   print(traceback.format_exc(),flush=True)
  world.pause()
 app.update();time.sleep(.1)
