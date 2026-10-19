"""Connect the frozen visual initial grasp to the frozen physical mobile switch.

Only plan inputs and orchestration are new. Initial grasp is RGB-D derived;
known articulation/body geometry supports motion and subsequent regrasp plans.
"""
import sys,json,textwrap,hashlib
from pathlib import Path
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
visual_runner=Path(spec['frozen_visual_source_dir'])/'run_7320_visual_manipulation.py'
visual_text=visual_runner.read_text()
# Execute the exact frozen setup and initial visual grasp, stopping before its
# short-opening orchestration. Its controller functions remain unmodified.
setup,initial=visual_text.split("try:\n rgbd_snapshot('initial')",1)
initial="rgbd_snapshot('initial')"+initial.split("\n if kind=='revolute':\n  # Preserve",1)[0]
exec(compile(setup,str(visual_runner)+'::setup','exec'),globals())
initial=textwrap.dedent(' '+initial)
switch_text=baseline_text.split('  # Execute the already-saved switch; retain the same physical scene.\n',1)[1].split("  profile_mark('OPEN_2')",1)[0]
switch_text=textwrap.dedent(switch_text)
switches=[];schedules=[];first_regrasp_state=None;manual_continuations=0
initial_closure_attempts=1;automatic_recoveries=[];goal_deg=90.
initial_visual_sequence=initial
_closure_start="  if job.get('timing_experiment',{}).get('contact_phase_original_closure_rate',False):policy['slow_closure_m_s']=.001*speed_scale"
closure_text=textwrap.dedent(_closure_start+baseline_text.split(_closure_start,1)[1].split("  profile_mark('OPEN_1')",1)[0])

def actual_state():
 return float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])

def body_at(state):
 return model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:float(state)})


def save_note(name,value):
 try:
  diagnostic_write(a.output/name,json.dumps(value,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)))
 except Exception as error:diagnostic_issue('optional_'+name,error)

def retry_initial_contact():
 """Reapproach the contact-centered visual pose; retain original closure limits."""
 global whole,phase,qvelocity,legal,initial_closure_attempts,grasp_success,grasp_state
 retry_T=tcp().copy()
 save_note(f'initial_closure_failure_{initial_closure_attempts:02d}.json',{'T_tcp':retry_T,'pad_loads_n':filtered(),'aperture_m':rows[-1]['aperture_m'],'source':'actual contact-centered visual grasp; no GT grasp pose'})
 saved_whole=whole
 whole={'switch_retreat':{'opening_m':min(.1,rows[-1]['aperture_m']+.02)}}
 try:exec(compile(switch_text.split("profile_mark('RETREAT')",1)[0],str(baseline)+'::unchanged_release','exec'),globals())
 finally:whole=saved_whole
 start=tcp().copy();escape=None
 for distance in (.03,.02,.01):
  for direction in (-start[:3,2],-start[:3,2]+np.array([0.,0.,.5]),np.array([0.,0.,1.])):
   direction=direction/np.linalg.norm(direction);seed=np.asarray(robot.get_joint_positions())[arm];candidate=[]
   for f in np.linspace(0.,1.,13):
    target=start.copy();target[:3,3]+=f*distance*direction;q=ik(target,seed)
    if q is None or model.margin(q)<=.05:break
    ok,why=collision.check(model.poses(q,base,width=opening),moving(),False)
    if not ok:break
    candidate.append(q);seed=q
   if len(candidate)==13:escape=candidate;break
  if escape is not None:break
 if escape is None:raise RuntimeError('LOCAL_VISUAL_RETRY_RETREAT_UNAVAILABLE_SCENE_PRESERVED')
 phase='CLEARANCE_RETREAT';qvelocity=np.zeros(6)
 for q in escape[1:]:
  distance=np.linalg.norm(model.poses(q,base,width=opening)['tcp_link'][:3,3]-tcp()[:3,3]);move(q,max(.25,1.5*distance/.003))
 start=tcp().copy();seed=np.asarray(robot.get_joint_positions())[arm];approach_retry=[]
 for f in np.linspace(0.,1.,21):
  target=retry_T.copy();target[:3,3]=(1-f)*start[:3,3]+f*retry_T[:3,3];q=ik(target,seed)
  if q is None:raise RuntimeError('LOCAL_VISUAL_RETRY_APPROACH_NO_IK_SCENE_PRESERVED')
  ok,why=collision.check(model.poses(q,base,width=opening),moving(),False)
  if not ok:raise RuntimeError('LOCAL_VISUAL_RETRY_APPROACH_'+why)
  approach_retry.append(q);seed=q
 phase='SECOND_APPROACH'
 for q in approach_retry[1:]:move(q,.15)
 initial_closure_attempts+=1;legal=False
 automatic_recoveries.append({'event':'VISUAL_CONTACT_RECENTER_RETRY','t':tick*dt,'attempt':initial_closure_attempts,'GT_grasp_pose':False})
 exec(compile(closure_text,str(baseline)+'::unchanged_closure','exec'),globals())
 grasp_success=bool(legal);grasp_state=actual_state()
 save_note('grasp_result.json',{'success':grasp_success,'closure_attempts':initial_closure_attempts,'state_at_grasp':grasp_state,'filtered_loads_n':filtered(),'simulation_s':tick*dt})

def slow_endpoint():
 # Reuse the already demonstrated measured-progress control body unchanged.
 # Its interactive pause/export footer is unnecessary inside this automatic run.
 text=(ROOT/'scripts/resume_revolute_endpoint.py').read_text()
 body=text.split('try:\n',1)[1].split('\nexcept Exception as error:',1)[0]
 exec(compile(textwrap.dedent(body),'resume_revolute_endpoint.py::existing_control','exec'),globals())

def available_arc(goal):
 """Select an available prefix on the acquired grasp's continuous IK branch."""
 theta=actual_state();E=tcp();origin=body_at(theta);seed=np.asarray(robot.get_joint_positions())[arm]
 bias=qtarget[arm]-seed;path=[];blocker=None
 states=np.linspace(theta,goal,max(2,int(abs(goal-theta)/np.deg2rad(1))+1))
 for state in states:
  target=body_at(state)@np.linalg.inv(origin)@E;q=ik(target,seed)
  if q is None:blocker='NO_IK';break
  if model.margin(q+bias)<=.05:blocker='EXISTING_JOINT_MARGIN';break
  P=model.poses(q+bias,base,width=rows[-1]['aperture_m'])
  ok,why=collision.check(P,body_at(state),True)
  if not ok:blocker=why;break
  path.append({'state':float(state),'q':q.tolist(),'T_tcp':target.tolist(),'margin_rad':model.margin(q+bias)});seed=q
 return path,bias,blocker

def execute_arc(path,bias):
 global arc,_arc_index,phase,command_state,planned_reference,qvelocity
 arc=path;previous=tcp();executed=[]
 profile_mark('OPEN_1' if regrasp_count==0 else 'OPEN_2')
 for _arc_index,waypoint in enumerate(arc):
  phase='OPEN_1' if regrasp_count==0 else 'OPEN_2';command_state=waypoint['state'];planned_reference=np.asarray(waypoint['T_tcp'])
  distance=np.linalg.norm(planned_reference[:3,3]-previous[:3,3]);duration=max(.25,1.5*distance/.003)*float(job.get('visual_mobile_motion_duration_scale',1.))
  if force_guard.pending:
   phase='FORCE_HOLD';hold(.25)
   if not force_guard.settled():raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
   force_guard.pending=False;duration*=2
  move(np.asarray(waypoint['q'])+bias,duration);previous=planned_reference.copy()
  executed.append(dict(waypoint,actual_state=actual_state()))
 qvelocity=np.zeros(6)
 schedules.append({'grasp_index':regrasp_count,'segments':executed})
 save_note('executed_schedule.json',schedules)

def plan_switch():
 """Reuse the existing resume planner with the physically acquired template."""
 from plan_revolute_transfer import plan
 import copy
 folder=a.output/f'switch_{regrasp_count+1:02d}';folder.mkdir(exist_ok=True)
 resume={'state':actual_state(),'base':list(base),'T_tcp':tcp().tolist(),'q_arm':np.asarray(robot.get_joint_positions())[arm].tolist(),'aperture_m':rows[-1]['aperture_m'],'T_template_closed':visual_template_closed.tolist()}
 planning_job=copy.deepcopy(job);planning_job['resume']=resume
 diagnostic_write(folder/'job.json',json.dumps(planning_job,indent=2))
 world.pause()
 try:summary=plan(planning_job,export,folder,wall_s=900)
 finally:world.play()
 if summary['found']:return json.loads((folder/'whole_plan.json').read_text())
 return None

def continue_manipulation():
 global whole,qvelocity,phase,status,success,first_regrasp_state,final_hold_completed,command_state,planned_reference
 target=np.deg2rad(90.)
 while actual_state()<target:
  path,bias,blocker=available_arc(target)
  # Use the existing planner's interior-switch fraction on this grasp's range,
  # rather than the old door plan's fixed switch angle. This is plan selection.
  need_switch=blocker is not None or regrasp_count==0
  if need_switch and len(path)>2:path=path[:max(2,int((len(path)-1)*.75)+1)]
  diagnostic_write(a.output/f'available_arc_{regrasp_count:02d}.json',json.dumps({'path':path,'blocker':blocker,'switch_selection':'75 percent of this acquired grasp feasible interval; existing planner heuristic','GT_initial_grasp':False},indent=2))
  if not need_switch:
   path=[point for point in path if point['state']<=target-np.deg2rad(4.)]
  try:execute_arc(path,bias)
  except RuntimeError as error:
   if str(error) not in ('PROBE_CARTESIAN_SPEED_LIMIT','CONTACT_LOST_PAUSE_IN_SAME_SCENE'):raise
   automatic_recoveries.append({'event':str(error),'t':tick*dt,'angle_deg':float(np.rad2deg(actual_state()))})
   need_switch=str(error)=='CONTACT_LOST_PAUSE_IN_SAME_SCENE'
  if actual_state()>=target:break
  if not need_switch:
   before=actual_state()
   try:slow_endpoint()
   except RuntimeError as error:
    if str(error) not in ('NO_IK_TRANSITION_TO_REGRASP','WORKSPACE_TRANSITION_TO_REGRASP','CONTACT_LOST_PAUSE_IN_SAME_SCENE','PROBE_CARTESIAN_SPEED_LIMIT'):raise
    automatic_recoveries.append({'event':str(error),'t':tick*dt,'angle_deg':float(np.rad2deg(actual_state()))})
    need_switch=True
   if actual_state()>=target:break
   if not need_switch and actual_state()>before:continue
  whole=plan_switch()
  if whole is None:
   status='LOCAL_SWITCH_PLAN_UNAVAILABLE_SCENE_PRESERVED';return
  entry={'release_angle_deg':float(np.rad2deg(actual_state())),'base_before':list(base),'source':'frozen RGB-D acquired template transported by known moving body'}
  exec(compile(switch_text,str(baseline)+'::unchanged_mobile_switch','exec'),globals())
  entry.update(regrasp_angle_deg=float(np.rad2deg(actual_state())),base_after=list(base),pad_loads_n=filtered().tolist(),regrasp_count=regrasp_count)
  switches.append(entry)
  if first_regrasp_state is None:first_regrasp_state=actual_state()
  diagnostic_write(a.output/'switch_events.json',json.dumps(switches,indent=2))
 phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(.5);final_hold_completed=True
 success=bool(grasp_success and regrasp_count>0 and actual_state()>=target)
 status='VISUAL_MOBILE_REGRASP_90_SUCCESS' if actual_state()>=target and success else 'VISUAL_MOBILE_REGRASP_SUCCESS' if success else 'CONTINUATION_REQUIRED'

try:
 try:exec(compile(initial_visual_sequence,str(visual_runner)+'::frozen_initial_grasp','exec'),globals())
 except RuntimeError as error:
  if str(error)!='BILATERAL_HOLD_NOT_ESTABLISHED':raise
  for retry in range(int(job.get('maximum_range',{}).get('template_closures',12))):
   try:retry_initial_contact();break
   except RuntimeError as retry_error:
    if str(retry_error)!='BILATERAL_HOLD_NOT_ESTABLISHED':raise
  if not grasp_success:raise RuntimeError('INITIAL_VISUAL_CONTACT_ALTERNATIVES_EXHAUSTED_SCENE_PRESERVED')
 visual_template_closed=body_at(0.)@np.linalg.inv(body_at(actual_state()))@tcp()
 diagnostic_write(a.output/'acquired_visual_template.json',json.dumps({'T_tcp_closed':visual_template_closed.tolist(),'source':'fresh RGB-D initial grasp after physical closure; known articulation transport','GT_grasp_template':False},indent=2))
 continue_manipulation()
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True)

# All recoverable exits preserve the same actor and its loaded control state.
# Evidence is exported even when physical continuation needs a local correction.
world.pause()
try:world.render()
except Exception as error:diagnostic_issue('optional_final_render',error)
def publish():
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
 diagnostic_write(a.output/'events.json',json.dumps(telemetry_events,indent=2))
 result={'status':status,'initial_visual_grasp_success':grasp_success,'initial_closure_attempts':int(globals().get('initial_closure_attempts',1)),'GT_initial_grasp_fallback':False,'classification':'VISUAL_INITIAL_GRASP_KNOWN_MODEL_MOBILE_REGRASP','final_angle_deg':float(np.rad2deg(actual_state())),'maximum_angle_deg':max_state,'base_moves':base_moves,'regrasp_count':regrasp_count,'switches':switches,'additional_after_first_regrasp_deg':None if first_regrasp_state is None else float(np.rad2deg(actual_state()-first_regrasp_state)),'milestone_success':bool(first_regrasp_state is not None and actual_state()-first_regrasp_state>=np.deg2rad(10)),'authored_90_success':bool(actual_state()>=np.deg2rad(90)),'final_hold_completed':final_hold_completed,'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'peak_pad_load_n':max((max(r['forces_n'].values()) for r in rows),default=None),'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),'telemetry_issues':telemetry_issues,'manual_continuations':manual_continuations,'automatic_recoveries':automatic_recoveries,'pid':os.getpid(),'scene_preserved':True,'known_dependencies':['scene and base setup','native collision and contact pairs','known hinge geometry/progress for manipulation','moving-body pose for regrasp planning; physical visual template reused'],'no_object_execution_commands':True,'fully_gt_free':False}
 diagnostic_write(a.output/'report.json',json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)));print('VISUAL_MOBILE_RESULT',json.dumps(result,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x)),flush=True)
try:publish()
except Exception as error:diagnostic_issue('final_publish',error)
if success:
 try:video.stdin.close();video.wait(timeout=30)
 except Exception as error:diagnostic_issue('video_close',error)
while app.is_running():
 if (a.output/'close_completed_scene').exists():app.close();break
 continuation=a.output/'continue_same_scene.py'
 if continuation.exists():
  command=continuation.read_text();continuation.rename(a.output/f'continuation_{time.time_ns()}.py');manual_continuations+=1
  try:world.play();exec(compile(command,str(continuation),'exec'),globals())
  except Exception as error:
   import traceback
   status=repr(error);print(traceback.format_exc(),flush=True)
  world.pause()
  try:publish()
  except Exception as error:diagnostic_issue('continuation_publish',error)
  if success:
   try:video.stdin.close();video.wait(timeout=30)
   except Exception as error:diagnostic_issue('video_close',error)
 app.update();time.sleep(.1)
