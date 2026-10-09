"""RGB-D moving-state inputs for one physical mobile regrasp.
Frozen servo/closure/release/base mechanisms, with perception inputs only.
GT dynamic state is read solely by passive evidence recording/evaluation.
"""
import sys,json,textwrap,types
from pathlib import Path
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
setup_runner=Path(spec['shadow_setup_runner']);setup_text=setup_runner.read_text().split("try:\n exec(compile(initial_visual_sequence",1)[0]
setup_text=setup_text.replace('from rgbd_hinge_tracking import RGBDHingeTracker','from rgbd_hinge_tracking_depth import RGBDDepthHingeTracker as RGBDHingeTracker')
exec(compile(setup_text,str(setup_runner)+'::observation_setup','exec'),globals())


def estimated_state():
 return tracker.state_at(tick*dt,phase) if tracker is not None else 0. # normal closed-start prior, never simulator readback


collision_body_closed=np.asarray(collision.moving_reference).copy()

def estimated_moving():
 D=body_at(estimated_state())@np.linalg.inv(body_at(0.))
 return D@collision_body_closed

original_initialize_tracker=initialize_tracker

def initialize_tracker(folder):
 original_initialize_tracker(folder)
 tracker.T_closed=collision_body_closed.copy()
 save_note('tracking/frame_convention.json',{'T_native_collision_body_closed':collision_body_closed,'T_model_link_closed':body_at(0.),'source':'known static closed geometry coordinate conversion; no runtime GT state'})

actual_state=estimated_state;moving=estimated_moving
original_capture_tracking=capture_tracking

def capture_tracking(force=False,label=None):
 record=original_capture_tracking(force,label)
 if record is not None:
  try:
   frame=scene['overview'][0].get_current_frame()
   record['camera_rendering_time']=frame.get('rendering_time');record['camera_rendering_frame']=frame.get('rendering_frame')
   with (tracking_dir/'estimates.jsonl').open('a') as stream:stream.write(json.dumps(record,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x))+'\n')
   with (tracking_dir/'GT_OFFLINE_EVALUATION_ONLY.jsonl').open('a') as stream:stream.write(json.dumps(tracking_gt[-1])+'\n')
  except Exception as error:diagnostic_issue('incremental_tracking_archive',error)
 return record

# Keep raw native joint readback in the original passive telemetry. It is not
# used by any state/target function below. Geometric checks use estimated pose;
# actual native contact guards, velocity/force limits and geometry are unchanged.
GT_expression="scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]"
initial_visual_sequence=initial_visual_sequence.replace(GT_expression,'estimated_state()')
switch_text=switch_text.replace(GT_expression,'estimated_state()').replace("regrasp_angle=rows[-1]['door_angle_deg']","regrasp_angle=float(np.rad2deg(estimated_state()))")

# The frozen visual algorithm uses the estimated angle for its known-hinge
# candidate ranking. Grasp geometry still comes from fresh SAM3 + RGB-D.
visual_source=visual_path.read_text()
visual_source=visual_source.replace("visual=infer(folder,g['job'],g['ROOT']);rm=", "visual=infer(folder,g['job'],g['ROOT']);g['initialize_tracker'](folder);rm=")
visual_source=visual_source.replace("theta=float(g['scene']['articulation'].get_joint_positions()[g['scene'].get('selected_asset_dof',0)])", "theta=g['estimated_state']()")
visual_source=visual_source.replace("initial=g['a'].output/'rgbd/initial'", "initial=g.get('refine_reference_dir',g['a'].output/'rgbd/initial')")
visual_module=types.ModuleType('rgbd_manipulation_grasp');visual_module.__file__=str(visual_path)+'::estimated_angle_input_adapter';exec(compile(visual_source,visual_module.__file__,'exec'),visual_module.__dict__);sys.modules['rgbd_manipulation_grasp']=visual_module


def estimated_switch(plan):
 """Original release/retreat/base, then refreshed visual pregrasp and closure."""
 global whole,phase,regrasp_count,refine_reference_dir,planned_reference,command_state,first_regrasp_state
 whole=plan
 planned_theta=float(plan['switch_state']);transported=body_at(planned_theta)@np.linalg.inv(body_at(0.))@visual_template_closed
 local_variant=np.linalg.inv(transported)@np.asarray(plan['second_station']['T_grasp'])
 before_approach=switch_text.split("profile_mark('APPROACH_2')",1)[0]
 exec(compile(before_approach,str(baseline)+'::unchanged_release_retreat_base','exec'),globals())
 capture_tracking(True,'after_base_relocation_visual_reobserve')
 refine_reference_dir=rgbd_snapshot('regrasp_reference')
 from rgbd_grasp_source import infer
 try:infer(refine_reference_dir,job,ROOT)
 except Exception as error:
  diagnostic_issue('regrasp_reference_segmentation',error)
  initial_handle=json.loads((a.output/'rgbd/initial/visual_handle.json').read_text())['anchor_world_m'];anchor=(tracker.delta(estimated_state())@np.r_[initial_handle,1])[:3]
  diagnostic_write(refine_reference_dir/'visual_handle.json',json.dumps({'anchor_world_m':anchor.tolist(),'source':'initial RGB-D handle transported by visual estimate; no GT fallback'}))
 profile_mark('APPROACH_2');phase='SECOND_APPROACH'
 for q in plan['second_station']['preplan'][1:]:move(q,1.5)
 # Current observed state, not planned/native articulation state, supplies the
 # regrasp target. Register same-state reference/pregrasp RGB-D for correction.
 pregrasp=rgbd_snapshot('regrasp_pregrasp');capture_tracking(True,'regrasp_pregrasp')
 T_est=tracker.delta(estimated_state())@visual_template_closed@local_variant
 T_refined,approach=visual_module.refine_pregrasp(globals(),pregrasp,T_est)
 save_note('regrasp_visual_target.json',{'source':'visual tracked moving pose + physically acquired visual grasp template + same-state RGB-D pregrasp registration','estimated_angle_rad':estimated_state(),'T_estimated':T_est,'T_refined':T_refined,'local_grasp_variant':local_variant,'GT_moving_pose_or_state_input':False})
 for q in approach[1:]:move(q,.15)
 closure_fragment=switch_text.split("profile_mark('regrasp_closure_settling')",1)[1]
 exec(compile("profile_mark('regrasp_closure_settling')"+closure_fragment,str(baseline)+'::unchanged_physical_regrasp','exec'),globals())

try:
 exec(compile(initial_visual_sequence,str(visual_runner)+'::estimated_state_initial_grasp','exec'),globals())
 capture_tracking(True,'initial_visual_grasp_complete')
 visual_template_closed=collision_body_closed@np.linalg.inv(estimated_moving())@tcp()
 save_note('acquired_visual_template.json',{'T_tcp_closed':visual_template_closed,'state_source':'RGB-D hinge estimate','GT_grasp_or_moving_pose_template':False})
 path,bias,blocker=available_arc(np.deg2rad(90.))
 if len(path)>2:path=path[:max(2,int((len(path)-1)*.75)+1)]
 execute_arc(path,bias);capture_tracking(True,'before_visual_release_plan')
 whole=plan_switch()
 if whole is not None:
  entry={'release_estimated_angle_deg':float(np.rad2deg(estimated_state())),'base_before':list(base),'target_source':'RGB-D estimated moving pose; no GT fallback'}
  estimated_switch(whole)
  first_regrasp_state=estimated_state()
  entry.update(regrasp_estimated_angle_deg=float(np.rad2deg(first_regrasp_state)),base_after=list(base),pad_loads_n=filtered().tolist(),regrasp_count=regrasp_count);switches.append(entry)
  # GT acquisition is archived separately and never bounds the next reference.
  offline_regrasp_truth=native_state_read()
  capture_tracking(True,'physical_visual_regrasp_complete')
  path,bias,blocker=available_arc(first_regrasp_state+np.deg2rad(12.));execute_arc(path,bias)
  phase='FINAL_HOLD';hold(.5);final_hold_completed=True;capture_tracking(True,'visual_regrasp_milestone_hold')
  success=True;status='ESTIMATED_STATE_PHYSICAL_SEQUENCE_COMPLETE'
 else:status='LOCAL_VISUAL_SWITCH_PLAN_UNAVAILABLE_SCENE_PRESERVED'
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True)
world.pause()
try:world.render()
except Exception as error:diagnostic_issue('optional_render',error)
try:
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
 save_note('tracking/estimates.json',tracking_frames);save_note('tracking/GT_OFFLINE_EVALUATION_ONLY.json',tracking_gt);save_note('events.json',telemetry_events);save_note('switch_events.json',switches)
 additional_GT=None if 'offline_regrasp_truth' not in globals() else float(np.rad2deg(native_state_read()-offline_regrasp_truth))
 physical_pass=bool(grasp_success and regrasp_count>0 and additional_GT is not None and additional_GT>=10.)
 result={'classification':'VISUAL_INITIAL_GRASP_VISUAL_MOVING_STATE_REGRASP_KNOWN_HINGE','status':status,'physical_milestone_PASS_offline':physical_pass,'initial_visual_grasp_success':bool(grasp_success),'final_estimated_angle_deg':float(np.rad2deg(estimated_state())),'final_angle_GT_offline_deg':float(np.rad2deg(native_state_read())),'maximum_angle_GT_offline_deg':max_state,'additional_after_regrasp_GT_offline_deg':additional_GT,'base_route_counter':base_moves,'physical_regrasps':regrasp_count,'switches':switches,'tracking_used_by_controller':True,'GT_dynamic_state_fallback':False,'GT_initial_grasp_fallback':False,'per_step_pose_field':'T_moving_link is visual estimated imported-body frame; GT pose stored only in separate tracking evaluation file','GT_recording_use':'passive raw telemetry and post-execution evaluation only; not state/target inputs','known_dependencies':['fixed hinge axis/origin and closed-link frame','robot kinematics and calibrated camera','known collision geometry/contact pairs','normal closed-start scene and initial base'],'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'manual_continuations':manual_continuations,'telemetry_issues':telemetry_issues,'scene_preserved':True,'pid':os.getpid()}
 save_note('report.json',result);print('ESTIMATED_REGRASP_RESULT',json.dumps(result),flush=True)
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
