"""Manipulation-aware RGB-D grasp and incremental short physical opening.
Only experiment orchestration, camera perturbation and endpoint selection live here.
Visual modules and known-model control helpers are loaded unchanged from snapshots.
"""
import sys,json,hashlib,textwrap
from pathlib import Path
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
baseline=Path(spec['baseline_runner']);baseline_text=baseline.read_text()
prefix,_,operation=baseline_text.partition("try:\n phase='SETTLE';hold(.5)")
start="phase='SETTLE';hold(.5)"
exec(compile(prefix,str(baseline),'exec'),globals())

def rgbd_snapshot(label):
 folder=a.output/'rgbd'/label;folder.mkdir(parents=True,exist_ok=True)
 camera=scene['overview'][0]
 if 'distance_to_image_plane' not in camera._custom_annotators:camera.add_distance_to_image_plane_to_frame()
 for _ in range(8):world.render()
 rgb=np.asarray(camera.get_rgba())[...,:3].copy();dep=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data()).copy()
 cv2.imwrite(str(folder/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.save(folder/'depth_m.npy',dep)
 meta={'K':np.asarray(camera.get_intrinsics_matrix()).tolist(),'T_world_camera_optical':matrix(*camera.get_world_pose(camera_axes='ros')).tolist(),'depth_units':'m optical Z','camera_source':'existing calibrated fixed overview RGB-D; no instance mask','T_world_tcp_measured':tcp().tolist(),'robot_q':np.asarray(robot.get_joint_positions()).tolist(),'base':list(base),'t_sim':tick*dt,'label':label}
 (folder/'camera.json').write_text(json.dumps(meta,indent=2));return folder

# Actual camera motion, fixed for all observations within this trial. No object
# geometry or handle pose is used to choose this predeclared perturbation.
cam=scene['overview'][0];C0=matrix(*cam.get_world_pose(camera_axes='ros'));delta=job['observation_perturbation']
C1=C0.copy();C1[:3,3]+=C0[:3,:3]@np.asarray(delta['translation_camera_m']);C1[:3,:3]=C0[:3,:3]@Rotation.from_euler('xyz',delta['rotation_camera_deg'],degrees=True).as_matrix()
cam.set_world_pose(position=C1[:3,3],orientation=np.roll(Rotation.from_matrix(C1[:3,:3]).as_quat(),1),camera_axes='ros')
(a.output/'camera_perturbation.json').write_text(json.dumps({'specified':delta,'T_before':C0.tolist(),'T_after':C1.tolist(),'camera_only_physics_unchanged':True},indent=2))
sys.path.insert(0,job['frozen_visual_source_dir'])
attempt_started=False;grasp_success=False;manipulation_success=False;grasp_state=None;final_hold_completed=False
try:
 rgbd_snapshot('initial')
 (a.output/'input_ready.json').write_text(json.dumps({'mode':job['grasp_source_mode'],'baseline_source_sha256':hashlib.sha256(baseline_text.encode()).hexdigest(),'pid':os.getpid()},indent=2))
 if job['grasp_source_mode']=='visual':
  from rgbd_manipulation_grasp import estimate_and_plan,refine_pregrasp
  chosen,T=estimate_and_plan(globals(),a.output/'rgbd/initial')
 terminal="  profile_mark('OPEN_1')" if kind=='revolute' else ";successful_body_TCP="
 initial=operation.partition(terminal)[0]
 if terminal not in operation:raise ValueError('frozen initial-grasp section not located')
 initial=' '+start+initial
 initial=initial.replace("move(qpre,.8);phase='APPROACH'","move(qpre,.8);rgbd_snapshot('pregrasp');phase='APPROACH'")
 if job['grasp_source_mode']=='visual':
  initial=initial.replace("rgbd_snapshot('pregrasp');phase='APPROACH'","rgbd_snapshot('pregrasp');T,approach=refine_pregrasp(globals(),a.output/'rgbd/pregrasp',T);phase='APPROACH'")
 attempt_started=True
 exec(compile(textwrap.dedent(initial),str(baseline)+'::initial_grasp','exec'),globals())
 grasp_success=bool(legal)
 try:rgbd_snapshot('after_grasp')
 except Exception as error:diagnostic_issue('after_grasp_capture',error)
 grasp_state=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
 (a.output/'grasp_result.json').write_text(json.dumps({'success':grasp_success,'closure_attempts':1,'state_at_grasp':grasp_state,'filtered_loads_n':filtered().tolist(),'simulation_s':tick*dt},indent=2))
 if kind=='revolute':
  # Preserve loaded command equilibrium, but prepare and execute one local
  # hinge segment at a time. A later NO_IK cannot cancel an earlier valid move.
  profile_mark('OPEN_1');E0=tcp();theta0=grasp_state
  origin=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta0})
  seed=np.asarray(robot.get_joint_positions())[arm];command_bias=qtarget[arm]-seed
  goal=grasp_state+np.deg2rad(job['manipulation_target_delta_deg'])
  states=[float(x['state']) for x in whole['path'] if grasp_state<=float(x['state'])<goal]+[goal]
  arc=[{'state':x} for x in states];previous=E0;executed=[]
  for _arc_index,waypoint in enumerate(arc):
   state=waypoint['state'];body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:state});target=body@np.linalg.inv(origin)@E0
   q=ik(target,seed)
   if q is None or model.margin(q+command_bias)<=.05:
    telemetry_events.append({'event':'WORKSPACE_REGRASP_REQUEST','t_sim':tick*dt,'reference_state':state,'actual_state':float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]),'scene_preserved':True})
    break
   phase='OPEN_1';command_state=state;planned_reference=target
   distance=np.linalg.norm(target[:3,3]-previous[:3,3]);duration=max(.25,1.5*distance/.003)
   if force_guard.pending:
    phase='FORCE_HOLD';hold(.25)
    if not force_guard.settled():raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
    force_guard.pending=False;duration*=2
   move(q+command_bias,duration);seed=q;previous=target
   executed.append({'reference_state':state,'T_tcp_reference':target.tolist(),'q':q.tolist(),'actual_state':float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])})
  (a.output/'manipulation_schedule.json').write_text(json.dumps({'execution':'incremental valid segments','GT_grasp_template':False,'segments':executed},indent=2))

 else:
  goal=grasp_state+job['manipulation_target_delta_m'];pull_to(goal,'PULL_VISUAL')
 phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(.5);final_hold_completed=True
 final_state=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
 # Post-execution outcome metrics, never control/validation gates.
 manipulation_success=bool(final_state-grasp_state >= (np.deg2rad(10.) if kind=='revolute' else .010))
 success=grasp_success and manipulation_success;status='VISUAL_GRASP_AND_MANIPULATION_SUCCESS' if success else 'SHORT_MANIPULATION_TARGET_NOT_REACHED'
 try:rgbd_snapshot('after_manipulation')
 except Exception as error:diagnostic_issue('after_manipulation_capture',error)
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True);success=False
finally:
 world.pause();world.render()
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
 try:video.stdin.close();video.wait(timeout=30)
 except Exception as error:diagnostic_issue('video_close',error)
 final_actual=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
 extension={'classification':'VISUAL_GRASP_KNOWN_ARTICULATION_MANIPULATION','grasp_success':grasp_success,'manipulation_success':manipulation_success,'final_hold_completed':final_hold_completed,'state_at_grasp':grasp_state,'final_actual_state':final_actual,'additional_actual_state':None if grasp_state is None else final_actual-grasp_state,'state_units':'rad' if kind=='revolute' else 'm','perturbation':job['observation_perturbation'],'GT_grasp_template_fallback':False,'visual_source_dir':job['frozen_visual_source_dir'],'motion_model':'existing known hinge path or measured-progress drawer pull_to; actual acquired grasp reference'}
 report={'mode':job['grasp_source_mode'],'status':status,'success':success,'first_physical_attempt':attempt_started,'closure_attempts':int(any(r['phase'] in ('CLOSE','SLOW_CLOSE','CLOSE_GRIPPER','CENTER','LOW_PRELOAD','FORCE_HOLD','VERIFY_PHYSICAL_CONTACT') for r in rows)),'legal_bilateral_hold':legal,'baseline_source_sha256':hashlib.sha256(baseline_text.encode()).hexdigest(),'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'peak_pad_load_n':max((max(r['forces_n'].values()) for r in rows),default=None),'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),'final_filtered_loads_n':filtered().tolist() if rows else None,'telemetry_issues':telemetry_issues,'scene_preserved':True,'no_object_execution_commands':True,'fully_gt_free_execution':False,'known_dependencies':['scene setup','base station','collision scene','contact pairing','known articulation model and native progress sensing','GT grasp comparison offline only'],'pid':os.getpid()}
 report.update(extension)
 (a.output/'report.json').write_text(json.dumps(report,indent=2));(a.output/'events.json').write_text(json.dumps(telemetry_events,indent=2));print('GRASP_TRIAL_RESULT',json.dumps(report),flush=True)
 while app.is_running():
  if (a.output/'close_completed_scene').exists():app.close();break
  app.update();time.sleep(.1)
