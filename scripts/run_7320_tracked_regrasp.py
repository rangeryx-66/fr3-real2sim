"""One mobile regrasp experiment, with passive RGB-D hinge tracking first.
Loads the exact frozen visual/mobile helpers; never edits the solved baseline.
GT evaluation is stored separately from estimator observations.
"""
import sys,json,textwrap,hashlib,importlib.util
from pathlib import Path
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
frozen_mobile=Path(spec['frozen_mobile_runner'])
text=frozen_mobile.read_text();prefix=text.split("try:\n try:exec(compile(initial_visual_sequence",1)[0]
exec(compile(prefix,str(frozen_mobile)+'::frozen_helpers','exec'),globals())
from rgbd_hinge_tracking import RGBDHingeTracker
from wrist_reconstruction.self_observation import robot_projection_mask
from interaction_identification.contact_probe import robot_only_model
tracking_mode=job.get('tracking_mode','shadow');tracker=None;tracking_last_tick=-10000;tracking_last_phase=None
tracking_gt=[];tracking_frames=[];tracking_issues=[];manual_continuations=0
native_moving_read=moving
native_state_read=lambda:float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
robot_visual_model=robot_only_model(ROOT/'config/piper.urdf')
tracking_dir=a.output/'tracking';tracking_dir.mkdir(exist_ok=True)


def initialize_tracker(folder):
 global tracker
 folder=Path(folder);meta=json.loads((folder/'camera.json').read_text());rgb=cv2.cvtColor(cv2.imread(str(folder/'rgb.png')),cv2.COLOR_BGR2RGB);depth=np.load(folder/'depth_m.npy')
 # Segmentation is RGB-only SAM3. A missing door mask uses the actually
 # observed handle mask, not a simulator mask or a GT geometry substitute.
 sam=job['wrist_experiment']['sam3'];env=dict(os.environ,PYTHONPATH=sam['pythonpath'],CUDA_VISIBLE_DEVICES=str(job['visual_gpu']),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1')
 try:
  with (tracking_dir/'panel_sam3.log').open('w') as log:subprocess.run([sam['python'],str(ROOT/'scripts/infer_sam3_part.py'),'--rgb',str(folder/'rgb.png'),'--prompt','microwave door','--checkpoint',sam['checkpoint'],'--output',str(tracking_dir/'panel_mask.npy')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
  mask=np.load(tracking_dir/'panel_mask.npy');mask_source='RGB-only SAM3 microwave door'
 except Exception as error:
  diagnostic_issue('shadow_panel_segmentation',error);mask=np.load(folder/'handle_mask.npy');mask_source='observed initial RGB-only handle mask'
 M0=body_at(0.);D=body_at(.1)@np.linalg.inv(M0);axis_known=Rotation.from_matrix(D[:3,:3]).as_rotvec()/.1;axis_known/=np.linalg.norm(axis_known)
 origin_known=np.linalg.lstsq(np.vstack((np.eye(3)-D[:3,:3],axis_known)),np.r_[D[:3,3],axis_known@M0[:3,3]],rcond=None)[0]
 tracker=RGBDHingeTracker(rgb,depth,meta['K'],meta['T_world_camera_optical'],mask,origin_known,axis_known,M0,t=tick*dt)
 save_note('tracking/known_geometry.json',{'axis_world':axis_known,'origin_world':origin_known,'T_moving_closed':M0,'source':'known fixed hinge geometry; closed-start prior; no runtime articulation state','mask_source':mask_source,'initial_features':len(tracker.pixels),'estimator_inputs':['actual RGB-D','camera calibration','robot self exclusion','known fixed hinge geometry','previous visual estimates']})


def capture_tracking(force=False,label=None):
 global tracking_last_tick,tracking_last_phase
 if tracker is None:return None
 if not force and tick-tracking_last_tick<240 and phase==tracking_last_phase:return None
 camera=scene['overview'][0];rgb=np.asarray(camera.get_rgba())[...,:3].copy();depth=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data()).copy();K=np.asarray(camera.get_intrinsics_matrix());C=matrix(*camera.get_world_pose(camera_axes='ros'));q=np.asarray(robot.get_joint_positions())
 P=robot_visual_model.poses(q[arm],base,finger_q=q[fingers]);excluded=robot_projection_mask(depth,K,C,robot_visual_model.robot_hulls,P)
 t=tick*dt;record=tracker.update(rgb,depth,t,excluded,phase)
 index=len(tracking_frames);filename=f'frame_{index:05d}.npz';np.savez_compressed(tracking_dir/filename,rgb=rgb,depth_m=depth,robot_excluded=excluded,K=K,T_world_camera_optical=C,robot_q=q,base=np.asarray(base),T_tcp=tcp())
 record.update(frame=filename,physics_step=tick,observation_t=t,label=label,control_mode=tracking_mode);tracking_frames.append(record)
 # These GT records are passive evidence only; never arguments to the tracker
 # or stored in its input NPZ. Evaluation is performed after execution.
 tracking_gt.append({'frame':filename,'t':t,'angle_rad_GT':native_state_read(),'T_moving_GT':native_moving_read().tolist(),'phase':phase})
 save_note('tracking/live.json',record)
 tracking_last_tick=tick;tracking_last_phase=phase
 return record


original_step=step

def step():
 result=original_step()
 try:capture_tracking()
 except Exception as error:diagnostic_issue('passive_rgbd_tracking',error)
 return result

# Initial pose generation/ranking and physical closure remain exactly the
# frozen visual algorithm in shadow mode. The wrapper adds no robot steps.
visual_path=Path(job['frozen_visual_source_dir'])/'rgbd_manipulation_grasp.py'
visual_spec=importlib.util.spec_from_file_location('rgbd_manipulation_grasp',visual_path);visual_module=importlib.util.module_from_spec(visual_spec);sys.modules['rgbd_manipulation_grasp']=visual_module;visual_spec.loader.exec_module(visual_module)
original_estimate_and_plan=visual_module.estimate_and_plan

def estimate_with_tracking(g,folder):
 result=original_estimate_and_plan(g,folder);initialize_tracker(folder);return result

visual_module.estimate_and_plan=estimate_with_tracking

try:
 exec(compile(initial_visual_sequence,str(visual_runner)+'::frozen_initial_grasp','exec'),globals())
 visual_template_closed=body_at(0.)@np.linalg.inv(body_at(actual_state()))@tcp()
 capture_tracking(True,'initial_grasp_complete')
 path,bias,blocker=available_arc(np.deg2rad(90.))
 if len(path)>2:path=path[:max(2,int((len(path)-1)*.75)+1)]
 execute_arc(path,bias);capture_tracking(True,'before_release_plan')
 whole=plan_switch()
 if whole is not None:
  entry={'release_angle_deg':float(np.rad2deg(actual_state())),'base_before':list(base)}
  exec(compile(switch_text,str(baseline)+'::unchanged_mobile_switch','exec'),globals())
  entry.update(regrasp_angle_deg=float(np.rad2deg(actual_state())),base_after=list(base),pad_loads_n=filtered().tolist(),regrasp_count=regrasp_count);switches.append(entry);first_regrasp_state=actual_state()
  capture_tracking(True,'physical_regrasp_complete')
  path,bias,blocker=available_arc(first_regrasp_state+np.deg2rad(12.));execute_arc(path,bias)
  phase='FINAL_HOLD';hold(.5);final_hold_completed=True;capture_tracking(True,'milestone_hold')
  success=bool(actual_state()-first_regrasp_state>=np.deg2rad(10.));status='SHADOW_PHYSICAL_REGRASP_CONTINUATION' if success else 'CONTINUATION_REQUIRED_SCENE_PRESERVED'
 else:status='LOCAL_SWITCH_PLAN_UNAVAILABLE_SCENE_PRESERVED'
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True)
world.pause()
try:world.render()
except Exception as error:diagnostic_issue('optional_render',error)
try:
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
 save_note('tracking/estimates.json',tracking_frames);save_note('tracking/GT_OFFLINE_EVALUATION_ONLY.json',tracking_gt);save_note('events.json',telemetry_events);save_note('switch_events.json',switches)
 result={'classification':'RGBD_TRACKING_SHADOW_KNOWN_MODEL_CONTROL','status':status,'success':bool(success),'initial_visual_grasp_success':bool(grasp_success),'final_angle_GT_offline_deg':float(np.rad2deg(native_state_read())),'maximum_angle_GT_offline_deg':max_state,'base_route_counter':base_moves,'physical_regrasps':regrasp_count,'switches':switches,'additional_after_regrasp_GT_offline_deg':None if first_regrasp_state is None else float(np.rad2deg(native_state_read()-first_regrasp_state)),'tracking_mode':tracking_mode,'tracking_frames':len(tracking_frames),'tracking_used_by_controller':False,'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'manual_continuations':manual_continuations,'telemetry_issues':telemetry_issues,'scene_preserved':True,'pid':os.getpid()}
 save_note('report.json',result);print('TRACKED_REGRASP_RESULT',json.dumps(result),flush=True)
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
