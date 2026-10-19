"""Isolated grasp-source experiment; execute frozen control helpers unchanged.
Shadow runs the original initial approach/closure. Visual mode replaces only the
candidate and uses an RGB-D re-observation at pregrasp. No frozen source is edited.
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

attempt_started=False
try:
 rgbd_snapshot('initial')
 (a.output/'input_ready.json').write_text(json.dumps({'mode':job['grasp_source_mode'],'baseline_source_sha256':hashlib.sha256(baseline_text.encode()).hexdigest(),'pid':os.getpid()},indent=2))
 if job['grasp_source_mode']=='visual':
  from rgbd_grasp_source import estimate_and_plan,refine_pregrasp
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
 success=bool(legal);status='PHYSICAL_GRASP_SUCCESS' if success else 'GRASP_NOT_ESTABLISHED'
 rgbd_snapshot('after_grasp')
except BaseException as error:
 import traceback
 status=repr(error);print(traceback.format_exc(),flush=True);success=False
finally:
 world.pause();world.render()
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
 try:video.stdin.close();video.wait(timeout=30)
 except Exception as error:diagnostic_issue('video_close',error)
 report={'mode':job['grasp_source_mode'],'status':status,'success':success,'first_physical_attempt':attempt_started,'closure_attempts':int(any(r['phase'] in ('CLOSE','SLOW_CLOSE','CLOSE_GRIPPER','CENTER','LOW_PRELOAD','FORCE_HOLD','VERIFY_PHYSICAL_CONTACT') for r in rows)),'legal_bilateral_hold':legal,'baseline_source_sha256':hashlib.sha256(baseline_text.encode()).hexdigest(),'simulation_s':tick*dt,'wall_s':time.perf_counter()-_profile_entry,'peak_pad_load_n':max((max(r['forces_n'].values()) for r in rows),default=None),'minimum_joint_margin_rad':min((r['margin_rad'] for r in rows),default=None),'final_filtered_loads_n':filtered().tolist() if rows else None,'telemetry_issues':telemetry_issues,'scene_preserved':True,'no_object_execution_commands':True,'fully_gt_free_execution':False,'known_dependencies':['scene setup','base station','collision scene','contact pairing','GT measurement only'],'pid':os.getpid()}
 (a.output/'report.json').write_text(json.dumps(report,indent=2));(a.output/'events.json').write_text(json.dumps(telemetry_events,indent=2));print('GRASP_TRIAL_RESULT',json.dumps(report),flush=True)
 while app.is_running():
  if (a.output/'close_completed_scene').exists():app.close();break
  app.update();time.sleep(.1)
