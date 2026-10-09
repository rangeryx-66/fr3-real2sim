"""Execute in the completed clean drawer actor, after offline prefix freezing.
Reuses the physical contact controller and progress-bounded pull unchanged.
No reset, object command, new gate or global planner.
"""
fullrange_target=float(model.manifest['source_joint_limits_rad']['upper'])
fullrange_stage=globals().get('fullrange_stage',0)+1
fullrange_prefix_steps=globals().get('fullrange_prefix_steps',tick)
fullrange_video_name=f'full_range_stage_{fullrange_stage:02d}.mp4'
try:
 video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(a.output/fullrange_video_name)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=open(a.output/f'full_range_video_{fullrange_stage:02d}.log','w'))
except Exception as error:video=None;diagnostic_issue('full_range_video',error)
note('AUTHORED_RANGE_EXTENSION_START',target_m=fullrange_target,stage=fullrange_stage,actor_pid=os.getpid())
fullrange_exception=None
try:
 pull_to(fullrange_target,f'PULL_{regrasp_count+2}')
 phase='FULL_RANGE_HOLD';qvelocity=np.zeros(6)
 if np.max(filtered())<.05:reference=None
 hold(2.)
except Exception as error:
 fullrange_exception=repr(error)
 import traceback
 print(traceback.format_exc(),flush=True)
finally:
 world.pause();world.render()
 fullrange_pass=actual_state()>=fullrange_target
 note('AUTHORED_RANGE_STAGE_END',target_m=fullrange_target,stage=fullrange_stage,full_range_pass=fullrange_pass,error=fullrange_exception)
 try:video.stdin.close();video.wait(timeout=30)
 except Exception as error:diagnostic_issue('full_range_video_close',error)
 profile_mark('full_range_export')
 export_raw_records('full_range_observations',rows);export_raw_records('full_range_physics_steps',native.physics_steps)
 result={'status':'AUTHORED_RANGE_SUCCESS' if fullrange_pass else 'SAME_SCENE_CONTINUATION_AVAILABLE','scene_preserved':True,'actor_pid':os.getpid(),'mode':'KNOWN_MODEL_DIAGNOSTIC','designed_target_travel_m':fullrange_target,'target_source':'authored prismatic URDF upper; not a measured mechanical limit','final_displacement_m':actual_state(),'maximum_displacement_m':max_state,'base_moves':base_moves,'regrasps':regrasp_count,'grasp_records':grasp_records,'release_events':[e for e in telemetry_events if e['event']=='PHYSICAL_RELEASE'],'peak_pad_load_n':max(max(r['forces_n'].values()) for r in rows),'minimum_joint_margin_rad':min(r['margin_rad'] for r in rows),'total_simulation_s':tick*dt,'total_wall_s':time.perf_counter()-_profile_entry,'error':fullrange_exception,'stage':fullrange_stage,'video':fullrange_video_name,'prefix_steps':fullrange_prefix_steps,'clean_prefix_unchanged':True,'telemetry_issues':telemetry_issues,'true_contact_loss_events':[e for e in telemetry_events if e['event']=='TRUE_CONTACT_LOSS_RECOVERY'],'recent_execution_events':[e for e in telemetry_events if e['event'] in ('WORKSPACE_TRANSITION_TO_REGRASP','STALL_REANCHOR_OR_REGRASP','TRUE_CONTACT_LOSS_RECOVERY')][-8:]}
 diagnostic_write(a.output/f'full_range_stage_{fullrange_stage:02d}.json',json.dumps(result,indent=2))
 diagnostic_write(a.output/'full_range_status.json',json.dumps(result,indent=2))
 diagnostic_write(a.output/'full_range_events.json',json.dumps(telemetry_events,indent=2))
 diagnostic_write(a.output/'full_range_telemetry_summary.json',json.dumps({'physics_control_steps':tick,'available_field_counts':telemetry_counts,'issues':telemetry_issues,'disabled_optional_apis':sorted(telemetry_disabled),'telemetry_never_used_as_gate':True},indent=2))
 print('FULL_RANGE_STAGE',json.dumps({k:v for k,v in result.items() if k not in ('grasp_records','release_events')}),flush=True)
 profile_mark('full_range_scene_pause')
