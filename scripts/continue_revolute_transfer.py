"""Same-scene next-station execution using the shared 7320 sequence verbatim.
The only per-object input is next_station_plan.json, produced from actual state.
"""
whole=json.loads((a.output/'next_station_plan.json').read_text())
continuation_error=None
try:
 # Execute the already-saved switch; retain the same physical scene.
 profile_mark('pre_release_hold')
 phase='SWITCH_SAFE_HOLD';qvelocity=np.zeros(6) # already paused; release promptly after a contact recovery
 switch_actual=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
 print('STATE_DIAGNOSTIC',phase,rows[-1]['door_angle_deg'],flush=True)
 profile_mark('RELEASE');_profile_activity='motion'
 reference=None;planned_reference=None;phase='RELEASE';effort=0.;mode='position'
 kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kp[fingers]=1000.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True)
 opening=float(whole['switch_retreat']['opening_m']);present=float(rows[-1]['aperture_m'])
 # Release slowly; no stale closed-position command replaces the preload hold.
 # A protected stop may leave residual handle contact. Keep the arm still
 # and continue only opening the jaws at the original release rate. Every
 # physics/contact check still runs; all other physical failures propagate.
 protected_release=bool(globals().get('status','').startswith('FINGER_BACK_OR_ROOT_HANDLE_LOAD') or 'FINGER_BACK_OR_ROOT_HANDLE_LOAD' in (a.output/'paused.json').read_text())
 if protected_release:qtarget[arm]=np.asarray(robot.get_joint_positions())[arm];qvelocity=np.zeros(6)
 if whole.get('physical_resume_input',{}).get('released_aperture_m') is None:
  for w in np.linspace(present,opening,max(2,int(abs(opening-present)/(.001*speed_scale*dt))+1)):
   qtarget[fingers]=[w/2,-w/2]
   try:step()
   except RuntimeError as release_error:
    if not (protected_release and str(release_error).startswith('FINGER_BACK_OR_ROOT_HANDLE_LOAD')):raise
    telemetry_events.append({'event':'PROTECTED_RELEASE_CONTINUES_OPENING_ONLY','t':tick*dt,'reason':str(release_error),'closing_effort_n':effort,'arm_motion_command':'stationary'})
  hold(.3)
  try:(a.output/'release_measurement.json').write_text(json.dumps({'target_m':opening,'actual_m':rows[-1]['aperture_m'],'error_m':rows[-1]['aperture_m']-opening,'pad_loads_n':filtered().tolist(),'classification':'DIAGNOSTIC','closing_effort_explicitly_cleared':True},indent=2))
  except Exception as error:diagnostic_issue('optional_write',error)
  if np.any(filtered()>.05):hold(1.)
  if np.any(filtered()>.05):raise RuntimeError('RELEASE_CONTACT_REMAINS_PAUSE_IN_SAME_SCENE')
 profile_mark('RETREAT')
 phase='CLEARANCE_RETREAT'
 for q in whole['switch_retreat']['q_path'][1:]:
  distance=np.linalg.norm(model.poses(q,base,width=opening)['tcp_link'][:3,3]-tcp()[:3,3]);move(q,max(.25,1.5*distance/.003))
 hold(.3)
 released_state=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
 print('STATE_DIAGNOSTIC',phase,rows[-1]['door_angle_deg'],flush=True)
 from isaacsim.core.prims import SingleXFormPrim
 from interactive_twin_recovery.mobile import scene_at
 profile_mark('BASE_MOVE');_profile_activity='motion'
 _cached_base_scene=None
 support=SingleXFormPrim('/World/r1a7_pedestal');chassis=SingleXFormPrim('/World/mobile_chassis');phase='BASE_ROUTE'
 for target_base in whole['base_route']['waypoints'][1:]:
  old_base=np.asarray(base).copy();delta_base=np.asarray(target_base)-old_base;duration=max(np.linalg.norm(delta_base[:2])/.01,abs(delta_base[3])/5.,dt)/speed_scale
  for f in np.linspace(0.,1.,max(2,int(duration/dt)))[1:]:
   pose=old_base+f*delta_base;quat=np.roll(Rotation.from_euler('z',pose[3],degrees=True).as_quat(),1)
   # Existing kinematic SE(2) platform, only after verified release/clearance.
   robot.set_world_pose(pose[:3],quat);support.set_world_pose([pose[0],pose[1],-.33],quat);chassis.set_world_pose([pose[0],pose[1],-.66],quat)
   base[:]=pose.tolist();B=transform(base[:3],[0,0,np.deg2rad(base[3])]);
   if tick%8==0:
    _base_query_started=time.perf_counter()
    if job.get('timing_experiment',{}).get('reuse_invariant_collision_solids',False) and _cached_base_scene is not None:
     collision=refresh_same_base_collision(_cached_base_scene,export,initial_base,base)
    else:collision=scene_at(ROOT,export,model,initial_base,base)
    _cached_base_scene=collision
    profile_cost('base_collision_scene_refresh',_base_query_started)
   step()
 base_moves+=1;hold(.3)
 now=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
 print('STATE_DIAGNOSTIC',phase,rows[-1]['door_angle_deg'],flush=True)
 profile_mark('APPROACH_2')
 phase='SECOND_APPROACH'
 for q in whole['second_station']['preplan'][1:]:move(q,1.5)
 for q in whole['second_station']['approach'][1:]:move(q,.15)
 # Reclose with the original bilateral verification and same preload command.
 profile_mark('regrasp_closure_settling');_profile_activity='auto'
 legal=False;loss_s=0.;slip_s=0.
 if job.get('timing_experiment',{}).get('contact_phase_original_closure_rate',False):policy['slow_closure_m_s']=.001*speed_scale
 closure=JawCenteredClosure(tcp(),policy);phase='CLOSE'
 for _ in range(int(100/dt)):
  state=rows[-1];P=poses(np.asarray(state['q']));centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]
  cmd=closure.update(tcp(),state['aperture_m'],filtered(),centers,dt);phase=cmd['state']
  # Use the existing controller's physical contact state, with no new
  # threshold, sample requirement, wait, or confirmation condition.
  if job.get('timing_experiment',{}).get('contact_phase_original_closure_rate',False) and cmd['state'] in ('CENTER','LOW_PRELOAD','FORCE_HOLD'):policy['slow_closure_m_s']=.001
  if cmd['mode']=='position':qtarget[fingers]=[cmd['opening_m']/2,-cmd['opening_m']/2]
  else:
   if mode!='effort':
    kp=np.zeros(len(names));kd=kp.copy();kp[arm]=10000.;kd[arm]=400.;kd[fingers]=40.;controller.set_gains(kps=kp,kds=kd,save_to_usd=True);mode='effort'
   effort=cmd['closing_effort_n']
  q=ik(cmd['T'],np.asarray(state['q'])[arm])
  if q is None:raise RuntimeError('CLOSURE_CENTERING_IK_FAILURE')
  qvelocity=np.clip((q-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=q;step()
  ready,detail=grip_window()
  if closure.state=='FORCE_HOLD' and ready:legal=True;break
 if not legal:raise RuntimeError('BILATERAL_HOLD_NOT_ESTABLISHED')
 observe_grasp()
 qvelocity=np.zeros(6);phase='FORCE_HOLD';reference=np.linalg.inv(moving())@tcp();hold(.5)
 regrasp_count+=1;regrasp_angle=rows[-1]['door_angle_deg']
 try:(a.output/'regrasp_contact.json').write_text(json.dumps({'angle_deg':regrasp_angle,'pad_loads_n':filtered().tolist(),'contact_window':detail,'base':base,'physical_closure':True},indent=2))
 except Exception as error:diagnostic_issue('optional_write',error)
 profile_mark('OPEN_2')
 E0=tcp();D0=moving();theta0=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)]);seed=np.asarray(robot.get_joint_positions())[arm];command_bias=qtarget[arm]-seed
 try:(a.output/('command_bias_'+str(regrasp_count)+'.json' if 'regrasp_count' in locals() else 'command_bias.json')).write_text(json.dumps({'q_measured':seed.tolist(),'command_q':qtarget[arm].tolist(),'bias':command_bias.tolist(),'purpose':'retain the loaded position-command equilibrium at path transition; unchanged gains and jaw preload'},indent=2))
 except Exception as error:diagnostic_issue('optional_write',error)
 origin=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta0});arc=[]
 # Reuse the saved path, aligning the grasp after real physical closure.
 recovery_reason=None
 for saved in whole['second_station']['path']:
  s=max(theta0,float(saved['state']));body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:s});D=body@np.linalg.inv(origin)
  target=D@E0;q=ik(target,seed)
  if q is None:
   recovery_reason='NO_IK_TRANSITION_TO_REGRASP';break
  arc.append({'q':q.tolist(),'state':s,'T_tcp':target.tolist(),'margin_rad':model.margin(q)});seed=q
 preflight={'known_model_whole_path':arc,'min_joint_margin_rad':min((x['margin_rad'] for x in arc),default=None),'planning_goal':whole['goal_state'],'GT_planning':True,'end_retreat':whole['end_retreat']}
 try:(a.output/'aligned_second_leg.json').write_text(json.dumps(preflight,indent=2))
 except Exception as error:diagnostic_issue('optional_write',error)
 previous=E0.copy()
 for _arc_index,waypoint in enumerate(arc):
  phase='OPEN_1' if regrasp_count==0 else 'OPEN_2';command_state=waypoint['state'];planned_reference=np.asarray(waypoint['T_tcp'])
  distance=np.linalg.norm(planned_reference[:3,3]-previous[:3,3]);duration=max(.25,1.5*distance/.003)
  if force_guard.pending:
   phase='FORCE_HOLD';hold(.25)
   if not force_guard.settled():raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
   force_guard.pending=False;duration*=2
  command=np.asarray(waypoint['q'])+command_bias
  if model.margin(command)<=.05:raise RuntimeError('COMMAND_MARGIN_BELOW_EXISTING_LIMIT')
  move(command,duration);previous=planned_reference.copy()
 # Small robot-reference continuation compensates contact compliance at the
 # unchanged 90-degree object limit. No object drive/state is commanded.
 if recovery_reason is None:
  for _ in range(4):
   if rows[-1]['door_angle_deg']>=goal_deg:break
   phase='OPEN_2_ENDPOINT';command_state+=np.deg2rad(.25)
   body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:command_state})
   planned_reference=body@np.linalg.inv(origin)@E0;q=ik(planned_reference,np.asarray(robot.get_joint_positions())[arm])
   if q is None:raise RuntimeError('NO_IK_TRANSITION_TO_REGRASP')
   command=q+command_bias
   if model.margin(command)<=.05:raise RuntimeError('WORKSPACE_TRANSITION_TO_REGRASP')
   distance=np.linalg.norm(planned_reference[:3,3]-tcp()[:3,3]);move(command,max(.25,1.5*distance/.003))
   qvelocity=np.zeros(6);hold(.3)
 # Smooth spline ends at zero arm velocity. Gripper effort/gains are retained.
 profile_mark('final_hold')
 phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(2.);ready,detail=grip_window()
 success=bool(max_state>=goal_deg);status='SUCCESS_PHYSICAL_AUTHORED_RANGE' if success else (recovery_reason or 'CONTINUE_FROM_CURRENT_STATE_TO_AUTHORED_TARGET')

except Exception as error:
 continuation_error=repr(error)
 raise
finally:
 world.pause();world.render()
 diagnostic_write(a.output/'transfer_continuation_status.json',json.dumps({'mode':'KNOWN_MODEL_DIAGNOSTIC','actual_angle_deg':rows[-1]['door_angle_deg'],'maximum_angle_deg':max_state,'base_moves':base_moves,'regrasp_count':regrasp_count,'target_deg':goal_deg,'success':success,'error':continuation_error,'scene_preserved':True,'phase':phase},indent=2))
 diagnostic_write(a.output/'transfer_continuation_events.json',json.dumps(telemetry_events,indent=2))
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
