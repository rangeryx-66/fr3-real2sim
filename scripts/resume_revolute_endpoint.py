"""Small same-grasp endpoint continuation after a preserved TCP speed stop.
Uses the existing IK, move, hold and step protections without changing thresholds.
No object-specific inputs, contact parameters, or object state commands.
"""
endpoint_error=None
try:
 profile_mark('endpoint_speed_recovery')
 phase='OPEN_2_ENDPOINT';qvelocity=np.zeros(6)
 telemetry_events.append({'event':'SPEED_STOP_LOCAL_RECOVERY','t':tick*dt,'angle_deg':rows[-1]['door_angle_deg'],'action':'retain jaw preload; zero arm velocity command; slower measured-progress endpoint reference'})
 hold(.5)
 E0=tcp();theta0=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
 origin=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta0})
 command_bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm]
 for _endpoint_attempt in range(30):
  if rows[-1]['door_angle_deg']>=goal_deg:break
  actual=float(scene['articulation'].get_joint_positions()[scene.get('selected_asset_dof',0)])
  command_state=min(np.deg2rad(goal_deg+.5),actual+np.deg2rad(.15))
  body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:command_state})
  planned_reference=body@np.linalg.inv(origin)@E0
  q=ik(planned_reference,np.asarray(robot.get_joint_positions())[arm])
  if q is None:raise RuntimeError('NO_IK_TRANSITION_TO_REGRASP')
  command=q+command_bias
  if model.margin(command)<=.05:raise RuntimeError('WORKSPACE_TRANSITION_TO_REGRASP')
  distance=np.linalg.norm(planned_reference[:3,3]-tcp()[:3,3])
  move(command,max(.5,1.5*distance/.001));qvelocity=np.zeros(6);hold(.1)
 profile_mark('final_hold');phase='FINAL_HOLD';qvelocity=np.zeros(6);hold(2.)
 success=bool(max_state>=goal_deg)
 status='SUCCESS_PHYSICAL_AUTHORED_RANGE' if success else 'CONTINUE_FROM_CURRENT_STATE_TO_AUTHORED_TARGET'
except Exception as error:
 endpoint_error=repr(error);raise
finally:
 world.pause();world.render()
 diagnostic_write(a.output/'endpoint_recovery_status.json',json.dumps({'actual_angle_deg':rows[-1]['door_angle_deg'],'max_angle_deg':max_state,'success':success,'error':endpoint_error,'scene_preserved':True,'base_moves':base_moves,'regrasp_count':regrasp_count},indent=2))
 diagnostic_write(a.output/'transfer_continuation_events.json',json.dumps(telemetry_events,indent=2))
 export_raw_records('observations',rows);export_raw_records('physics_steps',native.physics_steps)
