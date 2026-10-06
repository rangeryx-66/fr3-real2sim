"""Expand the existing entry with command-prefix continuation, not state replay."""
import ast

def augment(text):
    def replace(a,b):
        nonlocal text
        if text.count(a)!=1:raise RuntimeError('CONTINUATION_HOOK_CHANGED:'+a[:80])
        text=text.replace(a,b)
    replace(' def step():', ''' replay_base_origin=list(base);replay_route_cache=None
 def replay_base_command(frame):
  nonlocal B,collision,replay_route_cache,moving_initial
  from isaacsim.core.prims import SingleXFormPrim
  from interactive_twin_recovery.mobile import at_base,MobileRuntimeScene
  requested=frame.get('base_command')
  if requested is None:
   if replay_route_cache is not None:
    data=at_base(export,replay_base_origin,base)
    data['shapes']=[e for e in data['shapes'] if '/World/mobile_chassis' not in e['path']]
    collision=PhysicalScene(data,model,allowed);collision.moving_reference=moving_initial;replay_route_cache=None
   return
  if frame.get('retention_armed'):raise RuntimeError('REPLAY_BASE_MOVE_WITH_GRASP_FORBIDDEN')
  if max(filtered())>.02:raise RuntimeError('REPLAY_BASE_MOVE_REQUIRES_ACTUAL_RELEASE')
  if replay_route_cache is None:
   data=json.loads(json.dumps(export));data['shapes']=[e for e in data['shapes'] if '/World/mobile_chassis' not in e['path']]
   replay_route_cache=MobileRuntimeScene(ROOT,data,model,replay_base_origin)
   class CachedReplayCollision:
    def __getattr__(self,name):return getattr(replay_route_cache.scene,name)
    def check(self,P,moving_pose,allow_handle):
     replay_route_cache.moving_reference=moving_pose;return replay_route_cache.check(P,base)
    def contact_guard(self,contacts,P):return replay_route_cache.contact_guard(contacts,P)
   collision=CachedReplayCollision()
  requested=np.asarray(requested,float);quat=np.roll(Rotation.from_euler('z',requested[3],degrees=True).as_quat(),1)
  robot.set_world_pose(requested[:3],quat)
  SingleXFormPrim('/World/r1a7_pedestal').set_world_pose([requested[0],requested[1],-.33],quat)
  SingleXFormPrim('/World/mobile_chassis').set_world_pose([requested[0],requested[1],-.66],quat)
  base[:]=requested.tolist();B=transform(base[:3],[0,0,np.deg2rad(base[3])])
 def step():''')
    replace("   replay_frame=tape[tick];phase=", "   replay_frame=tape[tick];replay_base_command(replay_frame)\n   if not replay_frame.get('retention_armed') and reference is not None:\n    moving_initial=moving();reference=None;grasp_tcp=None;retention.reference=None;loss_s=0.;slip_s=0.\n   phase=")
    replace(";replay_last_compliant=gain_state", ";replay_last_compliant=gain_state\n    if not compliant:controller.apply_action(ArticulationAction(joint_efforts=np.zeros(len(arm)),joint_indices=arm))\n    if mode=='position':controller.apply_action(ArticulationAction(joint_efforts=np.zeros(len(fingers)),joint_indices=fingers))")
    replace("   if replay_frame.get('retention_armed') and reference is None:", "   if replay_frame.get('cartesian_input') is not None and memory is None:\n    from wrist_reconstruction.operation_memory import load_observed_model\n    estimate,sign,_=load_observed_model(job['continuation_memory'],job['skill']['joint_type'])\n    memory=InteractionMemory(tcp(),a.output);memory.estimate=estimate;memory.follow_sign=sign;memory.observe(tcp());fit_poses=memory.poses\n   if replay_frame.get('retention_armed') and reference is None:")
    # A cold checkpoint restoration repeats issued actuator commands. The
    # input-response sysID replay instead recomputes servo torque; that branch
    # is deliberately unsuitable here because its small feedback differences
    # can change a later physical regrasp. This is not measured-state replay.
    replace("  if compliant and (tape is None or replay_frame.get('cartesian_input') is not None):",
            "  if compliant and tape is None:")
    # Reuse the exact existing runtime API, including native closure, release,
    # route, arm safety and collision guards. No alternative controller.
    begin=text.index('   from wrist_reconstruction.session import run as run_skill')
    end=text.index("   skill_result=run_skill(runtime);",begin)
    api=text[begin:end]
    setup='''   tape=None;replay_last_compliant=None
   if memory is None:memory=InteractionMemory(tcp(),a.output)
   from wrist_reconstruction.operation_memory import load_observed_model
   estimate,sign,audit=load_observed_model(job['continuation_memory'],job['skill']['joint_type'])
   memory.estimate=estimate;memory.follow_sign=sign;memory.initial=tcp().copy();memory.observe(tcp());fit_poses=memory.poses
   drive=ConstrainedDrive(tcp(),memory.tangent(tcp()));drive.active=False
'''
    tail='''   checkpoint=json.loads(Path(job['continuation_checkpoint']).read_text());observation_origin=job['continuation_observed_origin']
   D=np.asarray(observation_origin['D_world_initial_to_checkpoint']);set_observed_moving(D)
   recovery.export=system_at_base()
   for shape in recovery.export['shapes']:
    if shape.get('rigid_body_path','')==runtime.capture.part_path:
     for key in ('world_transform','rigid_body_world_transform'):shape[key]=(D@np.asarray(shape[key])).tolist()
   recovery.visual=json.loads(json.dumps(observation_origin['visual_checkpoint']));runtime.initial_visual=json.loads(json.dumps(recovery.visual))
   runtime.capture.states=json.loads(json.dumps(job.get('inherited_capture_states',[])));runtime.capture.flush('CONTINUING_FROM_ISSUED_COMMANDS')
   recovery.initial_base=list(base);recovery.current_D=np.eye(4)
   recovery.initial_cloud=__import__('wrist_reconstruction.recovery',fromlist=['local_cloud']).local_cloud(runtime.capture,recovery.visual['anchor_world_m'])
   recovery.template=json.loads(Path(job['continuation_template']).read_text())
   recovery.observation_origin_state=float(checkpoint['current_state']);runtime.state_offset=float(checkpoint['current_state'])
   recovery.grasp_reference_ee=tcp().copy();recovery.grasp_reference_D=np.eye(4)
   saved_memory=memory
   if reference is None:recovery.released=True;release_opening=float(runtime.finger_q()[0]-runtime.finger_q()[1])
   try:
    runtime.phase('SYSTEM_CONTINUATION_SETTLE');runtime.hold(.25)
    if runtime.grip()[0] and force_guard.settled():runtime.resume_safe_compliance()
    else:recovery.run(runtime.state_offset,reason='SAFE_RELEASE_AFTER_LOAD_STOP')
   except RuntimeError:recovery.run(runtime.state_offset,reason='SAFE_RELEASE_AFTER_LOAD_STOP')
   skill_result=run_skill(runtime);fit=memory.final_fit();following=True;status=skill_result['status'];success=False
'''
    old="   ready,detail=grip_window();legal=bool(ready)\n   if not ready:raise RuntimeError('REPLAY_FINAL_HOLD_LOST')\n   status='REPLAY_COMPLETE';success=False"
    replace(old,setup+api+tail)
    ast.parse(text);return text
