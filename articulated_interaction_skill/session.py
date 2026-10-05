"""Bounded segment -> hold -> capture loop using frozen contact/compliance."""
import numpy as np

def run(r):
 p=r.policy;kind=p['joint_type'];last_capture=0.;path=0.;last=r.tcp()[:3,3].copy();start=r.time();segment=0
 def state():return r.memory.angle_deg(r.tcp()) if kind=='revolute' else r.memory.state(r.tcp())
 def snapshot(label,value):
  r.drive.active=False;r.phase('FINAL_HOLD');r.hold(p['hold_s']);ready,detail=r.grip()
  if r.memory.estimate is not None:value=state()
  r.capture.capture(label,value,r.time(),r.base,r.tcp(),r.grasp,ready,detail,articulation=r.memory.estimate)
  if not ready:raise RuntimeError('CAPTURE_HOLD_GRASP_LOST')
 snapshot('closed_after_real_grasp',0.)
 # Same four observed-frame probes and existing discovery estimator.
 for attempt in range(4):
  direction=r.memory.begin_attempt(attempt,r.time());r.drive.set_direction(direction,r.tcp());r.drive.active=True;r.phase('EXPLORATORY');r.effort.begin(segment,0.,r.time());segment+=1;probe_start=r.time()
  reason='LOW_EXCITATION'
  while r.time()-probe_start<35.:
   r.step()
   if r.tick()%8==0:
    r.memory.observe(r.tcp())
    if r.time()-probe_start>=1. and len(r.memory.poses)%30==0:r.memory.try_fit()
    if r.memory.estimate is not None:reason='COARSE_MODEL_AVAILABLE';break
    if r.time()-probe_start>=12. and r.memory.travel(r.memory.start_index)<.002:break
    if r.memory.travel()>.020:reason='PROBE_PATH_BUDGET';break
  r.effort.end(state(),r.time(),reason);r.memory.finish_attempt(r.time(),reason)
  if r.memory.estimate is not None:break
  r.drive.active=False;r.phase('PROBE_HOLD');r.hold(1.)
 if r.memory.estimate is None:raise RuntimeError('ARTICULATION_UNOBSERVABLE')
 value=state()
 if value>=p['capture_interval']:snapshot('discovery_reached_state',value);last_capture=value
 while segment<=p['maximum_segments']:
  if r.time()-start>=p['maximum_sim_s'] or path>=p['maximum_path_m']:raise RuntimeError('MULTISTATE_MOTION_BUDGET')
  value=state()
  if value>=p['targets'][-1]:break
  direction=r.memory.tangent(r.tcp());safe,why=r.increment_safe(direction,p['segment_m'])
  if not safe:
   r.drive.active=False
   # Do not attempt recovery after contact or physics failure. The handoff
   # requires a separate safe release and fresh observed grasp before any route.
   r.capture.flush('REPOSITION_REQUIRES_SAFE_REGRASP:'+why)
   raise RuntimeError('REPOSITION_REQUIRES_SAFE_REGRASP:'+why)
  from_rest=not r.drive.active
  if from_rest:r.drive.set_direction(direction,r.tcp())
  else:r.drive.refresh_tangent(direction,r.tcp())
  r.drive.active=True;r.phase('ESTIMATED_FOLLOW')
  r.effort.begin(segment,value,r.time());r.effort.segment['from_rest']=from_rest
  segment+=1;s0=r.tcp()[:3,3].copy();t0=r.time();resolved=False
  while r.time()-t0<p['segment_timeout_s']:
   r.step()
   if r.tick()%8==0:
    E=r.tcp();r.memory.observe(E);path+=float(np.linalg.norm(E[:3,3]-last));last=E[:3,3].copy()
    if getattr(r.memory,'pending',False):
     # Reuse frozen soft-gate handler. A pending confidence event must not
     # leave the directional drive disabled forever in a new orchestration.
     r.drive.active=False
     pause=r.memory.policy['confidence']['pause_s'];pause_start=r.time()
     while r.time()-pause_start<pause:
      r.step()
      if r.tick()%8==0:r.memory.observe(r.tcp())
     r.memory.resolve(r.time());resolved=True;break
    if np.linalg.norm(E[:3,3]-s0)>=p['segment_m']:break
  moved=float(np.linalg.norm(r.tcp()[:3,3]-s0));value=state();r.effort.end(value,r.time(),'SEGMENT_COMPLETE' if moved>=.0001 else 'INSUFFICIENT_MOTION')
  if moved<.0001 and not resolved:raise RuntimeError('SEGMENT_NO_USEFUL_MOTION')
  # Directional reference updates only; no repeated precision refit project.
  if value-last_capture>=p['capture_interval'] or value>=p['targets'][-1]:snapshot('incremental_capture',value);last_capture=value
 if state()-last_capture>p['minimum_capture_separation']:snapshot('last_reached_state',state())
 r.drive.active=False
 status='MULTISTATE_TARGET_COMPLETE' if state()>=p['targets'][-1] else 'MULTISTATE_SEGMENT_BUDGET'
 r.capture.flush(status);return {'status':status,'state':state(),'path_m':path,'segments':segment}
