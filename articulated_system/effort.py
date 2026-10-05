"""Windowed task resistance records, never friction parameters or pad-force sum."""
import json
from pathlib import Path
import numpy as np


def profile(capture_root, object_motion_evaluation=False):
 root=Path(capture_root)
 if not (root/'effort_samples.jsonl').exists() or not (root/'effort_segments.json').exists():return {'status':'MEASUREMENT_UNAVAILABLE'}
 object_rows=None
 if object_motion_evaluation:
  if not (root/'report.json').exists():raise RuntimeError('EFFORT_OBJECT_EVALUATION_REQUIRES_FINISHED_EPISODE')
  scope=json.loads((root/'evaluation_private/scope.json').read_text())
  if not scope.get('evaluation_only') or scope.get('controller_readback'):raise RuntimeError('EFFORT_EVALUATION_SCOPE_INVALID')
  object_rows=json.loads((root/'evaluation_private/object_trajectory.json').read_text())
  object_times=np.asarray([r['t'] for r in object_rows]);object_poses=np.asarray([r['T_object'] for r in object_rows])
 samples=[json.loads(x) for x in (root/'effort_samples.jsonl').read_text().splitlines()];segments=json.loads((root/'effort_segments.json').read_text());records=[]
 for segment in segments:
  S=[s for s in samples if s['segment']==segment['index']]
  if len(S)<3:continue
  t=np.array([s['t'] for s in S]);P=np.array([s['T_ee'] for s in S])[:,:3,3];travel=np.linalg.norm(P-P[0],axis=1);speed=np.linalg.norm(np.gradient(P,t,axis=0),axis=1)
  if object_rows is not None:
   indices=np.clip(np.searchsorted(object_times,t),0,len(object_times)-1);W=object_poses[indices]
   # An observed grasp-location marker, rigidly expressed in the moving link.
   # This offline evaluator separates actual part motion from EE compliance.
   local=(np.linalg.inv(W[0])@np.r_[P[0],1])[:3]
   P=np.einsum('nij,j->ni',W[:,:3,:3],local)+W[:,:3,3]
   travel=np.linalg.norm(P-P[0],axis=1);speed=np.linalg.norm(np.gradient(P,t,axis=0),axis=1)
  verified=np.array([s.get('full_tangential_measurement',False) for s in S]);F=np.array([s['effective_tangential_force_n'] if v else np.nan for s,v in zip(S,verified)])
  onset=np.flatnonzero(travel>=.00025);i=int(onset[0]) if len(onset) else None;rested=segment.get('from_rest',segment['index']==0)
  start_window=np.zeros(len(t),bool) if i is None else abs(t-t[i])<=.2
  moving=(t-t[0]>=.5)&(speed>=.00005)&(speed<=.00075)&(travel>=.00025)&verified
  def stats(values):
   values=values[np.isfinite(values)]
   return None if not len(values) else {'mean':float(values.mean()),'std':float(values.std()),'min':float(values.min()),'max':float(values.max()),'q10':float(np.quantile(values,.1)),'q90':float(np.quantile(values,.9)),'samples':len(values)}
  onset_force=stats(F[start_window]) if rested else None;steady=stats(F[moving]);proxy=[]
  for s in S:
   w=s.get('command_wrench_world');direction=s.get('direction_world');proxy.append(None if w is None or direction is None else float(np.dot(w[:3],direction)))
  records.append({**segment,'signal_source':'verified native PhysX normal+friction force on handle, measured samples only','measurement_verified_fraction':float(verified.mean()),'reference_frame':'world','force_units':'N','force_reference':'net force applied to handle from finger contact patches; NOT sum of finger normal loads','axis_torque_reference':'moment about EE-estimated axis line, when existing raw observer supplies it; not GT hinge torque','rested_start':rested,'EE_onset_time_s':None if i is None else float(t[i]),'onset_definition':'measured EE >=0.25mm, not independently observable object startup online','opening_task_resistance_at_onset_n':onset_force,'exact_minimum_breakaway_effort':False,'quasistatic_moving_tangential_force_n':steady,'quasistatic_actual_EE_speed_m_s':stats(speed[moving]),'moving_window_rule':'>=0.5s after segment start, speed 0.05–0.75mm/s, verified force, EE displacement>=0.25mm','command_proxy_source':'command Cartesian force projected along logged pull direction, NOT measured effort','command_proxy_range_n':stats(np.array([np.nan if x is None else x for x in proxy])),'direction_world':S[0].get('direction_world'),'T_world_grasp_start':S[0]['T_ee'],'raw_time_series':'effort_samples.jsonl','raw_segment':segment['index']})
 doc={'schema':'task-opening-resistance-windows-v1','friction_identification_performed':False,'opening_breakaway_effort':[r for r in records if r['rested_start']],'moving_effort_vs_state':records,'units':{'force':'N','time':'s','EE_motion':'m'},'limitation':'task opening resistance includes latch/seal/gravity and robot/grasp transients; no exact minimum friction claim; EE onset can include grasp compliance','measurement_completed':any(r['quasistatic_moving_tangential_force_n'] for r in records),'GT_input_to_controller':False}
 doc['motion_onset_source']='independent simulator moving-link pose, evaluated after episode; marker initialized from observed grasp TCP' if object_motion_evaluation else 'measured EE motion; compliance can precede actual part motion'
 doc['simulation_only_object_pose_used_offline']=bool(object_motion_evaluation)
 if object_motion_evaluation:
  for record in records:
   record['object_marker_onset_time_s']=record.pop('EE_onset_time_s')
   record['onset_definition']='moving-link rigid marker >=0.25mm; offline simulation evaluation, not exact minimum breakaway friction'
   record['object_marker_actual_speed_m_s']=record.pop('quasistatic_actual_EE_speed_m_s')
   record['marker_reference']='initial observed TCP location expressed in moving-link frame; not exact contact centroid'
   record['moving_window_rule']='>=0.5s after segment start, object-marker speed 0.05–0.75mm/s, verified force, object-marker displacement>=0.25mm'
  doc['limitation']='offline simulation task-opening resistance includes latch/seal/gravity and contact transients; no exact minimum friction claim; marker follows measured moving-link pose'
 filename='effort_profile_object_evaluation.json' if object_motion_evaluation else 'effort_profile.json'
 (root/filename).write_text(json.dumps(doc,indent=2));return doc
