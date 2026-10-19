"""Effective contact effort, with explicit proxy/observability provenance."""
import json
import csv
from pathlib import Path
import numpy as np


def effective_effort(contacts,dt,direction,axis=None,axis_point=None):
 d=np.asarray(direction,float);d/=np.linalg.norm(d);F=np.zeros(3);M=np.zeros(3);has_points=True
 for c in contacts:
  if not c.get('allowed_pad_target'):continue
  f=-np.asarray(c['impulse_world_ns'],float)/dt # stored impulse is on owned finger
  F+=f
  if axis_point is not None and 'contact_point_world_m' in c:M+=np.cross(np.asarray(c['contact_point_world_m'])-axis_point,f)
  elif axis_point is not None:has_points=False
 return {'force_on_handle_world_n':F.tolist(),'effective_tangential_force_n':float(F@d),'effective_tangential_force_magnitude_n':float(abs(F@d)),'estimated_axis_torque_nm':None if axis is None or axis_point is None or not has_points else float(np.asarray(axis)@M),'force_source':'native normal contact impulse projection only; excludes friction buffer; not full tangential effort','torque_source':'normal-component diagnostic about estimated axis; not full hinge torque'}


class EffortRecorder:
 def __init__(self,output):
  self.output=Path(output);self.stream=(self.output/'effort_samples.jsonl').open('w');self.segment=None;self.records=[]
 def begin(self,index,state,t):self.segment={'index':index,'start_estimated_state':state,'start_s':t};self.records.append(self.segment)
 def append(self,s,dt,direction,estimate):
  if self.segment is None:return
  r=estimate.get('revolute',{}) if estimate and estimate.get('joint_type')=='revolute' else {};e=effective_effort(s['contacts'],dt,direction,np.asarray(r['axis']) if r else None,np.asarray(r['point_on_axis']) if r else None)
  sensor=getattr(self,'sensor',None);full=sensor.sample(dt,np.asarray(r['axis']) if r else None,np.asarray(r['point_on_axis']) if r else None) if sensor else None
  if full and full.get('bilateral_buffer_verified',False):
   e.update(force_on_handle_world_n=full['force_on_handle_world_n'],effective_tangential_force_n=float(np.asarray(full['force_on_handle_world_n'])@direction),effective_tangential_force_magnitude_n=float(abs(np.asarray(full['force_on_handle_world_n'])@direction)),estimated_axis_torque_nm=full['estimated_axis_torque_nm'],force_source=full['source'],torque_source='normal+friction moment about EE-estimated axis; not exact GT hinge reaction')
  self.stream.write(json.dumps({'t':s['t'],'segment':self.segment['index'],'phase':s['phase'],'T_ee':s['T_tcp'],'direction_world':np.asarray(direction).tolist(),'command_wrench_world':s.get('constrained_drive',{}).get('command_wrench_world'),'full_tangential_measurement':bool(full and full.get('bilateral_buffer_verified',False)),'full_contact_sensor':full,**e})+'\n')
 def end(self,state,t,reason):
  if self.segment:self.segment.update(end_estimated_state=state,end_s=t,stop_reason=reason)
  self.segment=None;self.stream.flush()
 def close(self):
  self.stream.close();(self.output/'effort_segments.json').write_text(json.dumps(self.records,indent=2))


def finite_max(values):
 values=np.asarray(values);values=values[np.isfinite(values)]
 return None if not len(values) else float(values.max())


def summarize_effort(output,onset_m=.00025):
 """EE motion onset is a proxy for object onset, not a friction measurement.

 Frozen load/speed guards are untouched. The onset threshold here describes
 a measurement definition only. All samples and windows remain available.
 """
 root=Path(output);segments=json.loads((root/'effort_segments.json').read_text())
 samples=[json.loads(line) for line in (root/'effort_samples.jsonl').read_text().splitlines()]
 object_log=root/'evaluation_private/object_trajectory.json'
 truth=json.loads(object_log.read_text()) if object_log.exists() else []
 truth_t=np.array([x['t'] for x in truth]);truth_T=np.array([x['T_object'] for x in truth]) if truth else None
 records=[]
 for segment in segments:
  S=[s for s in samples if s['segment']==segment['index']]
  if len(S)<3:continue
  t=np.array([s['t'] for s in S]);P=np.array([s['T_ee'] for s in S])[:,:3,3]
  travel=np.linalg.norm(P-P[0],axis=1);velocity=np.linalg.norm(np.gradient(P,t,axis=0),axis=1)
  complete=all(s.get('full_tangential_measurement',False) and (s.get('full_contact_sensor') or {}).get('bilateral_buffer_verified',False) for s in S)
  if complete:F=np.array([s['effective_tangential_force_magnitude_n'] for s in S]);source='SIMULATOR_NORMAL_PLUS_FRICTION'
  else:
   values=[]
   for s in S:
    wrench=s.get('command_wrench_world');direction=s.get('direction_world')
    values.append(float('nan') if wrench is None else float(np.linalg.norm(wrench[:3])) if direction is None else float(abs(np.asarray(wrench[:3])@np.asarray(direction))))
   F=np.array(values);source='COMMAND_EQUIVALENT_PROXY' if all(s.get('direction_world') is not None for s in S) else 'COMMAND_RESULTANT_PROXY_LEGACY_DIRECTION_NOT_LOGGED'
  on=np.flatnonzero(travel>=onset_m)
  # Only rested starts may claim breakaway. Continuous 1mm segments do not.
  rested=segment.get('from_rest',segment['index']==0)
  onset=int(on[0]) if len(on) else None
  moving=(travel>=onset_m)&(velocity>=.00005)&(velocity<=.00075)&np.isfinite(F)
  values=F[moving];torques=[abs(s['estimated_axis_torque_nm']) for s,k in zip(S,moving) if k and s['estimated_axis_torque_nm'] is not None]
  record={**segment,'from_rest':rested,'effort_measurement_kind':source,'motion_onset_definition':'measured EE displacement >= 0.25 mm; object onset/slip not independently observed online',
          'motion_onset_delay_s':None if onset is None else float(t[onset]-segment['start_s']),
          'breakaway_effective_force_n':None if not rested or onset is None else finite_max(F[:onset+1]),
          'breakaway_status':'EE_ONSET_PROXY' if rested and onset is not None else 'NOT_FROM_REST' if not rested else 'NO_OBSERVED_ONSET',
          'moving_effective_force_n':None if not len(values) else float(np.mean(values)),
          'moving_force_std_n':None if not len(values) else float(np.std(values)),
          'moving_effort_estimated_axis_nm':None if not complete or not torques else float(np.mean(torques)),
          'moving_sample_count':int(moving.sum()),'actual_EE_displacement_m':float(travel[-1])}
  records.append(record)
  if truth and rested:
   nearest=np.array([int(np.argmin(abs(truth_t-x))) for x in t]);objects=truth_T[nearest]
   # Track the initially observed EE/grasp point fixed in the moving part.
   # Evaluation only; no GT hinge type/axis/angle is required or fed online.
   anchor=np.linalg.inv(objects[0])@np.r_[P[0],1.]
   actual=np.einsum('nij,j->ni',objects,anchor)[:,:3]
   motion=np.linalg.norm(actual-actual[0],axis=1);events=np.flatnonzero(motion>=onset_m)
   i=int(events[0]) if len(events) else None
   record['postrun_object_onset_evaluation']={'evaluation_only':True,'motion_threshold_m':onset_m,
     'force_at_object_onset_n':None if i is None else (float(F[i]) if np.isfinite(F[i]) else None),
     'force_ramp_max_before_object_onset_n':None if i is None else finite_max(F[:i+1]),
     'onset_delay_s':None if i is None else float(t[i]-segment['start_s']),
     'source':'independent object SE(3) log, tracked initial grasp point; never supplied to controller'}
 first=next((s for s in records if s['from_rest'] and s.get('start_estimated_state',0)==0),None)
 doc={'schema':'effective-contact-effort-v1','opening_breakaway_effort':first,
      'moving_effort_vs_state':records,'force_provenance':'normal+friction tensor measurement when available; otherwise commanded Cartesian proxy, explicitly NOT measured hinge effort; normal report alone retained as diagnostic',
      'torque_provenance':'proxy moment about estimated axis, not exact GT hinge reaction',
      'interpretation':'effective response under this frozen robot/controller/grasp; includes robot/grasp transients; not identified friction parameters',
      'exact_minimum_breakaway_force_proven':False,'raw_samples':'effort_samples.jsonl'}
 (root/'effort_summary.json').write_text(json.dumps(doc,indent=2))
 fields=['index','start_estimated_state','end_estimated_state','from_rest','effort_measurement_kind','breakaway_effective_force_n','moving_effective_force_n','moving_force_std_n','moving_effort_estimated_axis_nm','moving_sample_count','actual_EE_displacement_m','stop_reason']
 with (root/'effort_vs_state.csv').open('w',newline='') as stream:
  writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(records)
 return doc
