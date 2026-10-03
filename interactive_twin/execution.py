"""Fixed physics-probe protocol and PiPER-observable log projection.

The protocol consumes an EE-estimated model only. Privileged plant construction,
contact safety diagnostics and GT evaluation never enter these output logs.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from interactive_twin.sysid import command_hash, validate_log

DEFAULT_PROTOCOL=[
 {'probe_id':'P1','split':'train','duration_s':8.,'speed_m_s':.00025,'pattern':'opening'},
 {'probe_id':'P2','split':'train','duration_s':8.,'speed_m_s':.0005,'pattern':'opening'},
 {'probe_id':'P3','split':'train','duration_s':10.,'speed_m_s':.0005,'pattern':'stop_restart'},
 {'probe_id':'P4','split':'heldout','duration_s':8.,'speed_m_s':.000375,'pattern':'short_pulse'},
]

class PhysicsProtocol:
 def __init__(self,memory,drive,segments=None):
  self.memory=memory;self.drive=drive;self.segments=segments or DEFAULT_PROTOCOL
  self.original_speed=drive.speed_m_s;self.segment_start=None
  for s in self.segments:
   if not (0<s['speed_m_s']<=.0005 and 0<s['duration_s']<=12):raise ValueError('FROZEN_PHYSICS_PROBE_BUDGET')
  if [s['probe_id'] for s in self.segments]!=['P1','P2','P3','P4']:raise ValueError('FROZEN_TRAIN_HELDOUT_SPLIT')
 def start(self,s,T):
  self.drive.speed_m_s=s['speed_m_s'];self.drive.set_direction(self.memory.tangent(T),T);self.drive.active=True;self.segment_start=T[:3,3].copy()
 def update(self,s,t,T):
  active=True
  if s['pattern']=='stop_restart':active=t<3 or t>=6
  if s['pattern']=='short_pulse':active=1<=t<5
  self.drive.active=active
  if np.linalg.norm(T[:3,3]-self.segment_start)>=.001:
   self.drive.refresh_tangent(self.memory.tangent(T),T);self.segment_start=T[:3,3].copy()
 def finish(self):self.drive.speed_m_s=self.original_speed


def save_observable_logs(output,rows,commands,arm,job,report):
 output=Path(output);folder=output/'observable';folder.mkdir(exist_ok=True)
 if not rows:return
 robot_id=job.get('robot_model_id','f5dcc6f-frozen-piper')
 calibration_id=job.get('robot_calibration_id','PENDING_ROBOT_ONLY_CALIBRATION')
 controller_id=job.get('controller_id','cd61660-constrained-probe+physics-protocol-v1')
 q=np.array([r['q'] for r in rows])[:,arm];dt=1/240
 qdot=np.vstack([np.zeros((1,q.shape[1])),np.diff(q,axis=0)/dt])
 q=np.round(q/np.deg2rad(.001))*np.deg2rad(.001)
 qdot=np.round(qdot/.001)*.001
 # FK/EE is a robot observation. It is never a moving-object pose measurement.
 selected={p:[i for i,r in enumerate(rows) if r['phase']==p] for p in ['P1','P2','P3','P4','ROBOT_CALIBRATION','EXPLORATORY']}
 durations={s['probe_id']:s['duration_s'] for s in job.get('physics_protocol',DEFAULT_PROTOCOL)}
 durations['ROBOT_CALIBRATION']=10.
 # Replay prefixes still have to cover the full frozen segment duration.
 # Their overall final status says nothing about an earlier P1/P2/P3 segment.
 terminal=str(report.get('status','UNKNOWN'))
 clean_terminal=terminal in ('SUCCESS','PHYSICS_PROTOCOL_COMPLETE','REPLAY_COMPLETE',
                            'PHYSICS_REPLAY_COMPLETE','ROBOT_CALIBRATION_COMPLETE')
 failure_phase=report.get('failure_phase')
 last_sample_phase=rows[-1]['phase']
 results={}
 for p,indices in selected.items():
  expected_samples=int(durations[p]/dt) if p in durations else None
  # A sample is taken AFTER its physics step but has the applied command's
  # timestamp. N contiguous samples therefore cover N*dt, not (N-1)*dt.
  pairs_ok=all(i<len(commands) and commands[i].get('phase',p)==p for i in indices)
  command_times=[float(commands[i].get('t',rows[i]['t'])) for i in indices if i<len(commands)]
  observed_times=[float(rows[i]['t']) for i in indices]
  contiguous=bool(indices) and all(b==a+1 for a,b in zip(indices,indices[1:]))
  uniform=bool(indices) and (len(indices)==1 or np.allclose(np.diff(observed_times),dt,rtol=0.,atol=1e-7))
  aligned=pairs_ok and len(command_times)==len(observed_times) and np.allclose(command_times,observed_times,rtol=0.,atol=1e-7)
  command_horizon=(command_times[-1]-command_times[0]+dt) if command_times else 0.
  observed_horizon=(observed_times[-1]-observed_times[0]+dt) if observed_times else 0.
  if expected_samples is not None:
   complete=bool(len(indices)==expected_samples and contiguous and uniform and aligned and
                 abs(command_horizon-expected_samples*dt)<1e-6 and abs(observed_horizon-expected_samples*dt)<1e-6)
   completion_rule='full frozen segment: paired commands and physical responses at 240 Hz'
  else:
   # Exploratory motion has a variable, sensor-determined endpoint. Its fit
   # result, not a later held-out/manipulation failure, closes that observation.
   complete=bool(len(indices)>=5 and aligned and report.get('accepted_estimate_count',0)>0)
   completion_rule='adaptive exploratory segment ends with accepted EE-only estimate'
  # Only an explicitly attributed failure can invalidate a complete segment.
  # first_failure_state is often the *previous* step when a pre-step guard trips,
  # so it is retained as diagnostic rather than used to taint an earlier probe.
  attributed_failure=bool(not clean_terminal and failure_phase==p)
  if attributed_failure:complete=False
  results[p]={'path':None,'samples':len(indices),'complete':complete,
              'status':'COMPLETE' if complete else ('NOT_RUN' if not indices else 'INCOMPLETE'),
              'expected_samples':expected_samples,'expected_duration_s':expected_samples*dt if expected_samples is not None else None,
              'observed_response_horizon_s':observed_horizon,'applied_command_horizon_s':command_horizon,
              'paired_command_response':bool(aligned),'contiguous_physical_steps':bool(contiguous and uniform),
              'completion_rule':completion_rule,'episode_terminal_status':terminal,
              'explicit_failure_in_segment':attributed_failure,
              'terminal_last_sample_phase':last_sample_phase,
              'terminal_step_status_uncertain':bool(not clean_terminal and failure_phase is None and last_sample_phase==p)}
  if len(indices)<5:continue
  if not pairs_ok:continue
  start=rows[indices[0]]['t'];times=[round(rows[i]['t']-start,10) for i in indices]
  cartesian=[commands[i].get('cartesian_input') for i in indices]
  if any(value is not None for value in cartesian):
   canonical={'reference_world_m','direction_world','active'}
   legacy={'reference','direction','active'}
   if not all(isinstance(value,dict) and (canonical<=set(value) or legacy<=set(value)) for value in cartesian):
    results[p].update(complete=False,status='INCOMPLETE',projection_error='MIXED_OR_MISSING_CARTESIAN_INPUT');continue
   # The episode runner's actual command tape uses explicit world-frame keys.
   # Accept the early logger fixture aliases only at this projection boundary.
   cartesian=[value if canonical<=set(value) else {'reference_world_m':value['reference'],
               'direction_world':value['direction'],'active':value['active']} for value in cartesian]
   # These are the exogenous inputs to the unchanged feedback controller.
   # Plant-specific arm effort/gravity compensation remains in raw diagnostics,
   # rather than freezing a previous plant's feedback torque into the u hash.
   fields=['ee_reference_x_m','ee_reference_y_m','ee_reference_z_m',
           'direction_x','direction_y','direction_z','drive_active',
           'gripper_effort','finger_q1_command','finger_q2_command']
   values=[list(value['reference_world_m'])+list(value['direction_world'])+[float(bool(value['active'])),commands[i]['finger_effort']]+commands[i]['finger_position']
           for i,value in zip(indices,cartesian)]
   cmd={'time_s':times,'fields':fields,'kind':'cartesian_constrained_drive','values':values}
  else:
   fields=[f'arm_effort_j{k}' for k in range(1,7)]+['gripper_effort','finger_q1_command','finger_q2_command']
   cmd={'time_s':times,'fields':fields,'kind':'arm_effort','values':[commands[i]['arm_effort']+[commands[i]['finger_effort']]+commands[i]['finger_position'] for i in indices]}
  provenance={'source':'isaac_physics','robot_model_id':robot_id,'robot_calibration_id':calibration_id,'controller_id':controller_id,
   'simulator':'isaac_physx','complete':complete,
   'initialization_only':True,'robot_state_replayed':False,'object_state_replayed':False,'direct_object_actuation':False,'attachment':False,'command_applied_sha256':command_hash(cmd)}
  if p=='ROBOT_CALIBRATION':provenance['no_contact_supervisor_certified']=not any(c['force_n']>0 for i in indices for c in rows[i]['contacts'])
  # REAL_LOG_TO_SIM identifies the comparison contract. Prediction provenance
  # remains isaac_physics; it must never be relabelled as hardware observation.
  result={'schema_version':1,'mode':job.get('observation_mode','SIM_TO_SIM_BLIND_SYSID'),'episode_id':job.get('episode_id','DEV_7320'),
   'probe_id':p,'split':'heldout' if p=='P4' else ('calibration' if p=='ROBOT_CALIBRATION' else ('sensitivity' if p=='EXPLORATORY' else 'train')),
   'time_s':times,'commands':cmd,'signals':{'q_rad':q[indices].tolist(),'qdot_rad_s':qdot[indices].tolist(),
    'ee_T_world_tcp':[rows[i]['T_tcp'] for i in indices],'gripper_opening_m':[rows[i]['aperture_m'] for i in indices]},'provenance':provenance}
  validate_log(result)
  file=folder/(p+'.json');file.write_text(json.dumps(result));results[p].update(path=str(file),command_hash=command_hash(cmd))
 (folder/'index.json').write_text(json.dumps(results,indent=2))
