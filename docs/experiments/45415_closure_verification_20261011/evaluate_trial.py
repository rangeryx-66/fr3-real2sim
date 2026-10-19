"""Read-only post-experiment evidence extraction; never imported by the actor."""
from pathlib import Path
import json,pickle,gzip,numpy as np,collections,hashlib
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45415_closure_verification_20261011';trial=out/'physical_01'
report=json.loads((trial/'report.json').read_text());events=json.loads((trial/'events.json').read_text());grasp=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
rows=[]
for f in sorted((trial/'observations_chunks').glob('*.gz')):rows.extend(pickle.load(gzip.open(f,'rb')))
commands=[]
for f in sorted((trial/'closure_commands_chunks').glob('*.gz')):commands.extend(pickle.load(gzip.open(f,'rb')))
previous=None;speed_peak={'all':0.,'closure_before_contact':0.,'closure_after_contact':0.,'after_valid_grasp':None};speed_context={};first_contact=None;min_margin=None;peak=np.zeros(2);loaded_peak=np.zeros(2);counts=collections.Counter();targets=set();loss_intervals=[];gap=None;relative=0.;rotation=0.;first_protection=None;command_reopenings=[];previous_command=None
for c in commands:
 w=c['aperture_command_m']
 if c['mode']=='position' and w is not None:
  if previous_command is not None and w>previous_command+1e-7:command_reopenings.append(c)
  previous_command=w
for r in rows:
 peak=np.maximum(peak,list(r['forces_n'].values()));counts.update(k for k,v in r.get('telemetry',{}).items() if v is not None)
 if first_contact is None and max(r['forces_n'].values())>0:first_contact=r['t']
 for c in r['contacts']:
  if c.get('allowed_pad_target') and c.get('force_n',0)>0:targets.add(c['target'])
 loaded=bool(grasp and r['t']>=grasp['t'])
 if loaded:
  min_margin=r['margin_rad'] if min_margin is None else min(min_margin,r['margin_rad']);loaded_peak=np.maximum(loaded_peak,list(r['forces_n'].values()));relative=max(relative,r['relative_translation_slip_m']);rotation=max(rotation,r['relative_rotation_slip_deg'])
  positive=any(c.get('allowed_pad_target') and c.get('force_n',0)>0 for c in r['contacts'])
  if not positive and gap is None:gap=r['t']
  if positive and gap is not None:loss_intervals.append({'start_s':gap,'end_s':r['t'],'duration_s':r['t']-gap});gap=None
 if previous is not None:
  v=float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(previous['T_tcp'])[:3,3])/(r['t']-previous['t']));keys=['all']
  if r['phase'] in ['SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD']:keys.append('closure_after_contact' if first_contact is not None else 'closure_before_contact')
  if loaded:keys.append('after_valid_grasp')
  for k in keys:
   if speed_peak[k] is None or v>speed_peak[k]:speed_peak[k]=v;speed_context[k]={'t':r['t'],'phase':r['phase'],'loads_n':r['forces_n'],'command_increment_rad':float(np.max(abs(np.array(r['command_q_arm'])-previous['command_q_arm'])))}
  if loaded and v>.005 and first_protection is None:first_protection={'type':'EXISTING_LOADED_TCP_SPEED','t':r['t'],'velocity_m_s':v,'phase':r['phase'],'angle_deg':r['door_angle_deg']}
 previous=r
if gap is not None:loss_intervals.append({'start_s':gap,'end_s':rows[-1]['t']+1/240,'duration_s':rows[-1]['t']+1/240-gap})
first_force_hold=next(({'t':r['t'],'angle_deg':r['door_angle_deg'],'loads_n':r['forces_n'],'filtered_loads_n':r['telemetry']['pad_load_filtered_n']} for r in rows if r['phase']=='FORCE_HOLD'),None)
last=rows[-1];native=json.loads((trial/'physics_steps_chunks/index.json').read_text());streams=json.loads((trial/'EXPORT_COMPLETE.json').read_text());provenance=json.loads((trial/'active_source_provenance.json').read_text());preflight=json.loads((out/'PREFLIGHT.json').read_text())
result={'classification':'ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT / PHYSICAL_EVALUATION','physical_attempts':1,'correct_handle':'original-40','handle_contact_targets':sorted(targets),'bilateral_grasp_verified':bool(grasp),'grasp_event':grasp,'first_FORCE_HOLD':first_force_hold,'first_positive_handle_contact_s':first_contact,'physical_PASS_30deg':bool(report['success'] and grasp and report['max_actual_angle_after_valid_grasp_deg']>=30.0),'max_actual_angle_after_valid_grasp_deg':report['max_actual_angle_after_valid_grasp_deg'],'uncredited_pregrasp_max_angle_deg':max((r['door_angle_deg'] for r in rows if not grasp or r['t']<grasp['t']),default=None),'termination_status':report['status'],'first_existing_loaded_speed_violation':first_protection,'terminal_state':{k:last[k] for k in ['t','phase','door_angle_deg','aperture_m','margin_rad','forces_n','q','command_q_arm']},'peak_pad_loads_n_all':peak.tolist(),'peak_pad_loads_n_after_grasp':loaded_peak.tolist() if grasp else None,'max_TCP_velocity_m_s_by_phase':speed_peak,'speed_peak_context':speed_context,'minimum_joint_margin_rad_all':min(r['margin_rad'] for r in rows),'minimum_joint_margin_rad_after_grasp':min_margin,'max_handle_relative_translation_m_after_grasp':relative if grasp else None,'max_handle_relative_rotation_deg_after_grasp':rotation if grasp else None,'contact_loss_intervals_after_grasp':loss_intervals if grasp else None,'closure_command_records':len(commands),'unilateral_fix_branch_calls':sum(c['unilateral_centering_input'] for c in commands),'physical_position_command_reopenings':len(command_reopenings),'unilateral_reopenings':sum(c['unilateral_centering_input'] for c in command_reopenings),'runtime_repair_observer':json.loads((trial/'closure_fix_execution.json').read_text()),'opening_controller_identical_to_preflight':provenance['opening_controller_sha256']==preflight['unchanged_opening_controller_excerpt_sha256'],'observations':len(rows),'native_records':native['records'],'telemetry_available_counts':dict(counts),'export_streams_match_buffers':all(v['index'] is not None and v['buffer_records']==v['index']['records'] for v in streams['streams'].values()),'simulation_s':report['simulation_s'],'wall_s':report['wall_s'],'metrics_do_not_govern_execution':True}
(trial/'PHYSICAL_METRICS.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ['telemetry_available_counts','terminal_state']},indent=2))
