from pathlib import Path
import json,gzip,pickle,numpy as np,collections,sys
root=Path(sys.argv[1])
for output in root.glob('*/official_01'):
 if not (output/'report.json').exists() or (output/'BENCHMARK_A_METRICS.json').exists():continue
 report=json.loads((output/'report.json').read_text());events=json.loads((output/'events.json').read_text());grasp=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
 peak=np.zeros(2);records=0;counts=collections.Counter();max_valid=None;max_pull=None;raw_pre=None;speed_max=0.;joint_speed=0.;drift=0.;rot=0.;margin=None;previous=None;first_speed=None;first_joint_speed=None;gap=None;gaps=[];last=None;contact_count=0
 for f in sorted((output/'observations_chunks').glob('*.gz')):
  with gzip.open(f,'rb') as stream:rows=pickle.load(stream)
  for r in rows:
   records+=1;last=r;tele=r.get('telemetry',{});counts.update(k for k,v in tele.items() if v is not None);peak=np.maximum(peak,list(r['forces_n'].values()));contact_count+=len(r['contacts'])
   if not grasp or r['t']<grasp['t']:raw_pre=r['door_angle_deg'] if raw_pre is None else max(raw_pre,r['door_angle_deg'])
   else:
    max_valid=r['door_angle_deg'] if max_valid is None else max(max_valid,r['door_angle_deg'])
    if r['phase']=='FIXED_BASE_PINK_OPEN':max_pull=r['door_angle_deg'] if max_pull is None else max(max_pull,r['door_angle_deg'])
    v=0. if previous is None else float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(previous['T_tcp'])[:3,3])*240)
    speed_max=max(speed_max,v);qv=tele.get('joint_velocities');jv=float(np.max(np.abs(np.asarray(qv)[:6]))) if qv is not None else None
    if jv is not None:joint_speed=max(joint_speed,jv)
    if v>.005 and first_speed is None:first_speed={'t':r['t'],'phase':r['phase'],'actual_angle_deg':r['door_angle_deg'],'world_TCP_velocity_m_s':v,'actual_arm_q':r['q'][:6],'command_arm_q':r['command_q_arm'],'joint_velocities':qv}
    if jv is not None and jv>5/3 and first_joint_speed is None:first_joint_speed={'t':r['t'],'phase':r['phase'],'actual_angle_deg':r['door_angle_deg'],'joint_velocities':qv}
    drift=max(drift,r['relative_translation_slip_m']);rot=max(rot,r['relative_rotation_slip_deg']);margin=r['margin_rad'] if margin is None else min(margin,r['margin_rad'])
    positive=any(c.get('allowed_pad_target') and c.get('force_n',0)>0 for c in r['contacts'])
    if not positive and gap is None:gap=r['t']
    if positive and gap is not None:gaps.append({'start':gap,'end':r['t'],'duration_s':r['t']-gap});gap=None
   previous=r
 if gap is not None:gaps.append({'start':gap,'end':last['t']+1/240,'duration_s':last['t']+1/240-gap})
 passed=bool(report['success'] and report['initial_grasp'] and max_pull is not None and max_pull>=30.0)
 native=json.loads((output/'physics_steps_chunks/index.json').read_text())['records']
 data={'classification':'KNOWN_MODEL_DIAGNOSTIC_OFFLINE_EVALUATION','asset_id':report['asset_id'],'target_deg':30.0,'success_30_deg':passed,'grasp_confirmed':report['initial_grasp'],'max_actual_angle_all_phases_deg':report['max_actual_angle_deg'],'max_actual_angle_after_valid_grasp_deg':max_valid,'max_actual_angle_during_opening_deg':max_pull,'max_before_valid_grasp_deg_not_credited':raw_pre,'grasp_acquisition':grasp,'first_physical_divergence':first_speed or first_joint_speed or report.get('first_divergence'),'first_existing_TCP_speed_violation':first_speed,'first_existing_joint_speed_violation':first_joint_speed,'terminal_status':report['status'],'terminal_t':last['t'] if last else None,'terminal_actual_arm_q':last['q'][:6] if last else None,'terminal_command_arm_q':last['command_q_arm'] if last else None,'terminal_pad_loads_n':last['forces_n'] if last else None,'terminal_contacts':last['contacts'] if last else None,'observations':records,'native_records':native,'sensor_hz':240,'contact_records':contact_count,'telemetry_available_counts':dict(counts),'peak_pad_loads_n':peak.tolist(),'max_world_TCP_velocity_after_grasp_m_s':speed_max if grasp else None,'max_joint_velocity_after_grasp_rad_s':joint_speed if grasp else None,'max_relative_translation_m':drift if grasp else None,'max_relative_rotation_deg':rot if grasp else None,'minimum_loaded_joint_margin_rad':margin,'contact_loss_intervals':gaps,'simulation_s':report['simulation_s'],'wall_s':report['wall_s'],'evaluation_does_not_govern_execution':True}
 (output/'BENCHMARK_A_METRICS.json').write_text(json.dumps(data,indent=2));print(report['asset_id'],passed,max_pull,report['status'],records,flush=True)
