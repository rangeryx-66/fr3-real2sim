from pathlib import Path
import gzip,pickle,json,numpy as np,sys,collections
root=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/revolute10_fixed_20261011')
for output in root.glob('*/official_01'):
 if not (output/'report.json').exists() or (output/'physical_metrics.json').exists():continue
 report=json.loads((output/'report.json').read_text());events=json.loads((output/'events.json').read_text());grasp=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
 records=0;counts=collections.Counter();peak=np.zeros(2);velocity=0.;joint_velocity=0.;drift=0.;rot=0.;margin=None;previous=None;opening=[];first_dangerous=None;all_contacts=0;both_zero=[];gap=None
 for f in sorted((output/'observations_chunks').glob('*.gz')):
  with gzip.open(f,'rb') as stream:rows=pickle.load(stream)
  for r in rows:
   records+=1;tele=r.get('telemetry',{})
   counts.update(k for k,v in tele.items() if v is not None)
   peak=np.maximum(peak,list(r['forces_n'].values()));all_contacts+=len(r['contacts'])
   if first_dangerous is None and r['contacts']:
    for c in r['contacts']:
     if c.get('force_n',0)>0 and not c.get('allowed_pad_target',False):first_dangerous={'t':r['t'],'phase':r['phase'],'contact':c};break
   if grasp and r['t']>=grasp['t']:
    if previous is not None:velocity=max(velocity,float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(previous['T_tcp'])[:3,3])*240))
    v=tele.get('joint_velocities')
    if v is not None:joint_velocity=max(joint_velocity,float(np.max(np.abs(np.array(v)[:6]))))
    drift=max(drift,r['relative_translation_slip_m']);rot=max(rot,r['relative_rotation_slip_deg']);margin=r['margin_rad'] if margin is None else min(margin,r['margin_rad'])
    positive=any(c.get('force_n',0)>0 and c.get('allowed_pad_target',False) for c in r['contacts'])
    if not positive and gap is None:gap=r['t']
    if positive and gap is not None:both_zero.append({'start':gap,'end':r['t'],'seconds':r['t']-gap});gap=None
   if r['phase']=='FIXED_BASE_PINK_OPEN':opening.append(r['door_angle_deg'])
   previous=r
 if gap is not None:both_zero.append({'start':gap,'end':previous['t']+1/240,'seconds':previous['t']+1/240-gap})
 native=json.loads((output/'physics_steps_chunks/index.json').read_text()) if (output/'physics_steps_chunks/index.json').exists() else {}
 metrics={'classification':'OFFLINE_DIAGNOSTIC_ONLY','observation_records':records,'native_records':native.get('records'),'sensor_hz':240,'contact_records':all_contacts,'telemetry_available_counts':dict(counts),'peak_pad_loads_n':peak.tolist(),'grasp_acquisition':grasp,'opening_max_actual_angle_deg':max(opening) if opening else None,'max_world_TCP_velocity_after_grasp_m_s':velocity if grasp else None,'max_joint_velocity_after_grasp_rad_s':joint_velocity if grasp else None,'max_relative_translation_after_grasp_m':drift if grasp else None,'max_relative_rotation_after_grasp_deg':rot if grasp else None,'minimum_joint_margin_after_grasp_rad':margin,'contact_loss_intervals':both_zero,'first_non_handle_or_nonpad_contact':first_dangerous,'metrics_not_execution_gates':True}
 (output/'physical_metrics.json').write_text(json.dumps(metrics,indent=2));print(output.parent.name,json.dumps({k:v for k,v in metrics.items() if k not in ['telemetry_available_counts','first_non_handle_or_nonpad_contact','grasp_acquisition']}))
