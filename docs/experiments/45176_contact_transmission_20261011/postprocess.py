"""Passive post-run native contact evidence and before/after comparison."""
from pathlib import Path
import json,gzip,pickle,numpy as np,collections
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45176_contact_transmission_20261011';trial=out/'physical_01';old=ROOT/'results/oracle_door_development_20261011/45176/development_01'
m=json.loads((trial/'PHYSICAL_METRICS.json').read_text());events=json.loads((trial/'events.json').read_text());grasp=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
previous=None;curves=[];loaded_count=0;positive=np.zeros(2,int);first_zero=None;contact_examples=[];speed_by_phase=collections.defaultdict(float);oldrows=[json.loads(x) for x in (old/'live.jsonl').read_text().splitlines()]
for f in sorted((trial/'observations_chunks').glob('*.gz')):
 for r in pickle.load(gzip.open(f,'rb')):
  speed=0. if previous is None else float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(previous['T_tcp'])[:3,3])*240);speed_by_phase[r['phase']]=max(speed_by_phase[r['phase']],speed)
  loaded=bool(grasp and r['t']>=grasp['t'])
  if loaded:
   loaded_count+=1;positive+=np.array(list(r['forces_n'].values()))>0
   if first_zero is None and min(r['forces_n'].values())==0:first_zero={'t':r['t'],'angle_deg':r['door_angle_deg'],'loads_n':r['forces_n'],'relative_slip_m':r['relative_translation_slip_m'],'q':r['q'],'T_tcp':r['T_tcp'],'T_moving':r['T_moving_link'],'contacts':r['contacts'],'raw_contacts':r['telemetry']['raw_contact_records']}
   if len(contact_examples)<3:contact_examples.append({'t':r['t'],'phase':r['phase'],'angle_deg':r['door_angle_deg'],'pad_loads_n':r['forces_n'],'contacts':r['contacts'],'raw_contacts':r['telemetry']['raw_contact_records']})
  if int(r['telemetry']['physics_step_index'])%24==0:curves.append([r['t'],r['door_angle_deg'],r['relative_translation_slip_m']*1000,r['margin_rad'],speed*1000,*r['forces_n'].values()])
  previous=r
result={'classification':'POSTRUN_CONTACT_EVIDENCE_ONLY','previous_1Hz_sampled_max_deg':max(r['door_angle_deg'] for r in oldrows if r['phase']=='FIXED_BASE_PINK_OPEN'),'previous_native_records':0,'previous_exact_force_speed_contact_normals_unknown':True,'new_actual_max_deg':m['max_actual_angle_after_valid_grasp_deg'],'physical_PASS_30deg':m['physical_PASS_30deg'],'max_measured_TCP_by_phase_m_s':dict(speed_by_phase),'first_zero_pad_report_after_grasp':first_zero,'native_positive_pad_load_duty_after_grasp':(positive/loaded_count).tolist() if loaded_count else None,'loaded_records':loaded_count,'contact_examples':contact_examples,'terminal_contacts':previous['contacts'],'terminal_raw_contacts':previous['telemetry']['raw_contact_records'],'terminal_pose':{k:previous[k] for k in ['q','T_tcp','T_moving_link','door_angle_deg','aperture_m','margin_rad','forces_n']},'not_a_new_execution_gate':True}
(trial/'CONTACT_TRANSMISSION_EVIDENCE.json').write_text(json.dumps(result,indent=2))
a=np.array(curves);o=np.array([[r['t'],r['door_angle_deg'],r.get('relative_translation_slip_m',0)*1000,*r['forces_n'].values()] for r in oldrows]);fig,axes=plt.subplots(2,3,figsize=(14,8))
axes[0,0].plot(o[:,0],o[:,1],label='old: 1 Hz only');axes[0,0].plot(a[:,0],a[:,1],label='new: native archive');axes[0,0].axhline(30,ls='--',color='gray');axes[0,1].plot(a[:,0],a[:,2]);axes[0,2].plot(a[:,0],a[:,5],label='left');axes[0,2].plot(a[:,0],a[:,6],label='right');axes[1,0].plot(a[:,0],a[:,4]);axes[1,1].plot(a[:,0],a[:,3]);axes[1,1].axhline(.05,color='r',ls='--');
p=json.loads((out/'PATCH_DIAGNOSIS_AND_COMPATIBILITY.json').read_text())[0];chosen=json.loads((out/'PREFLIGHT.json').read_text())['cases'][0];axes[1,2].bar(['old visual','old native','new visual','new native'],[p['old_patch'][k]['bilateral_effective_patch_area_m2']*1e6 for k in ['visual','native']]+[chosen['patch'][k]['bilateral_effective_patch_area_m2']*1e6 for k in ['visual','native']]);axes[1,2].tick_params(axis='x',rotation=20)
for ax,title,unit in zip(axes.flat,['Actual door opening','Handle-relative drift','Raw pad loads','Measured TCP speed (24 Hz plot; 240 Hz metrics)','Joint margin','Predicted bilateral patch area (not measurement)'],['deg','mm','N','mm/s','rad','mm²']):
 ax.set_title(title,fontsize=10);ax.set_ylabel(unit);ax.grid(alpha=.2)
 if ax is not axes[1,2]:ax.set_xlabel('Simulation time (s)')
for ax in [axes[0,0],axes[0,2]]:ax.legend(fontsize=8)
fig.suptitle('45176 / ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — one physical trial');fig.tight_layout();fig.savefig(out/'COMPARISON.png',dpi=150)
print(json.dumps({k:v for k,v in result.items() if k not in ['contact_examples','terminal_contacts','terminal_raw_contacts','terminal_pose','first_zero_pad_report_after_grasp']},indent=2))
