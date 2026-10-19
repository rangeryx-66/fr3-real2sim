import json,pickle,gzip
from pathlib import Path
p=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_native_gripper_20261012/45249/physical_01');rows=[]
for f in sorted((p/'observations_chunks').glob('*.gz'))[-2:]:rows+=pickle.load(gzip.open(f,'rb'))
events=[];last=None
for x in rows:
 e=x['force_event'];st=e['status']
 if st!=last:events.append({'t':x['t'],'phase':x['phase'],'angle_deg':x['door_angle_deg'],'loads':x['forces_n'],'force_event':e,'drift_mm':1000*x['relative_translation_slip_m']});last=st
counts=[];previous=0
for x in rows:
 n=x['force_event']['consecutive_overload_windows']
 if n!=previous:counts.append({'t':x['t'],'counter':n,'mean_n':x['force_event']['window_mean_n'],'raw_n':x['force_event']['raw_peak_n']});previous=n
r={'overload_counter_transitions':counts,'tail_force_transitions':events,'last_samples':[{'t':x['t'],'angle_deg':x['door_angle_deg'],'force_event':x['force_event'],'forces':x['forces_n']} for x in rows[-5:]],'new_physical_trials':0,'mechanical_limit_claimed':False};(p/'FIRST_DIVERGENCE_FORCE.json').write_text(json.dumps(r,indent=2));print(json.dumps(events,indent=2))
