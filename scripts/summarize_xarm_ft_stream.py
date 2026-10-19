"""Streaming evidence integrity/metrics; never imported by a live controller."""
import argparse,gzip,pickle,json,collections
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('episode',type=Path);a=p.parse_args();root=a.episode
counts=collections.Counter();phases=collections.Counter();max_speed=collections.defaultdict(float);peaks=collections.defaultdict(float);previous=None;mind=1e9;maxd=0.;margin=1e9;angle=0.;missing=[]
for path in sorted((root/'observations_chunks').glob('*.gz')):
 with gzip.open(path,'rb') as f:rows=pickle.load(f)
 for r in rows:
  counts['rows']+=1;phase=r['phase'];phases[phase]+=1
  ft=r.get('wrist_ft',{});counts['FT_valid']+=int(ft.get('valid',False));counts['FT_missing']+=int(not ft)
  if not ft.get('valid'):missing.append(r['t'])
  margin=min(margin,r['margin_rad']);angle=max(angle,r['door_angle_deg'])
  tel=r.get('telemetry',{})
  for n,v in r['forces_n'].items():peaks['filtered_'+n]=max(peaks['filtered_'+n],v)
  for n,v in tel.get('pad_load_raw_n',{}).items():peaks['raw_'+n]=max(peaks['raw_'+n],v)
  if previous is not None:
   dt=r['t']-previous['t'];mind=min(mind,dt);maxd=max(maxd,dt)
   if dt>0:max_speed[phase]=max(max_speed[phase],float(np.linalg.norm(np.asarray(r['T_tcp'])[:3,3]-np.asarray(previous['T_tcp'])[:3,3])/dt))
  previous=r
out=dict(records=dict(counts),timestamp_interval_s=[mind,maxd],maximum_angle_deg=angle,minimum_joint_margin_rad=margin,
 phase_simulation_s={n:v/240 for n,v in phases.items()},maximum_measured_TCP_speed_m_s_by_phase=dict(max_speed),pad_peaks_n=dict(peaks),missing_FT_sample_times_s=missing,
 torque_semantics='wrist raw is incoming fixed-joint spatial reaction; arm/G1 projected reactions are not isolated drive torque',independent_motor_drive_output_available=False)
(root/'FT_STREAM_INTEGRITY.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
