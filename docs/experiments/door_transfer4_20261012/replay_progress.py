"""Replay real archived state streams through the actual shared progress rule."""
from pathlib import Path
import sys,json,gzip,pickle,numpy as np
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT/'scripts'),str(ROOT)]
from shared_door_execution import progress_checkpoint
out=ROOT/'results/door_transfer4_20261012';cases=[('45415_verified',ROOT/'results/45415_precontact_continuity_20261011/physical_01'),('45176_failed',ROOT/'results/45176_contact_transmission_20261011/physical_01')];reports=[]
for name,path in cases:
 events=[];checkpoint=None;maximum=-np.inf;count=0;loaded=False;grasp=json.loads((path/'report.json').read_text())['grasp_acquisition'];records=0
 for f in sorted((path/'observations_chunks').glob('*.gz')):
  for r in pickle.load(gzip.open(f,'rb')):
   if r['phase']!='FIXED_BASE_PINK_OPEN':continue
   records+=1;checkpoint,maximum,count=progress_checkpoint(checkpoint,maximum,np.deg2rad(r['door_angle_deg']),r['relative_translation_slip_m'],r['t']+1/240,.5,count,float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(r['T_moving_link'])[:3,3])))
   if count>=3:events.append({'t':r['t'],'angle_deg':r['door_angle_deg'],'drift_m':r['relative_translation_slip_m']});count=0
 reports.append({'case':name,'records':records,'first_stall_event':events[0] if events else None,'total_event_windows':len(events),'prediction_only_no_correction_dynamics_replayed':True})
(out/'PROGRESS_REPLAY.json').write_text(json.dumps({'method':'same progress_checkpoint function used by execution; original .5s confirmation scale and 3 temporal windows; geometric drift growth exceeding transmitted hinge-distance; no pad-unloading gate','cases':reports,'physical_attempts':0},indent=2));print(json.dumps(reports,indent=2))
