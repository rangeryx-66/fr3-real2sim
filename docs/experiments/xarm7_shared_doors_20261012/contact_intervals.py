from pathlib import Path
import sys,json,gzip,pickle,numpy as np
O=Path(sys.argv[1]);rows=[]
for p in sorted((O/'observations_chunks').glob('*.gz')):
 with gzip.open(p,'rb') as f:rows+=pickle.load(f)
events=json.loads((O/'events.json').read_text());grasp=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
if not grasp:raise SystemExit('No valid grasp; no loaded contact interval')
rows=[r for r in rows if r['t']>=grasp['t']];loads=np.array([[r['forces_n']['gripper_link1'],r['forces_n']['gripper_link2']] for r in rows]);filtered=np.array([r['telemetry']['pad_load_filtered_n'] for r in rows]);dt=1/240
def intervals(mask):
 out=[];start=None
 for i,x in enumerate(mask):
  if x and start is None:start=i
  if start is not None and (not x or i==len(mask)-1):
   end=i if not x else i+1;out.append({'start_t_s':rows[start]['t'],'end_t_s':rows[end-1]['t']+dt,'duration_s':(end-start)*dt});start=None
 return {'count':len(out),'maximum_duration_s':max((x['duration_s'] for x in out),default=0),'total_duration_s':sum(x['duration_s'] for x in out),'intervals':out}
r={'threshold_n':.05,'threshold_is_existing_contact_definition':True,'left_raw_unloaded':intervals(loads[:,0]<.05),'right_raw_unloaded':intervals(loads[:,1]<.05),'both_raw_unloaded':intervals(np.max(loads,axis=1)<.05),'both_filtered_unloaded':intervals(np.max(filtered,axis=1)<.05),'qualification':'per-pad report unloading/flicker is not intentional release or a proven physical loss of grasp; actual motion and existing contact-loss protection are reported separately'}
(O/'CONTACT_INTERVALS.json').write_text(json.dumps(r,indent=2));print({k:{x:y for x,y in v.items() if x!='intervals'} if isinstance(v,dict) else v for k,v in r.items()})
