"""Diagnostic-only discrete settled windows; frozen grasp runner is reused."""
from pathlib import Path
import runpy,json,os
import numpy as np
from real2sim.payload_skill_v2 import PayloadIDV2Skill

def capture(self,output_dir,protocol,grasp_result,mode,**kwargs):
 out=Path(output_dir);source=os.environ.get('TORQUE_DIAGNOSTIC_PAIR')
 if source:
  specs=json.load(open(Path(source)/'diagnostic_poses.json'))
 else:
  s=protocol['static'];ids=np.asarray(s['static_pose_id']);hold=np.asarray(s['static_hold'],bool)
  available=[int(x) for x in np.unique(ids[hold]) if x>=0]
  selected=[available[i] for i in np.unique(np.linspace(0,len(available)-1,min(8,len(available))).astype(int))]
  specs=[{'pose_id':pid,'q':np.asarray(s['q'])[np.flatnonzero((ids==pid)&hold)[-1]].tolist()} for pid in selected]
 chunks=[];audit=[];captured=[]
 for spec in specs:
  pid=spec['pose_id'];q=spec['q'];state=self._robot_state_for_q(np.array(q));self.backend.validate(state)
  if mode=='baseline' and 'opening_m' in spec:
   w=spec['opening_m']/2;self.plant.command({'op':'trajectory','names':['fr3_finger_joint1','fr3_finger_joint2'],'points':[{'t':1.,'q':[w,w]}],'gripper':True})
  answer=self.plant.command({'op':'trajectory','names':self.arm_names,'points':[{'t':2.,'q':q}]},timeout=180)
  self.plant.settle(1.0)
  start=self.plant.command({'op':'payload_record_start','mode':mode})
  if not start.get('ok'):audit.append({'pose_id':pid,'failure':'CONTACT_LOSS','detail':start});break
  self.plant.settle(.5)
  path=out/f'diagnostic_{mode}_pose{pid:02d}.npz';stop=self.plant.command({'op':'payload_record_stop','path':str(path.resolve())})
  audit.append({'pose_id':pid,'trajectory':answer,'record':stop})
  if not path.exists() or not stop.get('ok'):break
  with np.load(path,allow_pickle=True) as f:d={k:f[k] for k in f.files}
  n=len(d['t']);d['static_pose_id']=np.full(n,pid);d['static_hold']=np.ones(n,bool);d['q_ref']=np.tile(q,(n,1));d['dq_ref']=np.zeros((n,7));d['ddq_ref']=np.zeros((n,7));chunks.append(d)
  captured.append({'pose_id':pid,'q':d['q'].mean(0).tolist(),'opening_m':float(d['actual_opening_m'].mean())})
  (out/'diagnostic_poses.json').write_text(json.dumps(captured,indent=2))
 if chunks:
  d={k:np.concatenate([x[k] for x in chunks],axis=0) for k in chunks[0] if k not in ['guard_passed','mode','attachment_mode','motion_events_json']}
  # Keep real simulation timestamps; no relabelled trajectory interpolation.
  d.update(guard_passed=np.array([all(x['guard_passed'][0] for x in chunks)]),mode=np.array([mode]),attachment_mode=np.array(['NORMAL']),motion_events_json=np.array(['[]']))
  np.savez_compressed(out/f'system_id_{mode}_static.npz',**d)
 result={'mode':mode,'windows':len(chunks),'audit':audit,'diagnostic_only':True,'GT_used':False};(out/'diagnostic_capture.json').write_text(json.dumps(result,indent=2));return result
PayloadIDV2Skill.run=capture
runpy.run_path(str(Path(__file__).with_name('run_real2sim_pipeline.py')),run_name='__main__')
