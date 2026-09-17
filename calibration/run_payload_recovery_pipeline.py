"""Fresh payload-first paired capture; frozen estimator and q correction external."""
import os,json,runpy
from pathlib import Path
import numpy as np
from real2sim.payload_skill_v2 import PayloadIDV2Skill
from strict_pair_protocol import POLICY,window_quality

def capture(self,output_dir,protocol,grasp_result,mode,**kwargs):
 out=Path(output_dir);source=os.environ.get('CROSS_PAYLOAD_SOURCE');rows=[];accepted=[]
 if mode=='baseline':specs=json.load(open(Path(source)/'capture.json'))['accepted']
 else:
  s=protocol['static'];ids=np.array(s['static_pose_id']);hold=np.array(s['static_hold'],bool);available=[int(x) for x in np.unique(ids[hold]) if x>=0];chosen=[available[i] for i in np.unique(np.linspace(0,len(available)-1,min(8,len(available))).astype(int))];specs=[{'pose_id':pid,'q_ref':np.array(s['q'])[np.flatnonzero((ids==pid)&hold)[-1]].tolist()} for pid in chosen]
 reference=None
 def guard_snapshot(stage,pid=None,attempt=None):
  if mode=='baseline':return None
  guard=self.plant.command({'op':'payload_guard'})
  with (out/'handoff_diagnostics.jsonl').open('a') as f:
   f.write(json.dumps({'stage':stage,'pose_id':pid,'attempt':attempt,'guard':guard})+'\n')
  return guard
 guard_snapshot('capture_entry')
 for spec in specs:
  pid=spec['pose_id'];qref=np.array(spec['q_ref'])
  for attempt,settle in enumerate([1.,3.,5.]):
   self.backend.validate(self._robot_state_for_q(qref))
   if mode=='baseline':
    w=spec['opening_mean_m'];self.plant.command({'op':'trajectory','names':['fr3_finger_joint1','fr3_finger_joint2'],'points':[{'t':1.,'q':[w/2,w/2]}],'gripper':True})
   guard_snapshot('before_pose_move',pid,attempt)
   ans=self.plant.command({'op':'trajectory','names':self.arm_names,'points':[{'t':2.,'q':qref.tolist()}]},timeout=180);self.plant.settle(settle)
   guard_snapshot('after_pose_settle',pid,attempt)
   start=self.plant.command({'op':'payload_record_start','mode':mode})
   if not start.get('ok'):
    rows.append({'pose_id':pid,'attempt':attempt,'accepted':False,'reasons':[start.get('reason','PAYLOAD_GUARD_FAILED')],'start':start});(out/'capture.json').write_text(json.dumps({'mode':mode,'rows':rows,'accepted':accepted,'finished':False},indent=2));break
   self.plant.settle(.65);path=out/f'{mode}_pose{pid:02d}_attempt{attempt}.npz';stop=self.plant.command({'op':'payload_record_stop','path':str(path.resolve())})
   if not path.exists():rows.append({'pose_id':pid,'attempt':attempt,'accepted':False,'reasons':['NO_RECORD'],'stop':stop});break
   with np.load(path,allow_pickle=True) as f:d={k:f[k] for k in f.files}
   ix=d['t']>=d['t'][-1]-.5;w=spec['opening_mean_m'] if mode=='baseline' else float(d['actual_opening_m'][ix].mean())
   row=window_quality(d,qref,w,payload=mode!='baseline');row['recorded_q_only_rejections']=[x for x in row['reasons'] if x=='Q_REF_MISMATCH'];row['reasons']=[x for x in row['reasons'] if x!='Q_REF_MISMATCH']
   row.update(pose_id=pid,attempt=attempt,q_ref=qref.tolist(),path=str(path),trajectory=ans)
   if not stop.get('ok'):row['reasons'].append('CAPTURE_GUARD_FAILED');row['stop']=stop
   if mode!='baseline':
    from scipy.spatial.transform import Rotation
    T=d['T_TCP_object'][ix]
    if reference is None:reference=T[0]
    rel=np.linalg.inv(reference)[None]@T
    if np.max(np.linalg.norm(rel[:,:3,3],axis=1))>.003 or np.max(Rotation.from_matrix(rel[:,:3,:3]).magnitude())>np.deg2rad(5):row['reasons'].append('GLOBAL_RELATIVE_SLIP')
   row['guard_after_capture']=guard_snapshot('after_capture',pid,attempt);row['accepted']=not row['reasons'];rows.append(row)
   if row['accepted']:accepted.append(row)
   (out/'capture.json').write_text(json.dumps({'mode':mode,'rows':rows,'accepted':accepted,'finished':False},indent=2))
   if row['accepted'] or not stop.get('ok'):break
  if mode!='baseline' and rows and any(x in rows[-1]['reasons'] for x in ['GLOBAL_RELATIVE_SLIP','CAPTURE_GUARD_FAILED','PAYLOAD_ID_REQUIRES_FREE_SPACE_STABLE']):break
 result={'mode':mode,'rows':rows,'accepted':accepted,'finished':True,'GT_used':False};(out/'capture.json').write_text(json.dumps(result,indent=2));return {'clean_poses':len(accepted),'finished':True}
def prepare_static_only(self,output):
 output=Path(output);static=self._static_protocol(output.with_name(output.stem+'_static.json'))
 manifest={'schema':'cross_object_static_only/v1','static':static,'dynamic':None,'gt_used_for_estimation':False}
 output.write_text(json.dumps(manifest,indent=2));return manifest
PayloadIDV2Skill.prepare_protocol=prepare_static_only
PayloadIDV2Skill.run=capture
runpy.run_path(str(Path(__file__).with_name('run_real2sim_pipeline.py')),run_name='__main__')
