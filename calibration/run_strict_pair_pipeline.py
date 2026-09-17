"""Strict empty/payload capture adapter. Frozen execution/estimator untouched."""
import os,json,runpy
from pathlib import Path
import numpy as np
from real2sim.payload_skill_v2 import PayloadIDV2Skill
from strict_pair_protocol import POLICY,window_quality

manifest=Path(os.environ['STRICT_PAIR_MANIFEST'])
def prepare(self,output):
 return {'static':json.load(open(manifest)),'dynamic':None,'schema':'strict_pair_diagnostic/v1'}
def capture(self,output_dir,protocol,grasp_result,mode,**kwargs):
 out=Path(output_dir);spec=json.load(open(manifest));rows=[];accepted=[];empty=None
 if mode=='payload':empty=json.load(open(Path(os.environ['STRICT_PAIR_EMPTY'])/'strict_capture.json'))
 for pose in spec['poses']:
  pid=pose['pose_id'];qref=np.asarray(pose['q_ref']);w=spec['opening_m'];base=None
  if empty is not None:
   base=next((x for x in empty['accepted'] if x['pose_id']==pid),None)
   if base is None:continue
  for attempt,settle in enumerate(POLICY['settle_s']):
   self.backend.validate(self._robot_state_for_q(qref))
   if mode=='baseline':self.plant.command({'op':'trajectory','names':['fr3_finger_joint1','fr3_finger_joint2'],'points':[{'t':1.,'q':[w/2,w/2]}],'gripper':True})
   trajectory=self.plant.command({'op':'trajectory','names':self.arm_names,'points':[{'t':2.,'q':qref.tolist()}]},timeout=180)
   self.plant.settle(settle)
   start=self.plant.command({'op':'payload_record_start','mode':mode})
   if not start.get('ok'):
    rows.append({'pose_id':pid,'attempt':attempt,'accepted':False,'reasons':['CONTACT_LOSS'],'start':start});break
   self.plant.settle(.65)
   path=out/f'{mode}_pose{pid:02d}_attempt{attempt}.npz';stop=self.plant.command({'op':'payload_record_stop','path':str(path.resolve())})
   if not path.exists():rows.append({'pose_id':pid,'attempt':attempt,'accepted':False,'reasons':['NO_RECORD'],'stop':stop});break
   with np.load(path,allow_pickle=True) as f:d={k:f[k] for k in f.files}
   row=window_quality(d,qref,w,payload=mode=='payload');row.update(pose_id=pid,attempt=attempt,path=str(path),q_ref=qref.tolist(),trajectory=trajectory)
   if not stop.get('ok'):row['reasons'].append('CONTACT_LOSS')
   if base:
    pair=float(np.max(np.abs(np.asarray(row['q_mean'])-np.asarray(base['q_mean']))));row['q_pair_error_rad']=pair
    if pair>=POLICY['q_pair_max_rad']:row['reasons'].append('Q_PAIR_MISMATCH')
    if abs(row['opening_mean_m']-base['opening_mean_m'])>=POLICY['opening_max_m']:row['reasons'].append('PAIR_OPENING_MISMATCH')
   row['accepted']=not row['reasons'];rows.append(row)
   if row['accepted']:accepted.append(row)
   (out/'strict_capture.json').write_text(json.dumps({'mode':mode,'policy':POLICY,'rows':rows,'accepted':accepted,'finished':False},indent=2))
   if row['accepted'] or 'CONTACT_LOSS' in row['reasons']:break
 result={'mode':mode,'policy':POLICY,'rows':rows,'accepted':accepted,'finished':True,'GT_used':False};(out/'strict_capture.json').write_text(json.dumps(result,indent=2));return {'mode':mode,'clean_windows':len(accepted),'finished':True}
PayloadIDV2Skill.prepare_protocol=prepare
PayloadIDV2Skill.run=capture
runpy.run_path(str(Path(__file__).with_name('run_real2sim_pipeline.py')),run_name='__main__')
