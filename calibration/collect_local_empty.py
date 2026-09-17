"""Robot-only local samples, fixed endpoints and held-out center, no payload input torque/GT."""
import os,json,subprocess,time
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];O=R/'results/fr3_q_compensation';runs=[]
for ni,name in enumerate(['banana','mug']):
 d=json.load(open(O/f'{name}.json'));rows=[x for x in d['rows'] if not x['reasons']];out=O/'local_empty'/name;out.mkdir(parents=True,exist_ok=True)
 # Each pose triple has endpoints used for interpolation and a held-out center.
 poses=[];mapping=[]
 for i,row in enumerate(rows):
  qe=np.array(row['q_empty']);ql=np.array(row['q_loaded']);diff=ql-qe;length=float(np.linalg.norm(diff));axis=diff/max(length,1e-12);half=max(.004,length*1.25)
  with np.load(row['empty_path'],allow_pickle=True) as f:
   oldcmd=f['q_ref'][-1] if 'q_ref' in f else np.array(next(x for x in json.load(open(R/'results/fr3_clean_pair_fd_gate/matched_opening_repair/banana/manifest.json'))['poses'] if x['pose_id']==row['pose_id'])['q_ref'])
  cmdcenter=oldcmd+diff
  for side in [-1,0,1]:poses.append({'pose_id':i*3+side+1,'q_ref':(cmdcenter+side*half*axis).tolist()})
  mapping.append({'parent_pose_id':row['pose_id'],'local_ids':[i*3,i*3+1,i*3+2],'q_loaded':ql.tolist(),'axis':axis.tolist(),'half_range_rad':half,'source_empty':row['empty_path']})
 opening=float(np.median([x['opening_loaded'] for x in rows]));spec={'target':name,'seed':1020 if name=='banana' else 1039,'opening_m':opening,'poses':poses,'local_mapping':mapping,'method':'two_endpoints_plus_heldout_center','GT_used':False};manifest=out/'manifest.json';manifest.write_text(json.dumps(spec,indent=2))
 # Frozen baseline reset interface uses a robot-only center and the measured opening.
 center=out/'center.json';center.write_text(json.dumps({'q':poses[0]['q_ref'],'finger_q':[opening/2,opening/2]},indent=2))
 # Reuse established center schema from the existing capture rather than invent fields.
 source=R/'results/fr3_torque_rootcause/fresh'/name/'payload/excitation_center.json'
 cent=json.load(open(source));cent['finger_q']=[opening/2,opening/2];center.write_text(json.dumps(cent,indent=2))
 env=dict(os.environ,STRICT_PAIR_MANIFEST=str(manifest),PAYLOAD_JACOBIAN_REFRESH_TICKS='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');env.pop('PAYLOAD_ATTACHMENT_MODE',None)
 cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','calibration/launch_local_empty.py','--target',name,'--seed',str(spec['seed']),'--mode','ANYGRASP','--gpu','3','--port',str(20400+ni*4),'--domain',str(184+ni*4),'--payload-v2','--mass-com-only','--skip-scan','--baseline-only','--center-q',str(center),'--gripper-opening-mm',str(opening*1000),'--calibration-force','60','--calibration-mu','1.0','--output',str(out)]
 with (out/'runner.log').open('w') as f:
  try:ret=subprocess.run(cmd,cwd=R,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1800).returncode
  except subprocess.TimeoutExpired:ret=124
 runs.append({'object':name,'exit':ret,'requested_empty_poses':len(poses)});(O/'local_empty_progress.json').write_text(json.dumps(runs,indent=2))
(O/'local_empty_complete.json').write_text(json.dumps(runs,indent=2))
