"""Predeclared empty-first diagnostic batch; no GT or tuning."""
import os,json,subprocess,hashlib,time
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];O=Path(os.environ.get('STRICT_PAIR_OUTPUT',str(R/'results/fr3_strict_pair_validation')));O.mkdir(parents=True,exist_ok=True)
from strict_pair_protocol import POLICY
sources=[('mug',1039,'fresh/mug/payload'),('soup',1030,'fresh_retry/soup/payload'),('mustard',1002,'fresh_retry/mustard/payload'),('banana',1020,'fresh/banana/payload')]
frozen=['real2sim/fr3_robot_calibration.py','real2sim/payload_id_official_mass_com.py','real2sim/payload_id_v2.py','real2sim/payload_jacobian.py','calibration/sim_real2sim.py','calibration/hand_force_control.py','calibration/run_real2sim_pipeline.py','calibration/strict_pair_protocol.py','calibration/quasistatic_velocity_gate.py']
hashs={f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in frozen};(O/'frozen_protocol.json').write_text(json.dumps({'policy':POLICY,'force_N':60,'friction_mu':1.,'excitation_amplitude_frozen_rad':.3,'static_test_only':True,'GT_used_for_capture':False,'source_sha256':hashs},indent=2))
rows=[]
for i,(name,seed,rel) in enumerate(sources):
 src=R/'results/fr3_torque_rootcause'/rel;st=json.load(open(src/'payload_id_v2_protocol_static.json'));center=json.load(open(src/'excitation_center.json'));ids=np.array(st['static_pose_id']);hold=np.array(st['static_hold'],bool);available=[int(x) for x in np.unique(ids[hold]) if x>=0];chosen=[available[k] for k in np.unique(np.linspace(0,len(available)-1,min(8,len(available))).astype(int))]
 root=O/name;root.mkdir(exist_ok=True);manifest=root/'manifest.json';manifest.write_text(json.dumps({'target':name,'seed':seed,'source':str(src),'opening_m':float(np.sum(center['finger_q'])),'poses':[{'pose_id':pid,'q_ref':np.array(st['q'])[np.flatnonzero((ids==pid)&hold)[-1]].tolist()} for pid in chosen]},indent=2))
 env=dict(os.environ,STRICT_PAIR_MANIFEST=str(manifest),PAYLOAD_JACOBIAN_REFRESH_TICKS='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');env.pop('PAYLOAD_ATTACHMENT_MODE',None)
 row={'target':name,'seed':seed,'empty':str(root/'empty'),'payload':str(root/'payload'),'manifest':str(manifest)}
 for mode in ['empty','payload']:
  out=root/mode;out.mkdir(exist_ok=True)
  if mode=='payload':
   ep=root/'empty/strict_capture.json';ec=json.load(open(ep)) if ep.exists() else {};n=len(ec.get('accepted',[]));row['clean_empty_windows']=n
   if n<POLICY['min_clean_poses']:row['payload_status']='NOT_RUN_NO_CLEAN_EMPTY';break
   env['STRICT_PAIR_EMPTY']=str(root/'empty')
  cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','calibration/launch_strict_pair.py','--target',name,'--seed',str(seed),'--mode','ANYGRASP','--gpu','3','--port',str(20200+i*4),'--domain',str(160+i*4),'--payload-v2','--mass-com-only','--skip-scan','--calibration-force','60','--calibration-mu','1.0','--output',str(out)]
  if mode=='empty':cmd+=['--baseline-only','--center-q',str(src/'excitation_center.json'),'--gripper-opening-mm',str(float(np.sum(center['finger_q']))*1000)]
  else:cmd+=['--capture-payload']
  with open(out/'runner.log','w') as f:
   try:row[mode+'_exit']=subprocess.run(cmd,cwd=R,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1800).returncode
   except subprocess.TimeoutExpired:row[mode+'_exit']=124
 rows.append(row);(O/'progress.json').write_text(json.dumps(rows,indent=2))
changed=[f for f,h in hashs.items() if hashlib.sha256((R/f).read_bytes()).hexdigest()!=h]
(O/'complete.json').write_text(json.dumps({'rows':rows,'frozen_files_changed':changed,'finished_unix':time.time()},indent=2))
