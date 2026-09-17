"""Predeclared fresh four-object validation, at most five grasps each, no GT selection."""
import os,json,subprocess,hashlib,time
from pathlib import Path
R=Path(__file__).resolve().parents[1];O=R/'results/fr3_qcomp_cross_object';O.mkdir(exist_ok=True)
objects=[('soup',list(range(1030,1035))),('mustard',list(range(1000,1005))),('raisin',list(range(1005,1010))),('sugar',list(range(1025,1030)))]
frozen=['real2sim/payload_skill_v2.py','real2sim/fr3_robot_calibration.py','real2sim/payload_id_official_mass_com.py','real2sim/payload_jacobian.py','real2sim/payload_id_v2.py','calibration/strict_pair_protocol.py','calibration/quasistatic_velocity_gate.py','calibration/compare_q_baselines.py','calibration/sim_real2sim.py','calibration/run_real2sim_pipeline.py','calibration/hand_force_control.py']
hashes={p:hashlib.sha256((R/p).read_bytes()).hexdigest() for p in frozen};(O/'frozen_protocol.json').write_text(json.dumps({'objects':objects,'max_grasp_attempts':5,'force_N':60,'mu':1,'excitation_design_rad':.3,'capture':'STATIC_ONLY_UNCHANGED_POSE_DESIGN','dq_limit':.002,'opening_limit_m':.0002,'default_method':'DRAKE_Q_COMPENSATION','GT_selection':False,'source_sha256':hashes},indent=2))
rows=[];cases=[]
for index,(name,seeds) in enumerate(objects):
 for attempt,seed in enumerate(seeds):
  root=O/name/f'attempt_{attempt:02d}_seed{seed}';root.mkdir(parents=True,exist_ok=True);row={'object':name,'seed':seed,'attempt':attempt,'path':str(root)}
  env=dict(os.environ,PAYLOAD_JACOBIAN_REFRESH_TICKS='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');env.pop('PAYLOAD_ATTACHMENT_MODE',None)
  for mode in ['payload','empty']:
   out=root/mode;out.mkdir(exist_ok=True)
   cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','calibration/launch_cross_object.py','--target',name,'--seed',str(seed),'--mode','ANYGRASP','--gpu','3','--port',str(20500+index*4),'--domain',str(200+index*4),'--payload-v2','--mass-com-only','--skip-scan','--calibration-force','60','--calibration-mu','1','--output',str(out)]
   if mode=='payload':cmd+=['--capture-payload']
   else:
    env['CROSS_PAYLOAD_SOURCE']=str(root/'payload');cmd+=['--baseline-only','--center-q',str(root/'payload/excitation_center.json'),'--gripper-opening-mm',str(payload['accepted'][0]['opening_mean_m']*1000)]
   with (out/'runner.log').open('w') as f:
    try:ret=subprocess.run(cmd,cwd=R,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1800).returncode
    except subprocess.TimeoutExpired:ret=124
   row[mode+'_exit']=ret
   if mode=='payload':
    p=out/'capture.json';payload=json.load(open(p)) if p.exists() else {'accepted':[]};row['payload_clean_poses']=len(payload['accepted'])
    if len(payload['accepted'])<4:break
   else:
    p=out/'capture.json';empty=json.load(open(p)) if p.exists() else {'accepted':[]};pairs=[]
    for b in empty['accepted']:
     p=next(x for x in payload['accepted'] if x['pose_id']==b['pose_id']);pairs.append({'pose_id':b['pose_id'],'empty_path':b['path'],'payload_path':p['path']})
    row['paired_poses']=len(pairs)
    if len(pairs)>=4:cases.append({'name':name,'payload_dir':str(root/'payload'),'pairs':pairs})
  rows.append(row);(O/'progress.json').write_text(json.dumps(rows,indent=2));(O/'evaluation_cases.json').write_text(json.dumps(cases,indent=2))
  if any(c['name']==name for c in cases):break
changed=[p for p,h in hashes.items() if hashlib.sha256((R/p).read_bytes()).hexdigest()!=h];(O/'complete.json').write_text(json.dumps({'rows':rows,'frozen_files_changed':changed,'finished_unix':time.time()},indent=2))
env=dict(os.environ,CROSS_OUTPUT=str(O),PYTHONPATH='/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages',OPENBLAS_NUM_THREADS='1')
with (O/'analysis.log').open('w') as f:p=subprocess.run(['/data1/home/rangeryx/official_payload_env/bin/python','calibration/evaluate_cross_object.py'],cwd=R,env=env,stdout=f,stderr=subprocess.STDOUT)
(O/'analysis_exit.json').write_text(json.dumps({'returncode':p.returncode}))
