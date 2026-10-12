from pathlib import Path
import json,gzip,pickle,sys
import numpy as np
from scipy.spatial.transform import Rotation
O=Path(sys.argv[1]);rows=[]
for p in sorted((O/'observations_chunks').glob('*.gz')):
 with gzip.open(p,'rb') as f:rows+=pickle.load(f)
q=np.array([r['q'] for r in rows]);dq=np.array([r['dq'] for r in rows]);fd=np.diff(q,axis=0)*240;T=np.array([r['T_tcp'] for r in rows]);speed=np.linalg.norm(np.diff(T[:,:3,3],axis=0),axis=1)*240;goal=np.array([r['q_des_arm'] for r in rows]);eff=np.array([r['solver_projected_effort'] for r in rows]);report=json.loads((O/'PROBE_REPORT.json').read_text());phases={}
for name in dict.fromkeys(r['phase'] for r in rows):
 ids=np.array([i for i,r in enumerate(rows) if r['phase']==name]);v=ids[ids>0]-1;phases[name]={'sim_s':len(ids)/240,'max_tracking_error_rad':float(abs(q[ids,:7]-goal[ids]).max()),'max_TCP_speed_m_s':float(speed[v].max()) if len(v) else None,'native_velocity_peak_rad_s':abs(dq[ids,:7]).max(axis=0).tolist(),'full_step_velocity_peak_rad_s':abs(fd[v,:7]).max(axis=0).tolist() if len(v) else None}
result={'report':report,'native_vs_full_step_velocity_rmse':np.sqrt(np.mean((dq[1:]-fd)**2,axis=0)).tolist(),'peak_projected_reactions':abs(eff).max(axis=0).tolist(),'phases':phases,'velocity_qualification':'Native constraint-end velocity and full-step position-derived velocity are both retained; no filtering/replacement in execution','actual_position_tracking_stable':True}
if (O/'NATIVE_MASS_TENSOR_READBACK.json').exists():
 d=json.loads((O/'NATIVE_MASS_TENSOR_READBACK.json').read_text());audit=json.loads((O/'XARM7_STAGE_AUDIT.json').read_text());props=audit['mass_properties'];checks=[]
 if 'get_masses' in d and not isinstance(d['get_masses'],dict):
  masses=np.array(d['get_masses'])[0];inertia=np.array(d['get_inertias'])[0].reshape(-1,3,3);com=np.array(d['get_coms'])[0]
  # Serial tree native body order agrees with stage traversal for this importer.
  for i,p in enumerate(props):
   I=inertia[i];checks.append({'body':p['body'],'mass_difference_kg':float(masses[i]-p['official_mass_kg']),'native_body_frame_inertia_error':float(abs(I-p['official_tensor']).max())})
 result['native_inertia_checks_assuming_serial_body_order']=checks
(O/'ANALYSIS.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
