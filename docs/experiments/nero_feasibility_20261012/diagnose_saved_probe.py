from pathlib import Path
import gzip,pickle,json,numpy as np
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/nero_feasibility_20261012/integration_probe';records=[];drv=[]
for p in sorted((O/'observations_chunks').glob('*.gz')):records.extend(pickle.load(gzip.open(p,'rb')))
for p in sorted((O/'actuator_chunks').glob('*.gz')):drv.extend(pickle.load(gzip.open(p,'rb')))
m=json.loads((O/'APPROXIMATE_ACTUATOR.json').read_text());idx=m['arm_native_indices'];e=np.array([np.array(x['q'])[idx]-x['q_des_arm'] for x in records]);sat=np.array([x['saturated'] for x in drv]);tau=np.array([x['tau_command_nm'] for x in drv]);bad=next((r for r,err in zip(records,e) if abs(err).max()>.01),None)
samples=[]
for t in [0,.05,.1,.5,1,2,4,8,12,20.99]:
 k=min(range(len(records)),key=lambda k:abs(records[k]['t']-t));r=records[k];d=drv[min(k,len(drv)-1)];samples.append({'t':r['t'],'phase':r['phase'],'q_arm':np.array(r['q'])[idx].tolist(),'q_des':r['q_des_arm'],'error_rad':e[k].tolist(),'tau_nm':d['tau_command_nm'],'gravity_nm':d['tau_gravity_nm'],'dq_arm':np.array(r['dq'])[idx].tolist()})
result={'classification':'DIAGNOSTIC_ONLY_FROM_SAVED_ROBOT_ONLY_PROBE','names':[m['joint_names_native_order'][i] for i in idx],'maximum_joint_error_rad':abs(e).max(0).tolist(),'final_joint_error_rad':e[-1].tolist(),'torque_saturation_duty':sat.mean(0).tolist(),'peak_commanded_arm_torque_nm':abs(tau).max(0).tolist(),'first_over_0p01_rad_diagnostic':{'t':bad['t'],'q':bad['q'],'desired':bad['q_des_arm']} if bad else None,'samples':samples,'self_contact_impulses':'not collected in this robot-only probe; no invented contact-force attribution','no_door_episode_launched':True};(O/'TRACKING_DIVERGENCE.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
