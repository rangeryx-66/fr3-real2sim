from pathlib import Path
import sys,json,gzip,pickle
import numpy as np
O=Path(sys.argv[1]);records=[]
for p in sorted((O/'observations_chunks').glob('*.pkl.gz')):
 with gzip.open(p,'rb') as f:records+=pickle.load(f)
q=np.array([r['q'] for r in records]);dq=np.array([r['dq'] for r in records]);target=np.array([r['q_des_arm'] for r in records]);tcp=np.array([r['T_tcp'] for r in records]);dt=records[0]['physics_dt_s'];fd=np.diff(q,axis=0)/dt;speed=np.linalg.norm(np.diff(tcp[:,:3,3],axis=0),axis=1)/dt
phases={}
for phase in dict.fromkeys(r['phase'] for r in records):
 ix=np.array([i for i,r in enumerate(records) if r['phase']==phase]);valid=ix[ix>0]-1
 phases[phase]={'sim_s':len(ix)*dt,'max_joint_error_rad':float(abs(q[ix,:7]-target[ix]).max()),'final_joint_error_rad':(q[ix[-1],:7]-target[ix[-1]]).tolist(),'max_tcp_speed_m_s':float(speed[valid].max()) if len(valid) else None,'max_native_joint_velocity_rad_s':abs(dq[ix,:7]).max(axis=0).tolist(),'max_fd_joint_velocity_rad_s':abs(fd[valid,:7]).max(axis=0).tolist() if len(valid) else None}
contacts=[x for r in records for x in r['contacts']];pairs={}
for c in contacts:
 key=' | '.join(sorted([c['collider0'],c['collider1']]));info=pairs.setdefault(key,{'samples':0,'peak_impulse_ns':0.,'minimum_separation_m':0.})
 info['samples']+=1;info['peak_impulse_ns']=max(info['peak_impulse_ns'],float(np.linalg.norm(c['impulse_world_ns'])));info['minimum_separation_m']=min(info['minimum_separation_m'],c['separation_m'])
projected=np.array([r['solver_projected_effort'] for r in records if r['solver_projected_effort'] is not None]);external=np.array([r['externally_applied_effort'] for r in records if r['externally_applied_effort'] is not None]);act=json.loads((O/'APPROXIMATE_ACTUATOR.json').read_text());kp=np.array(act['kp_nm_rad']);kd=np.array(act['kd_nm_s_rad']);limit=np.array(act['maximum_arm_effort_nm']);drive_target=np.array([r['actuator']['implicit_position_target'] for r in records]);desired_v=np.array([r['dq_des_arm'] for r in records]);estimated_post_drive=kp*(drive_target-q[:,:7])+kd*(desired_v-dq[:,:7]);estimated_post_drive=np.clip(estimated_post_drive,-limit,limit)
result={'records':len(records),'sim_s':len(records)*dt,'max_joint_error_rad':float(abs(q[:,:7]-target).max()),'joint_error_peaks_rad':abs(q[:,:7]-target).max(axis=0).tolist(),'final_error_rad':(q[-1,:7]-target[-1]).tolist(),'max_tcp_speed_m_s':float(speed.max()),'native_minus_fd_velocity_rmse_rad_s':np.sqrt(np.mean((dq[1:]-fd)**2,axis=0)).tolist(),'native_velocity_peak':abs(dq).max(axis=0).tolist(),'position_difference_velocity_peak':abs(fd).max(axis=0).tolist(),'solver_projected_effort_peak':abs(projected).max(axis=0).tolist() if len(projected) else None,'external_effort_command_peak':abs(external).max(axis=0).tolist() if len(external) else None,'drive_effort_poststep_estimate_peak_nm':abs(estimated_post_drive).max(axis=0).tolist(),'estimated_drive_saturation_fraction':(abs(estimated_post_drive)>=limit*.999).mean(axis=0).tolist(),'effort_semantics':'external effort is API command; projected effort is solver reaction projection, not motor torque; poststep drive estimate excludes solver subiterations and is diagnostic only','phases':phases,'contact_samples':len(contacts),'contact_pairs':pairs}
(O/'ANALYSIS.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ['phases','contact_pairs']},indent=2))
