"""Read-only saved-probe diagnosis and bounded matched-station IK ablation."""
from pathlib import Path
import json,gzip,pickle,numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from nero_door_adapter import configuration,NeroModel,RobotPinkIK,scene_factory
from revolute10_fixed import authored_handle_colliders
from interactive_twin_recovery.mobile import at_base
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/nero_feasibility_20261012';c=configuration();results=[]
for asset in ['45134','45443','45176','45415']:
 p=O/'offline'/asset;plan=json.loads((p/'candidate.json').read_text()) if (p/'candidate.json').exists() else None
 if plan is None:results.append({'asset':asset,'no_selected_full7_plan':True});continue
 old=ROOT/'results/door_transfer4_20261012'/asset;job=json.loads((old/'planning_job.json').read_text()) if (old/'planning_job.json').exists() else json.loads((old/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());c['camera_calibration']=job['camera_calibration'];model=NeroModel(c,job['asset_root'],source);exp=json.loads(Path(job['collision_export']).read_text());allowed=authored_handle_colliders(exp,job['grasp_geometry_candidates'][0]);base=plan['base'];T=np.array(plan['T_grasp']);seed=np.array(plan['path'][0]['q']);width=plan['contact_geometry_audit']['physical_aperture_m'];sc=scene_factory(ROOT,at_base(exp,source['robot_base_pose'],base),model,allowed);variants=[]
 for value,label in [(-.6,'predefined'),(0.,'predefined'),(.6,'predefined'),(float(seed[2]),'matched_full7_grasp')]:
  solver=RobotPinkIK(c,{'joint3':value});q=solver.solve(T,base,seed,starts=4);path=[];blocker='NO_INITIAL_IK';first=None
  if q is not None:
   for degree in range(31):
    target=model.door_target(T,np.deg2rad(degree));qnew=solver.solve(target,base,q)
    if qnew is None:blocker='NO_CONTINUOUS_PINK_IK';first=degree;break
    q=qnew;ok,why=sc.check(model.poses(q,base,width=width),model.door_target(sc.moving_reference,np.deg2rad(degree)),True)
    if not ok:blocker=why;first=degree;break
    path.append({'angle_deg':degree,'q':q.tolist(),'margin_rad':model.margin(q)})
   if len(path)==31:blocker=None
  variants.append({'joint3_fixed_rad':value,'source':label,'range_deg':path[-1]['angle_deg'] if path else None,'initial_IK':q is not None,'blocker':blocker,'first_blocked_deg':first,'path':path,'same_base_and_native_tool':True})
 result={'asset':asset,'full7_range_deg':float(np.rad2deg(plan['planned_reached_state'])),'base':base,'T_grasp':T.tolist(),'variants':variants,'not_an_actual_PiPER_model':True};results.append(result);(p/'REDUNDANCY_ABLATION.json').write_text(json.dumps(result,indent=2))
(O/'FOUR_OBJECT_ABLATION.json').write_text(json.dumps(results,indent=2))
probe=O/'integration_probe';r=[]
for p in sorted((probe/'observations_chunks').glob('*.gz')):r.extend(pickle.load(gzip.open(p,'rb')))
m=json.loads((probe/'APPROXIMATE_ACTUATOR.json').read_text());idx=m['arm_native_indices'];q=np.array([x['q'] for x in r])[:,idx];dq=np.array([x['dq'] for x in r])[:,idx];dt=1/240;fd=np.diff(q,axis=0)/dt;velocity={'joint_names':[m['joint_names_native_order'][i] for i in idx],'rms_native_velocity_minus_position_difference':np.sqrt(np.mean((dq[1:]-fd)**2,axis=0)).tolist(),'rms_native_velocity_plus_position_difference':np.sqrt(np.mean((dq[1:]+fd)**2,axis=0)).tolist(),'native_velocity_peak_rad_s':abs(dq).max(0).tolist(),'finite_difference_velocity_peak_rad_s':abs(fd).max(0).tolist(),'interpretation':'instantaneous solver velocity and post-integration pose difference can disagree during projection/oscillation; this alone does not identify a firmware sign error','self_contact_measurement_missing':True};(probe/'VELOCITY_DIAGNOSTIC.json').write_text(json.dumps(velocity,indent=2))
t=np.array([x['t'] for x in r]);desired=np.array([x['q_des_arm'] for x in r]);fig,ax=plt.subplots(2,2,figsize=(12,7));ax[0,0].plot(t,q-desired);ax[0,0].set_ylabel('joint tracking error (rad)');ax[0,0].set_title('No-object approximate servo: did not hold/track');ax[0,1].plot(t,dq);ax[0,1].set_ylabel('native joint velocity (rad/s)');ax[1,0].plot(t,[x['aperture_m']*1000 for x in r]);ax[1,0].set_ylabel('measured aperture (mm)');Ttcp=np.array([x['T_tcp'] for x in r]);speed=np.linalg.norm(np.diff(Ttcp[:,:3,3],axis=0),axis=1)/dt;ax[1,1].plot(t[1:],speed*1000);ax[1,1].axhline(5,c='r',ls='--',label='future loaded TCP protection');ax[1,1].legend();ax[1,1].set_ylabel('measured TCP speed (mm/s)')
for a in ax.flat:a.set_xlabel('simulation time (s)');a.grid(alpha=.2)
fig.suptitle('NERO ROBOT-ONLY INTEGRATION PROBE — 240 Hz evidence, no door contact\nFinite-effort approximate actuator; not hardware-equivalent dynamics');fig.tight_layout(rect=[0,0,1,.92]);fig.savefig(O/'INTEGRATION_PROBE_DIAGNOSTIC.png',dpi=150)
print([(x['asset'],[v['range_deg'] for v in x.get('variants',[])]) for x in results]);print(velocity)
