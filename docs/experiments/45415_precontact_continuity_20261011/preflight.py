"""Archived diagnosis and mathematical replay only; does not initialize Isaac."""
from pathlib import Path
import json,hashlib,gzip,pickle,sys,numpy as np,pinocchio as pin
from scipy.spatial.transform import Rotation
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from piper_pink_fixed import PinkIK
from precontact_trajectory import segment_samples,repair_initial_sequence
from articulated_demo.kinematics import URDFChain,transform
out=ROOT/'results/45415_precontact_continuity_20261011';old=ROOT/'results/45415_closure_verification_20261011';job=json.loads((out/'job.json').read_text());plan=json.loads((out/'whole_plan.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=[]
for f in sorted((old/'physical_01/observations_chunks').glob('*.gz'))[:3]:rows.extend(pickle.load(gzip.open(f,'rb')))
ik=PinkIK(ROOT/'config/piper.urdf',[0,1.15,-1.35,0,.2,0]);data=ik.robot.createData();frame=ik.robot.getFrameId('tcp_link');B=transform(plan['base'][:3],[0,0,np.deg2rad(plan['base'][3])]);chain=URDFChain(ROOT/'config/piper.urdf')
def fk(q):
 pin.framesForwardKinematics(ik.robot,data,np.array(q));return B@data.oMf[frame].homogeneous.copy()
def jac(q):return pin.computeFrameJacobian(ik.robot,data,np.array(q),frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).copy()
def speed(a,b):return np.linalg.norm(np.array(b['T_tcp'])[:3,3]-np.array(a['T_tcp'])[:3,3])*240
pre=max([(speed(a,b),a,b) for a,b in zip(rows,rows[1:]) if b['phase']=='PREGRASP'],key=lambda t:t[0]);entry=next(i for i,r in enumerate(rows) if r['phase']=='SLOW_CLOSE');a,b=rows[entry-1:entry+1]
# Distinct tests: URDF-vs-Pin FK, Jacobian-vs-central finite differences,
# simulator feedback-vs-FK, and commanded velocity-vs-Jacobian prediction.
fk_errors=[];jac_errors=[];sim_errors=[]
for r in rows[::37]:
 q=np.array(r['q'][:6]);E=chain.root_to_link('tcp_link',dict(zip([f'joint{i}' for i in range(1,7)],q)));F=fk(q);fk_errors.append(np.max(abs(F-B@E)));sim_errors.append(np.linalg.norm(F[:3,3]-np.array(r['T_tcp'])[:3,3]))
 J=jac(q);FD=np.zeros((6,6));eps=1e-6
 for j in range(6):
  qp=q.copy();qm=q.copy();qp[j]+=eps;qm[j]-=eps;Ep=np.linalg.inv(B)@fk(qp);Em=np.linalg.inv(B)@fk(qm)
  FD[:3,j]=(Ep[:3,3]-Em[:3,3])/(2*eps);FD[3:,j]=Rotation.from_matrix(Ep[:3,:3]@Em[:3,:3].T).as_rotvec()/(2*eps)
 jac_errors.append(np.max(abs(J-FD)))
old_cmd=np.array(a['command_q_arm']);measured=np.array(a['q'][:6]);T=np.array(a['T_tcp']);raw=ik.solve(T,plan['base'],measured);offset=old_cmd-measured;fixed=raw+offset
baseline_snap=float(np.linalg.norm(fk(raw)[:3,3]-fk(old_cmd)[:3,3])*240);fixed_snap=float(np.linalg.norm(fk(fixed)[:3,3]-fk(old_cmd)[:3,3])*240)
segments=[];start=np.array(next(r['command_q_arm'] for r in rows if r['phase']=='PREGRASP'));path=plan['preplan'][1:]+[plan['approach'][0]]+plan['approach'][1:]
for i,end in enumerate(path):
 count_pre=len(plan['preplan'])-1;is_approach=i>count_pre;minimum=1.5 if i<count_pre else (.8 if i==count_pre else .12);cruise=.003 if is_approach else .030
 samples=list(segment_samples(ik,start,end,minimum,1/240,cruise));positions=np.array([start]+[q for q,v in samples]);velocities=np.array([np.zeros(6)]+[v for q,v in samples]);tcp=np.array([fk(q)[:3,3] for q in positions]);finite_speed=np.linalg.norm(np.diff(tcp,axis=0),axis=1)*240;dq=np.diff(positions,axis=0)*240
 segments.append({'index':i,'phase':'APPROACH' if is_approach else 'PREGRASP','duration_s':len(samples)/240,'max_command_TCP_m_s':float(max(finite_speed)),'max_command_joint_velocity_rad_s':float(np.max(abs(velocities))),'finite_difference_vs_analytic_velocity_rad_s':float(np.max(abs(dq-(velocities[:-1]+velocities[1:])/2))),'start_velocity_rad_s':velocities[0].tolist(),'end_velocity_rad_s':velocities[-1].tolist(),'exact_endpoint_error_rad':float(np.max(abs(positions[-1]-end)))})
 start=np.array(end)
text=Path(job['baseline_runner']).read_text();sequence=text.split("try:\n phase='SETTLE';hold(.5)",1)[1];initial="if True:\n phase='SETTLE';hold(.5)"+sequence.split("  profile_mark('OPEN_1')",1)[0];fixed_sequence=repair_initial_sequence(initial);compile(fixed_sequence,'corrected_initial_sequence','exec')
pink=Path(job['pink_runner']).read_text();opening='def body_at(theta):'+pink.split('def body_at(theta):',1)[1].split('initial_grasp=False',1)[0];runner=ROOT/'scripts/run_oracle_door_precontact_continuity.py'
peak_r=pre[2];peak_q=np.array(peak_r['q'][:6]);J=jac(peak_q);pred=float(np.linalg.norm(J[:3]@np.array(peak_r['telemetry']['command_q_velocity_arm'])))
result={'classification':'OFFLINE_ONLY_NO_PHYSICAL_TRIAL','physical_trials_consumed':0,'runner_path':str(runner),'runner_sha256':sha(runner),'helper_sha256':sha(ROOT/'scripts/precontact_trajectory.py'),'active_closure_sha256':sha(job['active_closure_source']),'unchanged_opening_controller_excerpt_sha256':hashlib.sha256(opening.encode()).hexdigest(),'diagnosis':{'PREGRASP':{'actual_peak_m_s':pre[0],'t':peak_r['t'],'jacobian_predicted_command_TCP_m_s':pred,'adjacent_speeds_m_s':[speed(rows[i-1],rows[i]) for i in range(max(1,rows.index(peak_r)-2),rows.index(peak_r)+3)],'cause':'Broad mid-segment fast motion: joint-rate-only cubic timing permits high Cartesian speed; not an isolated command handover spike.'},'SLOW_CLOSE_entry':{'actual_peak_m_s':speed(a,b),'t':b['t'],'command_equilibrium_offset_rad':offset.tolist(),'old_command_FK_snap_m_s':baseline_snap,'fixed_command_FK_snap_m_s':fixed_snap,'cause':'Measured-state IK resets gravity-deflected position equilibrium; one-step delta creates velocity-feedforward impulse. Preserve the acquired command-minus-feedback offset while still seeding IK from feedback.'}},'FK_and_Jacobian':{'max_URDF_vs_Pinocchio_matrix_error':max(fk_errors),'max_Jacobian_vs_central_difference_error':max(jac_errors),'max_simulator_TCP_vs_feedback_FK_position_error_m':max(sim_errors),'samples':len(fk_errors)},'trajectory_replay':{'segments':segments,'max_command_TCP_m_s':max(s['max_command_TCP_m_s'] for s in segments),'duration_s':sum(s['duration_s'] for s in segments),'same_joint_endpoints_and_branch':True,'quintic_zero_velocity_and_acceleration_at_segment_boundaries':True,'not_physical_validation':True},'replay':{'fixed_reopenings':0,'closure_replay_reused_from_frozen_verified_baseline':True},'source_provenance':{'frozen_inputs_unchanged':all(sha(p)==h for p,h in json.loads((out/'FROZEN_INPUT_HASHES.json').read_text()).items()),'whole_plan_identical':sha(out/'whole_plan.json')==sha(old/'whole_plan.json'),'grasp_plan_identical':sha(out/'grasp_plan.json')==sha(old/'grasp_plan.json')},'offline_supported':False}
result['offline_supported']=bool(result['runner_sha256']==job['expected_runner_sha256'] and result['source_provenance']['frozen_inputs_unchanged'] and max(jac_errors)<1e-7 and max(fk_errors)<1e-9 and fixed_snap<baseline_snap and all(s['exact_endpoint_error_rad']==0 for s in segments))
(out/'PREFLIGHT.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='trajectory_replay'},indent=2));print('TRAJECTORY',result['trajectory_replay']['duration_s'],result['trajectory_replay']['max_command_TCP_m_s'])
