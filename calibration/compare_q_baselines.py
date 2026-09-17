"""Offline diagnostic only: no production gate or estimator changes, no payload GT in corrections."""
import sys,json,copy,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from diagnose_fr3_torque_rootcause import find,load
from real2sim.fr3_robot_calibration import DEFAULT_FR3_URDF,DEFAULT_OFFICIAL_ROOT,make_fr3_arm_urdf,_make_drake_plant,_json_safe
from real2sim.payload_jacobian import tcp_jacobian_record
from real2sim.payload_id_v2 import static_gravity_regressor
from real2sim.payload_id_official_mass_com import _load_official_filtering,_official_process_torque,_torch_fit_mass_com_fixed_mass
from quasistatic_velocity_gate import configuration_velocity
from pydrake.all import JacobianWrtVariable
O=R/'results/fr3_q_compensation';O.mkdir(exist_ok=True)
flt,proc=_load_official_filtering(DEFAULT_OFFICIAL_ROOT)
def plant_at_opening(w):
 f=O/f'robot_opening_{w:.8f}.urdf';make_fr3_arm_urdf(DEFAULT_FR3_URDF,f);root=ET.parse(f).getroot();orig=ET.parse(DEFAULT_FR3_URDF).getroot()
 for name in ['fr3_leftfinger','fr3_rightfinger']:
  root.append(copy.deepcopy(orig.find(f"link[@name='{name}']")))
 for jn in ['fr3_finger_joint1','fr3_finger_joint2']:
  j=copy.deepcopy(orig.find(f"joint[@name='{jn}']"));j.set('type','fixed');origin=j.find('origin');xyz=np.fromstring(origin.get('xyz'),sep=' ');rot=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' '));axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');origin.set('xyz',' '.join(map(str,xyz+rot.apply(axis*w/2))))
  for x in list(j):
   if x.tag in ['axis','limit','dynamics','mimic']:j.remove(x)
  root.append(j)
 ET.ElementTree(root).write(f);plant,model=_make_drake_plant(f);return plant,model,plant.CreateDefaultContext()
def window(path,pid=None):
 d=load(path);idx=np.arange(len(d['t']))
 if pid is not None:idx=idx[(d['static_pose_id']==pid)&d['static_hold'].astype(bool)]
 if len(idx)<2:return None
 idx=idx[d['t'][idx]>=d['t'][idx[-1]]-.5];reason=[]
 if len(idx)<96:reason.append('SHORT_WINDOW')
 try:v=float(np.max(np.abs(configuration_velocity(d['t'][idx],d['q'][idx]))))
 except ValueError:v=float('inf');reason.append('BAD_TIMESTAMPS')
 if v>=.002:reason.append('NONZERO_DQ')
 tau=d['tau'][idx];h=len(tau)//2;drift=float(np.max(np.abs(tau[:h].mean(0)-tau[h:].mean(0))))
 if drift>=.05:reason.append('TORQUE_TRANSIENT')
 filtered,_=_official_process_torque(d,proc,flt)
 return dict(d=d,idx=idx,q=d['q'][idx].mean(0),tau=filtered[idx].mean(0),velocity=v,torque_drift=drift,reasons=reason,path=str(path),opening=float(np.mean(d['actual_opening_m'][idx])) if 'actual_opening_m' in d else None)
def main():
 cases={};base=R/'results/fr3_clean_pair_fd_gate';repair=base/'matched_opening_repair/banana';cap=json.load(open(repair/'payload/strict_capture.json'));ec=json.load(open(repair/'empty/strict_capture.json'))
 cases['banana']=(repair/'payload',[(r['pose_id'],Path(next(x for x in ec['accepted'] if x['pose_id']==r['pose_id'])['path']),Path(r['path']),None) for r in cap['accepted']])
 m=R/'results/fr3_torque_rootcause/fresh/mug';cases['mug']=(m/'payload',[(int(f.stem[-2:]),m/'empty'/f.name.replace('payload','baseline'),f,None) for f in sorted((m/'payload').glob('diagnostic_payload_pose*.npz'))])
 for n,rel in [('soup','payload_mass_com_diagnostic_v7_part_b/soup/attempt_01_seed1031/pairs/normal'),('mustard','payload_mass_com_diagnostic_v7_mustard_fresh/mustard/attempt_02_seed1002/pairs/normal')]:
  folder=R/'results'/rel;p=load(folder/'system_id_payload_static.npz');ids=np.unique(p['static_pose_id'][p['static_hold'].astype(bool)]);cases[n]=(folder,[(int(i),folder/'system_id_baseline_static.npz',folder/'system_id_payload_static.npz',int(i)) for i in ids if i>=0])
 summary={}
 for name,(folder,pairs) in cases.items():
  rows=[];accepted=[];reference=None
  for pid,bp,pp,legacy in pairs:
   if not bp.exists():continue
   b=window(bp,legacy);p=window(pp,legacy)
   if b is None or p is None:continue
   reasons=b['reasons']+p['reasons'];d=p['d'];ix=p['idx'];T=d['T_TCP_object'][ix];rel=np.linalg.inv(T[0])[None]@T;slip=float(np.max(np.linalg.norm(rel[:,:3,3],axis=1)));angle=float(np.max(Rotation.from_matrix(rel[:,:3,:3]).magnitude()))
   if slip>.003 or angle>np.deg2rad(5) or np.any(d['relative_motion_violation'][ix]):reasons.append('RELATIVE_SLIP')
   if not np.asarray(d['guard_passed']).all():reasons.append('CONTACT_GUARD')
   if b['opening'] is None or p['opening'] is None:reasons.append('MISSING_ACTUAL_OPENING')
   elif abs(b['opening']-p['opening'])>=.0002:reasons.append('OPENING_MISMATCH')
   opening=p['opening'] if p['opening'] is not None else float(sum(json.load(open(folder/'excitation_center.json'))['finger_q']))
   plant,model,ctx=plant_at_opening(opening)
   # Static actuator torque counters gravity. Payload is absent from this model.
   def robot_tau(q):plant.SetPositions(ctx,model,q);return -plant.CalcGravityGeneralizedForces(ctx)
   te=robot_tau(b['q']);tl=robot_tau(p['q']);correction=tl-te
   corrected=tcp_jacobian_record(d);Y=static_gravity_regressor(corrected)[ix].mean(0)
   row=dict(pose_id=pid,empty_path=str(bp),payload_path=str(pp),q_empty=b['q'],q_loaded=p['q'],q_mismatch_rad=p['q']-b['q'],q_mismatch_max_rad=float(np.max(np.abs(p['q']-b['q']))),dq_empty_max=b['velocity'],dq_loaded_max=p['velocity'],opening_empty=b['opening'],opening_loaded=p['opening'],tau_empty=b['tau'],tau_loaded=p['tau'],tau_robot_empty=te,tau_robot_loaded=tl,robot_q_correction=correction,raw_delta=p['tau']-b['tau'],corrected_delta=p['tau']-b['tau']-correction,Y=Y,T=T.mean(0),slip_m=slip,rotation_rad=angle,reasons=reasons,model_opening_source='ACTUAL' if p['opening'] is not None else 'COMMAND_ONLY_DIAGNOSTIC')
   rows.append(row)
   if not reasons:
    if reference is None:reference=T[0]
    rr=np.linalg.inv(reference)[None]@T
    if np.max(np.linalg.norm(rr[:,:3,3],axis=1))>.003 or np.max(Rotation.from_matrix(rr[:,:3,:3]).magnitude())>np.deg2rad(5):row['reasons'].append('GLOBAL_SLIP')
    else:accepted.append(row)
  result={'rows':rows,'accepted_poses':len(accepted),'rejection_counts':{k:sum(k in x['reasons'] for x in rows) for k in set(k for x in rows for k in x['reasons'])},'q_correction_all_pose_RMS_Nm':float(np.sqrt(np.mean([x['robot_q_correction']**2 for x in rows]))) if rows else None,'GT_in_correction_or_selection':False}
  if len(accepted)>=4:
   X=np.concatenate([x['Y'] for x in accepted]);methods={}
   # Fit before any payload GT is read. Pure empty affine baseline, with leave-one-pose-out audit.
   Q=np.array([x['q_empty'] for x in accepted]);tau=np.array([x['tau_empty'] for x in accepted]);q0=Q.mean(0);design=np.c_[np.ones(len(Q)),(Q-q0)/.01];rank=int(np.linalg.matrix_rank(design));pred=np.c_[np.ones(len(Q)),(np.array([x['q_loaded'] for x in accepted])-q0)/.01]@np.linalg.lstsq(design,tau,rcond=1e-8)[0]
   cv=[]
   for i in range(len(Q)):
    keep=np.arange(len(Q))!=i;cv.append(design[i]@np.linalg.lstsq(design[keep],tau[keep],rcond=1e-8)[0]-tau[i])
   result['local_empty_audit']={'training_points':len(Q),'affine_rank':rank,'dimensions':8,'heldout_pose_RMS_Nm':float(np.sqrt(np.mean(np.array(cv)**2))),'status':'INSUFFICIENT_LOCAL_SUPPORT' if rank<8 or len(Q)<10 else 'DIAGNOSTIC_ONLY','uses_payload_torque':False}
   for label,y in [('raw',np.concatenate([x['raw_delta'] for x in accepted])),('drake',np.concatenate([x['corrected_delta'] for x in accepted])),('empty_affine_exploratory',(np.array([x['tau_loaded'] for x in accepted])-pred).ravel())]:
    try:fit=_torch_fit_mass_com_fixed_mass(X,y,DEFAULT_OFFICIAL_ROOT)
    except ValueError as exc:fit={'accepted':False,'reason':str(exc)}
    methods[label]={'fit':fit,'torque':y}
   # Evaluation boundary: no GT has been used for gates, baseline or fits above.
   trial=json.load(open(next(folder.glob('*_ANYGRASP.json'))));mass=float(np.asarray(find(trial,'target_mass_kg')).ravel()[0]);cl=np.asarray(find(trial,'target_COM_local')).ravel()[:3];gt=[];com=[]
   plant,model,ctx=plant_at_opening(float(np.mean([x['opening_loaded'] for x in accepted])));W=plant.world_frame();tcp=plant.GetFrameByName('fr3_hand_tcp')
   for row in accepted:
    c=row['T'][:3,:3]@cl+row['T'][:3,3];com.append(c);plant.SetPositions(ctx,model,row['q_loaded']);J=plant.CalcJacobianTranslationalVelocity(ctx,JacobianWrtVariable.kV,tcp,c,W,W);g=J.T@np.array([0,0,mass*9.81]);gt.append(g);row['GT_torque_diagnostic']=g
   com=np.mean(com,axis=0);gt=np.array(gt)
   for label,m in methods.items():
    residual=m.pop('torque').reshape(-1,7)-gt;f=m['fit'];m.update(mass_error_percent=abs(f['mass_kg']-mass)/mass*100 if 'mass_kg' in f else None,COM_error_mm=float(np.linalg.norm(np.array(f['center_of_mass_m'])-com)*1000) if 'center_of_mass_m' in f else None,GT_residual_RMS_Nm=float(np.sqrt(np.mean(residual**2))),joint_bias=residual.mean(0),joint_RMS=np.sqrt(np.mean(residual**2,axis=0)))
   result['methods']=methods
  (O/f'{name}.json').write_text(json.dumps(_json_safe(result),indent=2));summary[name]={k:v for k,v in result.items() if k!='rows'};print(name,'accepted',len(accepted),result['rejection_counts'],flush=True)
 (O/'summary.json').write_text(json.dumps(_json_safe(summary),indent=2))
if __name__=='__main__':main()
