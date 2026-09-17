"""Evaluation-only torque decomposition; never imported by production estimation.
GT is used solely for independent synthetic recovery and residual diagnosis.
"""
import sys,json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from real2sim.fr3_robot_calibration import make_fr3_arm_urdf,_make_drake_plant,DEFAULT_FR3_URDF,DEFAULT_OFFICIAL_ROOT,_json_safe
from real2sim.payload_id_v2 import static_gravity_regressor
from real2sim.payload_id_official_mass_com import _torch_fit_mass_com_fixed_mass,_load_official_filtering,_official_process_torque
from pydrake.all import JacobianWrtVariable

def find(v,k):
 if isinstance(v,dict):
  if k in v:return v[k]
  for x in v.values():
   z=find(x,k)
   if z is not None:return z
 if isinstance(v,list):
  for x in v:
   z=find(x,k)
   if z is not None:return z

def load(p):
 with np.load(p,allow_pickle=True) as f:return {k:f[k] for k in f.files}
def fit(X,y,gt):
 f=_torch_fit_mass_com_fixed_mass(X,y,DEFAULT_OFFICIAL_ROOT)
 return {'mass_kg':f['mass_kg'],'COM_TCP_m':f['center_of_mass_m'],'COM_error_mm':float(np.linalg.norm(np.array(f['center_of_mass_m'])-gt)*1000),'residual_RMS_Nm':f['residual_rms_Nm'],'observability':f['observability'],'uncertainty':{k:v for k,v in f.items() if 'sigma' in k or 'covariance' in k or 'uncertainty' in k}}
def main():
 out=ROOT/'results/fr3_torque_rootcause';out.mkdir(exist_ok=True)
 plant,_=_make_drake_plant(make_fr3_arm_urdf(DEFAULT_FR3_URDF,out/'diagnostic_fr3.urdf'));ctx=plant.CreateDefaultContext();wf=plant.world_frame();tf=plant.GetFrameByName('fr3_hand_tcp')
 def kin(q,c,m):
  plant.SetPositions(ctx,q)
  X=plant.CalcRelativeTransform(ctx,wf,tf);R=X.rotation().matrix()
  # Independent COM-point Jacobian: no production Y or first moment conversion.
  Jc=plant.CalcJacobianTranslationalVelocity(ctx,JacobianWrtVariable.kV,tf,c,wf,wf)
  Jv=plant.CalcJacobianTranslationalVelocity(ctx,JacobianWrtVariable.kV,tf,np.zeros(3),wf,wf)
  Jw=plant.CalcJacobianAngularVelocity(ctx,JacobianWrtVariable.kV,tf,wf,wf)
  return Jc.T@np.array([0,0,m*9.81]),R,np.vstack([Jv,Jw]),X.translation()
 cases={'soup':'payload_mass_com_diagnostic_v7_part_b/soup/attempt_01_seed1031','mug':'payload_mass_com_diagnostic_v7_part_b/mug/attempt_04_seed1039','banana':'payload_mass_com_diagnostic_v7_part_b/banana/attempt_01_seed1021','mustard':'payload_mass_com_diagnostic_v7_mustard_fresh/mustard/attempt_02_seed1002'}
 filt,proc=_load_official_filtering(DEFAULT_OFFICIAL_ROOT);summary={}
 for name,rel in cases.items():
  run=ROOT/'results'/rel/'pairs/normal';b=load(run/'system_id_baseline_static.npz');p=load(run/'system_id_payload_static.npz')
  trial=json.load(open(next(run.glob('*_ANYGRASP.json'))));m=float(np.asarray(find(trial,'target_mass_kg')).ravel()[0]);cl=np.asarray(find(trial,'target_COM_local')).ravel()[:3]
  stale=np.asarray(find(trial,'target_COM_TCP_m')).ravel()[:3]
  center=json.load(open(run/'excitation_center.json'));opening=float(np.sum(center['finger_q'])*1000)
  bt,_=_official_process_torque(b,proc,filt);pt,_=_official_process_torque(p,proc,filt)
  Y=static_gravity_regressor(p); rows=[];Xlist=[];dlist=[];glist=[];dexact=[];glive=[];comlist=[];fixedlist=[]
  hand_com=np.fromstring(ET.parse(DEFAULT_FR3_URDF).getroot().find("link[@name='fr3_hand']/inertial/origin").attrib['xyz'],sep=' ')
  corrected=dict(p);JC=p['jacobian_TCP'].copy();offset=p['T_B_hand'][:,:3,:3]@hand_com
  for j,v in enumerate(offset):
   sk=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);JC[j,:3]+=sk@JC[j,3:]
  corrected['jacobian_TCP']=JC;Yfix=static_gravity_regressor(corrected)
  for pid in np.unique(p['static_pose_id'][p['static_hold'].astype(bool)]):
   if pid<0:continue
   ii=np.flatnonzero((p['static_pose_id']==pid)&p['static_hold'].astype(bool));jj=np.flatnonzero((b['static_pose_id']==pid)&b['static_hold'].astype(bool))
   if not len(jj):continue
   # Last 200 ms only; retain rejection explicitly.
   ii=ii[p['t'][ii]>=p['t'][ii[-1]]-.2];jj=jj[b['t'][jj]>=b['t'][jj[-1]]-.2]
   q=p['q'][ii].mean(0);qe=b['q'][jj].mean(0);T=p['T_TCP_object'][ii];cc=np.einsum('nij,j->ni',T[:,:3,:3],cl)+T[:,:3,3];c=cc.mean(0)
   drift=np.max(np.linalg.norm(cc-cc[0],axis=1));rot=max(Rotation.from_matrix(T[0,:3,:3].T@t[:3,:3]).magnitude() for t in T)
   speed=max(np.abs(p['dq'][ii]).max(),np.abs(b['dq'][jj]).max());qerr=float(np.abs(q-qe).max())
   reason=[]
   if drift>.003 or rot>np.deg2rad(5) or np.any(p['relative_motion_violation'][ii]):reason.append('RELATIVE_SLIP')
   if qerr>.003:reason.append('ACTUAL_Q_MISMATCH')
   if speed>.01:reason.append('NOT_STATIC')
   gt,R,J,pos=kin(q,c,m);ref,R,J,pos=kin(q,stale,m)
   temp={'T_B_TCP':np.eye(4)[None].copy(),'jacobian_TCP':J[None]};temp['T_B_TCP'][0,:3,:3]=R;Xd=static_gravity_regressor(temp)[0]
   Xm=Y[ii].mean(0);delta=pt[ii].mean(0)-bt[jj].mean(0)
   r={'pose_id':int(pid),'used':not reason,'rejection':reason,'q_loaded':q,'q_empty':qe,'q_pair_max_error_rad':qerr,'dq_max_rad_s':float(speed),'opening_command_mm':opening,'actual_opening_mm':None,'actual_opening_status':'NOT_RECORDED_IN_LEGACY_STREAM','tau_empty':bt[jj].mean(0),'tau_loaded':pt[ii].mean(0),'delta_tau_measured':delta,'tau_payload_GT':gt,'torque_residual':delta-gt,'gravity_TCP':R.T@np.array([0,0,-9.81]),'Y_COM':Xm[:,1:],'Y_mass_COM':Xm,'COM_GT_TCP_current':c,'COM_GT_TCP_trial':stale,'GT_reference_shift_mm':np.linalg.norm(c-stale)*1000,'jacobian_max_error':np.abs(J-p['jacobian_TCP'][ii].mean(0)).max(),'TCP_position_error_m':np.linalg.norm(pos-p['T_B_TCP'][ii,:3,3].mean(0))}
   rows.append(r)
   if qerr<=.003:Xlist.append(Xm);dlist.append(delta);glist.append(ref);dexact.append(Xd);glive.append(gt);comlist.append(c);fixedlist.append(Yfix[ii].mean(0))
  result={'source':str(run),'evaluation_only':True,'GT_never_used_by_production':True,'poses':rows,'accepted_static_poses':sum(r['used'] for r in rows),'diagnostic_only_pose_count':len(Xlist),'rejected_static_poses':sum(not r['used'] for r in rows),'mass_GT_kg':m}
  if len(Xlist)>=2:
   XX=np.concatenate(Xlist);dd=np.concatenate(dlist);gg=np.concatenate(glist);XD=np.concatenate(dexact);live=np.concatenate(glive);XF=np.concatenate(fixedlist);cg=np.mean(comlist,axis=0)
   result.update(synthetic_independent_Drake=fit(XD,gg,stale),synthetic_cross_Isaac=fit(XX,gg,stale),synthetic_corrected_Isaac=fit(XF,gg,stale),measured_corrected=fit(XF,dd,cg),measured=fit(XX,dd,cg),GT_COM_current_mean=cg,GT_COM_trial=stale,GT_COM_shift_mm=float(np.linalg.norm(cg-stale)*1000),residual_RMS_Nm=float(np.sqrt(np.mean((dd-live)**2))),per_joint_residual_mean_Nm=np.mean((dd-live).reshape(-1,7),axis=0),per_joint_residual_RMS_Nm=np.sqrt(np.mean((dd-live).reshape(-1,7)**2,axis=0)))
   rr=(dd-live).reshape(-1,7);grav=np.array([r['gravity_TCP'] for r in rows if r['q_pair_max_error_rad']<=.003]);design=np.c_[np.ones(len(grav)),grav];result['gravity_bias_in_sample_R2']=1-np.sum((rr-design@np.linalg.lstsq(design,rr,rcond=None)[0])**2)/max(np.sum((rr-rr.mean(0))**2),1e-20)
  (out/(name+'.json')).write_text(json.dumps(_json_safe(result),indent=2));summary[name]={k:v for k,v in result.items() if k!='poses'};print(name,json.dumps(_json_safe(summary[name])),flush=True)
 (out/'summary.json').write_text(json.dumps(_json_safe(summary),indent=2))
if __name__=='__main__':main()
