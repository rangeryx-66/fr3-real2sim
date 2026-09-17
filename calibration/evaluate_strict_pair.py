"""Strict clean-pair results; GT never used for acceptance or measured fitting."""
import json,sys,hashlib,os
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from real2sim.fr3_robot_calibration import DEFAULT_FR3_URDF,DEFAULT_OFFICIAL_ROOT,_make_drake_plant,make_fr3_arm_urdf,_json_safe
from real2sim.payload_id_official_mass_com import _torch_fit_mass_com_fixed_mass,_load_official_filtering,_official_process_torque
from real2sim.payload_id_v2 import static_gravity_regressor
from real2sim.payload_jacobian import tcp_jacobian_record
from diagnose_fr3_torque_rootcause import find,load
from strict_pair_protocol import POLICY
from quasistatic_velocity_gate import configuration_velocity
from pydrake.all import JacobianWrtVariable

def main():
 O=Path(os.environ.get('STRICT_PAIR_OUTPUT',str(R/'results/fr3_strict_pair_validation')));done=json.load(open(O/'complete.json'));summary={};flt,proc=_load_official_filtering(DEFAULT_OFFICIAL_ROOT)
 plant,_=_make_drake_plant(make_fr3_arm_urdf(DEFAULT_FR3_URDF,O/'evaluation_model.urdf'));ctx=plant.CreateDefaultContext();W=plant.world_frame();tcp=plant.GetFrameByName('fr3_hand_tcp')
 for run in done['rows']:
  name=run['target'];root=O/name;ep=root/'empty/strict_capture.json';pp=root/'payload/strict_capture.json';empty=json.load(open(ep)) if ep.exists() else {};payload=json.load(open(pp)) if pp.exists() else {};erows=empty.get('rows',[]);prows=payload.get('rows',[])
  counts={}
  for row in erows+prows:
   for reason in row.get('reasons',[]):counts[reason]=counts.get(reason,0)+1
  res={'target':name,'clean_empty_windows':len(empty.get('accepted',[])),'clean_payload_windows':len(payload.get('accepted',[])),'attempt_count':len(erows)+len(prows),'rejection_counts':counts,'status':'NO_CLEAN_PAIRED_DATA','fitting_performed':False,'GT_used_for_measured_fit':False,'run_status':run,'empty_dq_max_range':[min((r.get('dq_max_rad_s',float('inf')) for r in erows),default=None),max((r.get('dq_max_rad_s',0) for r in erows),default=None)]}
  accepted=payload.get('accepted',[])
  poses=[];Xs=[];ys=[];gs=[];coms=[]
  if accepted:
   trialpath=next((root/'payload').glob('*_ANYGRASP.json'));trial=json.load(open(trialpath));m=float(np.asarray(find(trial,'target_mass_kg')).ravel()[0]);cl=np.asarray(find(trial,'target_COM_local')).ravel()[:3]
   globalT=None
   for row in accepted:
    base=next(x for x in empty['accepted'] if x['pose_id']==row['pose_id']);p=tcp_jacobian_record(load(row['path']));b=load(base['path']);pi=np.asarray(p['t'])>=p['t'][-1]-.5;bi=np.asarray(b['t'])>=b['t'][-1]-.5
    if globalT is None:globalT=p['T_TCP_object'][0]
    from scipy.spatial.transform import Rotation
    rel=np.linalg.inv(globalT)[None]@p['T_TCP_object'];drift=float(np.linalg.norm(rel[:,:3,3],axis=1).max());angle=float(np.max(Rotation.from_matrix(rel[:,:3,:3]).magnitude()))
    if drift>.003 or angle>np.deg2rad(5):poses.append({'pose_id':row['pose_id'],'rejected':'GLOBAL_RELATIVE_SLIP','translation_m':drift,'angle_rad':angle});continue
    q=p['q'][pi].mean(0);qb=b['q'][bi].mean(0);pair=float(np.max(np.abs(q-qb)))
    if pair>=POLICY['q_pair_max_rad']:raise RuntimeError('accepted pair violates q threshold')
    pt,_=_official_process_torque(p,proc,flt);bt,_=_official_process_torque(b,proc,flt);delta=pt[pi].mean(0)-bt[bi].mean(0);Y=static_gravity_regressor(p)[pi].mean(0);T=p['T_TCP_object'][pi];c=np.mean(T[:,:3,:3]@cl+T[:,:3,3],axis=0)
    plant.SetPositions(ctx,q);RT=plant.CalcRelativeTransform(ctx,W,tcp).rotation().matrix();Jc=plant.CalcJacobianTranslationalVelocity(ctx,JacobianWrtVariable.kV,tcp,c,W,W);gt=Jc.T@np.array([0,0,m*9.81]);poses.append({'pose_id':row['pose_id'],'q_ref':row['q_ref'],'q_empty':qb,'q_loaded':q,'dq_empty':b['dq'][bi].mean(0),'dq_loaded':p['dq'][pi].mean(0),'dq_empty_corrected':configuration_velocity(b['t'][bi],b['q'][bi]).mean(0),'dq_loaded_corrected':configuration_velocity(p['t'][pi],p['q'][pi]).mean(0),'dq_empty_corrected_max':np.max(np.abs(configuration_velocity(b['t'][bi],b['q'][bi]))),'dq_loaded_corrected_max':np.max(np.abs(configuration_velocity(p['t'][pi],p['q'][pi]))),'tau_empty':bt[bi].mean(0),'tau_loaded':pt[pi].mean(0),'delta_tau':delta,'tau_payload_GT':gt,'residual':delta-gt,'gravity_TCP':RT.T@np.array([0,0,-9.81]),'Y_COM':Y[:,1:],'T_TCP_object':T,'opening_empty_m':base['opening_mean_m'],'opening_payload_m':row['opening_mean_m'],'q_pair_max_error_rad':pair,'COM_GT_TCP':c});Xs.append(Y);ys.append(delta);gs.append(gt);coms.append(c)
   if len(Xs)>=POLICY['min_clean_poses']:
    X=np.concatenate(Xs);y=np.concatenate(ys);g=np.concatenate(gs);c=np.mean(coms,axis=0);f=_torch_fit_mass_com_fixed_mass(X,y,DEFAULT_OFFICIAL_ROOT);gref=[]
    for pr in poses:
     if 'q_loaded' not in pr:continue
     plant.SetPositions(ctx,pr['q_loaded']);JG=plant.CalcJacobianTranslationalVelocity(ctx,JacobianWrtVariable.kV,tcp,c,W,W);gref.append(JG.T@np.array([0,0,m*9.81]))
    syn=_torch_fit_mass_com_fixed_mass(X,np.concatenate(gref),DEFAULT_OFFICIAL_ROOT);res.update(status='CLEAN_PAIRED_FIT',fitting_performed=True,estimate=f,synthetic_GT_diagnostic=syn,mass_error_percent=abs(f['mass_kg']-m)/m*100,COM_error_mm=np.linalg.norm(np.array(f['center_of_mass_m'])-c)*1000,synthetic_COM_error_mm=np.linalg.norm(np.array(syn['center_of_mass_m'])-c)*1000,per_joint_residual_RMS_Nm=np.sqrt(np.mean((y-g).reshape(-1,7)**2,axis=0)),per_joint_residual_bias_Nm=np.mean((y-g).reshape(-1,7),axis=0))
  res['poses']=poses;(root/'result.json').write_text(json.dumps(_json_safe(res),indent=2));summary[name]={k:v for k,v in res.items() if k!='poses'}
 (O/'summary.json').write_text(json.dumps(_json_safe(summary),indent=2));print(json.dumps(_json_safe(summary),indent=2))
 lines=['# 修复后严格 q/dq paired COM 验证','', '|物体|clean empty 窗口|clean payload 窗口|正式拟合|COM error mm|','|---|---:|---:|---|---:|']
 for name,res in summary.items():lines.append(f"|{name}|{res['clean_empty_windows']}|{res['clean_payload_windows']}|{res['status']}|{res.get('COM_error_mm','N/A')}|")
 lines+=['','没有合格数据时不得报告 COM 精度或宣称 1.64 mm 已复现；使用已审计的 actual q 差分静态 gate，阈值仍为 0.002 rad/s。每个不合格姿态最多三次到位采样；empty 没有至少4个干净窗口则不抓取 payload。','', '门槛与所有生产源码 hash 见 frozen_protocol.json；每次采样及拒绝原因见各 empty/payload 的 strict_capture.json 与 NPZ。GT 只用于诊断、never用于验收/选择或实测拟合。','', '保持 mass identified（通过可信度检查时），COM/inertia geometry fallback，直至有足够合格证据。','', '生产冻结文件变化：'+str(done['frozen_files_changed'])]
 (O/'REPORT.md').write_text('\n'.join(lines));(O/'ANALYSIS_COMPLETE.json').write_text(json.dumps({'objects':list(summary),'formal_fits':sum(x['fitting_performed'] for x in summary.values()),'source_hash_unchanged':not done['frozen_files_changed']},indent=2))
if __name__=='__main__':main()
