"""Independent static torque residual diagnosis; no GT enters production fitting."""
from diagnose_fr3_torque_rootcause import *
from real2sim.fr3_robot_calibration import _make_drake_plant, _json_safe
from real2sim.payload_jacobian import tcp_jacobian_record

def main():
 out=ROOT/'results/fr3_torque_rootcause';fresh=out/'fresh'
 plant,_=_make_drake_plant(make_fr3_arm_urdf(DEFAULT_FR3_URDF,out/'diagnostic_fr3.urdf'));ctx=plant.CreateDefaultContext();wf=plant.world_frame();tf=plant.GetFrameByName('fr3_hand_tcp')
 summaries={}
 folders=list(fresh.iterdir())
 if (out/'fresh_retry').exists():folders+=list((out/'fresh_retry').iterdir())
 for folder in sorted(folders):
  if not folder.is_dir():continue
  pp=folder/'payload/system_id_payload_static.npz';bp=folder/'empty/system_id_baseline_static.npz'
  if not pp.exists() or not bp.exists():continue
  p=tcp_jacobian_record(load(pp));b=load(bp);trial=json.load(open(next((folder/'payload').glob('*_ANYGRASP.json'))));mass=float(np.asarray(find(trial,'target_mass_kg')).ravel()[0]);cl=np.asarray(find(trial,'target_COM_local')).ravel()[:3]
  YY=static_gravity_regressor(p);rows=[];xx=[];yy=[];gg=[];cc=[]
  for pid in np.unique(p['static_pose_id']):
   pi=np.flatnonzero(p['static_pose_id']==pid);bi=np.flatnonzero(b['static_pose_id']==pid)
   if not len(bi):continue
   pi=pi[p['t'][pi]>=p['t'][pi[-1]]-.3];bi=bi[b['t'][bi]>=b['t'][bi[-1]]-.3]
   q=p['q'][pi].mean(0);qb=b['q'][bi].mean(0);T=p['T_TCP_object'][pi];coms=T[:,:3,:3]@cl+T[:,:3,3];com=coms.mean(0);plant.SetPositions(ctx,q);X=plant.CalcRelativeTransform(ctx,wf,tf);R=X.rotation().matrix();Jc=plant.CalcJacobianTranslationalVelocity(ctx,JacobianWrtVariable.kV,tf,com,wf,wf);gt=Jc.T@np.array([0,0,mass*9.81]);delta=p['tau'][pi].mean(0)-b['tau'][bi].mean(0);XM=YY[pi].mean(0)
   w=float(p['actual_opening_m'][pi].mean());wb=float(b['actual_opening_m'][bi].mean());qerr=float(np.abs(q-qb).max());drift=float(np.linalg.norm(coms-coms[0],axis=1).max());reasons=[]
   if qerr>.003:reasons.append('Q_PAIR_MISMATCH')
   if abs(w-wb)>.0002:reasons.append('OPENING_MISMATCH')
   if np.any(p['relative_motion_violation'][pi]) or drift>.003:reasons.append('RELATIVE_SLIP')
   speed=float(max(np.abs(p['dq'][pi].mean(0)).max(),np.abs(b['dq'][bi].mean(0)).max()));qspan=float(max(np.ptp(p['q'][pi],axis=0).max(),np.ptp(b['q'][bi],axis=0).max()))
   if speed>.002 or qspan>.001:reasons.append('NOT_SETTLED')
   row=dict(pose_id=int(pid),q=q,q_empty=qb,q_error_rad=qerr,actual_opening_m=w,empty_opening_m=wb,mean_dq_max=speed,q_span_max=qspan,tau_empty=b['tau'][bi].mean(0),tau_loaded=p['tau'][pi].mean(0),delta_tau_measured=delta,tau_payload_GT=gt,residual=delta-gt,gravity_TCP=R.T@np.array([0,0,-9.81]),Y_COM=XM[:,1:],COM_GT_TCP=com,used=not reasons,reasons=reasons);rows.append(row)
   if not reasons:xx.append(XM);yy.append(delta);gg.append(gt);cc.append(com)
  summary={'poses':rows,'accepted':len(xx),'total':len(rows),'evaluation_only':True}
  if len(xx)>=3:
   X=np.concatenate(xx);d=np.concatenate(yy);g=np.concatenate(gg);c=np.mean(cc,axis=0);f=fit(X,d,c);summary.update(measured=f,mass_error_percent=abs(f['mass_kg']-mass)/mass*100,GT_COM=c,per_joint_residual_RMS_Nm=np.sqrt(np.mean((d-g).reshape(-1,7)**2,axis=0)),residual_RMS_Nm=float(np.sqrt(np.mean((d-g)**2))))
  (folder/'diagnosis.json').write_text(json.dumps(_json_safe(summary),indent=2));summaries[folder.parent.name+'/'+folder.name]={k:v for k,v in summary.items() if k!='poses'}
 (out/'fresh_summary.json').write_text(json.dumps(_json_safe(summaries),indent=2));print(json.dumps(_json_safe(summaries),indent=2))
if __name__=='__main__':main()
