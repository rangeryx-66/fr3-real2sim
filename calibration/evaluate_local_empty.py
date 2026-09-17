"""Matched-sample comparison; local interpolation trained on empty only."""
import sys,json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from compare_q_baselines import window,O,_json_safe,_torch_fit_mass_com_fixed_mass,DEFAULT_OFFICIAL_ROOT,find
summary={}
for name in ['banana','mug']:
 old=json.load(open(O/f'{name}.json'));root=O/'local_empty'/name;spec=json.load(open(root/'manifest.json'));cap=json.load(open(root/'strict_capture.json'));rows=[];valid=[]
 for mapping in spec['local_mapping']:
  original=next(x for x in old['rows'] if x['pose_id']==mapping['parent_pose_id']);samples=[];reasons=[]
  for pid in mapping['local_ids']:
   row=next((x for x in cap['accepted'] if x['pose_id']==pid),None)
   if row is None:reasons.append('NO_ACCEPTED_EMPTY');break
   w=window(row['path']);samples.append(w)
   if abs(w['opening']-original['opening_loaded'])>=.0002:reasons.append('OPENING_MISMATCH')
  if len(samples)<3:rows.append({'pose_id':original['pose_id'],'reasons':reasons});continue
  a,c,b=samples;axis=b['q']-a['q'];den=axis@axis;ql=np.array(original['q_loaded']);alpha=float((ql-a['q'])@axis/den);beta=float((c['q']-a['q'])@axis/den);projection=a['q']+alpha*axis;off=float(np.max(np.abs(ql-projection)))
  if not 0<=alpha<=1:reasons.append('OUTSIDE_LOCAL_BRACKET')
  if off>.0002:reasons.append('UNSUPPORTED_ORTHOGONAL_DIRECTION')
  pred=a['tau']+alpha*(b['tau']-a['tau']);held=a['tau']+beta*(b['tau']-a['tau'])-c['tau'];res={'pose_id':original['pose_id'],'alpha':alpha,'orthogonal_distance_max_rad':off,'heldout_center_residual_Nm':held,'predicted_empty_tau':pred,'reasons':reasons,'endpoint_paths':[a['path'],b['path']],'heldout_path':c['path'],'original':original};rows.append(res)
  if not reasons:valid.append(res)
 result={'rows':rows,'common_poses':len(valid),'local_model':'one_dimensional_endpoint_interpolation_along_measured_q_mismatch','GT_used_for_baseline':False}
 if len(valid)>=4:
  X=np.concatenate([r['original']['Y'] for r in valid]);methods={}
  for name_method in ['raw','drake','local_empty']:
   ys=[]
   for r in valid:
    orig=r['original'];ys.append(orig['raw_delta'] if name_method=='raw' else orig['corrected_delta'] if name_method=='drake' else np.array(orig['tau_loaded'])-r['predicted_empty_tau'])
   try:f=_torch_fit_mass_com_fixed_mass(X,np.concatenate(ys),DEFAULT_OFFICIAL_ROOT)
   except ValueError as exc:f={'accepted':False,'reason':str(exc)}
   methods[name_method]={'fit':f,'ys':np.array(ys)}
  # GT evaluation only, after all three fits.
  folder=R/'results/fr3_clean_pair_fd_gate/matched_opening_repair/banana/payload' if name=='banana' else R/'results/fr3_torque_rootcause/fresh/mug/payload';trial=json.load(open(next(folder.glob('*_ANYGRASP.json'))));mass=float(np.asarray(find(trial,'target_mass_kg')).ravel()[0]);cl=np.asarray(find(trial,'target_COM_local')).ravel()[:3];c=np.mean([np.array(r['original']['T'])[:3,:3]@cl+np.array(r['original']['T'])[:3,3] for r in valid],0);gt=np.array([r['original']['GT_torque_diagnostic'] for r in valid])
  for method,m in methods.items():
   residual=m.pop('ys')-gt;f=m['fit'];m.update(GT_residual_RMS_Nm=float(np.sqrt(np.mean(residual**2))),joint_bias=residual.mean(0),joint_RMS=np.sqrt(np.mean(residual**2,axis=0)),mass_error_percent=abs(f['mass_kg']-mass)/mass*100 if 'mass_kg' in f else None,COM_error_mm=float(np.linalg.norm(np.array(f['center_of_mass_m'])-c)*1000) if 'center_of_mass_m' in f else None)
  result['methods']=methods;result['empty_heldout_RMS_Nm']=float(np.sqrt(np.mean(np.array([r['heldout_center_residual_Nm'] for r in valid])**2)))
 (O/f'{name}_local_comparison.json').write_text(json.dumps(_json_safe(result),indent=2));summary[name]={k:v for k,v in result.items() if k!='rows'}
(O/'local_comparison_summary.json').write_text(json.dumps(_json_safe(summary),indent=2));print(json.dumps(_json_safe(summary),indent=2))
