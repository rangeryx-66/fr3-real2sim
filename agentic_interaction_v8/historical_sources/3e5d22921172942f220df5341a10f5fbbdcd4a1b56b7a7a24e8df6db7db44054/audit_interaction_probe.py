"""Post-run audit and identification. Object truth is evaluation-only here."""
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from articulated_interaction.collision import InteractionScene
from piper_mobile_demo.model import Model
from interaction_identification.fitting import fit_articulation,evaluate
from piper_mobile_demo.model import origin

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for n in ['trial','source','asset-root','calibration']:p.add_argument('--'+n,type=Path,required=True)
 a=p.parse_args();report=json.loads((a.trial/'report.json').read_text());cal=json.loads(a.calibration.read_text());source=json.loads((a.source/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',a.asset_root,source);allowed=json.loads((a.trial/'association.json').read_text())['allowed_pad_targets'];owner=InteractionScene(ROOT,a.trial/'cooked_initial.json',ROOT/'config/piper_contact_ownership.json',allowed,model.manifest['moving_link'],calibration=cal)
 # Native gap check along each independently selected visual bar section.
 s=model.manifest['interaction_geometry']['selection'];n=model.asset_T[:3,:3]@s['outward_normal_root'];gap=[]
 for sec in s['sections']:
  c=(model.asset_T@np.r_[sec['anchor_root_m'],1])[:3];samples=c-np.linspace(sec['bar_depth_m']/2+.001,sec['bar_depth_m']/2+sec['visual_rear_gap_m']-.001,50)[:,None]*n;occupied=np.zeros(len(samples),bool)
  for shape in owner.scene:
   if shape.path not in allowed:continue
   v=shape.vertices@owner.moving_reference[:3,:3].T+owner.moving_reference[:3,3];eq=ConvexHull(v).equations;occupied|=np.max(samples@eq[:,:3].T+eq[:,3],axis=1)<=0
  gap.append({'section_fraction':sec['fraction'],'visual_rear_gap_m':sec['visual_rear_gap_m'],'native_gap_clear_fraction':float(np.mean(~occupied))})
 rows=json.loads((a.trial/'observations.json').read_text());truth=json.loads((a.trial/'evaluation_gt.json').read_text());physics=json.loads((a.trial/'physics_steps.json').read_text());errors=[];reference=None;slips=[]
 for i,(r,e) in enumerate(zip(rows,truth)):
  if r['phase'] not in ('CLOSE','SEEK_SECOND_PAD','PRELOAD','FORCE_HOLD','BLIND_PROBE'):continue
  q=np.asarray(r['q']);poses=model.poses(q[:6],report['base_fixed'],finger_q=q[6:]);L=np.asarray(e['T_moving_link']);ok,why=owner.check(poses,L,allow_pad=True)
  if not ok:errors.append({'sample':i,'phase':r['phase'],'reason':why});break
  ok,why=owner.check_self(poses,model.allowed)
  if not ok:errors.append({'sample':i,'phase':r['phase'],'reason':why});break
 for r,e in zip(rows,truth):
  if r['phase'] not in ('FORCE_HOLD','BLIND_PROBE'):continue
  relative=np.linalg.inv(np.asarray(e['T_moving_link']))@np.asarray(r['T_tcp'])
  if reference is None:reference=relative
  slips.append(float(np.linalg.norm(relative[:3,3]-reference[:3,3])))
 probe=[r for r in rows if r['phase']=='BLIND_PROBE'];fit=fit_articulation(np.array([r['T_tcp'] for r in probe])) if len(probe)>=12 else {'status':'INSUFFICIENT_PROBE_DATA'}
 forces=[list(r['ownership']['pad_forces_n'].values()) for r in probe];metal=sum(s['ownership']['metal_contacts']>0 for s in physics);wrong=sum(s['ownership']['pad_target_violations']>0 for s in physics)
 result={'status':'EVALUATED','native_gap_checks':gap,'all_closure_observation_geometry_valid':not errors,'geometry_failures':errors,'native_metal_physics_samples':metal,'native_wrong_target_physics_samples':wrong,'max_relative_slip_m':max(slips,default=None),'fit_from_EE_only':fit,'GT_actual_joint_displacement_rad':report.get('actual_joint_displacement_rad'),'minimum_probe_pad_forces_n':np.min(forces,axis=0).tolist() if forces else None,'legal_grasp':report['legal_preload_native_only_pending_offline_audit'] and not errors and metal==0 and wrong==0,'EE_2mm':report['ee_reached_2mm'],'raw_zero_triangle_intersection_not_required':True,'audit_scope':'all recorded closure/probe observation poses plus all native physics contact events; GT link pose used only for this evaluation'}
 joint=ET.parse(model.asset_urdf).getroot().find(f"joint[@name='{model.manifest['joint_name']}']");J=model.asset_T@model.asset.root_to_link(joint.find('parent').get('link'),{})@origin(joint);axis=J[:3,:3]@np.fromstring(joint.find('axis').get('xyz'),sep=' ');GT={'joint_type':joint.get('type'),'axis_world':(axis/np.linalg.norm(axis)).tolist(),'origin_world':J[:3,3].tolist()}
 result['evaluation_ground_truth']=GT;result['GT_joint_displacement_units']='m' if GT['joint_type']=='prismatic' else 'rad'
 if probe and 'joint_type' in fit:result['identification_errors']=evaluate(fit,GT,np.asarray(probe[0]['T_tcp'])[:3,3],np.array([r['T_tcp'] for r in probe]))
 probe_indices=[i for i,r in enumerate(rows) if r['phase']=='BLIND_PROBE']
 movement=0.
 if probe_indices:
  first=max(0,probe_indices[0]-1);last=probe_indices[-1];L0=np.asarray(truth[first]['T_moving_link']);L1=np.asarray(truth[last]['T_moving_link']);P0=np.asarray(rows[first]['T_tcp']);predicted=L1@np.linalg.inv(L0)@P0;movement=float(np.linalg.norm(predicted[:3,3]-P0[:3,3]));result['GT_probe_joint_displacement']=truth[last]['joint_rad']-truth[first]['joint_rad']
 result['GT_probe_motion_at_grasp_m']=movement
 result['physical_interaction_success']=result['legal_grasp'] and result['EE_2mm'] and bool(forces) and min(result['minimum_probe_pad_forces_n'])>=.2 and max(slips,default=1)<.001 and movement>=.001
 # Threshold is not door success: report actual angular/linear travel explicitly.
 (a.trial/'evaluation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
