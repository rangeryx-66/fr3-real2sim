"""Evaluate native pad grasp and motion; raw mesh intersections only diagnose."""
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model,geometry,origin
from piper_mobile_demo.owned_scene import OwnedFingerScene,Shape,intersects
from interaction_identification.fitting import fit_articulation,evaluate

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for n in ['trial','source','asset-root']:p.add_argument('--'+n,type=Path,required=True)
 a=p.parse_args();report=json.loads((a.trial/'report.json').read_text());rows=json.loads((a.trial/'observations.json').read_text());physics=json.loads((a.trial/'physics_steps.json').read_text());gt=json.loads((a.trial/'evaluation_gt.json').read_text());policy=report['calibration'];model=Model(ROOT/'config/piper.urdf',a.asset_root,json.loads((a.source/'report.json').read_text()))
 allowed=json.loads((a.trial/'association.json').read_text())['allowed_pad_targets'];owner=OwnedFingerScene(ROOT,a.trial/'cooked_initial.json',ROOT/'config/piper_contact_ownership.json',allowed,model.manifest['moving_link'])
 v=next(v for v in ET.parse(model.asset_urdf).getroot().find(f"link[@name='{model.manifest['door_link']}']").findall('visual') if Path(v.find('geometry/mesh').get('filename')).stem==model.manifest['interaction_geometry']['selection']['mesh'])
 visual=Shape(geometry(v,model.asset_urdf.parent),np.eye(4),False,'original_visual_handle',model.manifest['door_link'])
 phases={'FORCE_HOLD','BLIND_PROBE'};loaded=[(i,r) for i,r in enumerate(rows) if r['phase'] in phases];probe=[(i,r) for i,r in loaded if r['phase']=='BLIND_PROBE'];slip=[];reference=None
 for i,r in loaded:
  relative=np.linalg.inv(np.asarray(gt[i]['T_moving_link']))@np.asarray(r['T_tcp'])
  if reference is None:reference=relative
  slip.append(float(np.linalg.norm(relative[:3,3]-reference[:3,3])))
 diagnostic=[]
 for i,r in loaded[::max(1,len(loaded)//5)] if loaded else list(enumerate(rows))[-1:]:
  poses=model.poses(np.asarray(r['q'])[:6],report['base_fixed'],finger_q=np.asarray(r['q'])[6:]);visual.place(np.asarray(gt[i]['T_moving_link']))
  for raw in owner.raw:
   raw.place(poses[raw.body]);hit=intersects(raw,visual)
   diagnostic.append({'sample':i,'finger':raw.body,'official_nonpad_vs_original_visual_triangle_hit':hit,'acceptance_effect':'none'})
 metal=sum(s['ownership']['metal_contacts']>0 for s in physics);wrong=sum(s['ownership']['pad_target_violations']>0 for s in physics)
 forces=np.array([[r['ownership']['pad_forces_n'][n] for n in ('gripper_link1','gripper_link2')] for i,r in loaded]);probe_forces=np.array([[r['ownership']['pad_forces_n'][n] for n in ('gripper_link1','gripper_link2')] for i,r in probe]);travel=0.;movement=0.;joint_delta=None
 if probe:
  first=max(0,probe[0][0]-1);last=probe[-1][0];P0=np.asarray(rows[first]['T_tcp']);P1=np.asarray(rows[last]['T_tcp']);travel=float(np.linalg.norm(P1[:3,3]-P0[:3,3]));L0=np.asarray(gt[first]['T_moving_link']);L1=np.asarray(gt[last]['T_moving_link']);movement=float(np.linalg.norm((L1@np.linalg.inv(L0)@P0)[:3,3]-P0[:3,3]));joint_delta=gt[last]['joint_rad']-gt[first]['joint_rad']
 legal=bool(report['legal_preload_native_only_pending_offline_audit'] and metal==0 and wrong==0 and report['minimum_joint_margin_rad']>.05 and len(forces) and forces.min()>=.1 and forces.max()<=policy['max_pad_load_n'] and max(slip,default=1)<=policy['max_slip_m'])
 fit=fit_articulation(np.array([r['T_tcp'] for i,r in probe])) if len(probe)>=12 else {'status':'INSUFFICIENT_PROBE_DATA'}
 joint=ET.parse(model.asset_urdf).getroot().find(f"joint[@name='{model.manifest['joint_name']}']");J=model.asset_T@model.asset.root_to_link(joint.find('parent').get('link'),{})@origin(joint);axis=J[:3,:3]@np.fromstring(joint.find('axis').get('xyz'),sep=' ');truth={'joint_type':joint.get('type'),'axis_world':(axis/np.linalg.norm(axis)).tolist(),'origin_world':J[:3,3].tolist()}
 result={'legal_native_grasp':legal,'physical_interaction_success':legal and travel>=.002 and movement>=.0005,'actual_EE_pull_m':travel,'actual_object_motion_at_grasp_m':movement,'GT_probe_joint_displacement':joint_delta,'GT_displacement_units':'m' if truth['joint_type']=='prismatic' else 'rad','max_relative_slip_m':max(slip,default=None),'minimum_joint_margin_rad':report['minimum_joint_margin_rad'],'native_metal_physics_samples':metal,'native_wrong_target_physics_samples':wrong,'minimum_loaded_pad_forces_n':forces.min(0).tolist() if len(forces) else None,'raw_diagnostics':diagnostic,'raw_diagnostic_failure_effect':'none; no raw penetration threshold','fit_from_EE_only':fit,'evaluation_GT':truth,'wrist_wrench_used':False,'information_boundary':'GT only in this post-run evaluation'}
 if probe and 'joint_type' in fit:result['identification_errors']=evaluate(fit,truth,np.asarray(probe[0][1]['T_tcp'])[:3,3],np.array([r['T_tcp'] for i,r in probe]))
 (a.trial/'evaluation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
