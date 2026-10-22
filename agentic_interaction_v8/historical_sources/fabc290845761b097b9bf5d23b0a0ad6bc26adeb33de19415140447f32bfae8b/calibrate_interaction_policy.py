"""Freeze one global contact policy from a recorded successful canonical probe."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
import trimesh,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.owned_scene import meshes

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--canonical',type=Path,required=True);p.add_argument('--ownership',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wrist-canonical',type=Path,help='Optional independently validated reaction-wrench trace; does not change geometric tolerance or preload');a=p.parse_args();proof=json.loads((a.canonical/'full_validation.json').read_text());owner=json.loads(a.ownership.read_text());sha=hashlib.sha256(a.ownership.read_bytes()).hexdigest()
 if not proof.get('canonical_acceptance') or proof['ownership_manifest_sha256']!=sha:raise RuntimeError('canonical acceptance/ownership mismatch')
 steps=json.loads((a.canonical/'physics_steps.json').read_text());export=json.loads((a.canonical/'cooked_closed.json').read_text());cook_error=0.;target_offset=0.
 for e in export['shapes']:
  if e.get('owner') and e.get('raw_points') and e.get('convexes'):
   raw_mesh=meshes(e,False)[0];cooked_mesh=trimesh.util.concatenate(meshes(e,True));cook_error=max(cook_error,float(trimesh.proximity.closest_point_naive(raw_mesh,cooked_mesh.vertices)[1].max()),float(trimesh.proximity.closest_point_naive(cooked_mesh,raw_mesh.vertices)[1].max()))
  if 'canonical_handle' in e['path']:target_offset=max(target_offset,float(e['offsets']['contactOffset']['schema_value']))
 finger_offset=max(x['source_contact_offset_m'] for x in owner['fingers'].values());advance=0.;penetration=0.;forces=[];directions=[];impulses=[]
 for i,s in enumerate(steps):
  contacts=s.get('contacts',[]);penetration=max(penetration,max([-float(c['separation_m']) for c in contacts]+[0.]))
  if s['phase'] in ('CLOSE','CLOSURE_HOLD','PULL_DIAGNOSTIC','HOLD') and i:
   old=steps[i-1]
   for n,x in s['finger_world_poses'].items():
    if n not in old['finger_world_poses']:continue
    v=np.array(owner['fingers'][n]['pad_vertices']);T=np.array(x);P=np.array(old['finger_world_poses'][n]);advance=max(advance,float(np.linalg.norm(v@T[:3,:3].T+T[:3,3]-(v@P[:3,:3].T+P[:3,3]),axis=1).max()))
  if s['phase'] in ('CLOSURE_HOLD','HOLD','PULL_DIAGNOSTIC'):
   forces.extend(s['ownership']['pad_forces_n'].values())
  if s['phase']=='PULL_DIAGNOSTIC':
   impulses.append(np.sum([c['impulse_world_ns'] for c in contacts if c['owner']=='pad'],axis=0) if contacts else np.zeros(3))
   # Canonical robot pose can be FK-computed from q if absent in this older trace.
   if 'T_tcp' in s:directions.append(np.array(s['T_tcp'])[:3,3])
 # Read measured pull direction from the accepted observation trace, not object GT.
 obs=json.loads((a.canonical/'observations.json').read_text());dot=0.;pad_net=[];wrist=[]
 if a.wrist_canonical:
  proof_w=json.loads((a.wrist_canonical/'controller_validation.json').read_text())
  if not proof_w['canonical_acceptance'] or proof_w['ownership_manifest_sha256']!=sha:raise RuntimeError('wrist sensor calibration requires validated same-ownership canonical trace')
  obs=json.loads((a.wrist_canonical/'observations.json').read_text())
 # Force on a loaded finger points into its solid body, opposite the official
 # exposed pad face normal. This normal-load check has an unambiguous sign;
 # net tangential pull projection can be contaminated by normal imbalance.
 normals={}
 for n,x in owner['fingers'].items():
  mesh=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{n}.stl',force='mesh');normal=mesh.face_normals[x['official_pad_face_ids']].mean(0);normals[n]=-normal/np.linalg.norm(normal)
 for row in obs:
  net=np.zeros(3)
  for n in owner['fingers']:
   vectors=[c['impulse_world_ns'] for c in row['contacts'] if c['finger']==n and c['owner']=='pad'];vector=np.sum(vectors,axis=0) if vectors else np.zeros(3);dot+=float(vector@(np.asarray(row['finger_world_poses'][n])[:3,:3]@normals[n]));net+=vector/row['ownership']['physics_window_dt_s']
  pad_net.append(net);wrist.append(row.get('wrist_reaction_wrench_world'))
 sign=1. if dot>0 else -1.;sign_observable=abs(dot)>1e-8
 wrist_sign=None;wrist_observable=False
 if wrist and all(x is not None for x in wrist):
  w=np.asarray(wrist);f=sign*np.asarray(pad_net);quiet=np.linalg.norm(f,axis=1)<.001;bias=np.median(w[quiet,:3],axis=0) if quiet.any() else np.full(3,np.nan);active=np.linalg.norm(f,axis=1)>.1;correlation=float(np.sum((w[active,:3]-bias)*f[active]));wrist_observable=abs(correlation)>1e-3;wrist_sign=1. if correlation>0 else -1.
 # Reconstruction quality is a globally shared bound, not tuned per object.
 proxy_error_bound=.001
 tol=finger_offset+target_offset+cook_error+advance+penetration+proxy_error_bound
 result={'schema':'interaction-contact-policy-v1','ownership_sha256':sha,'canonical_acceptance':True,'canonical_path':str(a.canonical),'canonical_validation_sha256':hashlib.sha256((a.canonical/'full_validation.json').read_bytes()).hexdigest(),'penetration_tolerance_m':tol,'terms_m':{'finger_contact_offset':finger_offset,'canonical_target_contact_offset':target_offset,'measured_cooking_vertex_error':cook_error,'max_canonical_step_surface_advance':advance,'max_stable_canonical_separation_penetration':penetration,'global_proxy_quality_error_bound':proxy_error_bound},'preload_n':min(2.,.2*float(np.median(forces))),'finger_effort_limit_n':10.,'native_metal_impulse_tolerance_ns':0.,'force_sensor_sign':sign,'force_sign_observable':sign_observable,'raw_intersection_alone_is_failure':False,'physical_primary':'native pad/metal collider ownership; any actual metal impulse forbidden; clearly deep penetration forbidden','scope':'one frozen calibration for all assets; none of these terms is an object-specific adjustment'}
 result.update(force_sign_method='known official pad normal reaction, not tangential displacement correlation',wrist_reaction_sign=wrist_sign,wrist_reaction_sign_observable=wrist_observable,wrist_sensor_joint=obs[0].get('wrist_sensor_joint'))
 print('CALIBRATION TERMS',result['terms_m'],flush=True)
 if not np.isfinite(tol) or tol>.005:raise RuntimeError('canonical motion/error bound exceeds 5mm calibration sanity gate; inspect trace')
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
