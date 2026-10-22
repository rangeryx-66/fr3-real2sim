"""Cached full physics-step replay for the contact-aware canonical positive control."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np,trimesh
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.owned_scene import Shape,meshes,intersects
from articulated_interaction.collision import convex_depth

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);p.add_argument('--calibration',type=Path,required=True);a=p.parse_args();cal=json.loads(a.calibration.read_text());data=json.loads((a.trial/'cooked_closed.json').read_text());ownership=json.loads((ROOT/'config/piper_contact_ownership.json').read_text());fingers=[];targets=[];raw=[]
 for e in data['shapes']:
  if e.get('finger'):
   T=np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform']);fingers.extend(Shape(m,T,True,e['path'],e['finger'],e['owner']) for m in meshes(e))
  elif 'canonical_handle' in e['path']:targets.extend(Shape(m,np.asarray(e['world_transform']),True,e['path'],'canonical_handle') for m in meshes(e))
 for n in ownership['fingers']:
  m=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{n}.stl',force='mesh');pad=set(ownership['fingers'][n]['official_pad_face_ids']);m.update_faces([i not in pad for i in range(len(m.faces))]);raw.append(Shape(m,np.eye(4),False,'official_nonpad',n,'metal'))
 steps=json.loads((a.trial/'physics_steps.json').read_text());bad=[];micro=0
 for i,s in enumerate(steps):
  if s['ownership']['metal_contacts'] or s['ownership']['pad_target_violations']:bad.append({'sample':i,'reason':'NATIVE_FORBIDDEN_CONTACT'});break
  for shape in fingers+raw:shape.place(np.asarray(s['finger_world_poses'][shape.body]))
  for x in fingers:
   if x.owner=='pad':continue
   for y in targets:
    if intersects(x,y) and convex_depth(x,y)>cal['penetration_tolerance_m']:bad.append({'sample':i,'reason':'DEEP_NATIVE_SOLID_PENETRATION'});break
  for x in raw:
   for y in targets:
    if intersects(x,y):micro+=1
  if bad:break
 probe=[s for s in steps if s['phase']=='PULL_DIAGNOSTIC'];forces=[list(s['ownership']['pad_forces_n'].values()) for s in probe]
 def padcenter(s):return np.mean([(np.asarray(s['finger_world_poses'][n])@np.r_[np.asarray(o['pad_vertices']).mean(0),1])[:3] for n,o in ownership['fingers'].items()],axis=0)
 travel=float(np.linalg.norm(padcenter(probe[-1])-padcenter(probe[0]))) if probe else 0.
 result={'canonical_acceptance':not bad and bool(forces) and np.min(forces)>=.2 and travel>=.001,'failures':bad,'physics_step_samples':len(steps),'pull_displacement_m':travel,'minimum_pull_pad_forces_n':np.min(forces,axis=0).tolist() if forces else None,'raw_triangle_hit_diagnostics':micro,'raw_triangle_hit_alone_rejects':False,'native_metal_samples':sum(s['ownership']['metal_contacts']>0 for s in steps),'scope':'all native physics samples; cached cooked metal geometry and official raw surface diagnostics; canonical analytic box only'}
 result['ownership_manifest_sha256']=hashlib.sha256((ROOT/'config/piper_contact_ownership.json').read_bytes()).hexdigest()
 (a.trial/'controller_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
