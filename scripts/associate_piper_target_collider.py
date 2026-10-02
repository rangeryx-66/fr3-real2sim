"""Associate a frozen semantic-part point cloud with native scene collider IDs.

No object ID rule. Runtime contact classification uses the resulting collider
IDs, never a distance from a contact point to the point cloud.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np,trimesh
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from validate_piper_cooked_contact import mesh_from
p=argparse.ArgumentParser(description=__doc__);p.add_argument('export',type=Path);p.add_argument('points',type=Path);p.add_argument('output',type=Path);p.add_argument('--target-body',default='l_1');p.add_argument('--authored-mesh-stem',help='Explicit dataset semantic mesh ownership; must match one native authored collider');a=p.parse_args();shapes=[s for s in json.loads(a.export.read_text())['shapes'] if s.get('rigid_body_path','').split('/')[-1]==a.target_body and s.get('raw_points')];points=np.load(a.points)
if not shapes or len(points)<3:raise RuntimeError('semantic part/colliders missing')
distances=[]
for s in shapes:
 m=mesh_from(s,False)[0];m.apply_transform(np.asarray(s['world_transform']));distances.append(np.concatenate([trimesh.proximity.closest_point_naive(m,points[i:i+128])[1] for i in range(0,len(points),128)]))
distances=np.asarray(distances);nearest=np.argmin(distances,axis=0);votes=np.bincount(nearest,minlength=len(shapes));winner=int(np.argmax(votes));confidence=float(votes[winner]/votes.sum())
if a.authored_mesh_stem:
 import re
 token=re.sub('[^a-z0-9]','',a.authored_mesh_stem.lower())+'obj'
 matched=[i for i,s in enumerate(shapes) if token in re.sub('[^a-z0-9]','',s['path'].lower())]
 if len(matched)!=1:raise RuntimeError('explicit authored mesh ownership is not unique')
 winner=matched[0];confidence=float(votes[winner]/votes.sum())
if not a.authored_mesh_stem and confidence<.8:raise RuntimeError('semantic part does not identify a single collider reliably; require explicit surface ownership map')
r={'authored_mesh_stem':a.authored_mesh_stem,'explicit_unique_authored_mapping':bool(a.authored_mesh_stem),'method':('unique authored mesh identity from dataset semantic ownership' if a.authored_mesh_stem else 'nearest authored scene surface for frozen segmented part cloud; dominant component selected before execution'),'allowed_pad_targets':[shapes[winner]['path']],'winner_fraction':confidence,'point_count':len(points),'votes':[{ 'path':s['path'],'votes':int(v)} for s,v in zip(shapes,votes)],'does_not_classify_runtime_contacts_by_position':True};a.output.write_text(json.dumps(r,indent=2));print(r['allowed_pad_targets'],confidence)
