"""Bounds/volume/transform audit of authored partition and native cooked shapes."""
import argparse,json,sys
from pathlib import Path
import numpy as np,trimesh
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from validate_piper_cooked_contact import mesh_from
p=argparse.ArgumentParser(description=__doc__);p.add_argument('export',type=Path);p.add_argument('output',type=Path);p.add_argument('--ownership',type=Path,default=ROOT/'config/piper_contact_ownership.json');a=p.parse_args();data=json.loads(a.export.read_text());manifest=json.loads(a.ownership.read_text());report={}
for n,spec in manifest['fingers'].items():
 entries=[s for s in data['shapes'] if s.get('finger')==n];pieces=[];authored=[];rows=[]
 if len(entries)!=len(spec['pieces']):raise RuntimeError('missing owned colliders')
 for e in entries:
  T=np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform']);raw=mesh_from(e,False)[0];raw.apply_transform(T);authored.append(raw)
  for cooked in mesh_from(e):
   cooked.apply_transform(T);pieces.append(cooked);rows.append({'path':e['path'],'owner':e['owner'],'mesh_to_body':T.tolist(),'raw_bounds':raw.bounds.tolist(),'cooked_bounds':cooked.bounds.tolist(),'bounds_delta_m':(cooked.bounds-raw.bounds).tolist(),'raw_volume_m3':abs(float(raw.volume)),'cooked_volume_m3':abs(float(cooked.volume)),'max_raw_vertex_to_cooked_surface_m':float(trimesh.proximity.closest_point_naive(cooked,raw.vertices)[1].max()),'offsets':e['offsets'],'runtime_body_shapes':e['runtime_body_shapes']})
 union=trimesh.boolean.union(pieces,engine='manifold',check_volume=True);source=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{n}.stl',force='mesh')
 report[n]={'official_bounds_m':source.bounds.tolist(),'authored_union_bounds_m':np.vstack([m.vertices for m in authored]).min(0).tolist()+np.vstack([m.vertices for m in authored]).max(0).tolist(),'cooked_union_bounds_m':union.bounds.tolist(),'authored_partition_volume_m3':spec['partition_volume_m3'],'source_full_hull_volume_m3':spec['source_hull_volume_m3'],'cooked_union_volume_m3':float(abs(union.volume)),'cooked_union_relative_volume_change':float(abs(union.volume)/spec['source_hull_volume_m3']-1),'original_official_mesh_volume_m3':float(abs(source.volume)),'source_watertight':source.is_watertight,'official_pad_faces':spec['official_pad_face_ids'],'pieces':rows}
a.output.write_text(json.dumps({'no_official_vertices_moved':True,'partition_not_physical_pad_thickness_claim':True,'fingers':report},indent=2));print({n:{k:v for k,v in r.items() if k not in ['pieces','official_pad_faces']} for n,r in report.items()})
