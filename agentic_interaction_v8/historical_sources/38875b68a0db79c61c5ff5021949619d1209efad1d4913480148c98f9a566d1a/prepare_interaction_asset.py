"""Separate immutable visual meshes from concavity-preserving interaction proxies."""
import argparse,hashlib,json,shutil,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,trimesh,coacd
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import geometry,origin

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ranking',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--surface-resolution',type=int,default=200,help='Global reconstruction resolution, never scaled official geometry');p.add_argument('--concavity-mm',type=float,default=1.,help='Metric CoACD threshold for every asset');p.add_argument('--selection-rank',type=int,default=0,help='Index in geometry-qualified descending rear-clearance ranking; never an object-ID rule');a=p.parse_args();ranking=json.loads(a.ranking.read_text())
 choices=[r for r in ranking['ranking'] if r['sections'] and r['dimensions_m'][0]>.02543 and r['dimensions_m'][1]<.1 and r['dimensions_m'][0]/r['dimensions_m'][1]>1.6 and min(s['visual_rear_gap_m'] for s in r['sections'])>.005]
 if not choices:raise RuntimeError('NO_VISUAL_BAR_WITH_REAR_CLEARANCE')
 selected=sorted(choices,key=lambda r:-min(s['visual_rear_gap_m'] for s in r['sections']))[a.selection_rank]
 source=Path(selected['asset_root']);meta=json.loads((source/'manifest.json').read_text());urdf=source/'urdf'/f"{meta['asset_id']}.urdf";r=ET.parse(urdf).getroot();dest=a.output.resolve();dest.mkdir(parents=True,exist_ok=True);(dest/'urdf').mkdir(exist_ok=True);(dest/'interaction').mkdir(exist_ok=True)
 # Absolute visual references preserve original bytes and dimensions. No old USD cache.
 for link in r.findall('link'):
  for item in link.findall('visual')+link.findall('collision'):
   node=item.find('geometry/mesh')
   if node is not None:node.set('filename',str((urdf.parent/node.get('filename')).resolve()))
 target=r.find(f"link[@name='{selected['handle_link']}']");visual=next(v for v in target.findall('visual') if Path(v.find('geometry/mesh').get('filename')).stem==selected['mesh']);mesh=geometry(visual,urdf.parent)
 # Remove duplicate two-sided triangles, not surfaces or vertices.
 original_bounds=mesh.bounds.copy();original_faces=len(mesh.faces);mesh.update_faces(mesh.unique_faces());mesh.remove_unreferenced_vertices();trimesh.repair.fix_normals(mesh,multibody=True)
 source_closed=bool(mesh.is_watertight)
 # In link coordinates; origins and scale already applied exactly once.
 coacd.set_log_level('warn');parts=coacd.run_coacd(coacd.Mesh(mesh.vertices.astype(np.float64),mesh.faces.astype(np.int32)),threshold=a.concavity_mm/1000,real_metric=True,max_convex_hull=256,preprocess_mode=('off' if source_closed else 'auto'),preprocess_resolution=a.surface_resolution,resolution=3000,mcts_iterations=100,merge=True,decimate=False,extrude=False,seed=20261002)
 old=[]
 for col in list(target.findall('collision')):
  node=col.find('geometry/mesh')
  if node is not None and selected['mesh'] in Path(node.get('filename')).name:old.append(node.get('filename'));target.remove(col)
 if not old:raise RuntimeError('selected visual has no authored collision to replace')
 records=[]
 for i,(vertices,faces) in enumerate(parts):
  m=trimesh.Trimesh(vertices,faces,process=False);file=dest/'interaction'/f'handle_piece_{i:03}.stl';m.export(file,file_type='stl_ascii');col=ET.SubElement(target,'collision');g=ET.SubElement(col,'geometry');ET.SubElement(g,'mesh',filename=str(file),scale='1 1 1');records.append({'file':str(file),'bounds':m.bounds.tolist(),'volume_m3':abs(float(m.volume))})
 proxy_bounds=np.array([np.min([x['bounds'][0] for x in records],axis=0),np.max([x['bounds'][1] for x in records],axis=0)])
 bound_error=float(np.max(abs(proxy_bounds-mesh.bounds)))
 if bound_error>.0006 or np.any(proxy_bounds[0]>mesh.bounds[0]+1e-6) or np.any(proxy_bounds[1]<mesh.bounds[1]-1e-6):raise RuntimeError('proxy must not shrink source bounds; outward reconstruction bound is globally limited to 0.6mm')
 ET.indent(r);out=dest/'urdf'/urdf.name;ET.ElementTree(r).write(out,encoding='utf-8',xml_declaration=True)
 meta.update(moving_link=selected['moving_link'],door_link=selected['handle_link'],prepared_geometry_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),interaction_geometry={'method':'CoACD concavity-preserving decomposition of full-size source visual; no extrusion/inset','selection':selected,'original_asset_root':str(source),'original_urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'visual_source_sha256':hashlib.sha256(Path(visual.find('geometry/mesh').get('filename')).read_bytes()).hexdigest(),'pieces':records,'source_bounds':mesh.bounds.tolist(),'proxy_bounds':proxy_bounds.tolist(),'bounds_error_m':bound_error,'duplicate_faces_removed':original_faces-len(mesh.faces),'source_watertight':bool(mesh.is_watertight),'source_volume_m3':abs(float(mesh.volume)),'proxy_volume_sum_m3':sum(x['volume_m3'] for x in records),'quality_threshold_m':a.concavity_mm/1000,'preprocessing':('none' if source_closed else f'{a.surface_resolution}-resolution conservative surface reconstruction; source is open; not a physical thickness claim'),'physics_parameters_preserved':True})
 (dest/'manifest.json').write_text(json.dumps(meta,indent=2));print(meta['asset_id'],len(parts),bound_error,flush=True)
if __name__=='__main__':main()
