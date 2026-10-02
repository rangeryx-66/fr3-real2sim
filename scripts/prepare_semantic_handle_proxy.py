"""Dataset visual bar -> one PCA box and two supports, without CoACD.

The simplified contact proxy is explicit. Visuals, robot, object joints, masses,
installation and other collision geometry are preserved. No object-ID rules.
"""
import argparse,json,sys,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,trimesh
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import geometry
from articulated_demo.kinematics import URDFChain

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ranking',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 choices=[r for r in json.loads(a.ranking.read_text())['ranking'] if r['sections'] and r['dimensions_m'][0]>.02543 and r['dimensions_m'][1]<.1 and r['dimensions_m'][0]/r['dimensions_m'][1]>1.6 and min(x['visual_rear_gap_m'] for x in r['sections'])>.005]
 if not choices:raise RuntimeError('NO_SUPPORTED_VISIBLE_BAR')
 s=max(choices,key=lambda r:min(x['visual_rear_gap_m'] for x in r['sections']));source=Path(s['asset_root']);meta=json.loads((source/'manifest.json').read_text());urdf=source/'urdf'/f"{meta['asset_id']}.urdf";root=ET.parse(urdf).getroot();out=a.output.resolve();(out/'urdf').mkdir(parents=True,exist_ok=True);(out/'interaction').mkdir(exist_ok=True)
 for node in root.findall('.//geometry/mesh'):node.set('filename',str((urdf.parent/node.get('filename')).resolve()))
 link=root.find(f"link[@name='{s['handle_link']}']");visual=next(v for v in link.findall('visual') if Path(v.find('geometry/mesh').get('filename')).stem==s['mesh']);mesh=geometry(visual,urdf.parent);F=URDFChain(urdf).root_to_link(s['handle_link'],{});mesh.apply_transform(F)
 anchors=np.array([x['anchor_root_m'] for x in s['sections']]);center=anchors.mean(0);axis=np.linalg.svd(anchors-center,full_matrices=False)[2][0]
 if axis@np.asarray(s['axis_root'])<0:axis=-axis
 normal=np.asarray(s['outward_normal_root']);normal-=axis*(normal@axis);normal/=np.linalg.norm(normal);cross=np.cross(normal,axis);cross/=np.linalg.norm(cross);R=np.column_stack((axis,cross,normal))
 length=float(np.ptp(mesh.vertices@axis));width=float(s['dimensions_m'][1]);depth=max(x['bar_depth_m'] for x in s['sections']);rear=min(x['visual_rear_gap_m'] for x in s['sections'])
 # PCA straight bar. Supports occupy the end regions, leaving the central gap.
 specs=[('bar',center,[length,width,depth])]
 for side in [-1,1]:
  pos=center+side*.43*length*axis-(depth/2+rear/2)*normal
  specs.append(('support',pos,[.12*length,width,rear]))
 removed=[]
 for col in list(link.findall('collision')):
  node=col.find('geometry/mesh')
  if node is not None and s['mesh'] in Path(node.get('filename')).name:removed.append(node.get('filename'));link.remove(col)
 if not removed:raise RuntimeError('NO_MATCHING_HANDLE_COLLISION')
 records=[]
 for i,(kind,pos,size) in enumerate(specs):
  box=trimesh.creation.box(size);T=np.eye(4);T[:3,:3]=R;T[:3,3]=pos;T=np.linalg.inv(F)@T;box.apply_transform(T);file=out/'interaction'/f'handle_piece_{i:03}.stl';box.export(file,file_type='stl_ascii');node=ET.SubElement(link,'collision');g=ET.SubElement(node,'geometry');ET.SubElement(g,'mesh',filename=str(file),scale='1 1 1');records.append({'kind':kind,'file':str(file),'center_link_m':T[:3,3].tolist(),'rotation_link':T[:3,:3].tolist(),'size_m':list(size),'bounds':box.bounds.tolist(),'volume_m3':float(box.volume)})
 ET.indent(root);output=out/'urdf'/urdf.name;ET.ElementTree(root).write(output,encoding='utf-8',xml_declaration=True)
 # Nominal grasp anchor belongs to the semantic contact bar, not a raw-mesh fit.
 selection=dict(s);selection['axis_root']=axis.tolist();selection['outward_normal_root']=normal.tolist();selection['sections']=[{'fraction':0.,'anchor_root_m':center.tolist(),'bar_depth_m':depth,'visual_rear_gap_m':rear}]
 meta.update(moving_link=s['moving_link'],door_link=s['handle_link'],prepared_geometry_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),interaction_geometry={'method':'semantic PCA bar box + two support boxes; no CoACD','selection':selection,'original_asset_root':str(source),'original_urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'visual_source_sha256':hashlib.sha256(Path(visual.find('geometry/mesh').get('filename')).read_bytes()).hexdigest(),'pieces':records,'PCA_centerline_root':{'center_m':center.tolist(),'axis':axis.tolist(),'length_m':length},'cross_section_m':[width,depth],'rear_clearance_m':rear,'physics_parameters_preserved':True,'proxy_scope':'grasp/contact physics only; visual unchanged; a semantic approximation, not an official collision model','raw_intersection_policy':'diagnostic only; never an acceptance veto'})
 (out/'manifest.json').write_text(json.dumps(meta,indent=2));print(json.dumps({'asset_id':meta['asset_id'],'pieces':3,'length_m':length,'cross_section_m':[width,depth],'rear_clearance_m':rear},indent=2))
if __name__=='__main__':main()
