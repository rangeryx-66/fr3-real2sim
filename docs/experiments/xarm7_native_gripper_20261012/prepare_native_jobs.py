import json,copy,shutil,datetime
from pathlib import Path
import numpy as np
from xarm7_native_gripper import ROOT,OUT
from xarm7_native_planning import NativeModel
from piper_mobile_demo.model import geometry
from articulated_demo.kinematics import transform
from scipy.spatial.transform import Rotation
B=ROOT/'results/benchmarkA_30deg_20261011';old=ROOT/'results/xarm7_shared_doors_20261012'
reg=json.loads((OUT/'REGISTERED_CASES.json').read_text());ranking=json.loads((B/'DISCOVERY_GEOMETRY_RANKING.json').read_text())['ranking']
for asset in reg['asset_ids']:
 dest=OUT/asset;dest.mkdir(exist_ok=True)
 j=json.loads((old/('45134' if asset=='45249' else asset)/'job.json').read_text())
 if asset=='45249':
  src=B/'visual_inspection/45249/source';ar=B/'discovery_assets/45249';selection=reg['unseen_selection']
 else:
  src=Path(j['source']);ar=Path(j['asset_root']);selection=j['grasp_geometry_candidates'][0]
 if not (dest/'asset').exists():shutil.copytree(ar,dest/'asset')
 if not (dest/'source').exists():shutil.copytree(src,dest/'source')
 m=json.loads((dest/'asset/manifest.json').read_text());m.setdefault('interaction_geometry',{})['selection']=selection
 (dest/'asset/manifest.json').write_text(json.dumps(m,indent=2))
 j.update(asset_root=str(dest/'asset'),source=str(dest/'source'),output=str(dest/'physical_01'),whole_task_plan=str(dest/'planning/candidate.json'),plan=str(dest/'grasp_plan.json'),gpu=2,deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=8)).isoformat(),grasp_geometry_candidates=[selection],classification=reg['classification'],planning_base_seeds=[],episode_id='native_xarm_'+asset)
 j['skill_inputs']['handle_region']={'source':'oracle_handle','asset_id':asset,'mesh':reg['handles'][asset]}
 if asset=='45249':
  model=NativeModel(None,dest/'asset',json.loads((dest/'source/report.json').read_text()));exp={'shapes':[]}
  import xml.etree.ElementTree as ET
  urdf=ET.parse(model.asset_urdf).getroot()
  for l in urdf.findall('link'):
   n=l.get('name');T=model.asset_T@model.asset.root_to_link(n,{model.manifest['joint_name']:0.})
   for i,col in enumerate(l.findall('collision')):
    h=geometry(col,model.asset_urdf.parent).convex_hull;mesh=Path(col.find('geometry/mesh').get('filename')).stem
    exp['shapes'].append({'path':f'/World/asset/{n}/collision/{mesh}','type':'Mesh','rigid_body_path':f'/World/asset/{n}','world_transform':T.tolist(),'rigid_body_world_transform':T.tolist(),'convexes':[{'vertices':h.vertices.tolist(),'polygons':h.faces.tolist()}]})
  import trimesh
  setup=json.loads((B/'visual_inspection/45249/initial_scene_private.json').read_text());fixture=setup['fixture'];T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(fixture['quaternion_wxyz'],-1)).as_matrix();T[:3,3]=fixture['center'];h=trimesh.creation.box(fixture['size'])
  for n,h,T in [('fixture',h,T),('table',trimesh.creation.box([.9,.9,.05]),transform([.5,0,-.025]))]:exp['shapes'].append({'path':'/World/'+n,'type':'Mesh','rigid_body_path':'/World/'+n,'world_transform':T.tolist(),'rigid_body_world_transform':T.tolist(),'convexes':[{'vertices':h.vertices.tolist(),'polygons':h.faces.tolist()}]})
  (dest/'authored_offline_export.json').write_text(json.dumps(exp));j['collision_export']=str(dest/'authored_offline_export.json')
 else:j['collision_export']=str(ROOT/'results/door_transfer4_20261012'/asset/'cooked_initial.json')
 (dest/'planning_job.json').write_text(json.dumps(j,indent=2))
 print(asset,'prepared; no planning outcome consulted')
