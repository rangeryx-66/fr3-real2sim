from pathlib import Path
import json,copy,hashlib,datetime,xml.etree.ElementTree as ET
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
from xarm7_native_gripper import ROOT,configuration
from xarm7_native_planning import NativeModel
from piper_mobile_demo.model import geometry
from articulated_demo.fixture_geometry import fixture_box
from articulated_demo.kinematics import transform
O=ROOT/'results/native_g1_heldout_20261013';reg=json.loads((O/'REGISTERED_OBJECTS.json').read_text());c=configuration()
template=json.loads((ROOT/'results/xarm7_native_transfer_repair_20261012/45249/job.json').read_text())
protocol={'nominal_initial_base':[0.,-.6,-.1,0.],'nominal_base_rule':'same canonical scene seed for all held-out cases; no saved successful station or grasp plan','installation':{'x_m':.5,'y_m':-.05,'yaw_deg':-90.,'fixture_height_m':.04},'planning_base_seeds':[],'frozen_manipulation_sources':reg['frozen_source_sha256'],'target_rad':float(np.pi/6),'budget_s':300,'config_sha256':reg['native_configuration_sha256'],'no_saved_success_plan':True}
with (O/'EXECUTION_PROTOCOL.json').open('x') as f:json.dump(protocol,f,indent=2)
for row in reg['objects']:
 a=row['asset_id'];d=O/a;d.mkdir(exist_ok=True);asset=Path(row['asset_root']);src=d/'source';src.mkdir(exist_ok=True)
 m=json.loads((asset/'manifest.json').read_text());m['interaction_geometry']={'method':'frozen oracle handle candidate generator','selection':row['selection'],'pieces':[]};(d/'asset').mkdir(exist_ok=True)
 # Isolate metadata; geometry and collision files remain the untouched prepared input.
 for folder in ['urdf','meshes','collision']:(d/'asset'/folder).symlink_to(asset/folder,target_is_directory=True)
 (d/'asset/manifest.json').write_text(json.dumps(m,indent=2))
 source={'schema':'known-model-heldout-source/v1','asset_id':a,'asset_installation':protocol['installation'],'home_q':c['home_q'],'camera_offset_m':[.2,-.75,.36],'robot_base_pose':protocol['nominal_initial_base'],'support_bottom_z_m':-.56,'saved_grasp_poses':[],'task':'held-out normal closed start / fixed base / single grasp / actual >=30 deg'}
 (src/'report.json').write_text(json.dumps(source,indent=2));model=NativeModel(None,d/'asset',source)
 export={'schema':'authored-closed-geometry-offline/v1','not_native_PhysX_cooking':True,'shapes':[]};tree=ET.parse(model.asset_urdf).getroot()
 for l in tree.findall('link'):
  n=l.get('name');T=model.asset_T@model.asset.root_to_link(n,{model.manifest['joint_name']:0.})
  for col in l.findall('collision'):
   hull=geometry(col,model.asset_urdf.parent).convex_hull;mesh=Path(col.find('geometry/mesh').get('filename')).stem
   export['shapes'].append({'path':f'/World/articulated_pose/asset/Geometry/{n}/{mesh}'.replace('-',''),'type':'Mesh','rigid_body_path':f'/World/articulated_pose/asset/{n}','world_transform':T.tolist(),'rigid_body_world_transform':T.tolist(),'convexes':[{'vertices':hull.vertices.tolist(),'polygons':hull.faces.tolist()}]})
 fix=fixture_box(m,model.asset,model.asset_urdf,model.asset_T[:3,:3],model.asset_T[:3,3],.04)
 T=transform(fix['center']);T[:3,:3]=Rotation.from_quat(np.roll(fix['quaternion_wxyz'],-1)).as_matrix()
 for n,h,X in [('fixture',trimesh.creation.box(fix['size']),T),('table',trimesh.creation.box([.9,.9,.05]),transform([.5,0,-.025]))]:export['shapes'].append({'path':'/World/'+n,'type':'Mesh','rigid_body_path':'/World/'+n,'world_transform':X.tolist(),'rigid_body_world_transform':X.tolist(),'convexes':[{'vertices':h.vertices.tolist(),'polygons':h.faces.tolist()}]})
 (d/'authored_offline_export.json').write_text(json.dumps(export));j=copy.deepcopy(template)
 j.update(asset_id=a,asset_root=str(d/'asset'),source=str(src),output=str(d/'physical_01'),plan=str(d/'grasp_plan.json'),whole_task_plan=str(d/'planning/candidate.json'),collision_export=str(d/'authored_offline_export.json'),gpu=4,episode_id='native_g1_heldout_'+a,deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=24)).isoformat(),grasp_geometry_candidates=[row['selection']],benchmark_goal_rad=float(np.pi/6),authored_target_rad=m['source_joint_limits_rad']['upper'],planning_base_seeds=[],classification=reg['classification'])
 j['expected_runner_sha256']=reg['frozen_source_sha256'][str(ROOT/'scripts/run_xarm7_native_transfer_repair.py')]
 joint=tree.find("joint[@name='%s']"%m['joint_name'])
 j['skill_inputs']['handle_region']={'source':'oracle_handle','asset_id':a,'mesh':row['registered_handle_mesh']}
 j['skill_inputs']['articulation_model']={'type':'known_revolute','joint':m['joint_name'],'axis':joint.find('axis').get('xyz'),'authored_limits':m['source_joint_limits_rad'],'urdf_origin':joint.find('origin').attrib,'origin_frame':'parent','axis_frame':'joint'}
 for key in ['benchmark_root','development_root']:j.pop(key,None)
 (d/'planning_job.json').write_text(json.dumps(j,indent=2));print('INPUT_READY',a,flush=True)
extra={}
for key in ['baseline_runner','pink_runner','frozen_execution_runner','active_closure_source','shared_policy']:
 p=Path(template[key]);extra[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['NATIVE_CONFIG.json','xarm7_native_gripper.urdf','xarm7_native_dynamics.urdf']:
 p=ROOT/'results/xarm7_native_gripper_20261012'/name;extra[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(O/'EXTRA_FROZEN_INPUT_HASHES.json').write_text(json.dumps(extra,indent=2));print('INPUTS_READY_NO_PLANNING')
