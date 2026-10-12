import json
from pathlib import Path
import numpy as np
from xarm7_native_gripper import ROOT,OUT
from xarm7_native_planning import NativeModel,common_quality,scene_factory
from xarm7_native_transfer_contact import QUALITY
from plan_shared_door_skill import authored_handle_colliders,handle_surfaces,at_base
O=ROOT/'results/xarm7_native_transfer_repair_20261012';result={}
for a in ['45134','45443','45249']:
 d=O/a;j=json.loads((d/'planning_job.json').read_text());m=NativeModel(None,j['asset_root'],json.loads((Path(j['source'])/'report.json').read_text()));e=json.loads(Path(j['collision_export']).read_text());sel=j['grasp_geometry_candidates'][0];allowed=authored_handle_colliders(e,sel);surfaces=handle_surfaces(m,e,allowed,sel)
 oldj=json.loads((OUT/a/'job.json').read_text());old=json.loads(Path(oldj['whole_task_plan']).read_text());choices={'frozen_plan':old}
 if (d/'planning/candidate.json').exists():choices['new_shared_plan']=json.loads((d/'planning/candidate.json').read_text())
 items={}
 for label,p in choices.items():
  T=np.array(p['T_grasp']);b=p['base'];tangent=(m.door_target(T,1e-4)[:3,3]-T[:3,3]);tangent/=np.linalg.norm(tangent);quality=QUALITY(m,surfaces,T,b,tangent);legacy=common_quality(m,surfaces,T,b,tangent)
  scene=scene_factory(ROOT,at_base(e,json.loads((Path(j['source'])/'report.json').read_text())['robot_base_pose'],b),m,allowed);checks=[]
  for item in p['path']:
   ok,why=scene.check(m.poses(np.array(item['q']),b,width=p['contact_geometry_audit']['physical_aperture_m']),m.door_target(scene.moving_reference,item['state']),True);checks.append({'angle_deg':float(np.rad2deg(item['state'])),'ok':ok,'reason':why,'margin':m.margin(np.array(item['q']))})
  items[label]={'roll':p['roll_deg'],'pitch':p['pitch_deg'],'insertion_m':p['insertion_m'],'section':p['section_fraction'],'normal_channel':quality['native']['best_common_pose_aperture']['native_pad_normal_opening_component'],'new_score':quality['score'],'legacy_score':legacy['score'],'bilateral_patch_support':quality['native']['best_common_pose_aperture']['sides'],'path_checks':checks,'planned_reached_deg':float(np.rad2deg(p['planned_reached_state']))}
 result[a]=items
(O/'OFFLINE_COMPARISON.json').write_text(json.dumps(result,indent=2))
for a,r in result.items():print(a,{k:{x:v[x] for x in ['pitch','roll','normal_channel','new_score','legacy_score','planned_reached_deg']} for k,v in r.items()},flush=True)
