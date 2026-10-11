"""Offline diagnostics across original A; never overwrite prior results."""
from pathlib import Path
import sys,json,numpy as np
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from revolute10_fixed import Model,authored_handle_colliders
from common_aperture_contact import handle_surfaces,common_contact_quality
R=ROOT/'results/benchmarkA_30deg_20261011';out=ROOT/'results/door_transfer4_20261012';reg=json.loads((R/'REGISTERED_OBJECTS.json').read_text());ranking=json.loads((R/'DISCOVERY_GEOMETRY_RANKING.json').read_text())['ranking'];results=json.loads((R/'BENCHMARK_A_RESULTS.json').read_text());diagnostics=[]
for item,result in zip(reg['objects'],results['cases']):
 i=item['asset_id'];asset=R/'discovery_assets'/i;src=json.loads((R/i/'source/report.json').read_text());model=Model(ROOT/'config/piper.urdf',asset,src);selection=next(x for x in ranking if x['asset_id']==i and x['mesh']==item['eligibility_handle_mesh']);export=json.loads((R/i/'setup/cooked_initial.json').read_text());allowed=authored_handle_colliders(export,selection);normal=model.asset_T[:3,:3]@selection['outward_normal_root'];axis=model.asset_T[:3,:3]@selection['axis_root'];normal-=axis*(normal@axis);normal/=np.linalg.norm(normal);P=model.poses(model.home,src['robot_base_pose'],width=.1);offset=np.mean([(np.linalg.inv(P['tcp_link'])@P[f]@np.r_[model.pads[f].mean(0),1])[:3] for f in ['gripper_link1','gripper_link2']],axis=0);surfaces=handle_surfaces(model,export,allowed,selection);samples=[]
 for s in selection['sections']:
  T=np.eye(4);T[:3,:3]=np.column_stack((axis,np.cross(-normal,axis),-normal));T[:3,3]=(model.asset_T@np.r_[s['anchor_root_m'],1])[:3]-T[:3,:3]@offset;M0=model.asset_T@model.asset.root_to_link(selection['moving_link'],{selection['joint_name']:0.});D=model.asset_T@model.asset.root_to_link(selection['moving_link'],{selection['joint_name']:1e-4})@np.linalg.inv(M0);q=common_contact_quality(model,surfaces,T,src['robot_base_pose'],(D@T)[:3,3]-T[:3,3]);samples.append({'section':s['fraction'],'common_contact':q,'visual_rear_gap_m':s['visual_rear_gap_m'],'native_rear_gap_m':s['collision_rear_gap_m']})
 category='candidate/planning'
 if result['failure_category']=='IK/joint margin':category='hardware/configuration (executed workspace boundary)'
 elif result['failure_category'] in ['contact loss','grasp failure','collision/contact placement','speed protection']:category='contact-transmission/contact placement; existing protection retained'
 if result.get('actual_selected_grasp_mesh') and result['actual_selected_grasp_mesh']!=item['eligibility_handle_mesh']:category='candidate selection (wrong semantic region)'
 diagnostics.append({'asset_id':i,'original_failure_category':result['failure_category'],'shared_mechanism_category':category,'intended_handle':item['eligibility_handle_mesh'],'previous_selected_mesh':result.get('actual_selected_grasp_mesh'),'common_aperture_samples':samples,'collision_representation_warning':any(s['collision_rear_gap_m']<=0<s['visual_rear_gap_m'] for s in selection['sections']),'physical_attempts_this_diagnostic':0})
 print('AUDIT',i,category,diagnostics[-1]['collision_representation_warning'],flush=True)
(out/'BENCHMARK_A_CAPABILITY_AUDIT.json').write_text(json.dumps(diagnostics,indent=2))
