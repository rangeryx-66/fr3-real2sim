"""Post-exit diagnostic only; never imported by the Agent or used to correct poses."""
import argparse,json,time
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import native_g1_multiview_input as first_seed
from xarm7_native_planning import NativeModel
from revolute10_fixed import authored_handle_colliders
from handle_contact_quality import handle_surfaces
from evaluate_native_g1_multiview_predictions import first
from visualize_native_g1_gt_observed import sample,distance,stats,pca,project

p=argparse.ArgumentParser();p.add_argument('--parent',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--archived-plan',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True);run=a.parent/'episode'
assert (a.parent/'PROCESS_EXIT.json').exists(),'Evaluation only after scene exit'
rows=json.loads((run/'AGENT_OBSERVATIONS.json').read_text());folders=[Path(x['folder']) for x in rows if x['accepted_for_geometry']]
start=time.perf_counter();old=first_seed.build_geometry(folders[0],folders[1:],a.output/'first_seed_replay')
job=json.loads((a.parent/'job.json').read_text());model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
export=json.loads((run/'cooked_initial.json').read_text());selection=job['grasp_geometry_candidates'][0]
gt,cooked=handle_surfaces(model,export,authored_handle_colliders(export,selection),selection)
G=sample(gt,30000);axis=pca(G);visible=np.zeros(len(G),bool)
for folder in folders:
 meta=json.loads((folder/'camera.json').read_text());xy,valid,dz=project(G,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),np.load(folder/'depth_m.npy'));valid[valid]&=~np.load(folder/'robot_self_exclusion.npy')[xy[valid,1],xy[valid,0]];visible|=valid&(abs(dz)<=.002)
plan=json.loads((run/'visual_plan.json').read_text());T=np.asarray(plan['T']);before=json.loads(a.archived_plan.read_text());Tb=np.asarray(before['T'])
geom=[]
for name,folder in [('first_seed_same_fresh_RGBD',a.output/'first_seed_replay'),('incremental_fresh_live',run/'agent_geometry')]:
 P=np.load(folder/'points_world.npy');mesh=trimesh.load(folder/'observed_patches.ply',process=False);d=distance(gt,P);nearest=cKDTree(P).query(G)[0]
 geom.append(dict(mode=name,points=len(P),extent_GT_axis_mm=float(np.ptp(P@axis)*1000),point_error=stats(d),GT_visible_samples=int(visible.sum()),visible_surface_coverage_within_2mm=float(np.mean(nearest[visible]<=.002)),all_surface_coverage_within_2mm=float(np.mean(nearest<=.002)),over3mm_points=int(np.sum(d>.003)),at_identical_selected_grasp=first(model,T,plan['base'],[mesh]),at_identical_archived_grasp=first(model,Tb,before['base'],[mesh])))
mesh=trimesh.load(run/'agent_geometry/observed_patches.ply',process=False)
predictions=dict(measured_only=first(model,T,plan['base'],[mesh]))
for i,scale in enumerate(first_seed.SPEC['rear_scales']):
 rear=trimesh.load(run/'agent_geometry'/('uncertain_rear_%d.ply'%i),process=False);predictions['with_assumed_rear_'+str(scale)]=first(model,T,plan['base'],[mesh,rear])
R=Tb[:3,:3].T@T[:3,:3]
summary=dict(GT_use='post-exit diagnostic only; no poses corrected; no execution admission thresholds',same_fresh_capture_replay=geom,selected_prediction=predictions,selected_GT_visual_evaluation=first(model,T,plan['base'],[gt]),selected_GT_cooked_evaluation=first(model,T,plan['base'],cooked,True),archived_plan=str(a.archived_plan),candidate_change=dict(translation_mm=float(np.linalg.norm(T[:3,3]-Tb[:3,3])*1000),rotation_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(R)-1)/2,-1,1)))),fresh_section_fraction=plan['section_fraction'],archived_section_fraction=before['section_fraction']),wall_s=time.perf_counter()-start,physical_grasp_validation=False)
(a.output/'OFFLINE_EVALUATION.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k not in ['selected_prediction']},indent=2))
