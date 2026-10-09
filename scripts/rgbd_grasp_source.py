"""RGB-D grasp-pose adapter. No GT grasp/template inputs or fallback.
Known collision geometry is used only by the unchanged feasibility planner.
"""
import json,os,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from estimate_rgbd_handle import estimate

def infer(folder,job,root):
 folder=Path(folder);sam=job['wrist_experiment']['sam3']
 env=dict(os.environ,PYTHONPATH=sam['pythonpath'],CUDA_VISIBLE_DEVICES=str(job.get('visual_gpu',5)),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1')
 cmd=[sam['python'],str(root/'scripts/infer_sam3_part.py'),'--rgb',str(folder/'rgb.png'),'--prompt',job['visual_prompt'],'--checkpoint',sam['checkpoint'],'--output',str(folder/'handle_mask.npy')]
 with (folder/'sam3.log').open('w') as log:subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 return estimate(folder)

def plan_visual(visual,robot_model,collision,moving_pose,base,output):
 from interactive_twin.planning import plan_grasps
 # The existing visual bar family, with observed dimensions and official robot pads.
 import copy
 rows=[];result=None
 # Both anchors come from the same RGB-D estimator output. Native collision
 # can select among these poses, but never shifts or corrects either pose.
 for label,anchor in [('cross_section_center',visual['anchor_world_m']),('observed_front_surface',visual['front_surface_anchor_world_m'])]:
  v=copy.deepcopy(visual);v['anchor_world_m']=anchor
  from interactive_twin.manifest import DEFAULT_POLICY
  grid=copy.deepcopy(DEFAULT_POLICY['candidate_grid']);grid['depth_offsets_m']=visual.get('depth_candidate_order_m',[.002,0.])
  candidate=plan_grasps(robot_model,collision,moving_pose,v,base,budget=12,stop_after_first=True,wall_clock_s=180,candidate_grid=grid)
  for row in candidate['rows']:row['visual_anchor_hypothesis']=label
  rows.extend(candidate['rows']);result=candidate;result['rows']=rows
  if result['trial_candidates']:
   for row in result['trial_candidates']:row['visual_anchor_hypothesis']=label
   break
 result['visual_candidate_origins']='RGB-D cross-section estimate and observed surface; no GT pose correction'
 Path(output).write_text(json.dumps(result,indent=2));return result

def estimate_and_plan(g,folder):
 from interaction_identification.contact_probe import robot_only_model
 visual=infer(folder,g['job'],g['ROOT'])
 if g['job'].get('visual_hypothesis')=='observed_side_face':
  face=visual.get('observed_side_face')
  if face is not None:
   visual=dict(visual,anchor_world_m=face['anchor_world_m'],outward_normal_world=face['outward_normal_world'],depth_candidate_order_m=[0.,.002],along_handle_offset_m=float(g['job'].get('visual_along_handle_offset_m',0.)))
   (g['a'].output/'selected_visual_hypothesis.json').write_text(json.dumps(face,indent=2))
 rm=robot_only_model(g['ROOT']/'config/piper.urdf')
 result=plan_visual(visual,rm,g['collision'],g['moving'](),g['base'],g['a'].output/'visual_plan.json')
 (g['a'].output/'visual_source_boundary.json').write_text(json.dumps({'estimator_inputs':['RGB','optical-Z depth','camera intrinsics/extrinsics','RGB-only SAM3 mask'],'candidate_generator':'existing bar-side-pinch with robot-only pad geometry','GT_grasp_or_template_used':False,'GT_fallback':False,'known_remaining_dependencies':['scene initialization','base station','native collision scene','contact pair identification','passive articulation evaluation']},indent=2))
 if not result['trial_candidates']:raise RuntimeError('VISUAL_CANDIDATES_NOT_REACHABLE_SCENE_PRESERVED')
 chosen=result['trial_candidates'][0];return chosen,np.asarray(chosen['T'])

def refine_pregrasp(g,folder,old_T):
 import open3d as o3d
 initial=g['a'].output/'rgbd/initial';visual=infer(folder,g['job'],g['ROOT'])
 from interaction_identification.contact_probe import robot_only_model
 from wrist_reconstruction.self_observation import robot_projection_mask,common_release_points
 rm=robot_only_model(g['ROOT']/'config/piper.urdf');frames=[]
 for directory in (initial,Path(folder)):
  meta=json.loads((directory/'camera.json').read_text());q=np.asarray(meta['robot_q']);K=np.asarray(meta['K']);C=np.asarray(meta['T_world_camera_optical']);depth=np.load(directory/'depth_m.npy')
  poses=rm.poses(q[:6],meta['base'],finger_q=q[6:8])
  excluded=robot_projection_mask(depth,K,C,rm.robot_hulls,poses)
  np.save(directory/'robot_self_exclusion.npy',excluded)
  frames.append(dict(K=K,T_world_camera=C,depth_m=depth,robot_q_self_mask=excluded))
 # Reuse existing common-visibility observation; robot occlusion must not become
 # fictitious object motion. The anchor is the initial perception estimate.
 anchor=json.loads((initial/'visual_handle.json').read_text())['anchor_world_m']
 A,B=common_release_points(*frames,np.asarray(anchor))
 def cloud(p):
  c=o3d.geometry.PointCloud();c.points=o3d.utility.Vector3dVector(p);return c.voxel_down_sample(.003)
 # Same local RGB-D registration algorithm as the existing observer. Residuals
 # are reported only, without the legacy model-consistency execution veto.
 fit=o3d.pipelines.registration.registration_icp(cloud(A),cloud(B),.015,np.eye(4),o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
 D=np.asarray(fit.transformation);T=D@old_T;start=g['tcp']();seed=np.asarray(g['robot'].get_joint_positions())[g['arm']];approach=[seed.tolist()]
 rot=Rotation.from_matrix(T[:3,:3]@start[:3,:3].T).as_rotvec()
 for f in np.linspace(0,1,21)[1:]:
  E=start.copy();E[:3,3]=(1-f)*start[:3,3]+f*T[:3,3];E[:3,:3]=Rotation.from_rotvec(f*rot).as_matrix()@start[:3,:3]
  q=g['ik'](E,seed)
  if q is None:raise RuntimeError('VISUAL_SERVO_LOCAL_IK_SCENE_PRESERVED')
  approach.append(q.tolist());seed=q
 audit={'source':'initial vs pregrasp common-visible RGB-D, measured-q robot exclusion, and measured TCP','common_visible_points':len(A),'D_visual':D.tolist(),'T_before':old_T.tolist(),'T_after':T.tolist(),'translation_correction_m':float(np.linalg.norm(T[:3,3]-old_T[:3,3])),'rotation_correction_deg':float(np.rad2deg(Rotation.from_matrix(D[:3,:3]).magnitude())),'registration_fitness':fit.fitness,'registration_rmse_m':fit.inlier_rmse,'metrics_classification':'DIAGNOSTIC_ONLY','GT_pose_input':False}
 (g['a'].output/'visual_correction.json').write_text(json.dumps(audit,indent=2));return T,approach
