"""Rank RGB-D grasps by observed pad support and short known-hinge motion.
No GT handle/template pose is an input. Existing native collision and IK are
used for candidate choice, not a new execution certificate.
"""
import copy,json
from pathlib import Path
import numpy as np
from rgbd_grasp_source import infer
from scipy.spatial.transform import Rotation


def estimate_and_plan(g,folder):
 from interaction_identification.contact_probe import robot_only_model
 from interactive_twin.planning import plan_grasps
 from interactive_twin.manifest import DEFAULT_POLICY
 visual=infer(folder,g['job'],g['ROOT']);rm=robot_only_model(g['ROOT']/'config/piper.urdf')
 points=np.load(Path(folder)/'handle_points_world.npy');rows=[];candidates=[]
 # A visible front sheet is not an opposed-contact cross-section center.
 # Use the measured bar width as a thickness hypothesis instead of the poorly
 # constrained circle extrapolation. This is an explicit visual shape prior,
 # never a native/GT handle pose or a per-trial manually chosen offset.
 normal=np.asarray(visual['outward_normal_world']);normal/=np.linalg.norm(normal)
 width_center=np.asarray(visual['front_surface_anchor_world_m'])-normal*float(visual['dimensions_m'][1])/2
 contact_points=points-np.outer((points-width_center)@normal,normal)
 (g['a'].output/'visual_contact_hypothesis.json').write_text(json.dumps({'source':'RGB-D visible front surface plus measured width / 2 thickness hypothesis','anchor_world_m':width_center.tolist(),'measured_width_m':visual['dimensions_m'][1],'unobserved_thickness_is_prior':True,'GT_pose_input':False},indent=2))
 grid=copy.deepcopy(DEFAULT_POLICY['candidate_grid']);grid['depth_offsets_m']=[.002,0.]
 model=g['model'];theta=float(g['scene']['articulation'].get_joint_positions()[g['scene'].get('selected_asset_dof',0)])
 origin=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:theta})
 for label,anchor in [('visible_width_mid_depth',width_center.tolist()),('observed_front_surface',visual['front_surface_anchor_world_m']),('cross_section_center',visual['anchor_world_m'])]:
  v=copy.deepcopy(visual);v['anchor_world_m']=anchor
  result=plan_grasps(rm,g['collision'],g['moving'](),v,g['base'],budget=12,stop_after_first=False,wall_clock_s=180,candidate_grid=grid)
  for row in result['rows']:row['visual_anchor_hypothesis']=label
  rows.extend(result['rows'])
  for c in result['trial_candidates']:
   c['visual_anchor_hypothesis']=label;seed=np.asarray(c['q_grasp']);T=np.asarray(c['T'])
   # Estimated opposed-contact support in the real robot's distal pad coordinates.
   # Width changes do not change pad-local insertion/longitudinal coordinates.
   P=rm.poses(seed,g['base'],width=float(visual['dimensions_m'][1]));supports=[];depth_errors=[]
   for name in ('gripper_link1','gripper_link2'):
    local=(np.linalg.inv(P[name])@np.column_stack((contact_points,np.ones(len(contact_points)))).T).T[:,:3];bounds=rm.pads[name]
    local=local[(local[:,0]>=bounds[0,0])&(local[:,0]<=bounds[1,0])]
    supports.append(float(np.mean((local[:,1]>=bounds[0,1])&(local[:,1]<=bounds[1,1]))) if len(local) else 0.)
    depth_errors.append(float(np.median(abs(local[:,1]-bounds[:,1].mean()))) if len(local) else 1.)
   short=[];reason='sampled_15_deg';width=float(visual['dimensions_m'][1])
   for deg in range(1,16):
    state=theta+np.deg2rad(deg);body=model.asset_T@model.asset.root_to_link(model.manifest['moving_link'],{model.manifest['joint_name']:state});target=body@np.linalg.inv(origin)@T
    q=model.ik(target,g['base'],seed,starts=1)
    if q is None:reason='NO_IK';break
    ok,why=g['collision'].check(model.poses(q,g['base'],width=width),body,True)
    if not ok:reason=why;break
    short.append({'additional_deg':deg,'margin_rad':model.margin(q),'q':q.tolist()});seed=q
   coverage=float(np.mean(supports));reach=float(short[-1]['additional_deg']) if short else 0.
   c['manipulation_selection']={'width_prior_pad_support_fraction':coverage,'contact_support_source':'RGB-D width-derived mid-depth; unobserved thickness prior','pad_depth_error_m':float(np.mean(depth_errors)),'short_reachable_deg':reach,'short_path':short,'first_local_blocker':reason,'classification':'CANDIDATE_RANKING_ONLY','score':reach+15*coverage}
   candidates.append(c)
 candidates.sort(key=lambda c:(c['manipulation_selection']['score'],-c['manipulation_selection']['pad_depth_error_m'],c['minimum_joint_margin_rad']),reverse=True)
 result={'rows':rows,'trial_candidates':candidates,'best':candidates[0] if candidates else None,'ranking':'short reachable degrees + 15 * width-prior distal-pad contact support; depth centering then original margin tie-break','no_full_path_requirement':True,'known_motion_model_used_for_ranking':True,'GT_grasp_or_template_used':False}
 (g['a'].output/'visual_plan.json').write_text(json.dumps(result,indent=2))
 (g['a'].output/'visual_source_boundary.json').write_text(json.dumps({'estimator_inputs':['fresh actual RGB','optical Z depth','camera K/extrinsics','RGB-only SAM3 mask'],'grasp_pose_generation':'unchanged visual bar candidates using RGB-D axes/anchors and official robot pad geometry','GT_grasp_or_template_used':False,'GT_fallback':False,'known_dependencies':['scene setup','base station','native collision','contact pairing','known hinge motion for ranking and control']},indent=2))
 if not candidates:raise RuntimeError('NO_VISUAL_APPROACH_CANDIDATE_SCENE_PRESERVED')
 chosen=candidates[0];return chosen,np.asarray(chosen['T'])


def refine_pregrasp(g,folder,old_T):
 import open3d as o3d
 initial=g['a'].output/'rgbd/initial'
 # The second SAM mask is not an input to common-visible RGB-D registration.
 # The arm can occlude the handle; retain the initial visual pose and register
 # the remaining visible RGB-D instead of turning absent re-segmentation fatal.
 try:infer(folder,g['job'],g['ROOT'])
 except Exception as error:g['diagnostic_issue']('pregrasp_resegmentation',error)
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
