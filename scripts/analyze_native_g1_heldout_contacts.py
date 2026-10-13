"""Saved-data-only contact comparison. No scene, planner, or controller writes."""
from pathlib import Path
import json,gzip,pickle,collections,sys
import numpy as np
from scipy.spatial.transform import Rotation
from xarm7_native_gripper import NativeGripperGeometry,FINGER_BODIES
from xarm7_native_planning import PAD_FRAMES,LOGICAL
from articulated_demo.kinematics import URDFChain
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/native_g1_heldout_20261013'
reg=json.loads((O/'REGISTERED_OBJECTS.json').read_text());geometry=NativeGripperGeometry();pads={}
for body,label in zip(FINGER_BODIES,LOGICAL):
 mesh=geometry.meshes[body].copy();mesh.apply_transform(np.linalg.inv(PAD_FRAMES[body]));inner=mesh.vertices[:,2].min();faces=np.max(abs(mesh.triangles[:,:,2]-inner),axis=1)<1e-6;v=mesh.triangles[faces].reshape(-1,3);pads[label]=np.array([v.min(0),v.max(0)])
def summarize(samples):
 if len(samples)==0:return None
 v=np.asarray(samples);return {'min':float(v.min()),'median':float(np.median(v)),'max':float(v.max())}
results=[]
for case in reg['objects']:
 a=case['asset_id'];d=O/a;p=d/'physical_01';r=json.loads((p/'report.json').read_text());candidate=json.loads((d/'planning/candidate.json').read_text());patch=candidate['contact_geometry_audit']['patch'];m=json.loads((d/'asset/manifest.json').read_text());source=json.loads((d/'source/report.json').read_text());place=source['asset_installation'];B=np.eye(4);B[:3,:3]=Rotation.from_euler('z',place['yaw_deg'],degrees=True).as_matrix()@np.array([[0,0,1],[1,0,0],[0,1,0]]);B[:3,3]=[place['x_m'],place['y_m'],-m['scale_source_to_meters']*m['static_source_bounds'][0][1]+place['fixture_height_m']];chain=URDFChain(d/'asset/urdf'/f'{a}.urdf');joint=chain.joints[m['moving_link']];J=B@chain.root_to_link(m['moving_link'],{m['joint_name']:0.});axis=J[:3,:3]@joint.axis;origin=J[:3,3]
 meta=json.loads((p/'joint_sensor_index_metadata.json').read_text());arm=[meta['dof_names'].index('joint%d'%i) for i in range(1,8)];plan_states=np.asarray([v['state'] for v in candidate['path']]);plan_joints=np.asarray([v['q'] for v in candidate['path']]);plan_joint_errors=[]
 groups={};last_close=None;first_positive={};last_acquisition=(r.get('grasp_acquisition') or {}).get('t',float('inf'));contact_times={};first_phase=None;previous_cmd=None;active_steps=collections.Counter();stationary_steps=collections.Counter();initial_moving=None;dynamic_errors=[];postgrasp_progress=[];probe=[]
 for f in sorted((p/'observations_chunks').glob('*.gz')):
  for x in pickle.load(gzip.open(f,'rb')):
   phase=x['phase']
   measured=np.asarray(x['T_moving_link']);theta=np.deg2rad(x['door_angle_deg'])
   if initial_moving is None:initial_moving=measured.copy()
   Q=Rotation.from_rotvec(axis*theta).as_matrix();expected_R=Q@initial_moving[:3,:3];expected_p=origin+Q@(initial_moving[:3,3]-origin);dynamic_errors.append([float(np.linalg.norm(expected_p-measured[:3,3])),float(np.rad2deg(Rotation.from_matrix(expected_R@measured[:3,:3].T).magnitude()))])
   if x['t']>=last_acquisition:
    postgrasp_progress.append(x['door_angle_deg'])
    planned_q=np.array([np.interp(theta,plan_states,plan_joints[:,i]) for i in range(7)]);difference=np.asarray(x['q'])[arm]-planned_q;plan_joint_errors.append({'t':x['t'],'actual_deg':x['door_angle_deg'],'max_abs_joint_difference_rad':float(np.max(abs(difference))),'joint_differences_rad':difference.tolist()})
   if phase in ['TRANSMISSION_PROBE','LOCAL_CENTERING']:
    target=x.get('planned_T_tcp');probe.append({'t':x['t'],'phase':phase,'actual_deg':x['door_angle_deg'],'reference_deg':float(np.rad2deg(x['planned_articulation_state'])),'relative_drift_mm':x['relative_translation_slip_m']*1000,'tcp_reference_error_mm':None if target is None else float(np.linalg.norm(np.asarray(x['T_tcp'])[:3,3]-np.asarray(target)[:3,3])*1000)})
   command=np.r_[x['telemetry']['command_q_all'],x['base']];changing=previous_cmd is None or np.max(abs(command-previous_cmd))>1e-12;previous_cmd=command
   (active_steps if changing else stationary_steps)[phase]+=1
   if phase in ['SLOW_CLOSE','CENTER','LOW_PRELOAD','FORCE_HOLD']:
    last_close=x;group_phase=phase
   elif phase=='TRANSMISSION_PROBE':group_phase=phase
   elif phase=='OPEN_CONTINUE' and r.get('grasp_acquisition') and x['door_angle_deg']-r['grasp_acquisition']['angle_deg']<=5.:group_phase='FIRST_OPEN_5_DEG'
   else:continue
   g=groups.setdefault(group_phase,{'samples':0,'loads':[],'apertures':[],'command_apertures':[],'contacts':{n:[] for n in LOGICAL},'contact_steps':{n:0 for n in LOGICAL},'optional_contact_geometry_missing':0});g['samples']+=1;loads=[x['forces_n'][n] for n in LOGICAL];g['loads'].append(loads);g['apertures'].append(x['aperture_m']);g['command_apertures'].append(x['telemetry']['command_aperture_m'])
   for i,n in enumerate(LOGICAL):
    if loads[i]>.05 and n not in first_positive:first_positive[n]={'t':x['t'],'phase':phase,'aperture_m':x['aperture_m'],'command_aperture_m':x['telemetry']['command_aperture_m'],'load_n':loads[i]}
   touched=set()
   for c in x['contacts']:
    if not c['allowed_pad_target'] or c['force_n']<=0:continue
    n=c['finger']
    if c.get('contact_point_world_m') is None or c.get('native_normal_world') is None:
     g['optional_contact_geometry_missing']+=1;continue
    P=np.asarray(x['finger_world_poses'][n]);point=np.asarray(c['contact_point_world_m']);local=P[:3,:3].T@(point-P[:3,3]);normal=np.asarray(c['native_normal_world']);closing=P[:3,2];tangent=np.cross(axis,point-origin);tangent/=max(np.linalg.norm(tangent),1e-30);b=pads[n];inside=bool(np.all(local[:2]>=b[0,:2]) and np.all(local[:2]<=b[1,:2]));g['contacts'][n].append([c['force_n'],*local,abs(normal@closing),abs(normal@tangent),float(inside),abs(local[2]-b[0,2])]);touched.add(n)
   for n in touched:g['contact_steps'][n]+=1
 metrics={}
 for phase,g in groups.items():
  loads=np.asarray(g['loads']);out={'simulation_s':g['samples']/240,'mean_loads_n':loads.mean(0).tolist(),'load_duty_above_existing_0p05_n':(loads>.05).mean(0).tolist(),'aperture_m':summarize(g['apertures']),'command_aperture_m':summarize(g['command_apertures']),'optional_contact_geometry_missing':g['optional_contact_geometry_missing'],'contacts':{}}
  for n,points in g['contacts'].items():
   if not points:out['contacts'][n]={'positive_contact_records':0};continue
   z=np.asarray(points);w=z[:,0];out['contacts'][n]={'positive_contact_records':len(z),'contact_step_fraction':g['contact_steps'][n]/g['samples'],'force_weighted_local_contact_centroid_m':np.average(z[:,1:4],axis=0,weights=w).tolist(),'local_contact_bounds_m':[z[:,1:4].min(0).tolist(),z[:,1:4].max(0).tolist()],'force_weighted_abs_normal_alignment_to_closing':float(np.average(z[:,4],weights=w)),'force_weighted_abs_normal_alignment_to_opening_tangent':float(np.average(z[:,5],weights=w)),'force_weighted_fraction_inside_inner_pad_projection':float(np.average(z[:,6],weights=w)),'inner_pad_plane_distance_m':summarize(z[:,7])}
  metrics[phase]=out
 finalwidth=last_close['aperture_m'] if last_close else None;nearest=min(patch['native']['samples'],key=lambda x:abs(x['common_aperture_m']-finalwidth)) if finalwidth is not None else None
 results.append({'asset_id':a,'geometry':case['geometry_category'],'success':r['success'],'grasp_success':r['initial_grasp'],'visual_rear_gap_m':case['visual_rear_gap_m'],'collision_proxy_rear_gap_m':case['existing_proxy_rear_gap_m'],'chosen_pose':{k:candidate[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg','shared_selection_utility']},'predicted_best_common_aperture_m':patch['native']['best_common_pose_aperture']['common_aperture_m'],'predicted_best_common_support':patch['score'],'best_common_aperture_support':{k:patch[k]['best_common_pose_aperture'] for k in ['visual','native']},'nearest_static_planned_pose_sample_to_final_closure_aperture':nearest,'first_positive_pad_load':first_positive,'contact_phases':metrics,'postgrasp_max_additional_angle_deg':None if not postgrasp_progress else max(postgrasp_progress)-r['grasp_acquisition']['angle_deg'],'opening_execution_vs_saved_ik_path':None if not plan_joint_errors else {'maximum_joint_difference_record':max(plan_joint_errors,key=lambda v:v['max_abs_joint_difference_rad']),'last':plan_joint_errors[-1],'definition':'Measured arm q versus linear interpolation of the original saved endpoint-IK path at actual angle; descriptive only, no tracking gate'},'joint_to_moving_body_mapping_offline':{'maximum_position_error_mm':float(np.max(np.asarray(dynamic_errors)[:,0])*1000),'maximum_rotation_error_deg':float(np.max(np.asarray(dynamic_errors)[:,1])),'definition':'Known URDF joint axis/origin predicts delta applied to first measured closed body frame; static importer frame offset is held constant; diagnostic only'},'probe_summary':None if not probe else {'first':probe[0],'last':probe[-1],'max_actual_deg':max(x['actual_deg'] for x in probe),'reference_deg_range':[min(x['reference_deg'] for x in probe),max(x['reference_deg'] for x in probe)],'maximum_tcp_reference_error_mm':max((x['tcp_reference_error_mm'] for x in probe if x['tcp_reference_error_mm'] is not None),default=None)},'closure_end':None if last_close is None else {'t':last_close['t'],'aperture_m':last_close['aperture_m'],'command_aperture_m':last_close['telemetry']['command_aperture_m'],'loads_n':last_close['forces_n'],'tcp_position_shift_from_planned_grasp_mm':float(np.linalg.norm(np.asarray(last_close['T_tcp'])[:3,3]-np.asarray(candidate['T_grasp'])[:3,3])*1000)},'all_phase_command_activity_simulation_s':{k:{'command_changed':active_steps[k]/240,'command_stationary':stationary_steps[k]/240} for k in set(active_steps)|set(stationary_steps)}})
 print(a,r['success'],r['initial_grasp'],'final closure width',finalwidth,flush=True)
(O/'TEN_CASE_CONTACT_DIAGNOSTICS.json').write_text(json.dumps({'classification':'SAVED_DATA_OFFLINE_DIAGNOSTIC_ONLY','pad_bounds_logical_frame_m':{k:v.tolist() for k,v in pads.items()},'normal_sign_note':'absolute alignment; native collider ordering and signed impulses remain archived. No directional-friction inference.','static_sample_note':'nearest precomputed planned-pose aperture sample is not a replay of actual centered pose or dynamic contact','projection_note':'inner-pad XY footprint only; distances from its surface plane are reported separately; no new runtime contact or success gate','cases':results},indent=2))
