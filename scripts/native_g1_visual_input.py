"""Observed RGB-D/SAM3 geometry adapter; frozen native G1 scores/control reused.

GT scene surfaces enter only the existing collision checks and contact labels.
They never enter observed_geometry, closing_quality, or contact ray support.
"""
import copy, hashlib, inspect, json, os, subprocess, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh
from scipy import ndimage
from scipy.spatial.transform import Rotation


def save_rgbd(g, label):
    import cv2
    out=g['a'].output/'rgbd'/label;out.mkdir(parents=True,exist_ok=True)
    camera=g['scene']['overview'][0]
    if 'distance_to_image_plane' not in camera._custom_annotators:
        camera.add_distance_to_image_plane_to_frame()
    for _ in range(8):g['world'].render()
    rgb=np.asarray(camera.get_rgba())[...,:3].copy()
    depth=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data()).copy()
    cv2.imwrite(str(out/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.save(out/'depth_m.npy',depth)
    meta=dict(K=np.asarray(camera.get_intrinsics_matrix()).tolist(),T_world_camera_optical=g['matrix'](*camera.get_world_pose(camera_axes='ros')).tolist(),depth_units='m optical Z',camera_source='fixed scene overview; full frame; no GT handle framing/cropping or instance mask',robot_q=np.asarray(g['robot'].get_joint_positions()).tolist(),robot_joint_names=g['names'],base=list(g['base']),T_world_tcp_measured=g['tcp']().tolist(),t_sim=g['tick']*g['dt'])
    (out/'camera.json').write_text(json.dumps(meta,indent=2))
    from wrist_reconstruction.self_observation import robot_projection_mask
    q=np.asarray(meta['robot_q']);P=g['model'].poses(q[g['arm']],g['base'],finger_q=q[g['fingers']])
    np.save(out/'robot_self_exclusion.npy',robot_projection_mask(depth,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),g['model'].robot_hulls,P))
    return out


def segment(folder,job,root):
    from native_g1_observed_regions import extend_observed_mask
    sam=job['wrist_experiment']['sam3']
    env=dict(os.environ,PYTHONPATH=str(root/'scripts')+':'+str(root)+':'+sam['pythonpath'],CUDA_VISIBLE_DEVICES=str(job['visual_gpu']),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1')
    cmd=[sam['python'],str(root/'scripts/infer_sam3_observed.py'),'--folder',str(folder),'--prompt',job['visual_prompt'],'--checkpoint',sam['checkpoint']]
    if not job.get('_visual_proposal_used',False):cmd+=['--allow-proposal']
    try:
        with (folder/'sam3.log').open('w') as log:subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
    finally:
        if (folder/'visual_detection_attempts.json').exists():
            attempts=json.loads((folder/'visual_detection_attempts.json').read_text())['attempts']
            if len(attempts)>1:job['_visual_proposal_used']=True
    raw=np.load(folder/'handle_mask.npy').astype(bool);np.save(folder/'sam3_mask_raw.npy',raw)
    np.save(folder/'handle_mask.npy',raw&~np.load(folder/'robot_self_exclusion.npy'))
    extend_observed_mask(folder)


def reobserve_camera(g,folder):
    """One generic alternate full-frame view from observed panel and robot q."""
    sam=g['job']['wrist_experiment']['sam3'];env=dict(os.environ,PYTHONPATH=sam['pythonpath'],CUDA_VISIBLE_DEVICES=str(g['job']['visual_gpu']),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1')
    cmd=[sam['python'],str(g['ROOT']/'scripts/infer_sam3_part.py'),'--rgb',str(folder/'rgb.png'),'--prompt','cabinet','--checkpoint',sam['checkpoint'],'--output',str(folder/'observed_panel_mask.npy')]
    panel_mask=None
    try:
        with (folder/'panel_sam3.log').open('w') as log:subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
        panel_mask=np.load(folder/'observed_panel_mask.npy')
    except Exception as error:
        g['diagnostic_issue']('optional_panel_sam3_for_reobservation',error)
    meta=json.loads((folder/'camera.json').read_text());K=np.asarray(meta['K']);C=np.asarray(meta['T_world_camera_optical']);d=np.load(folder/'depth_m.npy');yy,xx=np.indices(d.shape);valid=np.isfinite(d)&(d>.02)&(d<3.)&~np.load(folder/'robot_self_exclusion.npy');valid=valid if panel_mask is None else valid&panel_mask;z=d[valid]
    Q=np.c_[(xx[valid]-K[0,2])*z/K[0,0],(yy[valid]-K[1,2])*z/K[1,1],z]@C[:3,:3].T+C[:3,3];rng=np.random.default_rng(61)
    if len(Q)>6000:Q=Q[rng.choice(len(Q),6000,replace=False)]
    best=None
    for _ in range(150):
        a,b,c=Q[rng.choice(len(Q),3,replace=False)];n=np.cross(b-a,c-a);n/=max(np.linalg.norm(n),1e-12)
        if abs(n[2])>.5:continue # known gravity, not a GT object normal
        keep=abs((Q-a)@n)<.003
        if best is None or keep.sum()>best[0]:best=(keep.sum(),Q[keep])
    if best is None:raise RuntimeError('NO_OBSERVED_PANEL_FOR_BOUNDED_REOBSERVATION')
    plane=best[1];target=np.median(plane,axis=0);_,_,V=np.linalg.svd(plane-target,full_matrices=False);normal=V[-1]
    observed=Q[abs((Q-target)@normal)<.06];lo,hi=np.quantile(observed,[.01,.99],axis=0);target=(lo+hi)/2
    corners=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
    if normal@(C[:3,3]-target)<0:normal=-normal
    side=np.cross([0,0,1.],normal);side/=np.linalg.norm(side)
    q=np.asarray(meta['robot_q']);P=g['model'].poses(q[g['arm']],g['base'],finger_q=q[g['fingers']]);robot_center=np.mean([P[n][:3,3] for n in g['model'].robot_hulls],axis=0)
    sign=1. if side@(target-robot_center)>=0 else -1.
    standoff=.85
    for standoff in np.linspace(.85,3.,45):
        eye=target+standoff*normal+sign*.30*side+np.array([0,0,.10]);forward=(target-eye)/np.linalg.norm(target-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward));camera_points=(corners-eye)@R;uv=camera_points@K.T;uv=uv[:,:2]/uv[:,2,None]
        if np.all(camera_points[:,2]>0) and np.all((uv[:,0]>=0)&(uv[:,0]<d.shape[1])&(uv[:,1]>=0)&(uv[:,1]<d.shape[0])):break

    g['scene']['overview'][0].set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros')
    (folder/'bounded_view_change.json').write_text(json.dumps(dict(source='visible depth plane, known gravity and measured robot geometry only',target_world=target.tolist(),eye_world=eye.tolist(),panel_inliers=len(plane),observed_framing_bounds_world=[lo.tolist(),hi.tolist()],standoff_m=float(standoff),GT_handle_pose_or_crop_used=False,rule='observed near-panel volume + camera FOV select standoff in 0.85..3m; 300mm side opposite robot centroid; +100mm height; same for all cases'),indent=2))


def observed_geometry(folder):
    """Only captured files. Loft visible profiles with a declared width-depth prior."""
    from estimate_rgbd_handle import estimate
    folder=Path(folder);v=estimate(folder);P=np.load(folder/'handle_points_world.npy')
    axis=np.asarray(v['axis_world']);normal=np.asarray(v['outward_normal_world']);side=np.cross(normal,axis);side/=np.linalg.norm(side)
    origin=np.asarray(v['front_surface_anchor_world_m']);F=np.column_stack((axis,side,normal));local=(P-origin)@F
    lo,hi=np.quantile(local[:,0],[.05,.95]);length=hi-lo;knots=[]
    for s in np.linspace(lo,hi,9):
        Q=local[abs(local[:,0]-s)<length/8]
        if len(Q)<8:Q=local[np.argsort(abs(local[:,0]-s))[:max(8,len(local)//12)]]
        left,right=np.quantile(Q[:,1],[.02,.98]);front=float(np.quantile(Q[:,2],.9));width=float(right-left)
        # Unseen rear face is assumed one OBSERVED local width behind front.
        knots.append(dict(s=float(s),left=float(left),right=float(right),front=front,back=front-width,width=width))
    parts=[]
    for a,b in zip(knots,knots[1:]):
        V=np.array([[k['s'],u,z] for k in (a,b) for u in (k['left'],k['right']) for z in (k['back'],k['front'])])
        parts.append(trimesh.convex.convex_hull(V@F.T+origin))
    mask=np.load(folder/'handle_mask.npy').astype(bool);meta=json.loads((folder/'camera.json').read_text());K=np.asarray(meta['K']);C=np.asarray(meta['T_world_camera_optical']);d=np.load(folder/'depth_m.npy');yy,xx=np.indices(d.shape)
    ring=ndimage.binary_dilation(mask,iterations=25)&~ndimage.binary_dilation(mask,iterations=4)&np.isfinite(d)&(d>.02)&(d<5)
    z=d[ring];Q=np.c_[(xx[ring]-K[0,2])*z/K[0,0],(yy[ring]-K[1,2])*z/K[1,1],z]@C[:3,:3].T+C[:3,3]
    # The normal was fitted from the same observed SAM ring, not the asset panel.
    plane=float(np.median(Q@normal));flat=Q[abs(Q@normal-plane)<.003]
    uv=(flat-origin)@F[:,:2];bounds=np.quantile(uv,[.01,.99],axis=0)
    corners=np.array([[s,u,z] for s in bounds[:,0] for u in bounds[:,1] for z in (plane-normal@origin-.01,plane-normal@origin)])
    panel=trimesh.convex.convex_hull(corners@F.T+origin)
    sections=[]
    for frac in (-.3,0.,.3):
        t=(lo+hi)/2+frac*length
        k={key:float(np.interp(t,[p['s'] for p in knots],[p[key] for p in knots])) for key in ('left','right','front','back','width')}
        center=np.array([t,(k['left']+k['right'])/2,(k['front']+k['back'])/2])@F.T+origin
        sections.append(dict(fraction=frac,anchor_world_m=center.tolist(),bar_depth_m=k['width'],observed_front_world_m=(np.array([t,(k['left']+k['right'])/2,k['front']])@F.T+origin).tolist()))
    out=folder/'observed_geometry.json'
    report=dict(source='RGB-D points selected by full-RGB SAM3; observed panel ring',GT_handle_surface_used=False,simulator_mask_used=False,axis_world=axis.tolist(),normal_world=normal.tolist(),dimensions_m=[length,float(np.median([p['width'] for p in knots])),float(np.median([p['width'] for p in knots]))],sections=sections,profile_knots=knots,assumptions=['unobserved local handle backside is one measured transverse width behind visible front','piecewise linear convex loft between nine observed longitudinal profiles','observed panel ring is extended to its observed rectangular bounds with 10mm inward thickness for scoring only','no original scene collider is changed'],parts=[dict(vertices=m.vertices.tolist(),faces=m.faces.tolist()) for m in parts],panel=dict(vertices=panel.vertices.tolist(),faces=panel.faces.tolist()),input_sha256=v['input_sha256'],legacy_estimate=v)
    hypotheses=[]
    for scale in (.75,1.,1.25):
        meshes=[]
        for a,b in zip(knots,knots[1:]):
            V=np.array([[k['s'],u,z] for k in (a,b) for u in (k['left'],k['right']) for z in (k['front']-scale*k['width'],k['front'])])
            m=trimesh.convex.convex_hull(V@F.T+origin);meshes.append(dict(vertices=m.vertices.tolist(),faces=m.faces.tolist()))
        hypotheses.append(dict(hidden_depth_scale=scale,parts=meshes))
    report['hidden_geometry_hypotheses']=hypotheses
    report['hypothesis_rule']='same observed front/lateral surfaces; hidden depth 0.75/1/1.25 times observed width; existing mean/lower-quartile weights aggregate all, never choose best imagined backside'
    out.write_text(json.dumps(report,indent=2));return report


def rank_surfaces(folder):
    data=json.loads((Path(folder)/'observed_geometry.json').read_text())
    parts=[trimesh.Trimesh(p['vertices'],p['faces'],process=False) for p in data['parts']]
    # Frozen API's visual/native labels both represent OBSERVATIONS + PRIOR here.
    combined=trimesh.util.concatenate(parts)
    hypotheses=[]
    for h in data['hidden_geometry_hypotheses']:
        mesh=trimesh.util.concatenate([trimesh.Trimesh(p['vertices'],p['faces'],process=False) for p in h['parts']])
        hypotheses.append((mesh,[mesh]))
    return combined,[combined],hypotheses


def closing_scene(model,folder):
    from piper_mobile_demo.owned_scene import Shape
    data=json.loads((Path(folder)/'observed_geometry.json').read_text())
    robot=[Shape(m,np.eye(4),True,'robot/'+n,n) for n,m in model.robot_hulls.items()]
    scene=[];allowed=[]
    for i,p in enumerate(data['parts']):
        name='observed/handle/'+str(i);allowed.append(name)
        scene.append(Shape(trimesh.Trimesh(p['vertices'],p['faces'],process=False),np.eye(4),True,name,'observed'))
    p=data['panel'];scene.append(Shape(trimesh.Trimesh(p['vertices'],p['faces'],process=False),np.eye(4),True,'observed/panel','observed'))
    return SimpleNamespace(robot=robot,scene=scene,allowed=set(allowed),moving_link='observed',moving_reference=np.eye(4),jaw_bounds={n:np.array([model.robot_hulls[n].vertices.min(0),model.robot_hulls[n].vertices.max(0)]) for n in ('gripper_link1','gripper_link2')})


def plan_observed(g,folder):
    import xarm7_native_planning as native
    from native_g1_visual_hypothesis_rank import rescore_visual_geometries as rescore_geometries
    data=observed_geometry(folder);model=g['model'];inv=np.linalg.inv(model.asset_T)
    selection=dict(mesh='OBSERVED_RGBD_PRIOR_NOT_GT',dimensions_m=data['dimensions_m'],axis_root=(inv[:3,:3]@data['axis_world']).tolist(),outward_normal_root=(inv[:3,:3]@data['normal_world']).tolist(),sections=[dict(x,anchor_root_m=(inv@np.r_[x['anchor_world_m'],1])[:3].tolist()) for x in data['sections']])
    job=copy.deepcopy(g['job']);job['grasp_geometry_candidates']=[selection];job.pop('cached_closing_geometry_audit',None);job['planning_base_seeds']=[]
    job['collision_export']=str(g['a'].output/'cooked_initial.json');job['visual_geometry_folder']=str(folder);job['visual_allowed_colliders']=list(g['allowed']);out=g['a'].output/'visual_planning'
    source=inspect.getsource(native.plan)
    injection='''
 text=text.replace("allowed=authored_handle_colliders(export,selection)","allowed=job['visual_allowed_colliders']")
 text=text.replace("surfaces=handle_surfaces(model,export,allowed,selection)","surfaces=rank_surfaces(job['visual_geometry_folder'])")
 text=text.replace("quality=closing_quality(model,sc0,T,initial,width)","quality=closing_quality(model,closing_scene(model,job['visual_geometry_folder']),T,initial,width)")
 text=text.replace("bases=[initial]+job.get('planning_base_seeds',[])+candidate_bases(initial,{'anchor_world_m':anchor.tolist(),'outward_normal_world':normal.tolist()},job['wrist_experiment']['mobile_scan'])","bases=[initial] # registered known-model station retained")
 anchor="    geometries.sort(key=lambda g:g['contact']['rank'],reverse=True)"
 robust="""    config=json.loads(Path(job['robust_uncertainty_config']).read_text())
    J=model.asset_T@model.asset.root_to_link(link,{joint:0.})
    geometries=rescore_geometries(geometries,model,surfaces,initial,J[:3,3],J[:3,:3]@model.asset.joints[link].axis,config,output)
"""
 text=text.replace(anchor,robust+anchor)
'''
    source=source.replace(' ns=dict(shared.__dict__)',injection+' ns=dict(shared.__dict__)')
    source=source.replace("ns['Model']=NativeModel", "ns.update(rank_surfaces=rank_surfaces,closing_scene=closing_scene,rescore_geometries=rescore_geometries);ns['Model']=NativeModel")
    ns=dict(native.__dict__,rank_surfaces=rank_surfaces,closing_scene=closing_scene,rescore_geometries=rescore_geometries)
    exec(compile(source,__file__+'::observed_input_only','exec'),ns)
    (g['a'].output/'VISUAL_PLANNING_ADAPTER_EXECUTED.py').write_text(source)
    candidate=ns['plan'](job,out,wall_s=float(job['visual_planning_wall_s']))
    if candidate is None:raise RuntimeError('BOUNDED_VISUAL_APPROACH_IK_SEARCH_EXHAUSTED')
    candidate['classification']='RGBD_SAM3_HANDLE / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION';candidate['contact_score_surface_source']='observed loft and declared prior; API visual/native both use this same geometry';candidate['station_source']='registered oracle station only; no saved grasp transform'
    (g['a'].output/'visual_plan.json').write_text(json.dumps(candidate,indent=2));return candidate,data


def record_perception_recovery(g):
    try:
        root=g['a'].output/'rgbd';entries=[json.loads(p.read_text()) for p in root.glob('initial*/visual_detection_attempts.json')]
        steps=[s for e in entries for s in e['attempts']]
        recovery=dict(crop_recovery_attempted=bool(g['job'].get('_visual_proposal_used')),crop_inferences=sum(s.get('kind')=='observed_depth_crop' for s in steps),handle_inferences=sum('raw_query_scores' in s for s in steps),camera_reobservations=len(list(root.glob('initial*/bounded_view_change.json'))),initial_captures=len(list(root.glob('initial*/depth_m.npy'))),auxiliary_panel_calls=len(list(root.glob('initial*/panel_sam3.log'))),pregrasp_capture_count=len(list(root.glob('pregrasp*/depth_m.npy'))),GT_fallback=False,detection_attempts=entries)
        (g['a'].output/'perception_recovery.json').write_text(json.dumps(recovery,indent=2))
    except Exception as error:g['diagnostic_issue']('perception_recovery_observer',error)


def prepare(g):
    attempts=[]
    g['job']['_visual_proposal_used']=False
    for attempt in range(g['job']['visual_reobservations']+1):
        if attempt:reobserve_camera(g,folder)
        folder=save_rgbd(g,'initial' if attempt==0 else 'initial_retry_'+str(attempt))
        try:
            segment(folder,g['job'],g['ROOT']);candidate,data=plan_observed(g,folder);break
        except Exception as error:
            attempts.append(repr(error));candidate=None
    record_perception_recovery(g)
    if candidate is None:
        (g['a'].output/'visual_failure.json').write_text(json.dumps(dict(stage='initial_perception_or_planning',attempts=attempts,GT_fallback=False),indent=2));raise RuntimeError('VISUAL_INPUT_UNAVAILABLE:'+str(attempts))
    g['visual_initial_folder']=folder;g['whole']=candidate;g['chosen']=dict(candidate);g['T']=np.asarray(candidate['T']);g['normal']=np.asarray(data['normal_world']);g['axis']=np.asarray(data['axis_world']);g['anchor_world']=np.asarray(data['legacy_estimate']['anchor_world_m']);g['anchor_link']=(np.linalg.inv(g['moving']())@np.r_[g['anchor_world'],1])[:3]
    # Recording close view uses only the now-observed handle, never a GT crop.
    eye=g['anchor_world']+g['normal']*.45+np.array([0,0,.20]);focus=g['anchor_world']+np.array([0,0,.02]);forward=(focus-eye)/np.linalg.norm(focus-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward));g['near'].set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros')
    boundary=dict(estimator_inputs=['fresh RGB','fresh optical-Z depth','K/extrinsics','SAM3 RGB-only mask','observed panel surface'],contact_scores='depth-completed observed loft + three hidden-depth hypotheses, original mean/lower-quartile weights; official robot geometry only',GT_grasp_fallback=False,remaining_known_dependencies=['registered initial station','scene mounting and robot/camera calibration','known hinge and dynamic body state','unchanged native scene collision checks','registered native collider identities for contact reporting'],fixed_perturbations_are_not_visual_confidence_bounds=True)
    (g['a'].output/'visual_source_boundary.json').write_text(json.dumps(boundary,indent=2))
    if g['job'].get('capture_shadow_only'):raise RuntimeError('SHADOW_CAPTURE_AND_PLANNING_COMPLETE_NO_MANIPULATION_ATTEMPT')


def refine(g):
    import open3d as o3d
    from wrist_reconstruction.self_observation import robot_projection_mask,common_release_points
    old=g['T'].copy();attempts=[]
    for attempt in range(g['job']['visual_reobservations']+1):
        folder=save_rgbd(g,'pregrasp' if attempt==0 else 'pregrasp_retry_'+str(attempt));frames=[]
        try:
            for directory in (g['visual_initial_folder'],folder):
                meta=json.loads((directory/'camera.json').read_text());q=np.asarray(meta['robot_q']);d=np.load(directory/'depth_m.npy');P=g['model'].poses(q[g['arm']],g['base'],finger_q=q[g['fingers']]);mask=robot_projection_mask(d,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),g['model'].robot_hulls,P);np.save(directory/'robot_self_exclusion.npy',mask)
                frames.append(dict(K=meta['K'],T_world_camera=meta['T_world_camera_optical'],depth_m=d,robot_q_self_mask=mask))
            A,B=common_release_points(*frames,g['anchor_world'])
            def cloud(points):
                c=o3d.geometry.PointCloud();c.points=o3d.utility.Vector3dVector(points);return c.voxel_down_sample(.003)
            fit=o3d.pipelines.registration.registration_icp(cloud(A),cloud(B),.015,np.eye(4),o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40));D=np.asarray(fit.transformation);T=D@old;start=g['tcp']();seed=np.asarray(g['robot'].get_joint_positions())[g['arm']];path=[seed.tolist()];rot=Rotation.from_matrix(T[:3,:3]@start[:3,:3].T).as_rotvec()
            for f in np.linspace(0,1,21)[1:]:
                X=start.copy();X[:3,3]=(1-f)*start[:3,3]+f*T[:3,3];X[:3,:3]=Rotation.from_rotvec(f*rot).as_matrix()@start[:3,:3];q=g['ik'](X,seed)
                if q is None:raise RuntimeError('VISUAL_PREGRASP_LOCAL_IK_UNAVAILABLE')
                ok,why=g['collision'].check(g['model'].poses(q,g['base'],width=.084),g['moving'](),False)
                if not ok:raise RuntimeError(why)
                path.append(q.tolist());seed=q
            g['T']=T;g['approach']=path
            audit=dict(source='common-visible initial/pregrasp RGB-D with measured native robot self-exclusion',GT_handle_pose_correction=False,D_visual=D.tolist(),T_before=old.tolist(),T_after=T.tolist(),common_points=len(A),fitness=fit.fitness,rmse_m=fit.inlier_rmse,pose_metrics='DIAGNOSTIC_ONLY; no threshold',attempt_errors=attempts)
            (g['a'].output/'visual_correction.json').write_text(json.dumps(audit,indent=2));record_perception_recovery(g);return
        except Exception as error:attempts.append(repr(error))
    raise RuntimeError('VISUAL_PREGRASP_REOBSERVATION_EXHAUSTED:'+str(attempts))


def adapt_setup(text):
    start=text.index("meta=model.manifest['interaction_geometry']['selection'];normal=")
    end=text.index('# Freeze and record',start)
    text=text[:start]+"normal=np.array([0.,-1.,0.]);axis=np.array([1.,0.,0.]);anchor_world=np.array([base[0],base[1],.1]);moving_initial=collision.moving_reference;anchor_link=(np.linalg.inv(moving_initial)@np.r_[anchor_world,1])[:3]\n"+text[end:]
    # The old fingerprint's metadata is archival only; initial scene camera is
    # fixed overview until the observed handle supplies a recording focus.
    text=text.replace("'selection':meta", "'selection':'recorded in original frozen manifest; not a visual input'")
    start=text.index("near=scene['overview'][1];eye=");end=text.index('\ntry:video=',start)
    text=text[:start]+"near=scene['overview'][1];near.set_world_pose(*scene['overview'][0].get_world_pose(camera_axes='ros'),camera_axes='ros');near.set_clipping_range(.02,3.)"+text[end:]
    return text
