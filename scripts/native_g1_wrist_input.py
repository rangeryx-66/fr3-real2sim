"""Rigid USD wrist sensor + actuator-executed views; frozen visual skill reused.

No scene-camera pixel/depth API is called here. GT geometry is confined to the
existing collision checker and the coarse cabinet survey envelope. Handle
fusion, view proposals and grasp/contact scores consume wrist observations.
"""
import inspect,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import native_g1_visual_input as visual
import native_g1_multiview_input as multi
import native_g1_metric_rgbd_geometry as metric
from native_g1_wrist_views import calibration,survey_candidates,observed_candidates,select_route


def install_camera(g):
    from isaacsim.sensors.camera import Camera
    from pxr import UsdPhysics
    c,X=calibration(g['job']['wrist_calibration'])
    parent=next(p for p in g['stage'].Traverse() if p.GetName()==c['parent_body'] and p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/World/Piper'))
    # URDF fixed transform, not a measured/GT object transform.
    chain=g['model'].chain
    local=np.linalg.inv(chain.root_to_link(c['parent_body'],{}))@chain.root_to_link('tcp_link',{})@X
    camera=Camera(str(parent.GetPath())+'/wrist_rgbd',name='xarm7_wrist_rgbd',resolution=tuple(c['resolution_wh']),frequency=c['frequency_hz'])
    camera.set_local_pose(translation=local[:3,3],orientation=np.roll(Rotation.from_matrix(local[:3,:3]).as_quat(),1),camera_axes='ros')
    camera.initialize();w,h=c['resolution_wh'];K=np.asarray(c['K'])
    camera.set_horizontal_aperture(w*.01);camera.set_vertical_aperture(h*.01*K[0,0]/K[1,1]);camera.set_focal_length(K[0,0]*.01)
    camera.set_clipping_range(*c['clipping_range_m']);camera.add_distance_to_image_plane_to_frame()
    g.update(wrist_sensor=camera,wrist_config=c,wrist_X=X,wrist_routes=[])
    (g['a'].output/'WRIST_CALIBRATION.json').write_text(json.dumps(dict(configuration=c,prim_path=str(camera.prim_path),parent_body_path=str(parent.GetPath()),T_parent_camera_optical=local.tolist(),T_tcp_camera_optical=X.tolist(),set_local_pose_calls=1,independent_world_pose_commands=0,rigid_USD_child=True),indent=2))


def self_mask(depth,K,C,model,poses):
    """Measured-q hull silhouettes, including triangles crossing near plane.

    The frozen external-camera helper skips near-crossing links; wrist sensing
    instead clips each hull triangle before projecting. No instance masks.
    """
    import cv2
    excluded=np.zeros(depth.shape,bool);V=np.linalg.inv(C)
    for name,mesh in model.robot_hulls.items():
        if name not in poses:continue
        T=V@poses[name];vertices=np.asarray(mesh.vertices)@T[:3,:3].T+T[:3,3]
        for face in mesh.faces:
            poly=list(vertices[face]);clipped=[]
            for a,b in zip(poly,poly[1:]+poly[:1]):
                ina,inb=a[2]>=.025,b[2]>=.025
                if ina:clipped.append(a)
                if ina!=inb:clipped.append(a+(b-a)*((.025-a[2])/(b[2]-a[2])))
            if len(clipped)<3:continue
            Q=np.asarray(clipped);uv=Q@K.T;uv=uv[:,:2]/uv[:,2,None]
            uv=np.clip(np.rint(uv),-1e6,1e6).astype(np.int32)
            mask=np.zeros(depth.shape,np.uint8);cv2.fillConvexPoly(mask,cv2.convexHull(uv),1)
            excluded|=mask.astype(bool)&np.isfinite(depth)&(depth>=Q[:,2].min())
    return excluded


def save_rgbd(g,label):
    import cv2
    from interaction_runtime.checkpoints import allocate
    out=allocate(g['a'].output/'rgbd',label)
    camera=g['wrist_sensor']
    # Render at the physically reached pose. No timeline pause or pose setter.
    for _ in range(8):g['world'].render()
    rgb=np.asarray(camera.get_rgba())[...,:3].copy()
    depth=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data()).copy()
    C=g['matrix'](*camera.get_world_pose(camera_axes='ros'));K=np.asarray(camera.get_intrinsics_matrix())
    q=np.asarray(g['robot'].get_joint_positions());P=g['model'].poses(q[g['arm']],g['base'],finger_q=q[g['fingers']]);fk=P['tcp_link']@g['wrist_X'];measured=g['tcp']()@g['wrist_X']
    cv2.imwrite(str(out/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.save(out/'depth_m.npy',depth)
    mask=self_mask(depth,K,C,g['model'],P);np.save(out/'robot_self_exclusion.npy',mask)
    meta=dict(K=K.tolist(),T_world_camera_optical=C.tolist(),robot_q=q.tolist(),robot_joint_names=g['names'],base=list(g['base']),T_world_tcp_measured=g['tcp']().tolist(),t_sim=g['tick']*g['dt'],depth_units='m optical Z',camera_source='rigid link7 child; actuator-executed xArm7 motion only',T_world_camera_FK=fk.tolist(),T_world_camera_rigid_measured_tcp=measured.tolist(),FK_camera_translation_error_m=float(np.linalg.norm(fk[:3,3]-C[:3,3])),FK_camera_rotation_error_deg=float(Rotation.from_matrix(fk[:3,:3]@C[:3,:3].T).magnitude()*180/np.pi),rigid_camera_translation_error_m=float(np.linalg.norm(measured[:3,3]-C[:3,3])),robot_self_mask_fraction=float(mask.mean()),diagnostics_do_not_gate_execution=True)
    (out/'camera.json').write_text(json.dumps(meta,indent=2));return out


def execute_route(g,route,label):
    g['phase']='WRIST_OBSERVATION_MOTION';g['profile_mark']('wrist_observation_motion')
    before=g['tick']*g['dt'];began=time.perf_counter()
    for q in route['q_path'][1:]:g['move'](np.asarray(q),.8)
    g['phase']='WRIST_OBSERVATION_SETTLE';g['hold'](g['wrist_config']['settle_s'])
    route=dict(route,label=label,t_start=before,t_end=g['tick']*g['dt'],motion_wall_s=time.perf_counter()-began,measured_q=np.asarray(g['robot'].get_joint_positions())[g['arm']].tolist(),base=list(g['base']))
    g['wrist_routes'].append(route)
    (g['a'].output/'WRIST_ROBOT_TRAJECTORIES.json').write_text(json.dumps(g['wrist_routes'],indent=2))


def prepare(g):
    install_camera(g);c=g['wrist_config'];X=g['wrist_X'];m=g['model'];attempts=[]
    g['job']['_visual_proposal_used']=False;start=time.perf_counter();seed_folder=None
    # Finite survey sweep. No recorded handle/grasp pose enters these targets.
    for ix,p in enumerate(survey_candidates(m,c)):
        q=np.asarray(g['robot'].get_joint_positions())[g['arm']]
        route,audit=select_route(m,g['solver'],g['collision'],g['base'],q,[p],c,X)
        row=dict(index=ix,planning=audit);attempts.append(row)
        if route is None:continue
        execute_route(g,route,'survey_%02d'%ix)
        folder=save_rgbd(g,'initial' if ix==0 else 'initial_retry_%d'%ix)
        try:
            visual.segment(folder,g['job'],g['ROOT']);first=metric.preserve(folder);seed_folder=folder;row['observed_points']=len(first['points']);break
        except Exception as error:row['perception_failure']=repr(error)
    (g['a'].output/'WRIST_SURVEY.json').write_text(json.dumps(attempts,indent=2))
    if seed_folder is None:raise RuntimeError('WRIST_OBSERVATION_UNAVAILABLE_NO_SCENE_CAMERA_FALLBACK')
    views=[];audit=[]
    for sign in [-1,1]:
        q=np.asarray(g['robot'].get_joint_positions())[g['arm']]
        proposals=[p for p in observed_candidates(first['points'],first['plane']['normal'],c) if np.sign(p['azimuth_deg'])==sign]
        route,log=select_route(m,g['solver'],g['collision'],g['base'],q,proposals,c,X,first['points']);audit.append(log)
        if route is None:continue
        execute_route(g,route,'observed_side_%d'%sign)
        views.append(save_rgbd(g,'wrist_side_%d'%sign))
    geometry=g['a'].output/'wrist_geometry';data=multi.build_geometry(seed_folder,views,geometry)
    (g['a'].output/'WRIST_OBSERVATION_RESULT.json').write_text(json.dumps(dict(views=[str(seed_folder)]+[str(v) for v in views],geometry=data,planning=audit,wall_s=time.perf_counter()-start,actual_robot_motion=True,camera_world_pose_setter_calls=0,scene_camera_RGBD_used=False,GT_handle_used=False,observation_only=g['job'].get('wrist_observation_only',False)),indent=2))
    g['profile_mark']('wrist_visual_grasp_planning')
    candidate,data=multi.plan_observed(g,geometry)
    # Reuse the frozen visual setup's state assignment, without recapture or
    # independent camera recovery. Grasp controller and ranking stay unchanged.
    ns=dict(visual.__dict__,save_rgbd=lambda state,label:seed_folder,segment=lambda *args:None,
            plan_observed=lambda state,folder:(candidate,data))
    exec(inspect.getsource(visual.prepare),ns);ns['prepare'](g)
    boundary=json.loads((g['a'].output/'visual_source_boundary.json').read_text())
    boundary.update(sensing='simulated rigid wrist RGB-D; actual actuator motion; not hardware-calibrated',scene_camera_depth_used=False,contact_scores='frozen open-patch multi-view rank; no GT handle surface',camera_housing_unvalidated=True,remaining_known_dependencies=boundary['remaining_known_dependencies']+['coarse cabinet collision envelope for survey only'])
    (g['a'].output/'visual_source_boundary.json').write_text(json.dumps(boundary,indent=2))
    if g['job'].get('wrist_observation_only'):raise RuntimeError('WRIST_OBSERVATION_AND_AUTOMATIC_PLAN_COMPLETE_NO_GRASP_ATTEMPT')
    # The frozen planner starts its pregrasp route at home. Return through the
    # already checked observation routes in reverse, using normal actuators.
    g['phase']='WRIST_RETURN_HOME'
    for route in reversed(g['wrist_routes']):
        for q in list(reversed(route['q_path']))[1:]:g['move'](np.asarray(q),.8)


def common_visible(before,after,anchor):
    """Two measured camera poses, optical-Z reprojection; no object GT warp."""
    from articulated_interaction_skill.capture import backproject
    A=np.load(before/'depth_m.npy');B=np.load(after/'depth_m.npy')
    ma=json.loads((before/'camera.json').read_text());mb=json.loads((after/'camera.json').read_text())
    K=np.asarray(mb['K']);Cb=np.asarray(mb['T_world_camera_optical']);Ca=np.asarray(ma['T_world_camera_optical'])
    maskA=np.load(before/'robot_self_exclusion.npy');maskB=np.load(after/'robot_self_exclusion.npy')
    valid=np.isfinite(A)&(A>.025)&(A<3)&~maskA
    P,_=backproject(A,np.asarray(ma['K']),Ca,valid,stride=2)
    P=P[np.linalg.norm(P-anchor,axis=1)<.10];Q=(P-Cb[:3,3])@Cb[:3,:3]
    uv=Q@K.T;uv=np.rint(uv[:,:2]/np.maximum(Q[:,2,None],1e-9)).astype(int)
    h,w=B.shape;ok=(Q[:,2]>.025)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    P=P[ok];Q=Q[ok];uv=uv[ok];x,y=uv[:,0],uv[:,1];z=B[y,x]
    ok=np.isfinite(z)&(z>.025)&(z<3)&~maskB[y,x]&(abs(z-Q[:,2])<.015)
    P=P[ok];uv=uv[ok];z=z[ok]
    Q=np.c_[(uv[:,0]-K[0,2])*z/K[0,0],(uv[:,1]-K[1,2])*z/K[1,1],z]@Cb[:3,:3].T+Cb[:3,3]
    return P,Q


def refine(g):
    # Frozen pregrasp ICP/IK correction with changed-camera common-visible
    # matching. The old helper explicitly requires a stationary scene camera.
    import open3d as o3d
    folder=save_rgbd(g,'pregrasp');A,B=common_visible(g['visual_initial_folder'],folder,g['anchor_world'])
    def cloud(P):
        c=o3d.geometry.PointCloud();c.points=o3d.utility.Vector3dVector(P);return c.voxel_down_sample(.003)
    fit=o3d.pipelines.registration.registration_icp(cloud(A),cloud(B),.015,np.eye(4),o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
    D=np.asarray(fit.transformation);old=g['T'].copy();T=D@old;start=g['tcp']();seed=np.asarray(g['robot'].get_joint_positions())[g['arm']];path=[seed.tolist()];rot=Rotation.from_matrix(T[:3,:3]@start[:3,:3].T).as_rotvec()
    for f in np.linspace(0,1,21)[1:]:
        target=start.copy();target[:3,3]=(1-f)*start[:3,3]+f*T[:3,3];target[:3,:3]=Rotation.from_rotvec(f*rot).as_matrix()@start[:3,:3]
        q=g['ik'](target,seed)
        if q is None:raise RuntimeError('VISUAL_PREGRASP_LOCAL_IK_UNAVAILABLE')
        ok,why=g['collision'].check(g['model'].poses(q,g['base'],width=.084),g['moving'](),False)
        if not ok:raise RuntimeError(why)
        path.append(q.tolist());seed=q
    g.update(T=T,approach=path)
    correction=json.dumps(dict(checkpoint=str(folder),source='common-visible calibrated wrist RGB-D reprojection at distinct measured camera poses',common_points=len(A),D_visual=D.tolist(),T_before=old.tolist(),T_after=T.tolist(),fitness=fit.fitness,rmse_m=fit.inlier_rmse,GT_handle_pose_correction=False,metrics_diagnostic_only=True),indent=2)
    (folder/'visual_correction.json').write_text(correction)
    (g['a'].output/'visual_correction.json').write_text(correction)
