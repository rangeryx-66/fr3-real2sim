"""Observation-only multi-view input adapter. Frozen execution is untouched.

Measured triangle patches are retained as open surfaces. Convex rear closures
are three explicit hypotheses, not measurements or confirmed free/solid space.
No asset mesh or handle pose is accepted by geometry construction.
"""
import copy, inspect, json, time, types
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
import native_g1_visual_input as visual
import native_g1_metric_rgbd_geometry as metric

SPEC = dict(reference='3103651', azimuths_deg=[-55, 0, 55], standoff_m=.7,
            height_m=.1, voxel_m=.0005, rear_scales=[.75, 1., 1.25],
            sensing='SIMULATION_SCENE_CAMERA_REPOSITION_NOT_ACTIVE_WRIST',
            observed_surfaces='open per-view measured-neighbor triangles',
            unknown_space='neither confirmed empty nor confirmed solid',
            controller_changes=False, scoring_weights_changed=False)


def fast_ray(mesh, origins, direction):
    """Exact nearest positive triangle intersections, cached R-tree backend.

    Matches frozen ray_surface's double-sided, pad-facing normal convention;
    acceleration changes no mesh, sampling grid or contact-score coefficient.
    """
    dirs=np.broadcast_to(direction, origins.shape).copy()
    face, ray, point=mesh.ray.intersects_id(origins, dirs, return_locations=True,
                                          multiple_hits=False)
    dist=np.full(len(origins), np.inf);normals=np.zeros_like(origins)
    if len(ray):
        dist[ray]=np.einsum('ij,j->i',point-origins[ray],direction)
        normals[ray]=mesh.face_normals[face]
        normals*=np.where(normals@direction>0,-1.,1.)[:,None]
    return dist,normals


def rescore(*args, **kwargs):
    import native_g1_robust_contact_ranking as frozen
    from native_g1_visual_hypothesis_rank import HypothesisContactRank
    one=types.FunctionType(frozen.RobustContactRank.sample.__code__,
                          dict(frozen.__dict__,ray_surface=fast_ray))
    class MultiViewRank(HypothesisContactRank):
        def sample(self,T,width,mu):
            original=self.surfaces;results=[]
            for hypothesis in original[2]:
                self.surfaces=hypothesis;results.append(one(self,T,width,mu))
            self.surfaces=original
            nominal=dict(results[1])
            for key in ['combined','contact_score','retention_score','task_score']:
                values=[s[key] for s in results]
                nominal[key]=float(self.cfg['mean_weight']*np.mean(values)+self.cfg['lower_quartile_weight']*np.quantile(values,.25))
            nominal['hidden_geometry_samples']=[dict(scale=s,combined=r['combined'],contact=r['contact_score'],retention=r['retention_score'],task=r['task_score']) for s,r in zip(SPEC['rear_scales'],results)]
            nominal['hypotheses_are_soft_ranking_not_vetoes']=True
            return nominal
    ns=dict(frozen.__dict__,RobustContactRank=MultiViewRank)
    exec(inspect.getsource(frozen.rescore_geometries),ns)
    return ns['rescore_geometries'](*args,**kwargs)


def build_geometry(seed_folder, view_folders, output):
    """All arguments are observed RGB-D folders, never an object asset."""
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    _surface_cache.pop(str(output.resolve()),None)
    for key in list(_closing_cache):
        if key[1]==str(output.resolve()):_closing_cache.pop(key)
    first=metric.preserve(seed_folder);views=[first];audit=[]
    metric.save_view(first,output/'single')
    for i,folder in enumerate(view_folders):
        try:
            seed=metric.projected_seed(folder,first['points'])
            view=metric.preserve(folder,seed,first['plane'])
            metric.save_view(view,output/('accepted_%02d'%i));views.append(view)
            audit.append(dict(folder=str(folder),accepted=True,points=len(view['points'])))
        except Exception as error:
            # A view without common visible depth is missing information, not
            # evidence that space is empty; other measured patches still count.
            audit.append(dict(folder=str(folder),accepted=False,reason=repr(error)))
    points,patch=metric.fuse(views,SPEC['voxel_m'])
    F=np.asarray(first['audit']['frame_world']);origin=np.median(points,axis=0)
    hypotheses=metric.uncertain_profiles(points,F,origin)
    np.save(output/'points_world.npy',points);patch.export(output/'observed_patches.ply')
    # Preserve the measured front. Only hull rear/lateral faces are uncertain
    # completion; discard front-facing hull caps, which would fill observed gaps.
    for i,h in enumerate(hypotheses):
        mesh=trimesh.util.concatenate(h['parts'])
        mesh.update_faces(mesh.face_normals@F[:,2]<.1)
        mesh.remove_unreferenced_vertices();mesh.export(output/('uncertain_rear_%d.ply'%i))
    Q=(points-origin)@F;lo,hi=Q[:,0].min(),Q[:,0].max();length=hi-lo
    sections=[]
    for frac in [-.3,0.,.3]:
        s=(lo+hi)/2+frac*length;q=Q[abs(Q[:,0]-s)<length/8]
        if len(q)<8:q=Q[np.argsort(abs(Q[:,0]-s))[:max(8,len(Q)//12)]]
        left,right=q[:,1].min(),q[:,1].max();front=q[:,2].max();width=right-left
        anchor=np.array([s,(left+right)/2,front-width/2])@F.T+origin
        sections.append(dict(fraction=frac,anchor_world_m=anchor.tolist(),bar_depth_m=float(width)))
    # Observed panel scoring slab, same declared 10mm inward prior as frozen
    # visual input; no native collider changes. Its extent comes from RGB-D.
    r=metric.panel(seed_folder,metric.load_seed(seed_folder));flat=r['P'][r['inliers']]
    uv=(flat-origin)@F[:,:2];bounds=np.quantile(uv,[.01,.99],axis=0)
    plane=float(np.asarray(first['plane']['center'])@F[:,2]-origin@F[:,2])
    corners=np.array([[s,u,z] for s in bounds[:,0] for u in bounds[:,1] for z in [plane-.01,plane]])
    panel=trimesh.convex.convex_hull(corners@F.T+origin)
    data=dict(source='calibrated RGB-D union; SAM3 seed and depth-consistent projected seeds',
              GT_handle_surface_used=False,simulator_mask_used=False,geometry_folder=str(output),
              axis_world=F[:,0].tolist(),normal_world=F[:,2].tolist(),
              dimensions_m=[float(length),float(np.median([s['bar_depth_m'] for s in sections])),float(np.ptp(Q[:,2]))],
              sections=sections,legacy_estimate=dict(anchor_world_m=origin.tolist()),
              panel=dict(vertices=panel.vertices.tolist(),faces=panel.faces.tolist()),
              views=audit,accepted_views=len(views),observed_points=len(points),
              observed_triangles=len(patch.faces),spec=SPEC,
              prior='unobserved rear 0.75/1/1.25 observed transverse width; mean/Q25 aggregation, no best-backside selection; not calibrated confidence',
              unknown_space_is_not_free=True)
    (output/'observed_geometry.json').write_text(json.dumps(data,indent=2))
    return data


_surface_cache={}
def rank_surfaces(folder):
    key=str(Path(folder).resolve())
    if key not in _surface_cache:
        measured=trimesh.load(Path(folder)/'observed_patches.ply',process=False)
        hypotheses=[]
        for i in range(3):
            assumed=trimesh.load(Path(folder)/('uncertain_rear_%d.ply'%i),process=False)
            combined=trimesh.util.concatenate([measured,assumed])
            hypotheses.append((combined,[combined]))
        nominal=hypotheses[1]
        _surface_cache[key]=(nominal[0],nominal[1],hypotheses)
    return _surface_cache[key]


_closing_cache={}
def closing_scene(model,folder):
    from piper_mobile_demo.owned_scene import Shape
    key=(id(model),str(Path(folder).resolve()))
    if key in _closing_cache:return _closing_cache[key]
    data=json.loads((Path(folder)/'observed_geometry.json').read_text())
    robot=[Shape(m,np.eye(4),True,'robot/'+n,n) for n,m in model.robot_hulls.items()]
    # Contact acquisition scoring uses measured open patches, not a convex
    # hull or a declaration that unknown rear geometry is solid.
    measured=trimesh.load(Path(folder)/'observed_patches.ply',process=False)
    name='observed/measured_handle'
    scene=[Shape(measured,np.eye(4),False,name,'observed')]
    p=data['panel'];scene.append(Shape(trimesh.Trimesh(p['vertices'],p['faces'],process=False),np.eye(4),True,'observed/panel','observed'))
    result=SimpleNamespace(robot=robot,scene=scene,allowed={name},moving_link='observed',moving_reference=np.eye(4),jaw_bounds={n:np.array([model.robot_hulls[n].vertices.min(0),model.robot_hulls[n].vertices.max(0)]) for n in ('gripper_link1','gripper_link2')})
    _closing_cache[key]=result
    return result


def plan_observed(g,folder):
    import xarm7_native_planning as native
    common_ns=dict(native.__dict__,ray_surface=fast_ray)
    source=inspect.getsource(native.common_quality).replace(' from handle_contact_quality import ray_surface\n','')
    exec(source,common_ns)
    proxy=SimpleNamespace(**native.__dict__)
    proxy.common_quality=common_ns['common_quality']
    proxy.plan=types.FunctionType(native.plan.__code__,dict(native.__dict__,common_quality=common_ns['common_quality']))
    ns=dict(visual.__dict__,native=proxy,observed_geometry=lambda f:json.loads((Path(f)/'observed_geometry.json').read_text()),rank_surfaces=rank_surfaces,closing_scene=closing_scene,rescore_geometries=rescore)
    source=inspect.getsource(visual.plan_observed)
    source=source.replace('    import xarm7_native_planning as native\n','').replace('    from native_g1_visual_hypothesis_rank import rescore_visual_geometries as rescore_geometries\n','')
    exec(compile(source,__file__+'::frozen_planner_observed_patches','exec'),ns)
    candidate,data=ns['plan_observed'](g,folder)
    candidate['contact_score_surface_source']='open measured multi-view patches plus separate uncertain rear faces; no GT handle surface'
    (g['a'].output/'visual_plan.json').write_text(json.dumps(candidate,indent=2))
    return candidate,data


def capture_views(g,folder):
    import cv2
    import omni.replicator.core as rep
    from wrist_reconstruction.self_observation import robot_projection_mask
    first=metric.preserve(folder);anchor=np.median(first['points'],axis=0)
    normal=np.asarray(first['plane']['normal']);side=np.cross([0,0,1.],normal);side/=np.linalg.norm(side)
    camera=g['scene']['overview'][0];original=camera.get_world_pose(camera_axes='ros');clip=camera.get_clipping_range()
    q=np.asarray(g['robot'].get_joint_positions());poses=g['model'].poses(q[g['arm']],g['base'],finger_q=q[g['fingers']])
    world=g['world'];world.pause();before=float(world.current_time);folders=[]
    try:
        camera.set_clipping_range(.05,3.)
        for i,angle in enumerate(SPEC['azimuths_deg']):
            out=g['a'].output/'rgbd'/('multi_%02d'%i);out.mkdir(parents=True,exist_ok=True)
            theta=np.deg2rad(angle);eye=anchor+SPEC['standoff_m']*(np.cos(theta)*normal+np.sin(theta)*side)+[0,0,SPEC['height_m']]
            forward=(anchor-eye)/np.linalg.norm(anchor-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward))
            camera.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros')
            rep.orchestrator.step(delta_time=0.,pause_timeline=True,rt_subframes=8)
            rgb=np.asarray(camera.get_rgba())[...,:3];depth=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data())
            if depth.ndim!=2 or depth.shape!=rgb.shape[:2]:raise ValueError('RGBD_EXPORT_SHAPE_MISMATCH')
            C=g['matrix'](*camera.get_world_pose(camera_axes='ros'));K=np.asarray(camera.get_intrinsics_matrix())
            cv2.imwrite(str(out/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.save(out/'depth_m.npy',depth)
            meta=dict(K=K.tolist(),T_world_camera_optical=C.tolist(),depth_units='m optical Z',camera_source=SPEC['sensing'],robot_q=q.tolist(),robot_joint_names=g['names'],base=list(g['base']),t_sim=float(world.current_time),azimuth_deg=angle)
            (out/'camera.json').write_text(json.dumps(meta,indent=2));np.save(out/'robot_self_exclusion.npy',robot_projection_mask(depth,K,C,g['model'].robot_hulls,poses));folders.append(out)
    finally:
        camera.set_world_pose(*original,camera_axes='ros');camera.set_clipping_range(*clip)
        rep.orchestrator.step(delta_time=0.,pause_timeline=True,rt_subframes=8)
        (g['a'].output/'MULTIVIEW_CAPTURE.json').write_text(json.dumps(dict(spec=SPEC,physics_time_before=before,physics_time_after=float(world.current_time),views=[str(p) for p in folders],camera_restored=True),indent=2))
        world.play()
    return folders


def prepare(g):
    # Reuse bounded SAM recovery and all downstream setup verbatim. Only the
    # geometric/planning function is replaced, in a private namespace.
    def capture_and_plan(state,folder):
        views=capture_views(state,folder)
        data=build_geometry(folder,views,state['a'].output/'multiview_geometry')
        candidate,_=plan_observed(state,Path(data['geometry_folder']))
        return candidate,data
    ns=dict(visual.__dict__,plan_observed=capture_and_plan)
    exec(inspect.getsource(visual.prepare),ns)
    ns['prepare'](g)
    boundary=json.loads((g['a'].output/'visual_source_boundary.json').read_text())
    boundary.update(contact_scores='measured open RGB-D patches plus explicitly separate rear hypotheses; frozen robust weights',multiview_sensing=SPEC['sensing'],GT_contact_score_surfaces=False)
    (g['a'].output/'visual_source_boundary.json').write_text(json.dumps(boundary,indent=2))


refine=visual.refine
