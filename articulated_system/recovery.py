"""Fresh RGB-D registration and bounded existing SE(2) recovery, no object GT."""
import copy,json,time
from pathlib import Path
import numpy as np
from articulated_interaction_skill.capture import backproject,matrix

def normalize_plan(plan):
    """Legacy nominal files use the existing shared family, never a new grid."""
    from interactive_twin.manifest import DEFAULT_POLICY
    result=copy.deepcopy(plan)
    result.setdefault('candidate_grid',copy.deepcopy(DEFAULT_POLICY['candidate_grid']))
    for index,candidate in enumerate(result['rows']):
        candidate.setdefault('candidate_index',index)
        if np.asarray(candidate['T']).shape!=(4,4):raise RuntimeError('INVALID_LEGACY_GRASP_TRANSFORM')
    for key in ('finger_swap_degrees','depth_offsets_m','along_handle_offsets_m'):
        if not result['candidate_grid'].get(key):raise RuntimeError('INCOMPLETE_GRASP_FAMILY_GRID')
    return result

def moving_cloud(recorder):
    P=[];world=recorder.scene['world'];world.render()
    for camera in recorder.cameras[:2]:
        frame=camera.get_current_frame();seg=frame['instance_id_segmentation']
        keys=[int(k) for k,v in seg['info']['idToLabels'].items() if recorder.part_path in str(v)]
        pts,_=backproject(np.asarray(frame['distance_to_image_plane']),np.asarray(camera.get_intrinsics_matrix()),matrix(*camera.get_world_pose(camera_axes='ros')),np.isin(seg['data'],keys),stride=4)
        P.append(pts)
    result=np.concatenate(P)
    if len(result)<200:raise RuntimeError('CURRENT_MOVING_PART_RGBD_UNAVAILABLE')
    return result

def register(source,target,initial):
    import open3d as o3d
    def cloud(p):
        c=o3d.geometry.PointCloud();c.points=o3d.utility.Vector3dVector(p);return c.voxel_down_sample(.003)
    A,B=cloud(source),cloud(target)
    fit=o3d.pipelines.registration.registration_icp(A,B,.015,np.asarray(initial),o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
    if fit.fitness<.5 or fit.inlier_rmse>.01:raise RuntimeError('CURRENT_HANDLE_REGISTRATION_UNCERTAIN')
    return np.asarray(fit.transformation),{'fitness':float(fit.fitness),'rmse_m':float(fit.inlier_rmse),'source':'fresh calibrated moving-part RGB-D, GT pose not read'}

class Recovery:
    def __init__(self,runtime,root,job,export,model,allowed):
        self.r=runtime;self.root=Path(root);self.job=job;self.export=export;self.model=model;self.allowed=allowed
        self.initial_cloud=moving_cloud(runtime.capture);self.initial_ee=runtime.tcp().copy();self.initial_base=list(runtime.base);self.count=0;self.history=[]
        self.plan=normalize_plan(json.loads(Path(job['plan']).read_text()))
        visual_path=Path(job.get('initial_visual',Path(job['source']).parent/'initial_visual_handle_world.json'))
        self.visual=json.loads(visual_path.read_text()) if visual_path.exists() else copy.deepcopy(runtime.initial_visual)
        self.visual['T_world_handle']=np.asarray(self.visual['T_world_handle']).copy().tolist()
        self.visual['T_world_handle'][0][3]=self.visual['anchor_world_m'][0]
        self.visual['T_world_handle'][1][3]=self.visual['anchor_world_m'][1]
        self.visual['T_world_handle'][2][3]=self.visual['anchor_world_m'][2]

    def observe(self,initial):
        now=moving_cloud(self.r.capture);D,audit=register(self.initial_cloud,now,initial);return D,audit,now

    def run(self,value):
        r=self.r;limit=self.job['system_capture']['maximum_repositions']
        if self.count>=limit:raise RuntimeError('REPOSITION_BUDGET_EXHAUSTED')
        self.count+=1;row={'index':self.count,'start_state_estimated':value,'initial_base':list(r.base),'object_reset':False};self.history.append(row)
        def save(): (r.capture.output/'reposition_history.json').write_text(json.dumps(self.history,indent=2))
        save()
        guess=r.tcp()@np.linalg.inv(self.initial_ee)
        try:D,audit,cloud=self.observe(guess)
        except Exception as error:
            row.update(status='PRE_RELEASE_OBSERVATION_FAILED',error=str(error));save();raise
        row['pre_release_observation']=audit;save()
        # Plan before release. All current handle/base targets derive from the
        # newly observed rigid registration, not from a simulator body pose.
        visual=copy.deepcopy(self.visual);H=D@np.asarray(visual['T_world_handle']);visual.update(T_world_handle=H.tolist(),anchor_world_m=H[:3,3].tolist(),axis_world=(D[:3,:3]@visual['axis_world']).tolist(),outward_normal_world=(D[:3,:3]@visual['outward_normal_world']).tolist(),source='current RGB-D registration')
        export=copy.deepcopy(self.export)
        # Existing mobile search adds the frozen chassis primitive itself.
        # Remove the duplicated execution export, not the physical collider.
        export['shapes']=[e for e in export['shapes'] if '/World/mobile_chassis' not in e['path']]
        for e in export['shapes']:
            if e.get('rigid_body_path','')==r.capture.part_path:
                e['world_transform']=(D@np.asarray(e['world_transform'])).tolist();e['rigid_body_world_transform']=(D@np.asarray(e['rigid_body_world_transform'])).tolist()
        plan=copy.deepcopy(self.plan)
        for index,c in enumerate(plan['rows']):
            # Legacy successful nominal plans predate this bookkeeping field.
            # Assign stable indices without changing any candidate pose.
            c.setdefault('candidate_index',index);c['T']=(D@np.asarray(c['T'])).tolist()
        from interactive_twin_recovery.mobile import recover,at_base
        export=at_base(export,self.initial_base,r.base)
        policy=json.loads((self.root/'configs/interactive_twin_recovery.yaml').read_text())['mobile']['search']
        result=recover(self.root,export,plan,visual,list(r.base),policy,seed=732061660,deadline=min(time.time()+600,r.deadline))
        row['planning']=result;save()
        if not result['selected']:raise RuntimeError('NO_SAFE_CONTINUATION_BASE_ROUTE')
        choice=result['selected'];row['planned_new_base']=choice['base'];r.plan_retreat(D);row['retreat_preplanned']=True;save();r.phase('RELEASE_CHECK_HOLD');r.drive.active=False;r.hold(2.)
        D0,_,_=self.observe(D)
        # Release is an incremental physical action. Abort and reclose if fresh
        # observations show retreat/self-close rather than assume it is locked.
        r.release_begin(D)
        try:
            for _ in range(20):
                r.release_increment(.001);r.hold(1.)
                D1,check,_=self.observe(D0)
                delta=D1@np.linalg.inv(D0)
                displacement=float(np.max(np.linalg.norm(cloud@delta[:3,:3].T+delta[:3,3]-cloud,axis=1)))
                if displacement>.001:raise RuntimeError('UNSAFE_RELEASE_OBSERVED_OBJECT_MOTION')
                if r.released():break
            else:raise RuntimeError('RELEASE_CONTACT_NOT_CLEARED')
        except Exception as error:
            row.update(status='SAFE_RELEASE_REJECTED',first_release_failure=str(error));save()
            try:r.reclose_at_current_pose();row['safe_reclose_completed']=True;save()
            except Exception as reclose_error:
                row['safe_reclose_error']=str(reclose_error);save();raise
            raise
        row['released']=True;r.retreat_home(choice);row['arm_retreat_completed']=True;save()
        r.base_route(choice);row['base_reposition_completed']=True;save()
        Dnew,audit,_=self.observe(D0);row['post_move_observation']=audit
        delta=Dnew@np.linalg.inv(D0)
        row['post_move_observed_displacement_m']=float(np.max(np.linalg.norm(cloud@delta[:3,:3].T+delta[:3,3]-cloud,axis=1)));save()
        if row['post_move_observed_displacement_m']>.001:raise RuntimeError('OBJECT_MOVED_DURING_REPOSITION')
        r.regrasp(choice,Dnew@np.linalg.inv(D));row['regrasp_completed']=True;row['status']='REPOSITIONED_REGRASPED';save()
        r.state_offset=value;r.reset_grasp_memory();return True
