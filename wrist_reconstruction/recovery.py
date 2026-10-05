"""Release/reobserve/replan around current RGB-D; no stale-world-pose veto."""
import ast,copy,json,time
from pathlib import Path
import numpy as np
from articulated_interaction_skill.capture import backproject,matrix
from articulated_system.recovery import Recovery as LegacyRecovery,normalize_plan,register
from wrist_reconstruction.capture import snapshot


def local_cloud(recorder,anchor,camera=None):
    P=[]
    # Pre-release stability uses existing sensor cameras, not instance masks.
    # Post-route target localization explicitly passes the actual wrist camera.
    for camera in [camera] if camera is not None else recorder.legacy_cameras[:2]:
        rgb,depth,_=snapshot(camera);K=np.asarray(camera.get_intrinsics_matrix());T=matrix(*camera.get_world_pose(camera_axes='ros'))
        points,_=backproject(depth,K,T,np.isfinite(depth),stride=2)
        points=points[np.linalg.norm(points-np.asarray(anchor),axis=1)<.10];P.append(points)
    result=np.concatenate(P)
    if len(result)<80:raise RuntimeError('OBSERVED_HANDLE_REGION_UNAVAILABLE')
    return result


class Recovery(LegacyRecovery):
    def __init__(self,runtime,root,job,export,model,allowed):
        self.r=runtime;self.root=Path(root);self.job=job;self.export=export;self.model=model;self.allowed=allowed
        self.initial_ee=runtime.tcp().copy();self.initial_base=list(runtime.base);self.count=0;self.history=[];self.plan=normalize_plan(json.loads(Path(job['plan']).read_text()))
        self.visual=copy.deepcopy(runtime.initial_visual);H=np.asarray(self.visual['T_world_handle']);H[:3,3]=self.visual['anchor_world_m'];self.visual['T_world_handle']=H.tolist()
        self.initial_cloud=local_cloud(runtime.capture,self.visual['anchor_world_m']);self.current_D=np.eye(4);self.released=False;self.grasp_reference_ee=self.initial_ee.copy();self.grasp_reference_D=np.eye(4)
        from wrist_reconstruction.planner import MobileWristPlanner
        from wrist_reconstruction.retreat import RetreatPlanner
        self.mobile=MobileWristPlanner(self);self.retreat=RetreatPlanner(self)

    def observe(self,initial,wrist=False):
        anchor=(np.asarray(initial)@np.r_[self.visual['anchor_world_m'],1])[:3]
        if wrist:self.r.observe_handle(anchor)
        for _ in range(8):self.r.step()
        P=local_cloud(self.r.capture,anchor,self.r.capture.camera if wrist else None)
        D,audit=register(self.initial_cloud,P,initial);audit['source']='unmasked live RGB-D local handle-region ICP; no simulator segmentation or body pose'
        audit['wrist_camera']=wrist;return D,audit,P

    def export_current(self,D):
        from interactive_twin_recovery.mobile import at_base
        data=copy.deepcopy(self.export);data['shapes']=[e for e in data['shapes'] if '/World/mobile_chassis' not in e['path']]
        for e in data['shapes']:
            if e.get('rigid_body_path','')==self.r.capture.part_path:
                for key in ('world_transform','rigid_body_world_transform'):e[key]=(D@np.asarray(e[key])).tolist()
        return at_base(data,self.initial_base,self.r.base)

    def visual_current(self,D):
        visual=copy.deepcopy(self.visual);H=D@np.asarray(visual['T_world_handle']);visual.update(T_world_handle=H.tolist(),anchor_world_m=H[:3,3].tolist(),axis_world=(D[:3,:3]@visual['axis_world']).tolist(),outward_normal_world=(D[:3,:3]@visual['outward_normal_world']).tolist(),source='current observed RGB-D, GT articulation unavailable');return visual

    def plan_current(self,D):
        from interactive_twin.planning import plan_grasps
        from interactive_twin_recovery.mobile import scene_at
        r=self.r;data=self.export_current(D);scene=scene_at(self.root,data,self.model,r.base,r.base)
        plan=plan_grasps(self.model,scene,scene.moving_reference,self.visual_current(D),list(r.base),seed=61,budget=self.job['system_capture']['regrasp_budget'],wall_clock_s=180,candidate_grid=self.plan['candidate_grid'])
        if not plan['trial_candidates']:raise RuntimeError('CURRENT_OBSERVATION_NO_SAFE_REGRASP')
        return {'base':list(r.base),'plan':plan,'route':{'waypoints':[list(r.base)],'translation_m':0.}}

    def move_base(self,choice):
        if not self.released:raise RuntimeError('MOBILE_REQUIRES_RELEASED_OBJECT')
        if choice['route'].get('translation_m',0)<1e-6 and choice['route'].get('rotation_deg',0)<1e-6:return
        if self.count>=self.job['system_capture']['maximum_repositions']:raise RuntimeError('REPOSITION_BUDGET_EXHAUSTED')
        self.count+=1;self.r.base_route(choice)
        (self.r.capture.output/'mobile_progress.json').write_text(json.dumps({'repositions':self.count,'base':list(self.r.base),'physical_release_verified':True,'route':choice['route']},indent=2))

    def grasp_recovery(self,D):
        from interactive_twin_recovery.mobile import recover
        policy=json.loads((self.root/'configs/interactive_twin_recovery.yaml').read_text())['mobile']['search']
        result=recover(self.root,self.export_current(D),self.plan,self.visual_current(D),list(self.r.base),policy,seed=61,deadline=min(time.time()+600,self.r.deadline))
        return result

    def run(self,value,capture_label=None):
        from wrist_reconstruction.planner import PlanningExhausted
        r=self.r;row={'index':len(self.history),'start_state_estimated':value,'initial_base':list(r.base),'object_reset':False,'regrasp_attempts':[]};self.history.append(row)
        def save():(r.capture.output/'reposition_history.json').write_text(json.dumps(self.history,indent=2))
        try:
            guess=r.tcp()@np.linalg.inv(self.grasp_reference_ee)@self.grasp_reference_D
            D,audit,cloud=self.observe(guess);self.current_D=D;row['pre_release_observation']=audit;save()
            plans=self.retreat.plans(D);row['retreat_preflight_alternatives']=len(self.retreat.rows);save()
            if not plans:raise PlanningExhausted('RETREAT_ALTERNATIVES_EXHAUSTED')
            r.phase('RELEASE_CHECK_HOLD');r.drive.active=False;r.hold(2.);D0,_,_=self.observe(D)
            r.configure_release(plans[0]['opening'],D)
            try:
                for _ in range(self.job['wrist_experiment']['retreat']['maximum_release_increments']):
                    r.release_increment(.001);r.hold(1.);D1,_,_=self.observe(D0);delta=D1@np.linalg.inv(D0)
                    if np.max(np.linalg.norm(cloud@delta[:3,:3].T+delta[:3,3]-cloud,axis=1))>.001:raise RuntimeError('UNSAFE_RELEASE_OBSERVED_OBJECT_MOTION')
                    if r.released():break
                else:raise RuntimeError('RELEASE_CONTACT_NOT_CLEARED')
            except BaseException:
                r.reclose_at_current_pose();raise
            self.released=True;row['released']=True;save()
            escaped=False
            for candidate in plans:
                if self.retreat.execute(candidate):escaped=True;break
            if not escaped:raise PlanningExhausted('RETREAT_PHYSICAL_ALTERNATIVES_EXHAUSTED')
            row['arm_retreat_completed']=True;save()
            if capture_label is not None:
                captured=r.capture.scan(capture_label,value,getattr(r,'saved_estimate',None));row['wrist_scan_completed']=captured['clean_wrist_capture'];save()
            # Reobserve after scan/base changes. Planning uncertainty is recoverable;
            # physical contact/safety errors continue to propagate unchanged.
            for attempt in range(self.job['system_capture']['regrasp_budget']):
                item={'attempt':attempt,'initial_base':list(r.base)};row['regrasp_attempts'].append(item);save()
                try:
                    Dnew,audit,_=self.observe(D0,wrist=True);self.current_D=Dnew;r.set_observed_moving(Dnew)
                    item['post_move_observation']=audit
                    delta=Dnew@np.linalg.inv(D0);item['post_move_observed_displacement_m']=float(np.max(np.linalg.norm(cloud@delta[:3,:3].T+delta[:3,3]-cloud,axis=1)))
                    item['world_consistency_role']='diagnostic only; fresh observation always replanned';save()
                    self.mobile.home();fresh=self.plan_current(Dnew);item['fresh_regrasp_plan']=fresh;save()
                except (PlanningExhausted,RuntimeError) as error:
                    reason=str(error);item['planning_error']=reason;save()
                    if not isinstance(error,PlanningExhausted) and not reason.startswith(('CURRENT_OBSERVATION_','CURRENT_HANDLE_REGISTRATION_','OBSERVED_HANDLE_REGION_','NO_SAFE_','SCAN_')):raise
                    self.mobile.home();result=self.grasp_recovery(self.current_D);item['mobile_grasp_search']=result;save()
                    if result['selected'] is None:continue
                    self.move_base(result['selected']);D0=self.current_D;continue
                r.regrasp(fresh,np.eye(4));self.released=False;item['physical_regrasp_completed']=True;row['regrasp_completed']=True;save()
                r.state_offset=value;r.reset_grasp_memory();self.grasp_reference_ee=r.tcp().copy();self.grasp_reference_D=Dnew.copy();row['status']='REOBSERVED_REPLANNED_REGRASPED';save()
                checkpoint={'stage':'stable_regrasp','estimated_state':value,'base':list(r.base),'current_observed_D':Dnew.tolist(),'repositions':self.count,'physical_state_restore_requires_command_reexecution':True,'attachment':False,'object_state_replay':False}
                (r.capture.output/'capture_checkpoint.json').write_text(json.dumps(checkpoint,indent=2));return True
            raise PlanningExhausted('REGRASP_RECOVERY_BUDGET_EXHAUSTED')
        except BaseException as e:
            row.update(status='FAILED',error=str(e));save();raise
