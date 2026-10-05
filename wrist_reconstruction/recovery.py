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
        self.initial_cloud=local_cloud(runtime.capture,self.visual['anchor_world_m']);self.current_D=np.eye(4)

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

    def run(self,value,capture_label=None):
        r=self.r;row={'index':len(self.history),'start_state_estimated':value,'initial_base':list(r.base),'object_reset':False};self.history.append(row)
        def save():(r.capture.output/'reposition_history.json').write_text(json.dumps(self.history,indent=2))
        try:
            guess=r.tcp()@np.linalg.inv(self.initial_ee)
            D,audit,cloud=self.observe(guess);row['pre_release_observation']=audit;save()
            # Keep the original finite recovery algorithm. Capture-only release
            # first stays at the current base; subsequent reachability failures
            # are eligible for the same SE2 search, never contact/fitting failures.
            if capture_label is not None:choice={'base':list(r.base),'route':{'waypoints':[list(r.base)],'translation_m':0.}}
            else:
                if self.count>=self.job['system_capture']['maximum_repositions']:raise RuntimeError('REPOSITION_BUDGET_EXHAUSTED')
                from interactive_twin_recovery.mobile import recover
                policy=json.loads((self.root/'configs/interactive_twin_recovery.yaml').read_text())['mobile']['search']
                result=recover(self.root,self.export_current(D),self.plan,self.visual_current(D),list(r.base),policy,seed=61,deadline=min(time.time()+600,r.deadline));row['planning']=result
                if not result['selected']:raise RuntimeError('NO_SAFE_CONTINUATION_BASE_ROUTE')
                choice=result['selected'];self.count+=1
            r.plan_retreat(D);r.phase('RELEASE_CHECK_HOLD');r.drive.active=False;r.hold(2.);D0,_,_=self.observe(D)
            r.release_begin(D)
            try:
                for _ in range(20):
                    r.release_increment(.001);r.hold(1.);D1,_,_=self.observe(D0);delta=D1@np.linalg.inv(D0)
                    if np.max(np.linalg.norm(cloud@delta[:3,:3].T+delta[:3,3]-cloud,axis=1))>.001:raise RuntimeError('UNSAFE_RELEASE_OBSERVED_OBJECT_MOTION')
                    if r.released():break
                else:raise RuntimeError('RELEASE_CONTACT_NOT_CLEARED')
            except BaseException:
                r.reclose_at_current_pose();raise
            row['released']=True;r.retreat_home(choice);row['arm_retreat_completed']=True;save()
            if capture_label is not None:
                r.capture.scan(capture_label,value,getattr(r,'saved_estimate',None));row['wrist_scan_completed']=True;save()
            r.base_route(choice);row['base_reposition_completed']=len(choice['route']['waypoints'])>1;save()
            Dnew,audit,_=self.observe(D0,wrist=True);row['post_move_observation']=audit
            delta=Dnew@np.linalg.inv(D0);row['post_move_observed_displacement_m']=float(np.max(np.linalg.norm(cloud@delta[:3,:3].T+delta[:3,3]-cloud,axis=1)))
            row['world_consistency_role']='diagnostic only; fresh target is replanned regardless of 1mm change';save()
            r.scan_home();fresh=self.plan_current(Dnew);row['fresh_regrasp_plan']=fresh;save()
            r.regrasp(fresh,Dnew@np.linalg.inv(D));row['regrasp_completed']=True;row['status']='REOBSERVED_REPLANNED_REGRASPED';save()
            self.current_D=Dnew;r.state_offset=value;r.reset_grasp_memory();return True
        except BaseException as e:
            row.update(status='FAILED',error=str(e));save();raise
