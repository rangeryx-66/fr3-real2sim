"""Finite live recovery for maximum-range mode; default baseline stays intact."""
import copy,json,time
import numpy as np
from scipy.spatial.transform import Rotation
from piper_mobile_demo.model import Model
from wrist_reconstruction.recovery import Recovery,release_motion
from wrist_reconstruction.max_range import ReleaseConsensus,template_variants
from wrist_reconstruction.planner import PlanningExhausted
from interactive_twin_recovery.mobile import candidate_bases,workspace_check


class MaximumRecovery(Recovery):
    def __init__(self,*args):
        super().__init__(*args);self.template=None;self.cycles=0;self.progress_last_wall=0.;self.last_observed_state=0.;self.progress_valid=False
        self.policy=self.job['wrist_experiment']['maximum_range'];self.tried={};self.cursor=[]
    def remember_success(self,D):
        ready,detail=self.r.grip()
        if not ready:return False
        H=np.asarray(self.visual_current(D)['T_world_handle']);E=self.r.tcp()
        robot_poses=self.model.poses(self.r.arm_q(),self.r.base,finger_q=self.r.finger_q());G=E@np.linalg.inv(robot_poses['tcp_link'])@robot_poses['gripper_base']
        if self.template is None:
            self.template={'T_world_handle_at_success':H.tolist(),'T_handle_TCP':(np.linalg.inv(H)@E).tolist(),
                'T_handle_gripper_success':(np.linalg.inv(H)@G).tolist(),'approach_direction_handle':(H[:3,:3].T@E[:3,2]).tolist(),
                'wrist_orientation_handle':(H[:3,:3].T@E[:3,:3]).tolist(),'finger_joint_positions':self.r.finger_q().tolist(),
                'actual_aperture_m':float(self.r.finger_q()[0]-self.r.finger_q()[1]),'contact_load_pattern':detail,
                'opening_role':'reference only; actual contact-aware closure determines endpoint','t':self.r.time()}
            (self.r.capture.output/'successful_grasp_template.json').write_text(json.dumps(self.template,indent=2))
        return True
    def observed_progress(self,fallback):
        if time.time()-self.progress_last_wall<5:
            return (self.last_observed_state,'FIXED_RGBD_HANDLE_REGISTRATION') if self.progress_valid else (fallback,'EE_PROXY_NOT_OBJECT_MEASUREMENT')
        self.progress_last_wall=time.time()
        guess=self.r.tcp()@np.linalg.inv(self.grasp_reference_ee)@self.grasp_reference_D
        try:
            D,audit,_=self.observe(guess);self.current_D=D
            if self.r.policy['joint_type']=='revolute':
                axis=np.asarray(self.r.memory.estimate['revolute']['axis']) if self.r.memory.estimate else np.array([0.,0,1])
                # Rotation is directly measured by RGB-D registration, not EE.
                # Sign follows the accepted observed-motion model, never GT.
                value=float(np.rad2deg(Rotation.from_matrix(D[:3,:3]).as_rotvec()@axis))*getattr(self.r.memory,'follow_sign',1.)
            else:
                axis=np.asarray(self.r.memory.estimate['prismatic']['axis']) if self.r.memory.estimate else -np.asarray(self.visual['T_world_handle'])[:3,2]
                anchor=np.r_[self.visual['anchor_world_m'],1];value=float(((D@anchor-anchor)[:3])@axis)*getattr(self.r.memory,'follow_sign',1.)
            self.last_observed_state=value;self.progress_valid=True
            with (self.r.capture.output/'observed_progress.jsonl').open('a') as f:f.write(json.dumps({'t':self.r.time(),'state':value,'source':'fixed RGB-D current handle-region registration','audit':audit,'D':D.tolist()})+'\n')
            return value,'FIXED_RGBD_HANDLE_REGISTRATION'
        except RuntimeError as e:
            self.progress_valid=False
            with (self.r.capture.output/'observed_progress.jsonl').open('a') as f:f.write(json.dumps({'t':self.r.time(),'status':'PERCEPTION_UNCERTAIN','error':str(e)})+'\n')
            return fallback,'EE_PROXY_NOT_OBJECT_MEASUREMENT'
    def confirm_release(self,baseline,D,row):
        monitor=ReleaseConsensus();attempts=0
        while attempts<self.policy['release_observation_attempts']:
            attempts+=1
            try:
                current=self.release_observation(D);amount,audit=release_motion(baseline,current)
                delta_center=np.asarray(current).mean(0)-np.asarray(baseline).mean(0)
                decision=monitor.observe(amount,delta_center)
                row.setdefault('release_observations',[]).append(dict(audit,displacement_m=amount,decision=decision))
                self.save()
                if decision=='STABLE':return
                if decision=='OBJECT_MOVED_DURING_RELEASE':raise RuntimeError(decision)
                # Pause further aperture commands while confirming a spike.
                self.r.hold(.3)
            except RuntimeError as e:
                if str(e)=='OBJECT_MOVED_DURING_RELEASE':raise
                row.setdefault('release_observations',[]).append({'decision':'PERCEPTION_UNCERTAIN','reason':str(e)});self.save();self.r.hold(.3)
        raise PlanningExhausted('RELEASE_PERCEPTION_RECOVERY_BUDGET_EXHAUSTED')
    def save(self):
        (self.r.capture.output/'reposition_history.json').write_text(json.dumps(self.history,indent=2))
    def escape(self,D,row):
        r=self.r;self.current_D=D;plans=self.retreat.plans(D);row['retreat_preflight_alternatives']=len(plans);self.save()
        if not plans:raise PlanningExhausted('RETREAT_ALTERNATIVES_EXHAUSTED')
        if not self.released:
            r.drive.active=False;r.hold(2.);baseline=self.release_observation(D);r.configure_release(plans[0]['opening'],D)
            try:
                for _ in range(self.job['wrist_experiment']['retreat']['maximum_release_increments']):
                    r.release_increment(.001);r.hold(1.);self.confirm_release(baseline,D,row)
                    if r.released():break
                else:raise PlanningExhausted('RELEASE_CONTACT_NOT_CLEARED')
            except BaseException:
                r.reclose_at_current_pose();raise
            self.released=True;row['released']=True;self.save()
        for plan in plans:
            if self.retreat.execute(plan):row['clearance_retreat']=True;self.save();return
        raise PlanningExhausted('RETREAT_PHYSICAL_ALTERNATIVES_EXHAUSTED')
    def candidates(self,D,family):
        H=np.asarray(self.visual_current(D)['T_world_handle'])
        if family=='template':return list(template_variants(self.template,H)) if self.template else []
        # Same semantic bar family, twelve diverse geometry-independent poses.
        E=H@np.asarray(self.template['T_handle_TCP']) if self.template else np.asarray(self.plan['rows'][0]['T'])
        rows=[]
        for i,(slide,depth,roll,swap,stand) in enumerate([
            (0,0,0,0,.04),(-.008,0,0,0,.04),(.008,0,0,0,.04),(0,-.004,0,0,.04),
            (0,.004,0,0,.04),(0,0,-10,0,.04),(0,0,10,0,.04),(0,0,0,180,.04),
            (-.005,-.002,0,180,.03),(.005,-.002,0,180,.05),(0,0,0,0,.02),(0,0,0,0,.065)]):
            T=E.copy();T[:3,3]+=H[:3,:3]@np.array([slide,0,depth]);T[:3,:3]=E[:3,:3]@Rotation.from_euler('xz',[roll,swap],degrees=True).as_matrix()
            rows.append({'candidate_index':i,'family':'generic-bar-side-pinch','T':T.tolist(),'standoff_m':stand})
        return rows
    def plan_variant(self,variant,base,start):
        r=self.r;scene=self.mobile.scene(base);T=np.asarray(variant['T']);fingers=np.array([.05,-.05])
        q=self.model.ik(T,base,seed=start,starts=5)
        if q is None:raise PlanningExhausted('NO_GRASP_IK')
        def check(q,b):return self.mobile.check(scene,q,b,fingers)
        ok,why,_=check(q,base)
        if not ok:raise PlanningExhausted('GRASP_'+why)
        pre=T.copy();pre[:3,3]-=variant['standoff_m']*T[:3,2];seed=self.model.ik(pre,base,seed=q,starts=3)
        if seed is None:raise PlanningExhausted('NO_PREGRASP_IK')
        edge=[]
        for fraction in np.linspace(0,1,21):
            A=T.copy();A[:3,3]=(1-fraction)*pre[:3,3]+fraction*T[:3,3];q=self.model.ik(A,base,seed=seed,starts=1)
            if q is None:raise PlanningExhausted('NO_APPROACH_IK')
            for middle in np.linspace(seed,q,max(2,int(np.max(abs(q-seed))/.025)+2)):
                ok,why,_=check(middle,base)
                if not ok:raise PlanningExhausted('APPROACH_'+why)
            edge.append(q.tolist());seed=q
        path=self.mobile.arm_path(scene,start,edge[0],base,fingers)
        if path is None:raise PlanningExhausted('NO_COLLISION_FREE_ARM_PATH')
        trial=dict(variant,q_pre=edge[0],q_grasp=edge[-1],approach=edge,preplan=path,minimum_joint_margin_rad=min(self.model.margin(np.asarray(x)) for x in edge+path))
        trial['unknown_probe_workspace']=workspace_check(self.model,scene,trial,base,self.visual_current(self.current_D),.002)
        if not trial['unknown_probe_workspace']['passed']:raise PlanningExhausted('NO_LOCAL_CONTINUATION_WORKSPACE')
        return trial
    def choose_base(self,D,row,excluded):
        r=self.r;visual=self.visual_current(D);locked=r.arm_q();coarse=[];start=time.time()
        for b in candidate_bases(r.base,visual,self.mobile.policy)[:self.policy['base_candidates']]:
            if time.time()>r.deadline or time.time()-start>self.mobile.policy['search_wall_s']:break
            if tuple(np.round(b,4)) in excluded:continue
            scene=self.mobile.scene(b);ok,why,gap=self.mobile.locked_valid(scene,locked,b)
            audit={'base':b,'status':why};row.setdefault('base_search',[]).append(audit)
            if not ok:continue
            for v in self.candidates(D,'template')[:1]+self.candidates(D,'generic')[:2]:
                q=self.model.ik(np.asarray(v['T']),b,seed=locked,starts=3)
                if q is None:continue
                ok,why,_=self.mobile.check(scene,q,b,[.05,-.05])
                if not ok:continue
                score=[self.model.margin(q),gap,-float(np.linalg.norm(np.subtract(b[:2],r.base[:2])))];coarse.append((score,b));audit['status']='COARSE_GRASP_IK';break
        coarse.sort(reverse=True,key=lambda x:x[0]);self.save()
        for _,b in coarse[:self.policy['full_base_paths']]:
            route=self.mobile.locked_route(list(r.base),b,locked)
            if not route['valid']:continue
            for v in self.candidates(D,'template')[:2]+self.candidates(D,'generic')[:1]:
                try:self.plan_variant(v,b,locked)
                except RuntimeError:continue
                return {'base':b,'route':route}
        return None
    def run(self,value,capture_label=None,reason='WORKSPACE_RECOVERY'):
        if self.cycles>=self.policy['operation_cycles']:raise PlanningExhausted('NO_RECOVERY_OPERATION_BUDGET_EXHAUSTED')
        self.cycles+=1;r=self.r;row={'index':len(self.history),'cycle':self.cycles,'reason':reason,'start_state_estimated':value,'initial_base':list(r.base),'regrasp_attempts':[]};self.history.append(row);self.save()
        # Observation uncertainty is retried without reopening or world reset.
        D=self.current_D;observed=False
        for retry in range(3):
            try:
                guess=self.current_D if self.released else r.tcp()@np.linalg.inv(self.grasp_reference_ee)@self.grasp_reference_D
                D,audit,_=self.observe(guess);row['pre_release_observation']=audit;observed=True;break
            except RuntimeError as e:row.setdefault('observation_retries',[]).append(str(e));self.save();r.hold(.3)
        if not observed:raise PlanningExhausted('HANDLE_REOBSERVE_BUDGET_EXHAUSTED')
        self.escape(D,row)
        if capture_label:
            try:
                result=r.capture.scan(capture_label,value,getattr(r,'saved_estimate',None));row['wrist_scan_completed']=result['clean_wrist_capture']
            except RuntimeError as e:row['scan_error']=str(e)
            self.save()
        excluded=set();physical={f:0 for f in ('template','generic')};consumed=set()
        for base_attempt in range(self.policy['actual_base_attempts']):
            for family in ('template','generic'):
                if physical[family]>=self.policy[family+'_closures']:continue
                try:Dnew,audit,_=self.observe(D,wrist=True);self.current_D=Dnew;r.set_observed_moving(Dnew)
                except RuntimeError as e:row.setdefault('wrist_reobserve_failures',[]).append(str(e));self.save();break
                for v in self.candidates(Dnew,family):
                    key=(tuple(np.round(r.base,4)),family,v['candidate_index'])
                    if key in consumed:continue
                    consumed.add(key);item={'family':family,'candidate_index':v['candidate_index'],'base':list(r.base),'current_wrist_observation':audit};row['regrasp_attempts'].append(item);self.save()
                    if physical[family]>=self.policy[family+'_closures']:break
                    try:trial=self.plan_variant(v,r.base,r.arm_q())
                    except RuntimeError as e:item['planning_error']=str(e);self.save();continue
                    physical[family]+=1;item['physical_closure_number']=physical[family];self.save()
                    choice={'base':list(r.base),'plan':{'trial_candidates':[trial]},'route':None}
                    try:r.regrasp(choice,np.eye(4))
                    except RuntimeError as e:
                        item['physical_regrasp_error']=str(e);self.save()
                        if str(e)=='SUSTAINED_CONTACT_LOSS':r.stop_failed_grasp_monitor()
                        r.halt_at_measured_state()
                        # Force/speed and dangerous native contacts are never
                        # ignored; only a checked safe escape can resume.
                        self.escape(Dnew,row);continue
                    self.released=False;item['physical_regrasp_completed']=True;row['regrasp_completed']=True;row['status']='REGRASPED_FROM_CURRENT_WRIST_OBSERVATION';self.save()
                    r.state_offset=value;r.reset_grasp_memory();self.grasp_reference_ee=r.tcp().copy();self.grasp_reference_D=Dnew.copy();self.remember_success(Dnew)
                    return True
            excluded.add(tuple(np.round(r.base,4)))
            choice=self.choose_base(self.current_D,row,excluded)
            if choice is None:row.setdefault('base_failures',[]).append('NO_ROUTE_OR_APPROACH');self.save();continue
            try:self.move_base(choice)
            except RuntimeError as e:row.setdefault('base_failures',[]).append(str(e));self.save();r.halt_at_measured_state();continue
            D=self.current_D
        row['status']='RECOVERY_BUDGET_EXHAUSTED';self.save();raise PlanningExhausted('NO_RECOVERY_BASE_ARM_REGRASP_BUDGET_EXHAUSTED')
