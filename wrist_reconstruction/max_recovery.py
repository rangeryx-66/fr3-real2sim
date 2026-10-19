"""Finite live recovery for maximum-range mode; default baseline stays intact."""
import copy,json,time
import numpy as np
from scipy.spatial.transform import Rotation
from piper_mobile_demo.model import Model
from wrist_reconstruction.recovery import Recovery,release_motion
from wrist_reconstruction.max_range import ReleaseConsensus,template_variants
from wrist_reconstruction.planner import PlanningExhausted
from wrist_reconstruction.constraint_policy import Category,State,ConstraintPolicy,TaskTermination
from interactive_twin_recovery.mobile import candidate_bases,workspace_check


class MaximumRecovery(Recovery):
    def __init__(self,*args):
        super().__init__(*args);self.template=None;self.cycles=0;self.progress_last_wall=0.;self.last_observed_state=0.;self.progress_valid=False
        self.policy=self.job['wrist_experiment']['maximum_range'];self.tried={};self.cursor=[]
        self.events=getattr(self.r,'constraints',None) or ConstraintPolicy(self.r.capture.output,self.r);self.r.constraints=self.events
        self.prior_failures=self.job.get('prior_failed_regrasp_candidates',[])
        if self.job.get('continuation_replay'):
            from pathlib import Path
            checkpoint=json.loads(Path(self.job['continuation_checkpoint']).read_text())
            # A cold restoration does not replenish the finite operation or
            # mobile budget. Conservatively debit carried physical failures.
            self.cycles=int(checkpoint.get('recovery_cycle',0))+len(self.prior_failures)
            self.count=int(checkpoint.get('base_moves',0))
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
    def motion_guess(self):
        guess=self.current_D.copy()
        if not self.released:
            predicted=guess@np.linalg.inv(self.grasp_reference_D)@self.grasp_reference_ee
            # A retained grasp may rotate relative to the handle. Keep the
            # measured object rotation and propagate only measured TCP motion.
            guess[:3,3]+=self.r.tcp()[:3,3]-predicted[:3,3]
        return guess
    def observed_progress(self,fallback):
        if time.time()-self.progress_last_wall<5:
            return (self.last_observed_state,'FIXED_RGBD_HANDLE_REGISTRATION') if self.progress_valid else (fallback,'EE_PROXY_NOT_OBJECT_MEASUREMENT')
        self.progress_last_wall=time.time()
        guess=self.motion_guess()
        try:
            D,audit,_=self.observe(guess);self.current_D=D
            if self.r.policy['joint_type']=='revolute':
                # From a normal closed setup, rotation magnitude is opening
                # excursion. An arbitrary EE-fit axis sign is not object state.
                value=float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(D[:3,:3]).as_rotvec())))
            else:
                axis=np.asarray(self.r.memory.estimate['prismatic']['axis']) if self.r.memory.estimate else -np.asarray(self.visual['T_world_handle'])[:3,2]
                anchor=np.r_[self.visual['anchor_world_m'],1];value=float(((D@anchor-anchor)[:3])@axis)*getattr(self.r.memory,'follow_sign',1.)
            value+=getattr(self,'observation_origin_state',0.)
            self.last_observed_state=value;self.progress_valid=True
            with (self.r.capture.output/'observed_progress.jsonl').open('a') as f:f.write(json.dumps({'t':self.r.time(),'state':value,'source':'fixed RGB-D current handle-region registration','audit':audit,'D':D.tolist()})+'\n')
            return value,'FIXED_RGBD_HANDLE_REGISTRATION'
        except RuntimeError as e:
            self.progress_valid=False
            decision=self.events.dispatch(str(e),'max_recovery.observed_progress','operation')
            if decision.category==Category.HARD:raise
            with (self.r.capture.output/'observed_progress.jsonl').open('a') as f:f.write(json.dumps({'t':self.r.time(),'status':'PERCEPTION_UNCERTAIN','error':str(e)})+'\n')
            return fallback,'EE_PROXY_NOT_OBJECT_MEASUREMENT'
    def confirm_release(self,baseline,D,row,baseline_index):
        monitor=ReleaseConsensus();attempts=0;last_observation_index=None
        while attempts<self.policy['release_observation_attempts']:
            attempts+=1
            try:
                current=self.release_observation(D)
                from wrist_reconstruction.self_observation import common_release_points
                folder=self.r.capture.output/'release_observer';pairs=[];anchor=(np.asarray(D)@np.r_[self.visual['anchor_world_m'],1])[:3]
                for camera in range(2):
                    with np.load(folder/f'observation_{baseline_index:03d}_camera_{camera}.npz') as A,np.load(folder/f'observation_{self.release_observation_index-1:03d}_camera_{camera}.npz') as B:pairs.append(common_release_points(A,B,anchor))
                before=np.concatenate([x[0] for x in pairs]);current=np.concatenate([x[1] for x in pairs])
                if len(before)<80:raise RuntimeError('RELEASE_COMMON_SURFACE_POINTS_INSUFFICIENT')
                amount,audit=release_motion(before,current);audit.update(common_visibility=True,common_points=len(before),t=self.r.time())
                delta_center=current.mean(0)-before.mean(0)
                decision=monitor.observe(amount,delta_center)
                row.setdefault('release_observations',[]).append(dict(audit,displacement_m=amount,decision=decision))
                self.save()
                if decision=='STABLE':return
                if decision=='OBJECT_MOVED_DURING_RELEASE':
                    # A repeatable 1mm offset can be elastic settling or a
                    # registration bias, not an uncontrolled fall. Verify
                    # successive current observations before reopening/reclosing.
                    if last_observation_index is not None:
                        stable_pairs=[]
                        for camera in range(2):
                            with np.load(folder/f'observation_{last_observation_index:03d}_camera_{camera}.npz') as A,np.load(folder/f'observation_{self.release_observation_index-1:03d}_camera_{camera}.npz') as B:stable_pairs.append(common_release_points(A,B,anchor))
                        previous=np.concatenate([x[0] for x in stable_pairs]);latest=np.concatenate([x[1] for x in stable_pairs])
                        if len(previous)>=80:
                            window_motion,window_audit=release_motion(previous,latest)
                            row['release_observations'][-1].update(window_motion_m=window_motion,window_audit=window_audit)
                            self.save()
                            if window_motion<=monitor.threshold:
                                row['release_observations'][-1]['decision']='STABLE_AFTER_OBSERVED_RELEASE_SHIFT'
                                self.events.dispatch('OBJECT_MOVED_DURING_RELEASE','max_recovery.release_settle','observation',absolute_shift_m=amount,window_motion_m=window_motion,handling='current observations stable; no fall confirmation')
                                self.save();return
                    # Keep aperture fixed and reobserve within the SAME release
                    # budget. Only persistent motion exhausts this perception pool.
                last_observation_index=self.release_observation_index-1
                # Pause further aperture commands while confirming a spike.
                self.r.hold(.3)
            except RuntimeError as e:
                if str(e)=='OBJECT_MOVED_DURING_RELEASE':raise
                row.setdefault('release_observations',[]).append({'decision':'PERCEPTION_UNCERTAIN','reason':str(e)});self.save();self.r.hold(.3)
        raise PlanningExhausted('RELEASE_PERCEPTION_RECOVERY_BUDGET_EXHAUSTED')
    def save(self):
        (self.r.capture.output/'reposition_history.json').write_text(json.dumps(self.history,indent=2))
    def escape(self,D,row,safety_release=False,plans=None):
        r=self.r;self.current_D=D;plans=self.retreat.plans(D) if plans is None else plans;row['retreat_preflight_alternatives']=len(plans);self.save()
        if not plans:raise PlanningExhausted('RETREAT_ALTERNATIVES_EXHAUSTED')
        if not self.released:
            self.events.transition(State.RELEASE,'max_recovery.escape')
            r.drive.active=False
            if not safety_release:r.hold(2.)
            baseline=self.release_observation(D);baseline_index=self.release_observation_index-1;r.configure_release(plans[0]['opening'],D)
            try:
                for _ in range(self.job['wrist_experiment']['retreat']['maximum_release_increments']):
                    r.release_increment(.001);r.hold(1.);self.confirm_release(baseline,D,row,baseline_index)
                    if r.released():break
                else:raise PlanningExhausted('RELEASE_CONTACT_NOT_CLEARED')
            except RuntimeError as error:
                decision=self.events.dispatch(str(error),'max_recovery.release','physical')
                r.halt_at_measured_state()
                if decision.category!=Category.HARD:
                    # Registration motion alone is not confirmed uncontrolled
                    # fall. Stop opening, safely reclose, settle and reobserve.
                    r.reclose_at_current_pose()
                raise
            self.released=True;self.needs_safety_release=False;row['released']=True;self.save()
        self.events.transition(State.CLEARANCE_RETREAT,'max_recovery.escape')
        for plan in plans:
            if self.retreat.execute(plan):row['clearance_retreat']=True;self.save();return
        raise PlanningExhausted('RETREAT_PHYSICAL_ALTERNATIVES_EXHAUSTED')
    def candidates(self,D,family):
        H=np.asarray(self.visual_current(D)['T_world_handle'])
        if family=='template':return list(template_variants(self.template,H)) if self.template else []
        # Same semantic bar family, twelve diverse geometry-independent poses.
        E=H@np.asarray(self.template['T_handle_TCP']) if self.template else D@np.asarray(self.plan['rows'][0]['T'])
        rows=[]
        for i,(slide,depth,roll,swap,stand) in enumerate([
            (0,0,0,0,.04),(-.008,0,0,0,.04),(.008,0,0,0,.04),(0,-.004,0,0,.04),
            (0,.004,0,0,.04),(0,0,-10,0,.04),(0,0,10,0,.04),(0,0,0,180,.04),
            (-.005,-.002,0,180,.03),(.005,-.002,0,180,.05),(0,0,0,0,.02),(0,0,0,0,.065)]):
            T=E.copy();T[:3,3]+=H[:3,:3]@np.array([slide,0,depth]);T[:3,:3]=E[:3,:3]@Rotation.from_euler('xz',[roll,swap],degrees=True).as_matrix()
            rows.append({'candidate_index':i,'family':'generic-bar-side-pinch','T':T.tolist(),'standoff_m':stand})
        return rows
    def plan_variant(self,variant,base,start):
        r=self.r;scene=self.mobile.scene(base);T=np.asarray(variant['T']);fingers=r.finger_q().copy()
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
                ok,why,_=self.mobile.check(scene,q,b,r.finger_q())
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
    def run(self,value,capture_label=None,reason='WORKSPACE_RECOVERY',force_mobile=False,finish_after_capture=False,safety_release=False):
        if self.cycles>=self.policy['operation_cycles']:raise PlanningExhausted('NO_RECOVERY_OPERATION_BUDGET_EXHAUSTED')
        if not hasattr(self,'events'):
            self.events=getattr(self.r,'constraints',None) or ConstraintPolicy(self.r.capture.output,self.r);self.r.constraints=self.events
        self.cycles+=1;r=self.r;row={'index':len(self.history),'cycle':self.cycles,'reason':reason,'start_state_estimated':value,'initial_base':list(r.base),'regrasp_attempts':[],'force_mobile':force_mobile};self.history.append(row);self.save()
        D=self.current_D;observed=False
        self.events.transition(State.REOBSERVE,'max_recovery.pre_release')
        safety=safety_release or getattr(self,'needs_safety_release',False) or self.events.dispatch(reason,'max_recovery.reason').category==Category.HARD or reason=='SAFE_RELEASE_AFTER_LOAD_STOP'
        for retry in range(3):
            try:
                guess=self.motion_guess()
                D,audit,_=self.observe(guess,step_frames=not safety);row['pre_release_observation']=audit;observed=True;break
            except TaskTermination:raise
            except RuntimeError as e:
                self.events.dispatch(str(e),'max_recovery.observe','candidate');row.setdefault('observation_retries',[]).append(str(e));self.save()
                if not safety:r.hold(.3)
        # An unavailable ICP cannot block safe release indefinitely. Use last
        # sensor-derived placement and unchanged checked collision planner.
        if not observed:row['pre_release_observation']='last accepted RGB-D placement; current registration unavailable'
        if getattr(self,'clearance_ready',False) and self.released and r.released():
            self.clearance_ready=False;row['clearance_retreat']='previous checked queued escape retained'
        else:self.escape(D,row,safety_release=safety)
        if capture_label:
            self.events.transition(State.WRIST_SCAN,'max_recovery.capture')
            for scan_attempt in range(2):
                try:
                    capture_value=0. if capture_label=='closed_after_real_grasp' and not self.job.get('continuation_replay') else value
                    result=r.capture.scan(capture_label,capture_value,getattr(r,'saved_estimate',None));row['wrist_scan_completed']=result['clean_wrist_capture']
                    if result['clean_wrist_capture']:break
                except TaskTermination:raise
                except RuntimeError as e:
                    decision=self.events.dispatch(str(e),'max_recovery.capture','scan');row['scan_error']=str(e)
                    if decision.category==Category.HARD:raise
                self.save()
            self.save()
            if finish_after_capture and row.get('wrist_scan_completed'):return True
        excluded=set();physical={f:0 for f in ('template','generic')};base_moves=0;visited_bases=[list(r.base)]
        consumed={(tuple(np.round(v['base'],4)),v['family'],v['candidate_index']) for v in self.prior_failures}
        def move_next_base():
            nonlocal base_moves,D
            while base_moves<self.policy['actual_base_attempts']:
                excluded.add(tuple(np.round(r.base,4)));choice=self.choose_base(self.current_D,row,excluded)
                if choice is None:row.setdefault('base_failures',[]).append('NO_ROUTE_OR_APPROACH');self.save();return False
                # A failed actual candidate is not selected again in this pool.
                excluded.add(tuple(np.round(choice['base'],4)))
                self.events.transition(State.MOVE_BASE,'max_recovery.mobile');base_moves+=1
                try:self.move_base(choice)
                except TaskTermination:raise
                except RuntimeError as e:
                    self.events.dispatch(str(e),'max_recovery.base','physical');row.setdefault('base_failures',[]).append(str(e));self.save();r.halt_at_measured_state();continue
                visited_bases.append(list(r.base));D=self.current_D;return True
            return False
        if force_mobile:
            # NO_IK/workspace cannot be "recovered" by repeatedly closing at
            # the same base. The robot must physically search/move SE(2).
            moved=False
            while base_moves<self.policy['actual_base_attempts']:
                old=base_moves
                if move_next_base():moved=True;break
                if base_moves==old:break
            if not moved:raise PlanningExhausted('NO_RECOVERY_MOBILE_CANDIDATES_EXHAUSTED')
        def try_family(family):
            nonlocal D
            if physical[family]>=self.policy[family+'_closures']:return False
            self.events.transition(State.REOBSERVE,'max_recovery.wrist')
            try:Dnew,audit,_=self.observe(D,wrist=True);self.current_D=Dnew;r.set_observed_moving(Dnew)
            except TaskTermination:raise
            except RuntimeError as e:
                self.events.dispatch(str(e),'max_recovery.wrist','candidate');row.setdefault('wrist_reobserve_failures',[]).append(str(e));self.save();return False
            self.events.transition(State.REGRASP,'max_recovery.'+family)
            for v in self.candidates(Dnew,family):
                key=(tuple(np.round(r.base,4)),family,v['candidate_index'])
                if key in consumed:continue
                consumed.add(key);item={'family':family,'candidate_index':v['candidate_index'],'base':list(r.base),'current_wrist_observation':audit,'T_world_handle':self.visual_current(Dnew)['T_world_handle'],'T_world_TCP_candidate':v['T']};row['regrasp_attempts'].append(item);self.save()
                if physical[family]>=self.policy[family+'_closures']:break
                try:trial=self.plan_variant(v,r.base,r.arm_q())
                except RuntimeError as e:
                    self.events.dispatch(str(e),'max_recovery.grasp_plan','preflight');item['planning_error']=str(e);self.save();continue
                physical[family]+=1;item['physical_closure_number']=physical[family];self.save()
                choice={'base':list(r.base),'plan':{'trial_candidates':[trial]},'route':None};self.released=False
                try:
                    r.regrasp(choice,np.eye(4));item['physical_closure_completed']=True
                    r.state_offset=value;r.reset_grasp_memory()
                except TaskTermination:raise
                except RuntimeError as e:
                    self.events.dispatch(str(e),'max_recovery.closure_and_settle','physical');item['physical_regrasp_error']=str(e);self.save()
                    self.prior_failures.append({'base':list(r.base),'family':family,'candidate_index':v['candidate_index'],'reason':str(e),'t':r.time(),'source':'actual closure/compliance failure'})
                    (r.capture.output/'failed_regrasp_candidates.json').write_text(json.dumps(self.prior_failures,indent=2))
                    if str(e)=='SUSTAINED_CONTACT_LOSS':r.stop_failed_grasp_monitor()
                    r.halt_at_measured_state();self.escape(Dnew,row,safety_release=True);continue
                self.released=False;item['physical_regrasp_completed']=True;row['regrasp_completed']=True;row['status']='REGRASPED_FROM_CURRENT_WRIST_OBSERVATION';self.save()
                self.grasp_reference_ee=r.tcp().copy();self.grasp_reference_D=Dnew.copy();self.remember_success(Dnew)
                self.events.transition(State.CONTINUE,'max_recovery.success');return True
            D=self.current_D;return False
        # Template family is exhausted across the bounded actual-base pool
        # before generic closure is attempted. First saved template never changes.
        while self.template is not None:
            if try_family('template'):return True
            if physical['template']>=self.policy['template_closures']:break
            if not move_next_base():break
        row['template_family_exhausted']=True;self.save()
        if try_family('generic'):return True
        # Generic fallback has the same finite base pool as the template stage,
        # rather than being restricted to the last template base by accident.
        current_base=tuple(np.round(r.base,4))
        for base in reversed(visited_bases[:-1]):
            if physical['generic']>=self.policy['generic_closures']:break
            if tuple(np.round(base,4))==current_base:continue
            scene=self.mobile.scene(base);locked=r.arm_q()
            ok,why,_=self.mobile.locked_valid(scene,locked,base)
            if not ok:
                row.setdefault('generic_base_failures',[]).append({'base':base,'reason':why});self.save();continue
            route=self.mobile.locked_route(list(r.base),base,locked)
            if not route['valid']:
                row.setdefault('generic_base_failures',[]).append({'base':base,'reason':'NO_LOCKED_ROUTE'});self.save();continue
            self.events.transition(State.MOVE_BASE,'max_recovery.generic_base')
            try:self.move_base({'base':base,'route':route})
            except TaskTermination:raise
            except RuntimeError as e:
                self.events.dispatch(str(e),'max_recovery.generic_base','physical');r.halt_at_measured_state();continue
            D=self.current_D
            if try_family('generic'):return True
        row['status']='RECOVERY_CANDIDATES_EXHAUSTED';self.save()
        raise PlanningExhausted('NO_RECOVERY_BASE_ARM_REGRASP_BUDGET_EXHAUSTED')
