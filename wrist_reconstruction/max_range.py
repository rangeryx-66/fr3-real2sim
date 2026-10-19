"""Maximum-range orchestration, isolated from the historical periodic skill.

No object truth enters this module. All reported online states have a source;
EE motion is never labelled actual object motion.
"""
import copy,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

class ReleaseConsensus:
    def __init__(self,threshold=.001,required=3):
        self.threshold=threshold;self.required=required;self.vectors=[]
    def observe(self,displacement,vector):
        if displacement<=self.threshold:
            self.vectors=[];return 'STABLE'
        vector=np.asarray(vector,float)
        if self.vectors and np.dot(vector,self.vectors[-1])<0:
            self.vectors=[]
        self.vectors.append(vector)
        return 'OBJECT_MOVED_DURING_RELEASE' if len(self.vectors)>=self.required else 'PERCEPTION_UNCERTAIN'


def template_variants(template,H):
    """Twelve deterministic local variants; the proven pose is always first."""
    local=np.asarray(template['T_handle_TCP']);H=np.asarray(H)
    specs=[(0,0,0,0,0,.04),(0,-.004,0,0,0,.04),(0,.004,0,0,0,.04),
           (0,0,-.004,0,0,.04),(0,0,.004,0,0,.04),(-.003,0,0,0,0,.04),
           (.003,0,0,0,0,.04),(0,0,0,-5,0,.04),(0,0,0,5,0,.04),
           (0,0,0,0,-5,.04),(0,0,0,0,5,.04),(0,0,0,0,180,.04)]
    for i,(x,y,z,roll,yaw,standoff) in enumerate(specs):
        L=local.copy();L[:3,3]+=[x,y,z]
        L[:3,:3]=Rotation.from_euler('xz',[roll,yaw],degrees=True).as_matrix()@L[:3,:3]
        yield {'candidate_index':i,'family':'successful-template','T':(H@L).tolist(),'standoff_m':standoff,
               'perturbation_handle':{'translation_m':[x,y,z],'roll_deg':roll,'yaw_deg':yaw}}


def operation_tangent(r):
    """Follow current sensor handle frame, independently of EE hinge fitting."""
    normal=np.asarray(r.recovery.visual_current(r.recovery.current_D)['outward_normal_world'],float)
    return normal/np.linalg.norm(normal)


def effort_validity(sample,threshold):
    reasons=[]
    if sample.get('relative_translation_slip_m',0)>threshold:reasons.append('RELATIVE_GRASP_MOTION')
    if sample.get('articulation_consistency_error_m',0) or 0: # model residual alone is diagnostic
        pass
    if min(sample.get('forces_n',{'left':0,'right':0}).values(),default=0)<.05:reasons.append('CONTACT_UNSTABLE')
    if sample.get('force_event',{}).get('status','NORMAL')!='NORMAL':reasons.append('FORCE_TRANSIENT')
    if sample.get('phase') not in ('ESTIMATED_FOLLOW','EXPLORATORY'):reasons.append('TRANSIENT_OR_HOLD')
    return ('INVALID_FOR_EFFORT' if reasons else 'VALID_EFFORT_SEGMENT'),reasons


class ValidEffort:
    """Keep original measurement chain; exclude invalid segments from summaries."""
    def __init__(self,wrapped,root):
        self.wrapped=wrapped;self.root=Path(root);self.stream=(self.root/'effort_validity.jsonl').open('a')
    def __getattr__(self,name):return getattr(self.wrapped,name)
    def begin(self,*args):return self.wrapped.begin(*args)
    def end(self,*args):return self.wrapped.end(*args)
    def append(self,s,dt,direction,estimate):
        status,reasons=effort_validity(s,.001)
        segment=self.wrapped.segment
        if segment is not None:
            segment.setdefault('validity_counts',{'VALID_EFFORT_SEGMENT':0,'INVALID_FOR_EFFORT':0})[status]+=1
            if reasons:segment.setdefault('invalid_reasons',[]).extend(x for x in reasons if x not in segment.get('invalid_reasons',[]))
            segment['effort_validity']='INVALID_FOR_EFFORT' if segment['validity_counts']['INVALID_FOR_EFFORT'] else 'VALID_EFFORT_SEGMENT'
        self.stream.write(json.dumps({'t':s['t'],'segment':None if segment is None else segment['index'],'status':status,'reasons':reasons})+'\n')
        self.wrapped.append(s,dt,direction,estimate)
    def close(self):
        self.stream.close();self.wrapped.close()
        valid=[r for r in self.wrapped.records if r.get('effort_validity')=='VALID_EFFORT_SEGMENT']
        (self.root/'valid_effort_segments.json').write_text(json.dumps(valid,indent=2))


def run(r):
    from wrist_reconstruction.constraint_policy import ConstraintPolicy,Category,State,TaskTermination
    p=r.policy;kind=p['joint_type'];recovery=r.recovery;out=r.capture.output
    events=getattr(r,'constraints',None) or ConstraintPolicy(out,r);r.constraints=events
    segment=0;path=0.;last=r.tcp()[:3,3].copy();maximum=0.;failures=[];capture_phase=False;capture_queue=[];motion_sign=1.
    journal=(out/'maximum_progress.jsonl').open('a')
    def current():
        return r.state_offset+(r.memory.angle_deg(r.tcp()) if kind=='revolute' else r.memory.state(r.tcp()))
    def checkpoint(stage,observe=True):
        nonlocal maximum
        if observe:value,source=recovery.observed_progress(current())
        else:value,source=(recovery.last_observed_state,'FIXED_RGBD_HANDLE_REGISTRATION_CACHED') if recovery.progress_valid else (current(),'EE_PROXY_NOT_OBJECT_MEASUREMENT')
        if source!='EE_PROXY_NOT_OBJECT_MEASUREMENT' and value>maximum:
            maximum=value;journal.write(json.dumps({'t':r.time(),'maximum_observed_state':maximum,'source':source})+'\n');journal.flush()
        data={'stage':stage,'current_state':value,'state_source':source,'max_observed_state':maximum,'EE_proxy_state':current(),
              'q':r.arm_q().tolist(),'base':list(r.base),'observed_handle':recovery.visual_current(recovery.current_D),
              'successful_grasp_template':recovery.template,'effort_segments':r.effort.records,'capture_states':r.capture.states,
              'recovery_cycle':recovery.cycles,'base_moves':recovery.count,'constraint_policy':events.checkpoint(),
              'command_history':'command_tape.jsonl','effort_validity':'effort_validity.jsonl',
              'physical_restore':'no object/contact state restore; new closed setup is default'}
        tmp=out/'max_range_checkpoint.tmp';tmp.write_text(json.dumps(data,indent=2));tmp.replace(out/'max_range_checkpoint.json')
        if source!='EE_PROXY_NOT_OBJECT_MEASUREMENT':
            label=(f"{r.capture.metadata['asset_id']}_max_{maximum:.2f}" if kind=='revolute' else f"{r.capture.metadata['asset_id']}_max_{100*maximum:.2f}cm")
            folder=out/'maximum_checkpoints';folder.mkdir(exist_ok=True)
            (folder/(label+'.json')).write_text(json.dumps(data,indent=2))
        return value,source
    def recover(reason,capture=None,final=False):
        nonlocal last
        decision=events.dispatch(reason,'max_range.recover',context='physical' if reason=='LOW_JOINT_MARGIN' else 'operation')
        mobile_required=decision.force_mobile;hard_pending=decision.category==Category.HARD;exhausted_poses=set()
        r.drive.active=False
        if reason in ('SUSTAINED_CONTACT_LOSS','CAPTURE_HOLD_GRASP_LOST'):r.stop_failed_grasp_monitor()
        r.halt_at_measured_state();frozen_state,state_source=checkpoint('RECOVERY_ENTRY',observe=False)
        failures.append({'t':r.time(),'reason':reason,'category':decision.category.value})
        while recovery.cycles<recovery.policy['operation_cycles'] and time.time()<r.deadline:
            try:
                result=recovery.run(frozen_state,capture_label=capture,reason=reason,
                                    force_mobile=mobile_required,finish_after_capture=final,safety_release=hard_pending)
                last=r.tcp()[:3,3].copy();checkpoint('RECOVERY_EXIT')
                if not final:events.transition(State.CONTINUE,'max_range.recover')
                return result
            except TaskTermination:raise
            except RuntimeError as error:
                decision=events.dispatch(str(error),'max_range.recovery_attempt',context='operation')
                mobile_required=mobile_required or decision.force_mobile;hard_pending=hard_pending or decision.category==Category.HARD
                failures.append({'t':r.time(),'reason':'RECOVERY_ATTEMPT_FAILED','detail':str(error),'category':decision.category.value})
                (out/'recovery_failures.json').write_text(json.dumps(failures,indent=2))
                if str(error)=='SUSTAINED_CONTACT_LOSS':r.stop_failed_grasp_monitor()
                r.halt_at_measured_state()
                if str(error).startswith(('CONFIRMED_UNCONTROLLED_OBJECT_','UNSAFE_TO_RECOVER')):
                    events.finish('HARD_PHYSICAL_SAFETY',trigger=str(error));raise TaskTermination('HARD_PHYSICAL_SAFETY:'+str(error))
                reason=str(error)
                if reason=='RETREAT_ALTERNATIVES_EXHAUSTED':
                    signature=(tuple(np.round(r.arm_q(),4)),tuple(np.round(r.base,4)),tuple(np.round(r.finger_q(),4)))
                    if signature in exhausted_poses:
                        failures.append({'t':r.time(),'reason':'BOUNDED_CLEARANCE_POOL_EXHAUSTED_AT_UNCHANGED_POSE','distinct_retreat_rows':len(recovery.retreat.rows)})
                        break
                    exhausted_poses.add(signature)
                # A failed scan retains its requested milestone; a single
                # perception or retreat candidate does not erase the request.
        terminal='WALL_BUDGET_EXHAUSTED' if time.time()>=r.deadline else ('HARD_PHYSICAL_SAFETY' if hard_pending and not recovery.released else 'NO_RECOVERY_AVAILABLE')
        # Exhausting grasp/base recovery does not erase a safely released
        # maximum. Scan it without spending another grasp cycle or regrasping.
        if terminal=='NO_RECOVERY_AVAILABLE' and maximum>=(60. if kind=='revolute' else .1) and recovery.released and r.released():
            events.transition(State.WRIST_SCAN,'max_range.exhausted_recovery_capture')
            try:r.capture.scan('maximum_safe_state',frozen_state,getattr(r,'saved_estimate',None))
            except TaskTermination:raise
            except RuntimeError as e:
                events.dispatch(str(e),'max_range.exhausted_recovery_capture','scan')
                failures.append({'t':r.time(),'reason':'MAXIMUM_SCAN_UNAVAILABLE','detail':str(e)})
        if terminal in ('NO_RECOVERY_AVAILABLE','HARD_PHYSICAL_SAFETY') and recovery.policy.get('keep_actor_for_recovery',False) and time.time()<r.deadline:
            from wrist_reconstruction.live_recovery import wait_for_recovery
            result=wait_for_recovery(r,recovery,events,frozen_state,reason)
            last=r.tcp()[:3,3].copy();checkpoint('LIVE_RECOVERY_RESUMED');return result
        events.finish(terminal,cycles=recovery.cycles,base_moves=recovery.count,failures=failures)
        raise TaskTermination(terminal)
    try:
        initial_issue=getattr(r,'initial_grasp_issue',None)
        if not initial_issue:recovery.remember_success(np.eye(4))
        from wrist_reconstruction.operation_memory import restore
        restore(r)
        if initial_issue:recover(initial_issue)
        if r.capture.job.get('continuation_replay'):
            if not r.grip()[0]:recover('CONTINUATION_REESTABLISH_GRASP')
        # Existing sensor-only operation memory normally bypasses discovery.
        for attempt in range(0 if r.memory.estimate is not None else 4):
            d=r.memory.begin_attempt(attempt,r.time());r.drive.set_direction(d,r.tcp());r.drive.active=True;r.phase('EXPLORATORY')
            t=r.time();r.effort.begin(segment,current(),t);segment+=1
            while r.time()-t<35 and time.time()<r.deadline:
                try:r.step()
                except RuntimeError as e:recover(str(e));break
                if r.tick()%8==0:
                    r.memory.observe(r.tcp())
                    if r.time()-t>=1 and len(r.memory.poses)%30==0:r.memory.try_fit()
                    if r.memory.estimate is not None:break
                    if r.time()-t>=12 and r.memory.travel(r.memory.start_index)<.002:break
            r.effort.end(current(),r.time(),'DISCOVERY');r.memory.finish_attempt(r.time(),'COARSE_MODEL_AVAILABLE' if r.memory.estimate else 'LOW_EXCITATION')
            if r.memory.estimate:break
        if r.memory.estimate is None:
            recover('ARTICULATION_UNOBSERVABLE_WITHIN_FROZEN_PROBE_BUDGET')
            restore(r)
            if r.memory.estimate is None:
                events.finish('NO_RECOVERY_AVAILABLE',scope='sensor-only operation model candidates exhausted')
                raise TaskTermination('NO_RECOVERY_AVAILABLE')
        status='WALL_BUDGET_EXHAUSTED'
        # segment/path/sim counters remain diagnostics. Only the original
        # absolute wall ceiling bounds safe continuous manipulation.
        while time.time()<r.deadline:
            try:value,source=checkpoint('CONTINUOUS_MANIPULATION')
            except TaskTermination:raise
            except RuntimeError as e:recover(str(e));continue
            goal=capture_queue[0] if capture_phase else p['targets'][-1]
            reached=source!='EE_PROXY_NOT_OBJECT_MEASUREMENT' and (value>=goal if motion_sign>0 else value<=goal+(.5 if kind=='revolute' else .002))
            if reached:
                r.drive.active=False
                if not capture_phase:
                    # Physical maximum comes first. Only then revisit sensor
                    # states using the same physical control and recovery path.
                    capture_phase=True
                    capture_queue=list(reversed(r.capture.config['capture']['key_states'][kind][:3]))
                    recover('POST_GOAL_WRIST_CAPTURE','maximum_safe_state')
                    motion_sign=-1.
                else:
                    target=capture_queue.pop(0)
                    recover('POST_GOAL_WRIST_CAPTURE',f'post_goal_state_{target}',final=not capture_queue)
                    if not capture_queue:status='TARGET_REACHED';break
                continue
            d=motion_sign*operation_tangent(r);safe,why=r.increment_safe(d,p['segment_m'])
            if not safe and why=='LOW_JOINT_MARGIN':why='JOINT_MARGIN_WARNING'
            if r.margin()<=p['warning_margin_rad'] or not safe:
                recover('JOINT_MARGIN_WARNING' if safe else why);continue
            rested=not r.drive.active
            if rested:r.drive.set_direction(d,r.tcp())
            else:r.drive.refresh_tangent(d,r.tcp())
            events.transition(State.PULL,'max_range.continuous')
            r.drive.active=True;r.phase('ESTIMATED_FOLLOW');r.effort.begin(segment,value,r.time());r.effort.segment['from_rest']=rested
            segment+=1;t=r.time();P=r.tcp()[:3,3].copy();issue=None
            while r.time()-t<p['segment_timeout_s'] and time.time()<r.deadline:
                try:r.step()
                except RuntimeError as e:issue=str(e);break
                if r.tick()%8==0:
                    E=r.tcp();path+=float(np.linalg.norm(E[:3,3]-last));last=E[:3,3].copy();r.memory.observe(E)
                    # Predict every control update, retaining original IK,
                    # margin, collision and drive speed/effort detectors.
                    tangent=motion_sign*operation_tangent(r);safe,why=r.increment_safe(tangent,p['segment_m'])
                    if not safe and why=='LOW_JOINT_MARGIN':why='JOINT_MARGIN_WARNING'
                    if r.margin()<=p['warning_margin_rad'] or not safe:
                        issue='JOINT_MARGIN_WARNING' if safe else why;r.drive.active=False;break
                    r.drive.refresh_tangent(tangent,E)
                    if getattr(r,'force_guard',None) is not None and r.force_guard.pending:
                        issue='SOFT_FORCE_WARNING';r.drive.active=False;break
                    if getattr(r.memory,'pending',False):
                        r.memory.pending=False
                        events.dispatch('MODEL_INCONSISTENCY','max_range.model_monitor',handling='diagnostic only; current observed handle frame drives operation')
                    if np.linalg.norm(E[:3,3]-P)>=p['segment_m']:break
            moved=float(np.linalg.norm(r.tcp()[:3,3]-P));r.effort.end(value,r.time(),issue or 'SEGMENT_COMPLETE')
            if issue=='SOFT_FORCE_WARNING':
                events.dispatch(issue,'max_range.force');r.drive.active=False;r.phase('SYSTEM_FORCE_SETTLE')
                try:
                    r.hold(.25);ready,detail=r.grip()
                    if not ready:recover('SUSTAINED_CONTACT_LOSS');continue
                    if not r.force_guard.settled():recover('SAFE_RELEASE_AFTER_LOAD_STOP');continue
                    r.force_guard.pending=False;r.drive.speed_m_s=min(r.drive.speed_m_s,.5*type(r.drive).speed_m_s)
                    r.drive.set_direction(motion_sign*operation_tangent(r),r.tcp());events.transition(State.CONTINUE,'max_range.force_settled')
                    checkpoint('TRANSIENT_FORCE_SETTLED_CONTINUE');continue
                except TaskTermination:raise
                except RuntimeError as e:recover(str(e));continue
            if issue or moved<.0001:recover(issue or 'NO_USEFUL_MOTION');continue
        r.drive.active=False
        if status!='TARGET_REACHED' and maximum>=(60. if kind=='revolute' else .1) and time.time()<r.deadline:
            try:r.hold(p['hold_s']);checkpoint('MAXIMUM_SAFE_STATE')
            except RuntimeError as e:recover(str(e))
            recover('FINAL_CLEAN_CAPTURE','maximum_safe_state',final=True)
        events.finish(status,maximum_observed=maximum);r.capture.flush(status)
        return {'status':status,'maximum_observed_state':maximum,'EE_proxy_state':current(),'path_m':path,'segments':segment,'recovery_cycles':recovery.cycles,'failures':failures}
    finally:
        (out/'recovery_failures.json').write_text(json.dumps(failures,indent=2));journal.close()
