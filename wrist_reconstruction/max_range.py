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
    specs=[(0,0,0,0,0,.04),(-.003,0,0,0,0,.04),(.003,0,0,0,0,.04),
           (0,0,-.002,0,0,.04),(0,0,.002,0,0,.04),(0,0,0,0,-5,.04),
           (0,0,0,0,5,.04),(0,0,0,-5,0,.04),(0,0,0,5,0,.04),
           (0,0,0,0,180,.04),(0,0,0,0,0,.025),(0,0,0,0,0,.06)]
    for i,(x,y,z,roll,yaw,standoff) in enumerate(specs):
        L=local.copy();L[:3,3]+=[x,y,z]
        L[:3,:3]=Rotation.from_euler('xz',[roll,yaw],degrees=True).as_matrix()@L[:3,:3]
        yield {'candidate_index':i,'family':'successful-template','T':(H@L).tolist(),'standoff_m':standoff,
               'perturbation_handle':{'translation_m':[x,y,z],'roll_deg':roll,'yaw_deg':yaw}}


def effort_validity(sample,threshold):
    reasons=[]
    if sample.get('relative_translation_slip_m',0)>threshold:reasons.append('RELATIVE_GRASP_MOTION')
    if sample.get('articulation_consistency_error_m',0) or 0: # model residual alone is diagnostic
        pass
    if min(sample.get('forces_n',{'left':0,'right':0}).values(),default=0)<.05:reasons.append('CONTACT_UNSTABLE')
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
    p=r.policy;kind=p['joint_type'];recovery=r.recovery;out=r.capture.output
    segment=0;path=0.;started=r.time();last=r.tcp()[:3,3].copy();maximum=0.;failures=[]
    journal=(out/'maximum_progress.jsonl').open('a')
    def current():
        return r.state_offset+(r.memory.angle_deg(r.tcp()) if kind=='revolute' else r.memory.state(r.tcp()))
    def checkpoint(stage):
        nonlocal maximum
        value,source=recovery.observed_progress(current())
        if source!='EE_PROXY_NOT_OBJECT_MEASUREMENT' and value>maximum:
            maximum=value;journal.write(json.dumps({'t':r.time(),'maximum_observed_state':maximum,'source':source})+'\n');journal.flush()
        data={'stage':stage,'current_state':value,'state_source':source,'max_observed_state':maximum,'EE_proxy_state':current(),
              'q':r.arm_q().tolist(),'base':list(r.base),'observed_handle':recovery.visual_current(recovery.current_D),
              'successful_grasp_template':recovery.template,'effort_segments':r.effort.records,'capture_states':r.capture.states,
              'recovery_cycle':recovery.cycles,'base_moves':recovery.count,'physical_restore':'live process only; no object or contact state restore'}
        tmp=out/'max_range_checkpoint.tmp';tmp.write_text(json.dumps(data,indent=2));tmp.replace(out/'max_range_checkpoint.json')
        return value,source
    def recover(reason,capture=None):
        nonlocal last
        r.drive.active=False;r.halt_at_measured_state();checkpoint('RECOVERY_ENTRY')
        failures.append({'t':r.time(),'reason':reason});(out/'recovery_failures.json').write_text(json.dumps(failures,indent=2))
        while recovery.cycles<recovery.policy['operation_cycles'] and time.time()<r.deadline:
            try:
                result=recovery.run(current(),capture_label=capture,reason=reason)
                last=r.tcp()[:3,3].copy();checkpoint('RECOVERY_EXIT');return result
            except RuntimeError as error:
                failures.append({'t':r.time(),'reason':'RECOVERY_ATTEMPT_FAILED','detail':str(error)})
                (out/'recovery_failures.json').write_text(json.dumps(failures,indent=2))
                r.halt_at_measured_state()
                # Confirmed uncontrolled release and unsafe loaded contact are
                # not made harmless by having unused search budget.
                if str(error).startswith(('OBJECT_MOVED_DURING_RELEASE','EXISTING_','PROBE_CARTESIAN_SPEED_LIMIT','DANGEROUS_NATIVE','UNSAFE_TO_RECOVER')):raise
                reason=str(error);capture=None
        raise RuntimeError('NO_RECOVERY_FROZEN_BUDGET_EXHAUSTED')
    try:
        recovery.remember_success(np.eye(4))
        # Reuse verified closed wrist data where possible; a failed scan is not
        # allowed to erase an established grasp or manipulation progress.
        if not r.capture.states:
            recover('INITIAL_CLEAN_CAPTURE','closed_after_real_grasp')
        from wrist_reconstruction.operation_memory import restore
        restore(r)
        for attempt in range(0 if r.memory.estimate is not None else 4):
            d=r.memory.begin_attempt(attempt,r.time());r.drive.set_direction(d,r.tcp());r.drive.active=True;r.phase('EXPLORATORY')
            t=r.time();r.effort.begin(segment,current(),t);segment+=1
            while r.time()-t<35:
                try:r.step()
                except RuntimeError as e:recover(str(e));break
                if r.tick()%8==0:
                    r.memory.observe(r.tcp())
                    if r.time()-t>=1 and len(r.memory.poses)%30==0:r.memory.try_fit()
                    if r.memory.estimate is not None:break
                    if r.time()-t>=12 and r.memory.travel(r.memory.start_index)<.002:break
            r.effort.end(current(),r.time(),'DISCOVERY');r.memory.finish_attempt(r.time(),'COARSE_MODEL_AVAILABLE' if r.memory.estimate else 'LOW_EXCITATION')
            if r.memory.estimate:break
        if r.memory.estimate is None:raise RuntimeError('ARTICULATION_UNOBSERVABLE_WITHIN_FROZEN_PROBE_BUDGET')
        status='TASK_BUDGET_EXHAUSTED'
        while segment<p['maximum_segments'] and r.time()-started<p['maximum_sim_s'] and path<p['maximum_path_m'] and time.time()<r.deadline:
            value,source=checkpoint('CONTINUOUS_MANIPULATION')
            if source!='EE_PROXY_NOT_OBJECT_MEASUREMENT' and value>=p['targets'][-1]:status='OBSERVED_TARGET_REACHED';break
            d=r.memory.tangent(r.tcp());safe,why=r.increment_safe(d,p['segment_m'])
            if r.margin()<=p['warning_margin_rad'] or not safe:
                recover('JOINT_MARGIN_WARNING' if safe else why);continue
            rested=not r.drive.active
            if rested:r.drive.set_direction(d,r.tcp())
            else:r.drive.refresh_tangent(d,r.tcp())
            r.drive.active=True;r.phase('ESTIMATED_FOLLOW');r.effort.begin(segment,value,r.time());r.effort.segment['from_rest']=rested
            segment+=1;t=r.time();P=r.tcp()[:3,3].copy();issue=None
            while r.time()-t<p['segment_timeout_s']:
                try:r.step()
                except RuntimeError as e:issue=str(e);break
                if r.tick()%8==0:
                    E=r.tcp();path+=float(np.linalg.norm(E[:3,3]-last));last=E[:3,3].copy()
                    r.memory.observe(E)
                    if getattr(r.memory,'pending',False):
                        r.drive.active=False
                        try:r.memory.resolve(r.time())
                        except RuntimeError as e:
                            # An unresolved model is not measured grasp slip.
                            r.memory.pending=False;failures.append({'t':r.time(),'reason':'MODEL_INCONSISTENCY','detail':str(e)})
                        break
                    if np.linalg.norm(E[:3,3]-P)>=p['segment_m']:break
            moved=float(np.linalg.norm(r.tcp()[:3,3]-P));r.effort.end(value,r.time(),issue or 'SEGMENT_COMPLETE')
            if issue or moved<.0001:recover(issue or 'NO_USEFUL_MOTION');continue
        r.drive.active=False;r.hold(p['hold_s']);checkpoint('MAXIMUM_SAFE_STATE')
        try:recover('FINAL_CLEAN_CAPTURE','maximum_safe_state')
        except RuntimeError as e:failures.append({'t':r.time(),'reason':'FINAL_CAPTURE_RECOVERY_EXHAUSTED','detail':str(e)})
        r.capture.flush(status)
        return {'status':status,'maximum_observed_state':maximum,'EE_proxy_state':current(),'path_m':path,'segments':segment,'recovery_cycles':recovery.cycles,'failures':failures}
    finally:
        journal.close()
