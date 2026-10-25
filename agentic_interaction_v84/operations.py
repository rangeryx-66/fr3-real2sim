"""Current-contact continuation around the original native acquisition/segments.

No manual grasp transforms, object commands or actuator changes. A held C
boundary has finite real-physics supervision; timeout unloads and parks.
"""
import hashlib,json,time
from contextlib import contextmanager
from unittest.mock import patch
import numpy as np
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.policy import SafetyStop
from interaction_runtime.segments import ExecutionBudgetExhausted


def contact_trusted(ops):
    r=ops.r
    if r.supervisor.latched is not None or r.grasp_reference is None or not r.rows:return False
    row=r.rows[-1]
    if not row['wrist_ft']['valid'] or not np.isfinite(row['joint_coordinate']):return False
    recent=[x for x in r.rows if x['t']>=row['t']-.5]
    if not recent:return False
    # Same original handover aperture/slip-change cells, allowing finite slip.
    if np.ptp([x['aperture_m'] for x in recent])>=.0005:return False
    if np.ptp([x['relative_translation_slip_m'] for x in recent])>=r.closure_policy['max_slip_m']:return False
    return bool(max(r.filtered())>=.05)


def operations_class(parent):
    class ABCOperations(parent):
        def __init__(self,r):
            super().__init__(r);self.abc_state='A';self.abc_contact_generation=0
            self._abc_retain=False;self.abc_history=[];self.abc_declared=[]

        def decision_packet(self,sequence,result=None):
            if self.g.get('recovery_state')=='LOADED_CRITIQUE':
                return dict(sequence=sequence,unloaded=False,abc=self.packet(),
                    last_result=result,segments=self.r.record.segments[-4:],
                    remaining_resources=self.r.episode_budget.remaining(self.r.tick*self.r.dt))
            return super().decision_packet(sequence,result)

        def request_plan(self,command):
            # In V8.3 this request was applied only on the prismatic branch.
            # Use the same original finite request->position-offset mapping.
            if self.articulation().joint_type=='revolute' and 'native_g1_preload_request_n' in command:
                from agentic_interaction_v81.preload_extension import select
                select(self,command)
            from .planning import schedule
            with schedule(self):result=super().request_plan(command)
            if result.get('plan'):self.abc_declared=list(command['planned_segments'])
            return result

        def release_park(self):
            last=self.r.record.segments[-1].get('result',{}) if self.r.record.segments else {}
            if (self._abc_retain and last.get('status')=='TARGET_REACHED'
                    and self.r.record.segments[-1].get('first_step',0)>=getattr(self,'_v81_block_first_record',0)
                    and last.get('physical_transmission_valid') and contact_trusted(self)):
                self.abc_state='C';self.g['recovery_state']='LOADED_CRITIQUE'
                return dict(safe=False,contact_retained=True,unloaded=False,
                    reason='FINITE_GUARDED_C_BOUNDARY_NOT_SAFE_PARK',
                    maximum_C_wall_s=20.,maximum_C_sim_s=5.)
            self.abc_state='A'
            result=super().release_park()
            if self.articulation().joint_type=='prismatic':self.g['v75_active_attempt']=0
            return result

        def rotate_segment(self,*args,**kwargs):
            action=kwargs.get('action') or (args[2] if len(args)>2 else {}) or {}
            captures=[]
            if action.get('execution_strategy','continuous')=='legacy':
                return super().rotate_segment(*args,**kwargs)
            import interaction_runtime.segments as segments
            from .monitor import factory
            # Explicit, isolated finite call binding. Frozen source unchanged.
            with patch.object(segments,'TransmissionMonitor',factory(self.r,self.articulation(),captures)):
                result=super().rotate_segment(*args,**kwargs)
            history=[x for m in captures for x in m.history]
            alerts=[m.first_advisory_s for m in captures if m.first_advisory_s is not None]
            result['abc_feedback']=dict(windows=history,original_reference_lead_unchanged=True,
                original_increasing_slip_stop_unchanged=True,
                nominal_warning_is_not_contact_failure=True,
                FIT_exclude_entire_segment=bool(alerts),
                first_nominal_warning_s=min(alerts) if alerts else None)
            return result

        def attempt(self,command):
            canonical=lambda x:json.dumps(x,sort_keys=True)
            declared={canonical(a) for a in self.abc_declared}
            if any(canonical(a) not in declared for a in command.get('segments',[])):
                return dict(executed=False,status='UNDECLARED_ACTION_REQUIRES_UNLOADED_PLAN')
            full=self._v81_declared_chain
            rail_full=self.g.get('v75_declared_chain')
            self._v81_declared_chain=command['segments']
            if self.articulation().joint_type=='prismatic':self.g['v75_declared_chain']=command['segments']
            self._abc_retain=bool(command.get('retain_contact',False))
            try:result=super().attempt(command)
            finally:
                self._abc_retain=False;self._v81_declared_chain=full
                if rail_full is not None:self.g['v75_declared_chain']=rail_full
            if result.get('safe_exit',{}).get('contact_retained'):
                self.abc_contact_generation+=1;self.abc_state='C'
                if self.articulation().joint_type=='prismatic':self.g['v75_active_attempt']=len(self.g['contact_attempts'])
                result['abc']=self.packet()
            else:self.abc_state='A'
            return result

        def continue_motion(self,command):
            if self.abc_state!='C' or not contact_trusted(self):
                return dict(executed=False,status='CONTACT_RECOVERY_REQUIRED',
                    safe_exit=self.release_park(),reason='CURRENT_CONTACT_NOT_TRUSTED')
            if command.get('contact_generation')!=self.abc_contact_generation:
                return dict(executed=False,status='STALE_CONTACT_GENERATION',remaining_goal=self.packet())
            actions=command['segments']
            # All target/motion/role fields must already have been approved and
            # native-checked in the fresh plan; a new reason cannot change them.
            canonical=lambda x:json.dumps(x,sort_keys=True)
            declared={canonical(a) for a in self.abc_declared}
            if any(canonical(a) not in declared for a in actions):
                return dict(executed=False,status='UNDECLARED_ACTION_REQUIRES_UNLOADED_PLAN')
            from agentic_interaction_v81.validation_queue import enqueue
            enqueue(self,command)
            self.actions=list(actions);self._v81_action_index=1;result=dict(segments=[],acquisition_reused=True)
            self.abc_state='B';self.g['recovery_state']='FINITE_CONTACT_OPERATION'
            try:
                for action in actions:
                    route=self.check_successor(action)
                    if not route.get('valid'):
                        result.update(status='PLANNING_REJECTED',native_check=route);break
                    segment=self.interact(self.action_target(action),action=action)
                    result['segments'].append(segment)
                    if segment.get('status')!='TARGET_REACHED':break
                self._abc_retain=bool(command.get('retain_contact',False)) and bool(result['segments']) and result['segments'][-1].get('status')=='TARGET_REACHED'
                result['safe_exit']=self.release_park()
                self.abc_contact_generation+=1
            except SafetyStop:raise
            except ExecutionBudgetExhausted as error:
                self._abc_retain=False;result.update(status='OPERATION_BUDGET_EXHAUSTED',reason=str(error),safe_exit=self.release_park())
            except RuntimeError as error:
                if str(error)!='LOCAL_IK_UNAVAILABLE':raise
                self._abc_retain=False
                result.update(status='PLANNING_REJECTED',reason=str(error),
                    safe_exit=self.release_park(),recoverable=True)
            finally:self._abc_retain=False
            if self.g['contact_attempts']:
                previous=self.g['contact_attempts'][-1]
                previous['segments'].extend(result['segments'])
                previous['last_record']=self.r.record.count
                previous['safe_exit']=result.get('safe_exit')
            result['abc']=self.packet();return result

        def packet(self):
            r=self.r;goal=r.job['v81_task_goal_native'];q=float(r.actual())
            return dict(state=self.abc_state,contact_generation=self.abc_contact_generation,
                actual_joint_q=q,original_goal_q=goal,remaining_native=goal-q,
                contact_trusted=contact_trusted(self),station_generation=r.base_generation,
                record_count=r.record.count,model_not_a_manipulation_gate=True,
                hard_latch=r.supervisor.latched)

        def feedback(self):
            # Loaded C returns synchronized numeric results immediately. Heavy
            # plots are rendered only after unloading, never in the hold loop.
            if self.g.get('recovery_state')=='LOADED_CRITIQUE':
                return dict(abc=self.packet(),segments=self.r.record.segments[-4:])
            path=self.r.output/'model/MODEL_PUBLICATION.json'
            key=(self.r.record.count,len(self.g['contact_attempts']),
                path.stat().st_mtime_ns if path.exists() else None)
            if key==getattr(self,'_abc_feedback_key',None):return self._abc_feedback_value
            from .waits import wait
            with wait(self.r,'READ_ONLY_FEEDBACK','contact plot and numeric publication'):
                value=super().feedback()
            self._abc_feedback_key=key;self._abc_feedback_value=value
            return value
    return ABCOperations
