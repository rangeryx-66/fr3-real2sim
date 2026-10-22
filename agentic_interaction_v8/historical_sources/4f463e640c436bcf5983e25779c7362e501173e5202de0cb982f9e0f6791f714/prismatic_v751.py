"""V7.5.1 engineering extension; all frozen predecessors remain unchanged."""
import numpy as np
from .prismatic_feedback_v75 import PrismaticEvidenceRuntimeV75
from .prismatic_operations_v75_r3 import PrismaticOperationsV75R3
from .prismatic_unloading_v751 import TransverseUnloading,causal_world_wrench


class PrismaticRuntimeV751(PrismaticEvidenceRuntimeV75):
    def __init__(self,job):
        if job.get('engineering_revision')!='V7.5.1':raise ValueError('V7.5.1 engineering registration required')
        super().__init__(job)
        self.transverse_unloading_enabled=False;self.unloading=None;self.unloading_epoch=np.zeros(3);self.unloading_closure=None
        original=self.Closure
        def closure(T,policy):
            self.unloading=None;self.unloading_epoch=np.zeros(3)
            self.unloading_closure=original(T,policy)
            return self.unloading_closure
        self.Closure=closure

    def begin_reference(self):
        super().begin_reference()
        self.unloading_epoch=self.unloading.offset.copy()if self.unloading is not None else np.zeros(3)

    def cartesian_step(self,goal,phase):
        if (self.transverse_unloading_enabled and self.grasp_reference is not None
                and self.policy.phase(phase)=='LOADED' and not self._decelerating):
            if self.reference_drive is None:self.begin_reference()
            if self.unloading is None:
                if self.unloading_closure is None:raise RuntimeError('UNLOADING_CLOSURE_CONTEXT_UNAVAILABLE')
                c=self.unloading_closure
                self.unloading=TransverseUnloading(self.closure_policy,self.contact_evidence.axis,
                    self.filtered_loads.maxlen,c.command[:3,3]-c.origin[:3,3])
            # Causal, trailing filter only; same native load window as closure.
            recent=list(self.rows)[-self.filtered_loads.maxlen:]
            force=np.mean([causal_world_wrench(r,self.job['ft_calibration']['bias_sensor'])[:3]for r in recent],axis=0)
            centres=[]
            for native,logical in [('left_finger','gripper_link1'),('right_finger','gripper_link2')]:
                T=self.pose(native)@self.pad_frames[native]
                centres.append((T@np.r_[self.model.pads[logical].mean(0),1])[:3])
            update=self.unloading.update(self.tick,self.dt,force,centres)
            if update is not None:self.event('V751_TRANSVERSE_UNLOADING',**update)
            goal=np.array(goal,copy=True);goal[:3,3]+=self.unloading.offset-self.unloading_epoch
        return super().cartesian_step(goal,phase)


class PrismaticOperationsV751(PrismaticOperationsV75R3):
    def request_plan(self,command):
        enabled=command.get('transverse_unloading',False)
        if type(enabled)is not bool:return dict(plan=False,reason='BOOLEAN_UNLOADING_SELECTION_REQUIRED')
        if self.g['recovery_state']!='UNLOADED_OBSERVATION'or self.r.supervisor.latched is not None:
            return dict(plan=False,reason='VERIFIED_UNLOADED_BOUNDARY_REQUIRED')
        result=super().request_plan(command)
        if result.get('plan'):
            self.r.transverse_unloading_enabled=enabled
            self.r.event('V751_AGENT_TRAJECTORY_SELECTION',transverse_unloading=enabled,
                         reason=command.get('reason'),grip_authority_changed=False)
            result['transverse_unloading']=enabled
        return result
