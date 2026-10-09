"""Object-level event policy. Detection/physics thresholds stay in their owners.

A HARD event stops the current action, not automatically the object task.
Only an explicit terminal condition may end bounded recovery.
"""
from dataclasses import dataclass, field, asdict
from enum import Enum
import json, time
from pathlib import Path


class Category(str, Enum):
    HARD = 'HARD_PHYSICAL_SAFETY'
    RECOVERABLE = 'RECOVERABLE_PLANNING_OR_PERCEPTION'
    DATA = 'DATA_QUALITY_ONLY'
    DIAGNOSTIC = 'DIAGNOSTIC_ONLY'


class State(str, Enum):
    GRASP='GRASP'; PULL='PULL'; SAFE_HOLD='SAFE_HOLD'; RELEASE='RELEASE'
    CLEARANCE_RETREAT='CLEARANCE_RETREAT'; MOVE_BASE='MOVE_BASE'
    REOBSERVE='REOBSERVE'; REGRASP='REGRASP'; WRIST_SCAN='WRIST_SCAN'
    CONTINUE='CONTINUE'; DONE='DONE'; UNRECOVERABLE='UNRECOVERABLE'


@dataclass(frozen=True)
class Event:
    code: str
    origin: str
    context: str = 'operation'
    evidence: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Decision:
    category: Category
    next_state: State
    stop_current_action: bool = False
    force_mobile: bool = False
    known: bool = True


HARD = ('DANGEROUS_LOADED_CONTACT', 'FINGER_BACK_OR_ROOT_HANDLE_LOAD',
        'DANGEROUS_NATIVE', 'HARD_FORCE_STOP_', 'EXISTING_ARM_SPEED_LIMIT',
        'PROBE_CARTESIAN_SPEED_LIMIT', 'EXISTING_ARM_EFFORT_LIMIT',
        'INVALID_TASK_TORQUE', 'CONFIRMED_UNCONTROLLED_OBJECT_FALL',
        'CONFIRMED_UNCONTROLLED_OBJECT_REBOUND', 'UNSAFE_TO_RECOVER')
INTEGRITY = ('FROZEN_', 'WRIST_HOOK_', 'WRIST_PARENT_', 'ROBOT_JACOBIAN_',
             'PHYSICS_CLOCK_', 'INVALID_SAVED_', 'ESTIMATE_PROVENANCE_',
             'MOVING_BODY_ASSOCIATION_', 'OFFICIAL_UNSPLIT_', 'OWNERSHIP_',
             'BASE_CHANGED', 'REPLAY_', 'WRIST_SESSION_HOOK_')
DATA = ('QA_', 'OBJECT_WOULD_BE_', 'SENSOR_SEGMENTATION_', 'SYNCHRONIZED_FRAME_',
        'INITIAL_RGBD_OBJECT_', 'INSUFFICIENT_CLEAN_', 'CHECKPOINT_',
        'SAME_OBSERVED_STATE_', 'SAM3_SENSOR_')
DIAGNOSTIC = ('GRASP_RELATIVE_MOTION', 'RELATIVE_GRASP_MOTION',
              'SUSTAINED_CONTACT_PLANE_DRIFT', 'SUSTAINED_RELATIVE_SLIP',
              'FINAL_TRUE_RELATIVE_SLIP', 'MODEL_INCONSISTENCY',
              'MODEL_CONFIDENCE_', 'MODEL_CONSISTENCY_', 'NORMAL')
RECOVERABLE = ('NO_', 'SCAN_', 'MOBILE_', 'CAMERA_', 'HANDLE_', 'CURRENT_',
               'OBSERVED_', 'RELEASE_', 'UNSAFE_RELEASE_OBSERVED_OBJECT_MOTION',
               'OBJECT_MOVED_DURING_RELEASE', 'PERCEPTION_', 'GRASP_',
               'APPROACH_', 'HOME_', 'RETREAT_', 'FAILED_CLOSURE_',
               'JOINT_ESCAPE_', 'LOW_JOINT_MARGIN', 'JOINT_MARGIN_WARNING',
               'WORKSPACE_', 'OPERATIONAL_CONTINUATION_', 'MODEL_FREE_',
               'RECOVERY_', 'REGRASP_', 'BILATERAL_', 'CLOSURE_',
               'SUSTAINED_CONTACT_LOSS', 'CAPTURE_HOLD_GRASP_LOST', 'TACTILE_RETENTION_',
               'SOFT_FORCE_WARNING', 'SAFE_RELEASE_', 'INITIAL_CLEAN_CAPTURE',
               'REQUIRED_KEY_STATE_CAPTURE', 'FINAL_CLEAN_CAPTURE', 'POST_GOAL_WRIST_CAPTURE',
               'CONTINUATION_REESTABLISH_GRASP', 'EXISTING_LOW_PRELOAD_FORCE_LIMIT',
               'REPOSITION_', 'BASE_ROUTE_', 'BASE_SCAN_', 'CHASSIS_',
               'SELF_COLLISION', 'DANGEROUS_GEOMETRY_COLLISION',
               'MISSING_CONTACT_DIAGNOSTIC', 'ARTICULATION_UNOBSERVABLE_',
               'UNOBSERVABLE', 'ESTIMATED_FOLLOW_TIMEOUT')


def classify(event):
    code=event.code
    preflight=event.context in ('preflight', 'candidate')
    # A collision prediction never authorizes executing that candidate.
    if code.startswith(HARD) and not preflight:
        if code.startswith('HARD_FORCE_STOP_') and event.context=='protective_release':
            return Decision(Category.HARD, State.RELEASE, True)
        return Decision(Category.HARD, State.SAFE_HOLD, True)
    if code == 'LOW_JOINT_MARGIN' and event.context == 'physical':
        return Decision(Category.HARD, State.SAFE_HOLD, True)
    if code.startswith(INTEGRITY):
        return Decision(Category.DIAGNOSTIC, State.SAFE_HOLD, True, known=False)
    if code.startswith(DATA):return Decision(Category.DATA, State.WRIST_SCAN)
    if code.startswith(DIAGNOSTIC):return Decision(Category.DIAGNOSTIC, State.CONTINUE)
    if code.startswith(RECOVERABLE) or preflight:
        mobile=any(x in code for x in ('IK','WORKSPACE','JOINT_MARGIN','ARM_PATH','NO_USEFUL_MOTION'))
        if event.context == 'scan':return Decision(Category.RECOVERABLE, State.WRIST_SCAN)
        return Decision(Category.RECOVERABLE, State.SAFE_HOLD, True, mobile)
    return Decision(Category.DIAGNOSTIC, State.SAFE_HOLD, True, known=False)


class TaskTermination(RuntimeError):
    """Only the four authorized terminal conditions, plus implementation fault."""
    pass


class ConstraintPolicy:
    def __init__(self, output, runtime=None):
        self.output=Path(output);self.runtime=runtime;self.state=State.GRASP
        self.sequence=0
    def _write(self,row):
        row.update(sequence=self.sequence,wall_s=time.time());self.sequence+=1
        if self.runtime is not None:
            row.update(t=self.runtime.time(),base=list(self.runtime.base))
        with (self.output/'constraint_events.jsonl').open('a') as stream:
            stream.write(json.dumps(row,default=str)+'\n')
    def transition(self,state,origin,event=None):
        state=State(state)
        if state in (State.DONE,State.UNRECOVERABLE):
            raise ValueError('Terminal states require explicit finish(), never event dispatch')
        previous=self.state;self.state=state
        self._write(dict(kind='transition',previous=previous,next=state,origin=origin,event=event))
    def dispatch(self,code,origin,context='operation',**evidence):
        if str(code) in ('CUTOFF_05_00','EPISODE_WALL_CLOCK_BUDGET'):
            self.finish('WALL_BUDGET_EXHAUSTED',origin=origin,trigger=str(code));raise TaskTermination('WALL_BUDGET_EXHAUSTED')
        event=Event(str(code),origin,context,evidence);decision=classify(event)
        previous=self.state;self.state=decision.next_state
        self._write(dict(kind='event',event=asdict(event),decision=asdict(decision),previous=previous,next=self.state))
        if not decision.known:
            raise TaskTermination('IMPLEMENTATION_ERROR:'+event.code)
        return decision
    def finish(self,reason,**evidence):
        if reason not in ('HARD_PHYSICAL_SAFETY','ACTUAL_MECHANICAL_LIMIT',
                          'NO_RECOVERY_AVAILABLE','WALL_BUDGET_EXHAUSTED','TARGET_REACHED'):
            raise ValueError('Unsupported object termination: '+reason)
        self.state=State.UNRECOVERABLE if reason in ('HARD_PHYSICAL_SAFETY','NO_RECOVERY_AVAILABLE') else State.DONE
        self._write(dict(kind='terminal',reason=reason,next=self.state,evidence=evidence))
        return reason
    def checkpoint(self):return {'state':self.state.value,'event_sequence':self.sequence}
