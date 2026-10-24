"""Explicit V8.1 request menu; native drive authority is never changed."""
from contextlib import contextmanager
import math
from unittest.mock import patch

BASELINE=(.5,.75,1.,1.5)
STUDY=(2.,2.5,3.)


def request(value,*,study=False,approved=()):
    allowed=BASELINE+(STUDY if study else tuple(approved))
    if type(value) not in (int,float) or not math.isfinite(value) or value not in allowed:
        raise ValueError('EXPLICIT_PRELOAD_REQUEST_MENU_REQUIRED')
    return float(value)


def for_job(job,value):
    if any(v not in STUDY for v in job.get('v81_evidence_approved_preloads',())):
        raise ValueError('UNREGISTERED_PRELOAD_EVIDENCE_POLICY')
    return request(value,study=bool(job.get('v81_preload_study')),
        approved=job.get('v81_evidence_approved_preloads',()))


def active_requests(job):
    values=BASELINE+(STUDY if job.get('v81_preload_study') else
        tuple(job.get('v81_evidence_approved_preloads',())))
    return tuple(for_job(job,value) for value in values)


def select(ops,command):
    """Same unloaded checks and original position mapping, without a 1.5 cap."""
    from interaction_runtime.prismatic_preload_v71 import preload_policy
    r=ops.r
    value=for_job(r.job,command.get('native_g1_preload_request_n',r.closure_policy['preload_n']))
    if (ops.g['recovery_state']!='UNLOADED_OBSERVATION' or r.supervisor.latched is not None
            or r.grasp_reference is not None
            or max(r.filtered())>r.policy.data['force']['unloaded_pad_n']
            or not r.supervisor.force.settled()):
        raise ValueError('PRELOAD_CHANGE_REQUIRES_VERIFIED_UNLOADED_STATE')
    # Retain the ORIGINAL finite request bound (10 N in this simulation),
    # nonlinear aperture Jacobian, drive kp/kd and measured-angle effort cap.
    r.closure_policy=preload_policy(r.closure_policy,value)
    ops.g.update(policy=r.closure_policy,v75_planned_preload=value,
        v75_declared_chain=command['planned_segments'])
    return value


@contextmanager
def menu_scope(ops):
    """Replace only the frozen menu-admission method in this isolated call.

    All inherited radial/observation/memory and whole-chain checks still run.
    The method below mirrors V75 admission, replacing its literal menu only.
    """
    if not ops.r.job.get('v81_integrated_repair'):
        yield;return
    from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75 as Owner
    from interaction_runtime.prismatic_v73 import PrismaticOperationsV73 as AcquisitionOwner
    import interaction_runtime.prismatic_memory_v75_r3 as memory
    import interaction_runtime.prismatic_operations_v75_r3 as conditioned
    allowed=active_requests(ops.r.job)
    def admitted(self,command):
        from .declaration_preflight import check
        self.current_plan_ready=False
        try:
            check(command.get('planned_segments'),self.articulation())
            select(self,command)
        except ValueError as error:
            return dict(plan=False,reason=str(error),recoverable=self.r.supervisor.latched is None)
        return super(Owner,self).request_plan(command)
    def acquisition(self,command):
        # Only V73's literal menu guard is replaced. Preserve its input record,
        # parent execution (including all V75 checks above this owner), feedback
        # and original native closure/drive mapping. No controller is copied.
        import json
        from agent_command_protocol_v12 import atomic_json
        value=for_job(self.r.job,command.get('native_g1_preload_request_n',self.r.closure_policy['preload_n']))
        if self.current_plan_ready:
            selected=json.loads((self.r.output/'visual_plan.json').read_text())
            keys=['T','T_grasp','section_fraction','insertion_m','roll_deg','pitch_deg',
                'preplan','approach','contact_geometry_audit','contact_score_surface_source']
            folder=self.r.output/'attempt_inputs';folder.mkdir(exist_ok=True)
            atomic_json(folder/('%02d.json'%len(self.g['contact_attempts'])),dict(
                selected={k:selected[k] for k in keys if k in selected},requested_preload_n=value,
                measured_start_q=self.r.actual(),observations=[r['folder'] for r in self.g['agent_observations']],GT_grasp_used=False))
        result=super(AcquisitionOwner,self).attempt(command)
        try:result['contact_inspection']=self.feedback().get('contact_feedback')
        except Exception as error:result['contact_inspection_error']=repr(error)
        return result
    # Both modules retain an imported reference to this menu. The functions
    # continue to own failure-condition fingerprints and candidate exclusion.
    with patch.object(Owner,'request_plan',admitted), \
         patch.object(memory,'LEGAL_REQUESTS',allowed), \
         patch.object(conditioned,'LEGAL_REQUESTS',allowed), \
         patch.object(AcquisitionOwner,'attempt',acquisition):
        yield


def instrument(r):
    """Passive controller-request telemetry; do not change one command byte."""
    original=r.Closure
    def closure(T,policy):
        value=original(T,policy);update=value.update;origin=[None]
        def observed(*args,**kwargs):
            result=update(*args,**kwargs)
            r.v81_closure_request=dict(mode=result['mode'],state=result['state'],
                closing_effort_n=result.get('closing_effort_n'),
                opening_m=result.get('opening_m'))
            if result['mode']=='effort':
                if origin[0] is None:origin[0]=float(r.robot.get_joint_positions()[r.controller.master])
                r.v81_closure_request['unclipped_master_rad']=(origin[0]+result['closing_effort_n']*
                    abs(r.controller.geometry.width_derivative(origin[0]))/r.c['gripper_kp_nm_rad'])
            return result
        value.update=observed
        return value
    r.Closure=closure
    previous=r.record.coordinate_fields
    def fields(row):
        import numpy as np
        values=previous(row);cmd=getattr(r,'v81_closure_request',{})
        values.update(closure_internal_effort_request_n=np.float32(
            cmd.get('closing_effort_n') if cmd.get('closing_effort_n') is not None else np.nan),
            native_master_measured_rad=np.float32(row['q'][r.controller.master]),
            native_master_target_rad=np.float32(r.controller.geometry.angle(r.jaw_width)),
            mapping_unclipped_master_rad=np.float32(cmd.get('unclipped_master_rad',np.nan)),
            native_master_cap_nm=np.float32(r.controller.caps[r.controller.master]),
            native_position_mapping_limit=np.uint8(
                r.controller.geometry.angle(r.jaw_width)<=0 or r.controller.geometry.angle(r.jaw_width)>=.85))
        return values
    r.record.coordinate_fields=fields
