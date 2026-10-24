"""Versioned integrated engineering activation; frozen physical owners intact."""
import math
from agent_command_protocol_v12 import atomic_json

PROFILE='v81_force_replan_validation_v1'
ACTIVE_PROFILE='v81_active_recovery_v1'
PRISMATIC_ACTIVE_PROFILE='v83_prismatic_active_recovery_v1'
REVOLUTE_VALIDATION_PROFILE='v83_revolute_model_validation_v1'
from .orchestration_extension import PROFILE as ORCHESTRATION_PROFILE, AUTH as ORCHESTRATION_AUTH


def apply(spec,job):
    profile=spec.get('integration_profile')
    if ORCHESTRATION_AUTH in spec and profile!=ORCHESTRATION_PROFILE:
        raise ValueError('ORCHESTRATION_AUTHORIZATION_REQUIRES_OPT_IN_PROFILE')
    if profile is None:
        if spec.get('preload_study'):raise ValueError('PRELOAD_STUDY_REQUIRES_INTEGRATED_PROFILE')
        return
    if profile not in (PROFILE,ACTIVE_PROFILE,PRISMATIC_ACTIVE_PROFILE,REVOLUTE_VALIDATION_PROFILE,ORCHESTRATION_PROFILE) or spec.get('phase')!='engineering':
        raise ValueError('EXPLICIT_INTEGRATED_ENGINEERING_PROFILE_REQUIRED')
    if 'preload_study' in spec and type(spec['preload_study']) is not bool:
        raise ValueError('BOOLEAN_PRELOAD_STUDY_REQUIRED')
    job.update(v81_integrated_repair=True,v81_preload_study=bool(spec.get('preload_study',False)))
    if profile in (ACTIVE_PROFILE,REVOLUTE_VALIDATION_PROFILE,ORCHESTRATION_PROFILE):
        job.update(v81_active_recovery=True,v81_minimum_reverse_native=math.radians(10.))
    if profile in (REVOLUTE_VALIDATION_PROFILE,ORCHESTRATION_PROFILE):
        job['v83_continuous_video_hz']=5
    if profile==ORCHESTRATION_PROFILE:
        from .orchestration_extension import policy, FLAG
        job.update({FLAG:True, ORCHESTRATION_AUTH:policy(spec)})
    if profile==PRISMATIC_ACTIVE_PROFILE:
        job.update(v81_active_recovery=True,v81_minimum_reverse_native=.04,
                   v83_continuous_video_hz=5)
    from pathlib import Path
    import json
    policy=json.loads((Path(__file__).parent/'PRELOAD_EVIDENCE_POLICY.json').read_text())
    job['v81_evidence_approved_preloads']=policy['approved_agent_requests_n']
    if job:
        joint=job.get('articulation_v7',{}).get('joint_type')
        if profile==ACTIVE_PROFILE and joint and joint!='revolute':
            raise ValueError('ACTIVE_RECOVERY_PROFILE_CURRENTLY_REVOLUTE_ONLY')
        if profile in (REVOLUTE_VALIDATION_PROFILE,ORCHESTRATION_PROFILE) and joint and joint!='revolute':
            raise ValueError('V83_REVOLUTE_VALIDATION_PROFILE_REQUIRES_REVOLUTE')
        if profile==PRISMATIC_ACTIVE_PROFILE and joint and joint!='prismatic':
            raise ValueError('V83_PRISMATIC_PROFILE_REQUIRES_PRISMATIC')
        if joint:job['v81_task_goal_native']=(.18 if profile==PRISMATIC_ACTIVE_PROFILE
            else math.pi/2 if joint=='revolute' else .15)


def progress(r,row):
    if not r.job.get('v81_integrated_repair'):return
    q=float(row['joint_coordinate']);goal=r.job['v81_task_goal_native']
    previous=getattr(r,'v81_task_progress',None)
    first_goal=(previous or {}).get('first_goal_record')
    if first_goal is None and q>=goal:
        first_goal=getattr(getattr(r,'record',None),'count',None)
    r.v81_task_progress=dict(requested_final_target_native=goal,
        maximum_measured_native=max(q,previous['maximum_measured_native']) if previous else q,
        minimum_measured_native=min(q,previous['minimum_measured_native']) if previous else q,
        current_measured_native=q,unit='rad' if r.job['articulation_v7']['joint_type']=='revolute' else 'm',
        global_goal_erased_by_shorter_action=False,first_goal_record=first_goal)


def packet(ops):
    from .preload_extension import BASELINE,STUDY
    from .validation_queue import requests,model_state
    progress=getattr(ops.r,'v81_task_progress',{})
    atomic_json(ops.r.output/'TASK_PROGRESS.json',progress)
    return dict(task_progress=progress,pending_validation=requests(ops),numeric_model_state=model_state(ops),
        preload_menu=dict(requests_n=list(BASELINE)+(list(STUDY) if ops.r.job.get('v81_preload_study')
            else list(ops.r.job.get('v81_evidence_approved_preloads',()))),
            controlled_study=bool(ops.r.job.get('v81_preload_study')),
            request_is_not_measured_force=True,actuator_authority_unchanged=True),
        measured_phase_costs=getattr(ops,'_v81_phase_costs',{}),
        active_budget_model='Measured acquisition/release and useful motion costs; previous whole-block cost is historical, not a universal next-action cost',
        priority=('Keep pending VALIDATION and complete it at the next suitable safe boundary when a qualified frozen model is ready. '
            'Numerical delay/failure alone must not block meaningful manipulation or qualified FIT; recover from the current state.'
            if ops.r.job.get('v81_active_recovery') else
            'Qualified bidirectional FIT then complete pending independent VALIDATION, before optional range extension. Recover in this scene from ordinary rejections.'))
