"""Versioned integrated engineering activation; frozen physical owners intact."""
import math
from agent_command_protocol_v12 import atomic_json

PROFILE='v81_force_replan_validation_v1'


def apply(spec,job):
    profile=spec.get('integration_profile')
    if profile is None:
        if spec.get('preload_study'):raise ValueError('PRELOAD_STUDY_REQUIRES_INTEGRATED_PROFILE')
        return
    if profile!=PROFILE or spec.get('phase')!='engineering':
        raise ValueError('EXPLICIT_INTEGRATED_ENGINEERING_PROFILE_REQUIRED')
    if 'preload_study' in spec and type(spec['preload_study']) is not bool:
        raise ValueError('BOOLEAN_PRELOAD_STUDY_REQUIRED')
    job.update(v81_integrated_repair=True,v81_preload_study=bool(spec.get('preload_study',False)))
    from pathlib import Path
    import json
    policy=json.loads((Path(__file__).parent/'PRELOAD_EVIDENCE_POLICY.json').read_text())
    job['v81_evidence_approved_preloads']=policy['approved_agent_requests_n']
    if job:
        joint=job.get('articulation_v7',{}).get('joint_type')
        if joint:job['v81_task_goal_native']=math.pi/2 if joint=='revolute' else .15


def progress(r,row):
    if not r.job.get('v81_integrated_repair'):return
    q=float(row['joint_coordinate']);goal=r.job['v81_task_goal_native']
    previous=getattr(r,'v81_task_progress',None)
    r.v81_task_progress=dict(requested_final_target_native=goal,
        maximum_measured_native=max(q,previous['maximum_measured_native']) if previous else q,
        minimum_measured_native=min(q,previous['minimum_measured_native']) if previous else q,
        current_measured_native=q,unit='rad' if r.job['articulation_v7']['joint_type']=='revolute' else 'm',
        global_goal_erased_by_shorter_action=False)


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
        priority='Qualified bidirectional FIT then complete pending independent VALIDATION, before optional range extension. Recover in this scene from ordinary rejections.')
