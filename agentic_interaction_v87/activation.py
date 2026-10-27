"""Explicit engineering registration; no activation on historical baselines."""
import math


def activate(spec,job,existing,base_profile):
    shadow=dict(spec,integration_profile=base_profile)
    shadow.pop('controlled_interruption_fixture',None)
    shadow.pop('graspgenx',None)
    shadow.pop('gpu_capacity_evidence',None)
    shadow.pop('constraint_policy',None)
    shadow.pop('persistent_goal',None)
    shadow.pop('coordination_policy',None)
    shadow.pop('native_assets',None)
    if spec.get('persistent_goal') is not None or spec.get('coordination_policy') is not None:
        from agentic_interaction_v81.integration import ABC_PROFILE
        shadow['integration_profile']=ABC_PROFILE
    shadow.pop('staged_exit',None)
    shadow.pop('staged_exit_fixture',None)
    existing(shadow,job)
    from .persistent_goal import activate as persistent
    persistent(spec,job)
    fixture=spec.get('controlled_interruption_fixture')
    if fixture is not None:
        if (not isinstance(fixture,dict) or set(fixture)!={'kind','progress_native'}
                or fixture['kind']!='recoverable_execution_interrupt'
                or type(fixture['progress_native'])not in (int,float)
                or not math.isfinite(fixture['progress_native'])or fixture['progress_native']<=0):
            raise ValueError('FINITE_EXPLICIT_V87_ENGINEERING_FIXTURE_REQUIRED')
        job['v87_controlled_interruption_fixture']=dict(fixture)
    job['v87_continuous_recovery']=True
    if 'coordination_policy' in spec:
        from agentic_interaction_v88 import POLICY
        if (spec['coordination_policy'] != POLICY or spec.get('phase') != 'engineering'
                or spec.get('staged_exit') is not True):
            raise ValueError('COORDINATION_REQUIRES_EXPLICIT_ENGINEERING_STAGED_EXIT')
        job.update(v88_coordination_policy=POLICY, v86_object_state_telemetry=True)
    if 'native_assets' in spec:
        if not job.get('v88_coordination_policy'):
            raise ValueError('NATIVE_ASSET_MIGRATION_REQUIRES_COORDINATED_ENGINEERING')
        from agentic_interaction_v88.assets import verify
        verify(spec['native_assets'])
        job['v88_native_assets'] = dict(spec['native_assets'])
    if 'constraint_policy' in spec:
        from .constraint_feedback import POLICY
        if (spec['constraint_policy'] != POLICY or spec.get('phase') != 'engineering'
                or spec.get('staged_exit') is not True):
            raise ValueError('CLASSIFIED_CONSTRAINT_POLICY_REQUIRES_ENGINEERING_STAGED_EXIT')
        job['v87_constraint_policy'] = POLICY
    if 'staged_exit' in spec:
        if spec['staged_exit'] is not True:raise ValueError('EXPLICIT_STAGED_EXIT_OPT_IN_REQUIRED')
        job['v87_staged_exit']=True
    if 'staged_exit_fixture' in spec:
        fixture=spec['staged_exit_fixture']
        if (not job.get('v87_staged_exit') or spec.get('phase')!='engineering'
                or fixture!={'kind':'reject_preferred_parking_route_once_after_task_goal'}):
            raise ValueError('EXPLICIT_ENGINEERING_STAGED_EXIT_FIXTURE_REQUIRED')
        job['v87_staged_exit_fixture']=dict(fixture)
    registration=spec.get('gpu_capacity_evidence')
    if registration is not None:
        from pathlib import Path
        if (not isinstance(registration,dict)or set(registration)!={'path','sha256'}
                or not isinstance(registration['path'],str)or not Path(registration['path']).is_absolute()
                or not isinstance(registration['sha256'],str)or len(registration['sha256'])!=64):
            raise ValueError('EXPLICIT_MEASURED_SPLIT_GPU_EVIDENCE_REQUIRED')
        job['v87_gpu_capacity_evidence']=dict(registration)
    config=spec.get('graspgenx')
    if config is not None:
        from pathlib import Path
        if (not isinstance(config,dict)or set(config)!={'python','source','gpu'}
                or type(config['gpu'])is not int or config['gpu']<0
                or not all(isinstance(config[k],str)and Path(config[k]).is_absolute()for k in ['python','source'])):
            raise ValueError('EXPLICIT_NATIVE_G1_GRASPGENX_RESOURCES_REQUIRED')
        job['v87_graspgenx']=dict(config)

