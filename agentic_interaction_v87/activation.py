"""Explicit engineering registration; no activation on historical baselines."""
import math


def activate(spec,job,existing,base_profile):
    shadow=dict(spec,integration_profile=base_profile)
    shadow.pop('controlled_interruption_fixture',None)
    existing(shadow,job)
    fixture=spec.get('controlled_interruption_fixture')
    if fixture is not None:
        if (not isinstance(fixture,dict) or set(fixture)!={'kind','progress_native'}
                or fixture['kind']!='recoverable_execution_interrupt'
                or type(fixture['progress_native'])not in (int,float)
                or not math.isfinite(fixture['progress_native'])or fixture['progress_native']<=0):
            raise ValueError('FINITE_EXPLICIT_V87_ENGINEERING_FIXTURE_REQUIRED')
        job['v87_controlled_interruption_fixture']=dict(fixture)
    job['v87_continuous_recovery']=True

