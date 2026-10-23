"""Explicit recovery-debug resources. No controller or protection overrides."""
from datetime import datetime,timedelta,timezone
PROFILE='recovery_gate_debug_v1'
VALUES=dict(episode_wall_budget_s=3600.,episode_sim_budget_s=1800.,
    agent_wait_timeout_s=600.,safe_exit_wall_reserve_s=600.,safe_exit_sim_reserve_s=120.)
FULL_RANGE_PROFILE='full_range_integration_debug_v1'
PROFILES={PROFILE:VALUES,FULL_RANGE_PROFILE:dict(episode_wall_budget_s=5400.,
    episode_sim_budget_s=2700.,agent_wait_timeout_s=600.,
    safe_exit_wall_reserve_s=600.,safe_exit_sim_reserve_s=120.)}


def apply(spec,job):
    profile=spec.get('debug_budget_profile')
    if profile is None:return
    if profile not in PROFILES or spec.get('phase')!='engineering':
        raise ValueError('EXPLICIT_ENGINEERING_DEBUG_BUDGET_REQUIRED')
    values=PROFILES[profile]
    for key,value in [('wall_budget_s',values['episode_wall_budget_s']),
                      ('sim_budget_s',values['episode_sim_budget_s']),
                      ('agent_wait_s',values['agent_wait_timeout_s'])]:
        if spec.get(key)!=value:raise ValueError('DEBUG_BUDGET_CONFLICT:'+key)
    job.update(values,v81_debug_budget_profile=profile)
    job['deadline_shanghai']=(datetime.now(timezone.utc)+timedelta(seconds=values['episode_wall_budget_s'])).isoformat()


def window(r):
    resources=r.episode_budget.remaining(r.tick*r.dt)
    return min(float(r.job['agent_wait_timeout_s']),resources['wall_s']),resources
