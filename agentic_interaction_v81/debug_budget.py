"""Explicit recovery-debug resources. No controller or protection overrides."""
from datetime import datetime,timedelta,timezone
PROFILE='recovery_gate_debug_v1'
VALUES=dict(episode_wall_budget_s=3600.,episode_sim_budget_s=1800.,
    agent_wait_timeout_s=600.,safe_exit_wall_reserve_s=600.,safe_exit_sim_reserve_s=120.)


def apply(spec,job):
    profile=spec.get('debug_budget_profile')
    if profile is None:return
    if profile!=PROFILE or spec.get('phase')!='engineering':
        raise ValueError('EXPLICIT_ENGINEERING_DEBUG_BUDGET_REQUIRED')
    for key,value in [('wall_budget_s',3600),('sim_budget_s',1800),('agent_wait_s',600)]:
        if spec.get(key)!=value:raise ValueError('DEBUG_BUDGET_CONFLICT:'+key)
    job.update(VALUES,v81_debug_budget_profile=PROFILE)
    job['deadline_shanghai']=(datetime.now(timezone.utc)+timedelta(seconds=3600)).isoformat()


def window(r):
    resources=r.episode_budget.remaining(r.tick*r.dt)
    return min(float(r.job['agent_wait_timeout_s']),resources['wall_s']),resources
