"""Fixed four-hour, goal-driven ownership for the registered two-object trial.

Only action/search deadlines renew. The absolute experiment deadline never
renews. This grants no new physical motion, clearance, load or reset authority.
"""
import hashlib,json,math,time
from datetime import datetime,timezone
from pathlib import Path

FLAG='v87_persistent_goal'
KIND='fixed_two_unseen_four_hour_v1'
KNOWN_KIND='fixed_known_four_hour_v88'


def enabled(r):return bool(getattr(r,'job',{}).get(FLAG))


def activate(spec,job):
    declaration=spec.get('persistent_goal')
    if declaration is None:return
    if (spec.get('phase')!='engineering' or spec.get('staged_exit') is not True
            or not isinstance(declaration,dict)
            or set(declaration)!={'kind','registration','sha256','formal_start_unix_s'}
            or declaration['kind'] not in (KIND,KNOWN_KIND) or 'orchestration_authorization'in spec
            or spec.get('wall_budget_s')!=14400 or spec.get('sim_budget_s')!=14400):
        raise ValueError('EXPLICIT_FIXED_FOUR_HOUR_PERSISTENT_REGISTRATION_REQUIRED')
    source=Path(declaration['registration'])
    if not source.is_absolute() or hashlib.sha256(source.read_bytes()).hexdigest()!=declaration['sha256']:
        raise ValueError('PERSISTENT_REGISTRATION_COMMITMENT_MISMATCH')
    registered=json.loads(source.read_text());asset=spec['scene'].removeprefix('asset_')
    cases=[c for c in registered['fixed_cases']if c['asset_id']==asset]
    start=declaration['formal_start_unix_s']
    if (len(cases)!=1 or cases[0]['wall_budget_s']!=14400 or type(start)not in (int,float)
            or not math.isfinite(start) or start<=0 or start>time.time()+1.):
        raise ValueError('FROZEN_ASSET_AND_REAL_FORMAL_START_REQUIRED')
    deadline=float(start)+14400.
    job.update({FLAG:dict(declaration,deadline_unix_s=deadline,asset_id=asset),
        'episode_wall_budget_s':14400.,'episode_sim_budget_s':14400.,
        'deadline_shanghai':datetime.fromtimestamp(deadline,timezone.utc).isoformat(),
        'v86_object_state_telemetry':True})


def remaining_wall(r):
    return max(0.,r.job[FLAG]['deadline_unix_s']-time.time())


def finish_assessment(ops,completion):
    """No strategy-exhaustion/cost/retry-count exemption in this experiment."""
    r=ops.r
    if r.supervisor.latched is not None:
        return dict(allowed=True,reason='INDEPENDENT_HARD_SAFETY_BLOCK',task_complete=False)
    # External owner records a direct human cancellation separately from the
    # Agent's ordinary failed-plan finish request. Do not use STOP_AND_EXPORT:
    # that marker interrupts runtime.step itself, including guarded parking.
    request=r.output/'USER_REQUESTED_STOP.json'
    if request.is_file():
        try:
            user=json.loads(request.read_text())
            valid=(isinstance(user,dict) and user.get('schema')=='explicit-user-episode-stop/v1'
                and user.get('episode_id')==r.job.get('episode_id')
                and user.get('authorization')=='direct_user_message'
                and isinstance(user.get('user_request'),str) and bool(user['user_request'].strip())
                and r.job[FLAG].get('formal_start_unix_s',math.inf)<=user['requested_unix_s']<=time.time())
        except (OSError,ValueError,KeyError,TypeError):valid=False
        if valid:return dict(allowed=True,reason='USER_EXPLICIT_CANCEL',task_complete=False,user_request=str(request))
    if all(completion[k]for k in ('actual_goal_reached','meaningful_reverse','complete_independent_validation')):
        return dict(allowed=True,reason='MEASURED_TASK_LIFECYCLE_COMPLETE',task_complete=True,completion=completion)
    if remaining_wall(r)<=0:
        return dict(allowed=True,reason='REGISTERED_FOUR_HOUR_DEADLINE',task_complete=False)
    resources=r.episode_budget.remaining(r.tick*r.dt)
    if resources['wall_s']<=0 or resources['sim_s']<=0:
        return dict(allowed=True,reason='PROTECTED_EXIT_RESOURCE_BOUNDARY',task_complete=False,resources=resources)
    return dict(allowed=False,status='CONTINUATION_REQUIRED',recoverable=True,
        reason='LOCAL_FAILURE_DOES_NOT_END_REGISTERED_GOAL',completion=completion,
        remaining_total_wall_s=remaining_wall(r),original_goal_native=r.job['v81_task_goal_native'])


def renew_decision(r,began,wait,sequence):
    """A reasoning subdeadline asks for another strategy; it cannot close World.

    This records an idle/late-Agent interval honestly, never as useful search.
    The external Agent must continue observations, search and actual actions.
    """
    from agent_command_protocol_v12 import atomic_json
    resources=r.episode_budget.remaining(r.tick*r.dt)
    remaining=min(remaining_wall(r),resources['wall_s'])
    if resources['sim_s']<=0:remaining=0.
    if remaining<=0:return None
    extension=min(float(r.job['agent_wait_timeout_s']),remaining)
    record=dict(sequence=sequence,reason='AGENT_SUBDEADLINE_STRATEGY_SWITCH_REQUIRED',
        renewed_wait_s=extension,remaining_total_wall_s=remaining,
        absolute_deadline_unix_s=r.job[FLAG]['deadline_unix_s'],
        world_recreated=False,useful_search_claimed=False)
    r.event('PERSISTENT_AGENT_SUBDEADLINE',**record)
    atomic_json(r.output/'PERSISTENT_STRATEGY_SWITCH_REQUIRED.json',record)
    return time.perf_counter()-began+extension,time.time()+extension
