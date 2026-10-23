"""One authoritative live state, distinct from historical WAITING evidence."""
import time
from agent_command_protocol_v12 import atomic_json


def write(r,sequence,state,**extra):
    value=dict(episode_id=r.job['episode_id'],sequence=sequence,state=state,
        updated_unix_s=time.time(),station_generation=r.base_generation,
        observation_generation=len(r.g.get('observation_generations',[])),
        remaining_work=r.episode_budget.remaining(r.tick*r.dt),
        safe_exit_wall_reserve_s=r.job['safe_exit_wall_reserve_s'],**extra)
    atomic_json(r.output/'AGENT_SESSION_STATUS.json',value)
    return value
