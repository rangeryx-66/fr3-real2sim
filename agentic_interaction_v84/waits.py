"""Small event ledger: measured waits, explicit dependencies, no timer changes."""
import time
from contextlib import contextmanager
from agent_command_protocol_v12 import atomic_json


@contextmanager
def wait(runtime,reason,dependency,**extra):
    begin=time.time();sim=runtime.tick*runtime.dt
    try:yield
    finally:
        rows=getattr(runtime,'abc_waits',[])
        rows.append(dict(start_unix_s=begin,end_unix_s=time.time(),
            wall_s=time.time()-begin,start_sim_s=sim,end_sim_s=runtime.tick*runtime.dt,
            reason=reason,dependency=dependency,**extra))
        runtime.abc_waits=rows
        atomic_json(runtime.output/'ABC_WAITS.json',dict(waits=rows))
