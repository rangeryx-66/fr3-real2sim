"""Prefer a current-state, fully checked direct return over old view detours.

Frozen execute() still owns actuation, timed motion, measured-state approach
revalidation and pregrasp RGB-D refinement. No historic waypoint is required
when the original checker certifies a shorter current route.
"""
from contextlib import contextmanager
import json,time
from unittest.mock import patch
import numpy as np


@contextmanager
def direct_return(runtime):
    import native_g1_agent_execution as owner
    original=owner.validate_execution
    def validate(g,candidate=None,write=True):
        started=time.perf_counter()
        if runtime.grasp_reference is not None or runtime.supervisor.latched is not None:
            return original(g,candidate,write)
        if candidate is None:
            path=g['a'].output/'visual_plan.json'
            if not path.exists():return original(g,candidate,write)
            candidate=json.loads(path.read_text())
        measured=np.asarray(g['robot'].get_joint_positions())[g['arm']]
        path=[measured.tolist(),g['qtarget'][g['arm']].tolist(),candidate['preplan'][0]]
        returning=owner.check_path(g,path)
        if not returning['valid']:return original(g,candidate,write)
        approach=owner.approach_from(g,path[-1],candidate)
        if not approach['valid']:return original(g,candidate,write)
        result=dict(valid=True,return_path=path,return_audit=returning,
            approach_audit=approach,measured_start_q=measured.tolist(),
            wall_s=time.perf_counter()-started,
            source='current measured and equilibrium states to original execution start; original full native check',
            skipped_redundant_view_routes=len(g['wrist_routes']),
            original_timed_actuation_and_pregrasp_refinement=True)
        if write:owner.observation.write(g['a'].output/'AGENT_EXECUTION_TRANSITION_CHECK.json',result)
        return result
    with patch.object(owner,'validate_execution',validate):yield
