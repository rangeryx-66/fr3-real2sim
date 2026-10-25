"""Prune only exact final utilities dominated by a fully certified winner.

Same strict winner comparison as frozen plan_shared_door_skill.plan. Never
prune a better utility, before a valid whole-chain winner, or a live execution
check. Mirrors the existing prismatic optimization without changing scores.
"""
from contextlib import contextmanager
from copy import deepcopy
from unittest.mock import patch
from agent_command_protocol_v12 import atomic_json


def dominated_checker(original,audit,pool=None):
    def checked(owner,candidate):
        utility=candidate['shared_selection_utility']
        best=getattr(owner,'_abc_certified_utility',None)
        if best is not None and utility<=best:
            record=dict(valid=False,reason='DOMINATED_BY_FULLY_NATIVE_CERTIFIED_UTILITY',
                pruned_not_collision_failure=True,native_checks_not_claimed=True,
                shared_selection_utility=utility,verified_winner_utility=best)
            if pool is not None:pool.append(deepcopy(candidate))
            audit.append(record);return record
        result=original(owner,candidate)
        audit.append(dict(utility=utility,fully_checked=True,valid=result.get('valid')))
        if result.get('valid'):owner._abc_certified_utility=utility
        return result
    return checked


@contextmanager
def schedule(ops):
    if ops.articulation().joint_type!='revolute':
        yield;return  # original prismatic dominated-check mechanism remains
    import agentic_interaction_v81.replanning as planning
    import native_g1_execution_candidates as native
    original=planning.candidate_gate;audit=[]
    @contextmanager
    def gate(actions,horizon=None,pool=None):
        # Install AFTER the whole-chain gate, so only its complete success can
        # establish a winner. A base-only valid but chain-invalid pose cannot.
        with original(actions,horizon,pool) as coverage:
            full=native.CandidateCheck.__call__
            with patch.object(native.CandidateCheck,'__call__',dominated_checker(full,audit,pool)):
                yield coverage
    try:
        with patch.object(planning,'candidate_gate',gate):yield
    finally:
        atomic_json(ops.r.output/'ABC_PLAN_SCHEDULE.json',dict(events=audit,
            scoring_and_strict_winner_comparison_unchanged=True,
            live_execution_checks_pruned=False))
