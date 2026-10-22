"""Task-conditioned candidate memory, never a saved pose or safety override.

A completed forward trajectory is not disproved by a later reverse failure.
Reconsider only that already demonstrated direction AND measured interval, at
the same legal preload, after verified exit and fresh observation. Every new
pose still comes from the current observed candidate pool and native checks.
"""
from contextlib import contextmanager
import hashlib
import json
import math
from unittest.mock import patch


def intervals(actions, start):
    result = []
    for action in actions:
        target = action.get('target_q')
        if (action.get('coordinate_unit') != 'm' or type(target) not in (int, float)
                or not math.isfinite(target) or not math.isfinite(start) or target == start):
            return []
        result.append(dict(direction=1 if target > start else -1,
                           low=min(start, target), high=max(start, target)))
        start = target
    return result


def evidence(attempt, recorded):
    park = (attempt.get('safe_exit') or {}).get('park') or {}
    if (not (attempt.get('acquisition') or {}).get('acquired')
            or not (attempt.get('safe_exit') or {}).get('safe')
            or not park.get('reached') or not park.get('unloaded')
            or attempt.get('error') or attempt.get('operation_status')):
        return None
    start = attempt['acquisition'].get('joint_coordinate')
    if type(start) not in (int, float) or not math.isfinite(start):
        return None
    rows = [s for s in recorded
            if attempt['first_record'] <= s['first_step'] < attempt['last_record']]
    successes, failures, sources = [], set(), []
    for segment in rows:
        result = segment.get('result') or {}
        action = segment.get('declared_action') or {}
        task = intervals([action], start)
        end = result.get('joint_coordinate')
        if (not segment.get('completed') or not task
                or type(end) not in (int, float) or not math.isfinite(end)):
            return None
        direction = task[0]['direction']
        status = result.get('status')
        if (status == 'TARGET_REACHED' and result.get('requested_target_reached')
                and result.get('physical_transmission_valid')
                and direction * (end - start) > 0):
            successes.append(dict(direction=direction, low=min(start, end), high=max(start, end)))
        elif (status == 'TRANSMISSION_FAILURE'
              or (status == 'SAFE_PARTIAL_PROGRESS'
                  and result.get('reason') == 'PREDICTED_LOAD_MARGIN_REQUIRES_UNLOAD')):
            failures.add(direction)
        else:
            return None  # Unknown/acquisition/budget/hard faults stay excluded.
        sources.append(dict(segment_id=segment['id'], role=segment['role'],
                            status=status, direction=direction, start=start, end=end))
        start = end
    if not failures:
        return None
    return dict(successes=successes, failed_directions=failures, sources=sources)


def conditioned(attempts, output, requested, *, original, actions, start, fresh, recorded):
    blocked, old = original(attempts, output, requested)
    tasks = intervals(actions, start)
    reconsidered = []
    remaining = set(blocked)
    if fresh and tasks:
        for family in sorted(blocked):
            conditions = [r for r in old['failed_conditions']
                          if tuple(r['family']) == family and not r['reconsideration_eligible']]
            proofs = []
            for condition in conditions:
                if condition['prior_requested_preload_n'] != requested:
                    break
                proof = evidence(attempts[condition['attempt_index']], recorded)
                if proof is None:
                    break
                proofs.append(proof)
            else:
                if not proofs:
                    continue
                failed = set().union(*(p['failed_directions'] for p in proofs))
                success = [s for p in proofs for s in p['successes']]
                supported = all(t['direction'] not in failed and any(
                    s['direction'] == t['direction'] and s['low'] <= t['low']
                    and t['high'] <= s['high'] for s in success) for t in tasks)
                if supported:
                    remaining.remove(family)
                    reconsidered.append(dict(family=list(family),
                        requested_intervals=tasks,
                        prior_measured_results=[s for p in proofs for s in p['sources']],
                        reason='DIFFERENT_DIRECTION_WITHIN_ACTUALLY_COMPLETED_SUPPORT'))
    memory = dict(old)
    memory.pop('conditions_sha256')
    memory.update(legacy_conditions_sha256=old['conditions_sha256'],
        blocked_families=[list(f) for f in sorted(remaining)],
        task_conditioned_reconsideration=reconsidered,
        fresh_observation_after_contact=bool(fresh), requested_task_intervals=tasks,
        policy='same episode, same preload, demonstrated direction/interval only; fresh poses and full native checks still required')
    memory['conditions_sha256'] = hashlib.sha256(
        json.dumps(memory, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return remaining, memory


@contextmanager
def task_memory(ops, command):
    """Explicit prismatic-only seam; restore both frozen owners on exceptions."""
    import interaction_runtime.prismatic_operations_v75_r3 as operations
    import interaction_runtime.prismatic_memory_v75_r3 as owner
    original = owner.recovery_memory
    start = float(ops.r.actual())
    recorded = tuple(ops.r.record.segments)
    def memory(attempts, output, requested):
        fresh = any(v.get('accepted_for_geometry') and
                    v['world_sim_s'] > getattr(ops, '_last_contact_finished_world_s', float('inf'))
                    for v in ops.g['agent_observations'])
        return conditioned(attempts, output, requested, original=original,
                           actions=command['planned_segments'], start=start,
                           fresh=fresh, recorded=recorded)
    with patch.object(operations, 'recovery_memory', memory), patch.object(owner, 'recovery_memory', memory):
        yield
