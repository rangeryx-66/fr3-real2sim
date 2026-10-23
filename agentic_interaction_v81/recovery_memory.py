"""Task-conditioned candidate memory, never a saved pose or safety override.

A failed long excursion is not a permanent ban on every task for that family.
Default reconsideration stays inside completed measured support. An explicit
Agent request may instead explore an unfailed direction, or a shorter entire
excursion bounded by a recorded healthy prefix. Neither is declared successful
in advance. Fresh geometry, verified exit and original native checks remain.
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


def healthy_excursion(result):
    """Conservative observed prefix; never relabel the failed whole action."""
    startup = result.get('prismatic_startup') or {}
    if (not result.get('fit_prefix_transmission_valid')
            or startup.get('first_suspect_s') is None):
        return None
    try:
        windows = startup['observed_windows']
        if not windows or any(not all(math.isfinite(float(w[k])) for k in
                ['duration_s','t_s','drawer_progress_m']) for w in windows):
            return None
        reserve_s = max(float(w['duration_s']) for w in windows)
        cutoff = min(float(result['fit_valid_until_s']), float(startup['first_suspect_s']))-reserve_s
        margin = float(startup['contact_reference_allowance_m'])+float(startup['maximum_measured_tracking_offset_m'])
        prior = [w for w in windows if float(w['t_s']) <= cutoff]
        resolved = [w for w in prior if w.get('motion_resolved') and
                    w.get('next_phase') == 'OPEN_CONTINUE' and w.get('transmission_ratio_actual_TCP',0) >= .1]
        distance = sum(float(w['drawer_progress_m']) for w in prior)-margin
        if (len(resolved) < 2 or not all(math.isfinite(x) for x in (reserve_s,cutoff,margin,distance))
                or reserve_s <= 0 or margin <= 0 or distance <= 0):
            return None
    except (KeyError, TypeError, ValueError):
        return None
    return dict(maximum_requested_excursion_m=distance, prefix_end_s=cutoff,
                excluded_final_window_s=reserve_s, position_margin_m=margin,
                source='recorded transmission-qualified prefix minus existing lead and measured servo offset',
                whole_action_still_failed=True, new_task_success_unproven=True)


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
    successes, failures, sources, healthy = [], set(), [], []
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
            prefix = healthy_excursion(result)
            if prefix is not None:
                healthy.append(dict(direction=direction, segment_id=segment['id'], **prefix))
        else:
            return None  # Unknown/acquisition/budget/hard faults stay excluded.
        sources.append(dict(segment_id=segment['id'], role=segment['role'],
                            status=status, direction=direction, start=start, end=end))
        start = end
    if not failures:
        return None
    return dict(successes=successes, failed_directions=failures, sources=sources, healthy=healthy)


def conditioned(attempts, output, requested, *, original, actions, start, fresh, recorded, exploratory=False):
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
                reason = 'DIFFERENT_DIRECTION_WITHIN_ACTUALLY_COMPLETED_SUPPORT' if supported else None
                limits = []
                directions = {t['direction'] for t in tasks}
                # Exploration is explicitly requested by the Agent, never a
                # hidden automatic acceptance or a change to physical guards.
                if not supported and exploratory and len(directions) == 1:
                    direction = next(iter(directions))
                    if direction not in failed:
                        supported = True
                        reason = 'EXPLICIT_UNFAILED_DIRECTION_EXPLORATION_NOT_DEMONSTRATED_SUPPORT'
                    else:
                        relevant = [p for p in proofs if direction in p['failed_directions']]
                        for proof in relevant:
                            bounds = [h for h in proof['healthy'] if h['direction'] == direction]
                            if not bounds:break
                            limits.extend(bounds)
                        else:
                            # Bound the ENTIRE acquisition, not each action:
                            # splitting a long pull cannot reset this allowance.
                            travel = sum(t['high']-t['low'] for t in tasks)
                            if limits and travel <= min(h['maximum_requested_excursion_m'] for h in limits):
                                supported = True
                                reason = 'EXPLICIT_SHORTER_TOTAL_EXCURSION_FROM_RECORDED_HEALTHY_PREFIX'
                if supported:
                    remaining.remove(family)
                    reconsidered.append(dict(family=list(family),
                        requested_intervals=tasks,
                        prior_measured_results=[s for p in proofs for s in p['sources']],
                        healthy_excursion_evidence=limits, reason=reason,
                        exploratory_not_a_success_claim=reason != 'DIFFERENT_DIRECTION_WITHIN_ACTUALLY_COMPLETED_SUPPORT'))
    memory = dict(old)
    memory.pop('conditions_sha256')
    memory.update(legacy_conditions_sha256=old['conditions_sha256'],
        blocked_families=[list(f) for f in sorted(remaining)],
        task_conditioned_reconsideration=reconsidered,
        fresh_observation_after_contact=bool(fresh), requested_task_intervals=tasks,
        explicit_Agent_exploration=bool(exploratory),
        policy='same episode/preload and verified exit; completed support by default; explicit changed-task exploration only; native checks and all physical protections unchanged')
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
    exploratory = command.get('exploratory_task_reconsideration', False)
    if type(exploratory) is not bool:raise ValueError('BOOLEAN_TASK_RECONSIDERATION_REQUIRED')
    if exploratory:
        from pathlib import Path
        observations = ops.g['agent_observations']
        if not observations:raise ValueError('CURRENT_INSPECTED_WRIST_GEOMETRY_REQUIRED')
        # The original owner may accept a current observed-depth hypothesis
        # later in request_plan. Its accepted/fresh geometry is checked at the
        # actual scoring seam below, after that normal acceptance path.
        folder = Path(observations[-1]['folder'])
        hashes = {hashlib.sha256(p.read_bytes()).hexdigest() for name in
                  ['rgb.png','agent_inspection.png','selected_handle_overlay.png','depth_handle_proposals.png'] if (p:=folder/name).is_file()}
        if command.get('inspected_image_sha256') not in hashes or not command.get('reason'):
            raise ValueError('CURRENT_INSPECTED_WRIST_IMAGE_AND_AGENT_REASON_REQUIRED')
    def memory(attempts, output, requested):
        fresh = any(v.get('accepted_for_geometry') and
                    v['world_sim_s'] > getattr(ops, '_last_contact_finished_world_s', float('inf'))
                    for v in ops.g['agent_observations'])
        return conditioned(attempts, output, requested, original=original,
                           actions=command['planned_segments'], start=start,
                           fresh=fresh, recorded=recorded, exploratory=exploratory)
    with patch.object(operations, 'recovery_memory', memory), patch.object(owner, 'recovery_memory', memory):
        yield
