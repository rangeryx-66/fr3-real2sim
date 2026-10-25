"""Opt-in human-authorized task-wall credit; no physical authority changes.

Only a newly launched actor installs this frontend. Accepted immutable records
are authenticated to its launcher, which follows the same cumulative credit.
The launch authorization reference is procedural human provenance, not proof
of human identity supplied by a text string or a new control authority.
"""
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import time

PROFILE = 'v83_revolute_orchestration_extension_v1'
FLAG = 'v83_orchestration_extension'
AUTH = 'orchestration_authorization'
KEY_ENV = 'V83_ORCHESTRATION_LEDGER_KEY'
OPERATION = 'extend_orchestration_wall'
MAX_SINGLE_S = 1800
LEDGER = 'orchestration_extensions'


def policy(spec):
    value = spec.get(AUTH)
    if (not isinstance(value, dict) or set(value) - {'human_authorization_ref', 'max_total_extra_wall_s', 'parked_decision_renewal_s'}
            or not {'human_authorization_ref', 'max_total_extra_wall_s'} <= set(value)
            or not isinstance(value['human_authorization_ref'], str)
            or not value['human_authorization_ref'].strip()
            or type(value['max_total_extra_wall_s']) is not int
            or not 1 <= value['max_total_extra_wall_s'] <= 86400):
        raise ValueError('EXPLICIT_HUMAN_REFERENCE_AND_FINITE_INTEGER_CREDIT_CAP_REQUIRED')
    renewal = value.get('parked_decision_renewal_s', 0)
    if type(renewal) is not int or not 0 <= renewal <= MAX_SINGLE_S:
        raise ValueError('EXPLICIT_PARKED_RENEWAL_INTEGER_0_TO_1800_REQUIRED')
    return dict(value, parked_decision_renewal_s=renewal)


def validate_request(command):
    fields = {'operation', 'protocol_version', 'sequence', 'binding', 'request_nonce',
              'human_authorization_ref', 'delta_wall_s', 'reason'}
    if set(command) != fields:
        raise ValueError('STRICT_ORCHESTRATION_WALL_REQUEST_REQUIRED')
    binding = command['binding']
    if (not isinstance(binding, dict) or set(binding) != {'episode_id', 'source_commit', 'actor_pid', 'session_nonce'}
            or type(binding['actor_pid']) is not int or binding['actor_pid'] <= 0
            or any(not isinstance(binding[k], str) or not binding[k] for k in
                   ('episode_id', 'source_commit', 'session_nonce'))):
        raise ValueError('EXPLICIT_EPISODE_SOURCE_ACTOR_SESSION_BINDING_REQUIRED')
    if (not isinstance(command['request_nonce'], str) or len(command['request_nonce']) != 32
            or any(c not in '0123456789abcdef' for c in command['request_nonce'])):
        raise ValueError('UNIQUE_128_BIT_REQUEST_NONCE_REQUIRED')
    if type(command['delta_wall_s']) is not int or not 1 <= command['delta_wall_s'] <= MAX_SINGLE_S:
        raise ValueError('WALL_CREDIT_MUST_BE_INTEGER_1_TO_1800_SECONDS')
    for k in ('human_authorization_ref', 'reason'):
        if not isinstance(command[k], str) or not command[k].strip():
            raise ValueError('EXPLICIT_HUMAN_REFERENCE_AND_REASON_REQUIRED')


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def signature(value, key):
    return hmac.new(key.encode(), canonical(value), hashlib.sha256).hexdigest()


def immutable_json(path, value):
    """Publish once atomically, with no overwrite of any accepted record."""
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name('.' + path.name + '.' + secrets.token_hex(8))
    try:
        with temporary.open('xb') as stream:
            stream.write(canonical(value)); stream.flush(); os.fsync(stream.fileno())
        os.link(temporary, path)  # atomic publication, fails if final already exists
    finally:
        if temporary.exists():temporary.unlink()


class WallCredit:
    """Delegate unchanged sim/reserve contracts; retain the original clock."""
    def __init__(self, original):
        self.original = original
        self.total_extra_wall_s = 0

    def __getattr__(self, name):return getattr(self.original, name)

    def remaining(self, sim_s):
        result = self.original.remaining(sim_s)
        # Never add credit to clamped remaining(): that would stop elapsed time
        # consuming the extension once the original budget expired.
        result['wall_s'] = max(0., self.original.wall + self.total_extra_wall_s
            - (self.original.now() - self.original.started) - self.original.reserve_wall)
        return result


class ActorLedger:
    def __init__(self, output, binding, authorization, budget, key, unix_now=time.time):
        self.output = Path(output); self.binding = dict(binding)
        self.authorization = dict(authorization); self.budget = budget
        self.key = key; self.unix_now = unix_now; self.closed = False
        self.nonces = set(); self.records = 0; self.previous = None
        if not isinstance(key, str) or len(key) < 64:
            raise ValueError('LAUNCHER_ACTOR_LEDGER_KEY_REQUIRED')
        if any((self.output/LEDGER).glob('*.json')):
            raise ValueError('FRESH_ACTOR_LEDGER_REQUIRED_NO_RESUME_OR_REPLAY')

    def packet(self):
        return dict(binding=self.binding, authorization=self.authorization,
            accepted_extra_wall_s=self.budget.total_extra_wall_s,
            maximum_single_extension_s=MAX_SINGLE_S, operation=OPERATION,
            only_at_verified_parked_live_decision_boundary=True,
            original_started_monotonic=self.budget.original.started,
            original_wall_s=self.budget.original.wall,
            effective_wall_s=self.budget.original.wall+self.budget.total_extra_wall_s,
            original_sim_s=self.budget.original.sim,
            original_exit_wall_s=self.budget.original.reserve_wall,
            original_exit_sim_s=self.budget.original.reserve_sim,
            current_actor_must_have_installed_extension_at_launch=True)

    def accept(self, command, evidence, decision_deadline_unix_s, *, standing_renewal=False):
        validate_request(command)
        if self.closed or evidence.get('closed'):
            raise ValueError('CLOSED_ACTOR_CANNOT_BE_EXTENDED')
        if (not math.isfinite(decision_deadline_unix_s)
                or (not standing_renewal and self.unix_now() >= decision_deadline_unix_s)):
            raise ValueError('EXPIRED_DECISION_BOUNDARY')
        if standing_renewal and command['delta_wall_s'] != self.authorization.get('parked_decision_renewal_s', 0):
            raise ValueError('PARKED_RENEWAL_NOT_EXPLICITLY_AUTHORIZED')
        if standing_renewal and decision_deadline_unix_s+command['delta_wall_s'] <= self.unix_now():
            raise ValueError('PARKED_DECISION_TOO_STALE_FOR_FINITE_RENEWAL')
        if command['binding'] != self.binding:
            raise ValueError('EXTENSION_BINDING_MISMATCH')
        if command['human_authorization_ref'] != self.authorization['human_authorization_ref']:
            raise ValueError('UNREGISTERED_HUMAN_AUTHORIZATION_REFERENCE')
        if command['request_nonce'] in self.nonces:
            raise ValueError('EXTENSION_NONCE_REPLAY')
        required = ('unloaded', 'park_reached', 'robot_stationary', 'object_stationary',
                    'released', 'no_grasp_reference', 'force_settled', 'finite_state', 'sensors_trusted')
        if evidence.get('safety_latch') is not None or not all(evidence.get(k) is True for k in required):
            raise ValueError('VERIFIED_PARKED_UNLOADED_UNLATCHED_TRUSTED_BOUNDARY_REQUIRED')
        # Serialization rejects NaNs anywhere in the accepted state evidence.
        canonical(evidence)
        original = self.budget.original
        before = self.budget.remaining(evidence['sim_s'])
        if (before['wall_s'] <= 0 and not standing_renewal) or before['sim_s'] <= 0:
            raise ValueError('LIVE_UNEXPIRED_WORK_BOUNDARY_REQUIRED')
        total = self.budget.total_extra_wall_s + command['delta_wall_s']
        if total > self.authorization['max_total_extra_wall_s']:
            raise ValueError('HUMAN_AUTHORIZED_CUMULATIVE_WALL_CAP_EXCEEDED')
        after_wall = max(0., original.wall+total-(original.now()-original.started)-original.reserve_wall)
        if after_wall <= 0:raise ValueError('AUTHORIZED_CREDIT_DOES_NOT_RESTORE_LIVE_WORK_WINDOW')
        entry = dict(schema='v83-human-orchestration-wall-credit-v1', ordinal=self.records+1,
            binding=self.binding, request=command, authorization=self.authorization,
            previous_sha256=self.previous, accepted_unix_s=self.unix_now(),
            original=dict(started_monotonic=original.started, wall_s=original.wall, sim_s=original.sim,
                          exit_wall_s=original.reserve_wall, exit_sim_s=original.reserve_sim),
            prior_total_extra_wall_s=self.budget.total_extra_wall_s, total_extra_wall_s=total,
            wall_remaining_before_s=before['wall_s'],
            wall_remaining_after_s=after_wall,
            authorization_mode='standing_parked_wait' if standing_renewal else 'explicit_decision_command',
            decision_deadline_before_unix_s=decision_deadline_unix_s,
            boundary=evidence, physical_limits_modified=False, original_clock_reset=False)
        signed = dict(entry, hmac_sha256=signature(entry, self.key))
        immutable_json(self.output/LEDGER/('%04d.json' % (self.records+1)), signed)
        self.previous = hashlib.sha256(canonical(signed)).hexdigest()
        self.records += 1; self.nonces.add(command['request_nonce'])
        self.budget.total_extra_wall_s = total
        return dict(extended=True, total_extra_wall_s=total, effective_wall_s=original.wall+total,
            original_exit_reserves_preserved=True, physical_limits_modified=False,
            next_decision_window_uses_effective_remaining_wall=True, ledger_sha256=self.previous)

    def renew_wait(self, evidence, deadline):
        delta = self.authorization.get('parked_decision_renewal_s', 0)
        if not delta:raise ValueError('PARKED_RENEWAL_NOT_EXPLICITLY_AUTHORIZED')
        command = dict(operation=OPERATION, protocol_version=1, sequence=-1,
            binding=self.binding, request_nonce=secrets.token_hex(16),
            human_authorization_ref=self.authorization['human_authorization_ref'], delta_wall_s=delta,
            reason='Standing human launch authorization: continue bounded waiting only after fresh verified native parked/unloaded checks')
        result = self.accept(command, evidence, deadline, standing_renewal=True)
        return dict(result, renewed_wait_s=delta)


def install(runtime):
    if not runtime.job.get(FLAG):return
    job = runtime.job
    budget = WallCredit(runtime.episode_budget)
    manager = ActorLedger(runtime.output,
        dict(episode_id=job['episode_id'], source_commit=job['v83_orchestration_source_commit'],
             actor_pid=os.getpid(), session_nonce=job['v83_orchestration_session_nonce']),
        job[AUTH], budget, os.environ.get(KEY_ENV))
    runtime.episode_budget = budget
    runtime.orchestration_extension = manager


def measured_evidence(ops):
    """Read native current state only; no stepping, settling or latch clearing."""
    import numpy as np
    from dataclasses import asdict
    from native_g1_safe_park import verify_park
    r = ops.r
    if (r.stopped or (r.output/'PROCESS_COMPLETION.json').exists()
            or (r.output/'EPISODE_RESULT.json').exists()):
        raise ValueError('CLOSED_OR_STOPPED_ACTOR')
    if (ops.g['recovery_state'] != 'UNLOADED_OBSERVATION'
            or not ops.g.get('at_safe_park') or not ops.g.get('last_exit_plan')):
        raise ValueError('VERIFIED_NATIVE_PARK_REQUIRED')
    boundary = r.station_boundary(record=False); boundary.check()
    verified = verify_park(ops.g, ops.g['last_exit_plan'])
    row = r.rows[-1]
    numeric = np.r_[r.robot.get_joint_positions(), r.robot.get_joint_velocities(),
                    r.qtarget, r.filtered(), row['joint_coordinate_velocity'], row['aperture_m'],
                    row['wrist_ft']['raw_wrench_sensor'],
                    row['wrist_ft']['gravity_compensated_wrench_world'],
                    np.asarray(row['T_tcp']).ravel()]
    return dict(unloaded=bool(boundary.unloaded_verified and verified['unloaded']),
        park_reached=bool(verified['reached']), robot_stationary=boundary.robot_stationary,
        object_stationary=boundary.object_stationary, released=boundary.released,
        no_grasp_reference=r.grasp_reference is None and ops.g.get('reference') is None,
        force_settled=bool(r.supervisor.force.settled()), finite_state=bool(np.isfinite(numeric).all()),
        sensors_trusted=bool(row.get('wrist_ft', {}).get('valid')),
        safety_latch=r.supervisor.latched, sim_s=r.tick*r.dt,
        native_boundary=asdict(boundary), park=verified, closed=False)


def handle(ops, command, deadline):
    manager = getattr(ops.r, 'orchestration_extension', None)
    if manager is None:
        return dict(extended=False, reason='EXTENSION_NOT_INSTALLED_AT_LAUNCH')
    try:return manager.accept(command, measured_evidence(ops), deadline)
    except (ValueError, KeyError, TypeError) as error:
        return dict(extended=False, reason=str(error), physical_motion_executed=False)


def renew_parked_wait(ops, deadline, sequence=None):
    manager = getattr(ops.r, 'orchestration_extension', None)
    if manager is None:return dict(extended=False, reason='EXTENSION_NOT_INSTALLED_AT_LAUNCH')
    try:return manager.renew_wait(measured_evidence(ops), deadline)
    except (ValueError, KeyError, TypeError) as error:
        # A paused, released observation pose is recoverable. With explicit
        # standing authorization, use the SAME original guarded exit before
        # requesting clock credit; never certify an unparked state as parked.
        from .task_recovery import unloaded
        r=ops.r
        if (str(error)!='VERIFIED_NATIVE_PARK_REQUIRED' or not r.job.get('v84_abc')
                or not manager.authorization.get('parked_decision_renewal_s',0)
                or not unloaded(ops) or r.stopped or (r.output/'STOP_AND_EXPORT').exists()):
            return dict(extended=False, reason=str(error), physical_motion_executed=False)
        from .session_status import write as status
        from agent_command_protocol_v12 import atomic_json
        status(r,sequence,'RUNNING',operation='AUTHORIZED_TIMEOUT_SAFE_PARK',unloaded_decision=False)
        began=time.monotonic();start_sim=r.tick*r.dt
        safe=ops.release_park()  # hard safety exceptions propagate, never retry through
        cost=dict(wall_s=time.monotonic()-began,start_sim_s=start_sim,end_sim_s=r.tick*r.dt)
        atomic_json(r.output/('ORCHESTRATION_RECOVERY_%03d.json'%sequence),
            dict(reason='AUTHORIZED_DECISION_WAIT_RECOVERY',safe_exit=safe,
                original_task_goal_preserved=True,latch_cleared=False,cost=cost))
        if not safe.get('safe'):
            return dict(extended=False,reason='TIMEOUT_SAFE_PARK_UNCONFIRMED',safe_exit=safe)
        ops.current_plan_ready=False
        if hasattr(ops,'_v81_plan_ticket'):ops._v81_plan_ticket=None
        r.world.pause()  # restore the original unloaded decision boundary
        try:
            result=manager.renew_wait(measured_evidence(ops),deadline)
            return dict(result,guarded_park_recovery=True,current_plan_invalidated=True,park_recovery_cost=cost)
        except (ValueError,KeyError,TypeError) as refusal:
            return dict(extended=False,reason=str(refusal),safe_exit=safe)


def accepted_total(output, binding, authorization, key):
    """Launcher trusts only actor-authenticated immutable accepted records."""
    total = 0; previous = None; nonces = set()
    for i, path in enumerate(sorted((Path(output)/LEDGER).glob('*.json')), 1):
        signed = json.loads(path.read_text()); entry = dict(signed)
        mac = entry.pop('hmac_sha256')
        if not hmac.compare_digest(mac, signature(entry, key)):
            raise ValueError('EXTENSION_LEDGER_AUTHENTICATION_FAILED')
        request = entry['request']; validate_request(request)
        if (entry['binding'] != binding or request['binding'] != binding or entry['ordinal'] != i
                or entry['authorization'] != authorization or entry['previous_sha256'] != previous
                or entry['prior_total_extra_wall_s'] != total
                or request['human_authorization_ref'] != authorization['human_authorization_ref']
                or request['request_nonce'] in nonces):
            raise ValueError('EXTENSION_LEDGER_BINDING_CHAIN_OR_REPLAY_FAILURE')
        if entry.get('authorization_mode') == 'standing_parked_wait' and request['delta_wall_s'] != authorization.get('parked_decision_renewal_s', 0):
            raise ValueError('UNAUTHORIZED_STANDING_RENEWAL')
        total += request['delta_wall_s']
        if total != entry['total_extra_wall_s'] or total > authorization['max_total_extra_wall_s']:
            raise ValueError('EXTENSION_LEDGER_CUMULATIVE_CAP_FAILURE')
        nonces.add(request['request_nonce'])
        previous = hashlib.sha256(canonical(signed)).hexdigest()
    return total


def wait_actor(actor, job, output, key=None, now=time.monotonic, poll_s=1.):
    if not job.get(FLAG):
        return actor.wait(timeout=float(job['episode_wall_budget_s'])+180)
    started = now(); credit = 0
    binding = dict(episode_id=job['episode_id'], source_commit=job['v83_orchestration_source_commit'],
                   actor_pid=actor.pid, session_nonce=job['v83_orchestration_session_nonce'])
    while True:
        result = actor.poll()
        if result is not None:return result
        try:
            observed = accepted_total(output, binding, job[AUTH], key)
            if observed < credit:raise ValueError('ACCEPTED_EXTENSION_LEDGER_REMOVED')
            credit = observed
        except (ValueError, KeyError, TypeError, OSError) as error:
            # Deny new unverified credit; retain already authenticated time.
            # Do not abandon the owned actor on an audit error. The original
            # launcher timeout/cleanup still owns its eventual termination.
            from agent_command_protocol_v12 import atomic_json
            atomic_json(Path(output)/'ORCHESTRATION_WATCHDOG_FAULT.json',dict(
                reason=str(error), retained_verified_credit_s=credit, extra_credit_denied=True))
        remaining = float(job['episode_wall_budget_s'])+180+credit-(now()-started)
        if remaining <= 0:raise subprocess.TimeoutExpired(actor.args, float(job['episode_wall_budget_s'])+180+credit)
        try:return actor.wait(timeout=min(poll_s, remaining))
        except subprocess.TimeoutExpired:pass
