"""Fail-closed terminal evidence; never clears a latch or commands motion.

Simulation pause, native stop acknowledgement, parking and process exit are
separate facts. A closed simulator cannot retrospectively prove safe parking.
"""
import json
import math
from pathlib import Path
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def registry_path(environment):
    return Path(str(environment['actor_lock'])+'.safety_hold.json')


def require_clearance(path):
    # Deliberately no boolean bypass/reset API. Existing unsafe scenes need
    # independently verified remediation before any new physical launch.
    if Path(path).exists():
        raise RuntimeError('UNRESOLVED_TERMINAL_SAFETY_HOLD:'+str(path))


def latest_exit(result):
    result=result or {}
    if 'safe_exit' in result:return result['safe_exit'] or {}
    for row in reversed(result.get('trace', [])):
        exit_result=row.get('result', {}).get('safe_exit')
        if exit_result is not None:return exit_result
    return {}


def classify(latch, exit_result, agent_parked=False):
    park=exit_result.get('park') or {}
    verified=bool(latch is None and exit_result.get('safe') is True
        and park.get('reached') is True and park.get('unloaded') is True
        and park.get('measured_collision_check', {}).get('valid') is True)
    return dict(status='SAFE_PARK_VERIFIED' if verified else
        'HARD_STOP_UNPARKED' if latch is not None else 'EXIT_UNVERIFIED',
        latch_present=latch is not None, fault=latch,
        agent_park_flag=bool(agent_parked), safe_park_verified=verified,
        requires_intervention=not verified, motion_resume_allowed=False,
        controller_stop_verified=False,
        controller_stop_scope='No post-fault controller acknowledgement/stationary dwell; simulator exit is not hardware braking.',
        unload=exit_result, latch_cleared=False)


def persist(runtime, result=None, pause=False):
    latch=runtime.supervisor.latched
    if pause:
        runtime.world.pause()  # no step, play, joint command or latch mutation
    data=classify(latch, latest_exit(result), runtime.g.get('at_safe_park',False))
    row=runtime.rows[-1] if getattr(runtime,'rows',None) else {}
    velocities=row.get('joint_velocities', [])
    arm=getattr(runtime,'arm',range(len(velocities)))
    values=[float(velocities[i]) for i in arm] if len(velocities) else []
    data.update(episode_id=runtime.job['episode_id'],
        simulation_paused_verified=(not runtime.world.is_playing())
            if hasattr(runtime.world,'is_playing') else None,
        last_measurement_t=row.get('t'),
        last_measured_object_coordinate=row.get('joint_coordinate'),
        last_arm_velocity_abs_max=max(map(abs,values))
            if values and all(math.isfinite(x) for x in values) else None,
        post_stop_stationary_window_available=False,
        physical_state_not_inferred_from_process_exit=True)
    data=result_packet(data)
    atomic_json(runtime.output/'TERMINAL_SAFETY.json', data)
    path=runtime.job.get('terminal_safety_registry')
    if data['requires_intervention'] and path:
        atomic_json(Path(path), dict(episode_id=runtime.job['episode_id'],
            status=data['status'], evidence=str(runtime.output/'TERMINAL_SAFETY.json'),
            reason='Physical exit unverified; no automatic reset or new contact.'))
    return data


def process_outcome(output, returncode):
    path=Path(output)/'TERMINAL_SAFETY.json'
    evidence=json.loads(path.read_text()) if path.exists() else {}
    return dict(process_returncode=returncode,
        terminal_safety_status=evidence.get('status','UNVERIFIED_MISSING_EVIDENCE'),
        physical_safe_park_verified=evidence.get('safe_park_verified') is True,
        requires_intervention=evidence.get('requires_intervention',True),
        process_exit_is_not_task_success=True)
