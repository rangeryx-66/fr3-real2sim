"""Versioned file transport, independent of perception, physics and policy.

Each sequence is consumed exactly once. A bad message gets a PROTOCOL_ERROR
ACK and consumes that sequence, with no robot action; send a corrected message
at the next advertised sequence. ACCEPTED and COMPLETED ACKs are separate.
All operations use the same schema and atomic publisher. Safety exceptions are
not caught here: the existing operation owner still handles them.
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from publish_agent_command_atomic import publish as atomic_publish

VERSION = 1
OPERATIONS = frozenset(('observe', 'inspect', 'evaluate_view', 'move_and_observe',
    'build_geometry', 'validate_execution', 'attempt_grasp_and_probe',
    'inspect_contact_result', 'safe_release_retreat', 'reobserve_and_replan',
    'fresh_mode_observation', 'plan_panel_push', 'execute_panel_push', 'finish'))


class ProtocolError(ValueError):
    pass


def atomic_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False).encode()
    fd, name = tempfile.mkstemp(prefix='.' + path.name, suffix='.pending', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)


def decode(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result: raise ProtocolError('duplicate JSON key: ' + key)
            result[key] = value
        return result
    try:
        return json.loads(payload, object_pairs_hook=pairs,
            parse_constant=lambda x: (_ for _ in ()).throw(ProtocolError('nonfinite JSON: ' + x)))
    except (ValueError, UnicodeError) as error:
        raise ProtocolError(str(error)) from error


def validate(command, sequence):
    def require(ok, why):
        if not ok: raise ProtocolError(why)
    def number(x):
        return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
    def vector(x, n):
        return isinstance(x, list) and len(x) == n and all(number(v) for v in x)
    require(isinstance(command, dict), 'command must be a JSON object')
    require(type(command.get('protocol_version')) is int and command['protocol_version'] == VERSION, 'protocol_version must be 1')
    require(type(command.get('sequence')) is int and command['sequence'] == sequence, 'sequence does not match inbox slot')
    op = command.get('operation')
    require(isinstance(op, str) and op in OPERATIONS, 'unknown/missing operation')
    if op == 'evaluate_view':
        target = command.get('target'); require(isinstance(target, dict), 'target object required')
        if 'T_camera' in target:
            require(isinstance(target['T_camera'], list) and len(target['T_camera']) == 4
                and all(vector(row, 4) for row in target['T_camera']), 'finite 4x4 T_camera required')
        else:
            require(vector(target.get('eye_world_m'), 3) and vector(target.get('look_at_world_m'), 3), 'finite eye/look vectors required')
    if op == 'move_and_observe':
        require(type(command.get('evaluated_command')) is int and 0 <= command['evaluated_command'] < sequence, 'previous evaluated_command sequence required')
    if op == 'attempt_grasp_and_probe':
        actions = command.get('segments')
        require(isinstance(actions, list) and 1 <= len(actions) <= 4, 'finite registered segment list required')
        for action in actions:
            require(isinstance(action, dict), 'segment object required')
            require(number(action.get('target_deg')), 'finite target_deg required')
            require(isinstance(action.get('role'), str) and action['role'].startswith(('FIT', 'VALIDATION')), 'segment role required')
            require(action.get('profile') in ('constant', 'smooth_variation'), 'known segment profile required')
            require(type(action.get('hold')) is bool, 'boolean segment hold required')
            for key in ('reference_speed_m_s', 'reference_acceleration_m_s2', 'speed_scale'):
                if key in action: require(number(action[key]) and action[key] > 0, 'positive finite ' + key + ' required')
    if op == 'plan_panel_push':
        rectangle = command.get('rectangle')
        require(isinstance(rectangle, list) and len(rectangle) == 4 and all(type(x) is int for x in rectangle), 'integer image rectangle required')
        require(0 <= rectangle[0] < rectangle[2] and 0 <= rectangle[1] < rectangle[3], 'ordered nonnegative rectangle required')
        require(number(command.get('target_deg')), 'finite target_deg required')
        if 'roll_deg' in command: require(number(command['roll_deg']), 'finite roll_deg required')
    return command


def publish(directory, sequence, command):
    # Serialize publishers as well as readers; the old existence/replace pair
    # alone could overwrite the same slot when two publishers raced.
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    value = dict(command, protocol_version=VERSION, sequence=sequence)
    validate(value, sequence)
    with (directory / '.publisher.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        result = atomic_publish(directory, sequence, value)
    return dict(result, protocol_version=VERSION, sequence=sequence, acknowledgment='pending')


def receive(path, sequence, responses):
    path = Path(path); responses = Path(responses)
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    try:
        command = validate(decode(payload), sequence)
    except ProtocolError as error:
        ack = dict(sequence=sequence, protocol_version=VERSION, status='PROTOCOL_ERROR',
            reason=str(error), command_sha256=digest, robot_action_executed=False,
            recoverable=True, next_sequence=sequence + 1)
        atomic_json(responses / ('%03d.json' % sequence), ack)
        return None, ack
    ack = dict(sequence=sequence, protocol_version=VERSION, status='ACCEPTED',
        operation=command['operation'], command_sha256=digest)
    atomic_json(responses / ('%03d.accepted.json' % sequence), ack)
    return command, ack


def complete(responses, sequence, entry):
    value = dict(entry, sequence=sequence, protocol_version=VERSION, status='COMPLETED')
    atomic_json(Path(responses) / ('%03d.json' % sequence), value)


def wire_owner(source):
    """One explicit transport seam in the pinned operation owner, all ops alike.

    Imports are inside the generated function, so no new callback dependencies
    are silently inherited via dynamically constructed global namespaces.
    """
    old = "        command=json.loads(path.read_text());op=command['operation'];began=time.perf_counter()"
    new = """        from agent_command_protocol_v12 import receive, complete
        command,ack=receive(path,sequence,responses)
        if command is None:
            trace.append(dict(sequence=sequence,protocol_error=ack))
            write(g,'AGENT_ACTION_TRACE.json',trace)
            sequence+=1
            continue
        op=command['operation'];began=time.perf_counter()"""
    end = "observation.write(responses/('%03d.json'%sequence),entry)"
    if source.count(old) != 1 or source.count(end) != 1:
        raise ValueError('Pinned Agent owner transport seam changed')
    # A safety exception still terminates the operation immediately. Publish
    # its result ACK before the original owner exits; never swallow the stop.
    fatal = "if g['recovery_state']!='UNLOADED_OBSERVATION':termination='INTERFACE_OR_SAFETY_FAILURE';write(g,'INTERFACE_FAILURE.json',result);break"
    ack_fatal = "if g['recovery_state']!='UNLOADED_OBSERVATION':\n                complete(responses,sequence,dict(command=command,result=result))\n                termination='INTERFACE_OR_SAFETY_FAILURE';write(g,'INTERFACE_FAILURE.json',result);break"
    if source.count(fatal) != 1: raise ValueError('Pinned safety exception boundary changed')
    return source.replace(old, new).replace(end, 'complete(responses,sequence,entry)').replace(fatal, ack_fatal)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--sequence', type=int, required=True)
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(publish(args.directory, args.sequence, decode(args.input.read_bytes()))))
