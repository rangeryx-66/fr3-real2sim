"""V8 atomic transport plus explicitly validated V8.1 station requests."""
import argparse,fcntl,hashlib,json,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from agent_command_protocol_v12 import (
    VERSION,ProtocolError,decode,validate as original_validate,atomic_json,complete,atomic_publish)


def validate(command,sequence):
    if not isinstance(command,dict):return original_validate(command,sequence)
    op=command.get('operation')
    if op=='build_geometry' and 'planning_target_q' in command:
        value=command['planning_target_q']
        if type(value) not in (int,float) or not math.isfinite(value):
            raise ProtocolError('finite native-coordinate planning horizon required')
    if op not in ('evaluate_station','relocate_station'):return original_validate(command,sequence)
    # Preserve original protocol-version/sequence/type checks. Additional
    # operations cannot smuggle hand-written joint/grasp/object-state fields.
    original_validate(dict(command,operation='inspect'),sequence)
    common={'operation','protocol_version','sequence','reason'}
    if op=='evaluate_station':
        if set(command)-common-{'target_xy_m','inspected_image_sha256'}:raise ProtocolError('unknown station fields')
        xy=command.get('target_xy_m')
        if not isinstance(xy,list) or len(xy)!=2 or not all(type(x) in (int,float) and math.isfinite(x) for x in xy):
            raise ProtocolError('finite XY target required')
        h=command.get('inspected_image_sha256')
        if not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h):
            raise ProtocolError('current wrist image SHA256 required')
    else:
        if set(command)-common-{'evaluated_command'}:raise ProtocolError('unknown station fields')
        n=command.get('evaluated_command')
        if type(n) is not int or not 0<=n<sequence:raise ProtocolError('previous evaluated station sequence required')
    if not isinstance(command.get('reason'),str) or not command['reason'].strip():raise ProtocolError('Agent reason required')
    return command


def publish(directory,sequence,command):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    value=dict(command,protocol_version=VERSION,sequence=sequence);validate(value,sequence)
    with (directory/'.publisher.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        result=atomic_publish(directory,sequence,value)
    return dict(result,protocol_version=VERSION,sequence=sequence,acknowledgment='pending',extension='V81_STATION')


def receive(path,sequence,responses):
    path=Path(path);responses=Path(responses);payload=path.read_bytes();sha=hashlib.sha256(payload).hexdigest()
    try:command=validate(decode(payload),sequence)
    except ProtocolError as error:
        ack=dict(sequence=sequence,protocol_version=VERSION,status='PROTOCOL_ERROR',reason=str(error),
            command_sha256=sha,robot_action_executed=False,recoverable=True,next_sequence=sequence+1)
        atomic_json(responses/('%03d.json'%sequence),ack);return None,ack
    ack=dict(sequence=sequence,protocol_version=VERSION,status='ACCEPTED',operation=command['operation'],command_sha256=sha)
    atomic_json(responses/('%03d.accepted.json'%sequence),ack);return command,ack


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',required=True,type=Path)
    p.add_argument('--sequence',required=True,type=int);p.add_argument('--json',required=True,type=Path)
    a=p.parse_args();print(json.dumps(publish(a.episode/'agent_commands',a.sequence,decode(a.json.read_bytes()))))
