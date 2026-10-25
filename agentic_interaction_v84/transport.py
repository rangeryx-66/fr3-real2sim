"""Strict long-motion contract and explicit current-contact generations."""
from agentic_interaction_v81.transport import validate as legacy_validate
from agent_command_protocol_v12 import ProtocolError


def validate(command,sequence):
    op=command.get('operation') if isinstance(command,dict) else None
    if op not in ('acquire_execute','continue_motion'):return legacy_validate(command,sequence)
    allowed={'operation','protocol_version','sequence','reason','segments','retain_contact',
        'contact_generation','inspected_image_sha256','native_g1_preload_request_n','recovery_strategy'}
    if set(command)-allowed:raise ProtocolError('unknown long-motion fields')
    if not isinstance(command.get('reason'),str) or not command['reason'].strip():
        raise ProtocolError('finite long-motion purpose required')
    if type(command.get('retain_contact',False))is not bool:raise ProtocolError('boolean retention required')
    if op=='acquire_execute':
        h=command.get('inspected_image_sha256')
        if not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h):
            raise ProtocolError('current actually inspected plan image required')
    if op=='continue_motion' and (type(command.get('contact_generation'))is not int or command['contact_generation']<1):
        raise ProtocolError('current positive contact generation required')
    forwarded={k:v for k,v in command.items() if k not in ('retain_contact','contact_generation')}
    forwarded['operation']='attempt_grasp_and_probe'
    legacy_validate(forwarded,sequence)
    for action in command.get('segments',[]):
        if action.get('execution_strategy','continuous') not in ('legacy','continuous'):
            raise ProtocolError('registered legacy/continuous strategy required')
    return command


def receive(path,sequence,responses):
    from unittest.mock import patch
    import agentic_interaction_v81.transport as legacy
    with patch.object(legacy,'validate',validate):return legacy.receive(path,sequence,responses)


def publish(episode,sequence,command):
    import fcntl,json,time
    from pathlib import Path
    from unittest.mock import patch
    import agentic_interaction_v81.transport as legacy
    from agent_command_protocol_v12 import atomic_publish
    episode=Path(episode);value=dict(command,protocol_version=1,sequence=sequence)
    validate(value,sequence)
    state=json.loads((episode/'AGENT_SESSION_STATUS.json').read_text())
    if not state.get('abc_contact'):
        with patch.object(legacy,'validate',validate):return legacy.publish(episode/'agent_commands',sequence,value)
    if (episode/'PROCESS_COMPLETION.json').exists() or state['state']!='WAITING' or state['sequence']!=sequence or time.time()>=state['deadline_unix_s']:
        raise ProtocolError('STALE_OR_CLOSED_CONTACT_BOUNDARY')
    if value['operation'] not in ('continue_motion','safe_release_retreat','finish','inspect'):
        raise ProtocolError('only finite continuation, release, finish or numeric inspection while held')
    directory=episode/'agent_commands'
    with (directory/'.publisher.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        live=json.loads((episode/'AGENT_SESSION_STATUS.json').read_text())
        if live!=state:raise ProtocolError('CONTACT_STATE_CHANGED_RETRY')
        return atomic_publish(directory,sequence,value)


if __name__=='__main__':
    import argparse,json
    from pathlib import Path
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--sequence',type=int,required=True);p.add_argument('--json',type=Path,required=True)
    a=p.parse_args();print(json.dumps(publish(a.episode,a.sequence,json.loads(a.json.read_text()))))
