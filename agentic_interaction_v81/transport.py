"""V8 atomic transport plus explicitly validated V8.1 station requests."""
import argparse,fcntl,hashlib,json,math,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from agent_command_protocol_v12 import (
    VERSION,ProtocolError,decode,validate as original_validate,atomic_json,complete,atomic_publish)


def validate(command,sequence):
    if not isinstance(command,dict):return original_validate(command,sequence)
    op=command.get('operation')
    if op=='relocate_observe_prepare':
        fields={'operation','protocol_version','sequence','reason','target_xy_m',
            'inspected_image_sha256','planned_segments','planning_target_q',
            'native_g1_preload_request_n','transverse_unloading','contact_following'}
        if set(command)-fields:raise ProtocolError('no pose or recovery override allowed')
        station={k:command[k]for k in ('protocol_version','sequence','reason','target_xy_m','inspected_image_sha256')if k in command}
        validate(dict(station,operation='evaluate_station'),sequence)
        plan={k:v for k,v in command.items()if k not in ('target_xy_m',)}
        validate(dict(plan,operation='prepare_grasp'),sequence)
        original_validate(dict(protocol_version=command.get('protocol_version'),sequence=sequence,
            operation='attempt_grasp_and_probe',segments=command.get('planned_segments')),sequence)
        if 'native_g1_preload_request_n' in command:
            value=command['native_g1_preload_request_n']
            if isinstance(value,bool)or value not in (.5,.75,1.,1.5):raise ProtocolError('tested preload menu only')
        return command
    if op in ('prepare_grasp','observe_view','select_observed_target'):
        forwarded=dict(command,operation='evaluate_view'if op=='observe_view'else'build_geometry')
        if op=='select_observed_target':forwarded['start_new_target_association']=True
        validate(forwarded,sequence)
        return command
    if op=='build_geometry' and 'exploratory_task_reconsideration' in command:
        value=command['exploratory_task_reconsideration']
        if type(value) is not bool:raise ProtocolError('boolean task reconsideration required')
        if value:
            h=command.get('inspected_image_sha256')
            if not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h):
                raise ProtocolError('current inspected wrist image SHA256 required')
            if not isinstance(command.get('reason'),str) or not command['reason'].strip():
                raise ProtocolError('explicit Agent changed-task reason required')
    if op=='build_geometry' and 'start_new_target_association' in command:
        if type(command['start_new_target_association']) is not bool:
            raise ProtocolError('boolean target-association selection required')
        if command['start_new_target_association']:
            h=command.get('inspected_image_sha256')
            if (not isinstance(h,str)or len(h)!=64
                    or any(c not in '0123456789abcdef'for c in h)
                    or not command.get('reason')or not command.get('observed_handle_hypothesis')):
                raise ProtocolError('current inspected explicit observed-target selection required')
    if op=='redeclare_actions':
        allowed={'operation','protocol_version','sequence','reason','segments','inspected_image_sha256'}
        if set(command)-allowed:raise ProtocolError('action declaration cannot modify a pose or preload')
        original_validate(dict(command,operation='attempt_grasp_and_probe'),sequence)
        h=command.get('inspected_image_sha256')
        if not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h):
            raise ProtocolError('current inspected plan image SHA256 required')
        if not isinstance(command.get('reason'),str) or not command['reason'].strip():
            raise ProtocolError('Agent action-declaration reason required')
        return command
    if op=='build_geometry' and 'planning_target_q' in command:
        value=command['planning_target_q']
        if type(value) not in (int,float) or not math.isfinite(value):
            raise ProtocolError('finite native-coordinate planning horizon required')
    if op=='build_geometry' and 'recovery_search_round' in command:
        value=command['recovery_search_round']
        if type(value) is not int or value not in (0,1):
            raise ProtocolError('bounded recovery search round 0 or 1 required')
    if op=='build_geometry' and 'decline_current_plan' in command:
        item=command['decline_current_plan']
        if (command.get('recovery_search_round')!=1 or not isinstance(item,dict)
                or set(item)!={'image_sha256','reason'}
                or not isinstance(item['image_sha256'],str) or len(item['image_sha256'])!=64
                or any(c not in '0123456789abcdef'for c in item['image_sha256'])
                or not isinstance(item['reason'],str) or not item['reason'].strip()):
            raise ProtocolError('explicit inspected current-plan rejection required')
    if op not in ('evaluate_station','relocate_station'):return original_validate(command,sequence)
    # Preserve original protocol-version/sequence/type checks. Additional
    # operations cannot smuggle hand-written joint/grasp/object-state fields.
    original_validate(dict(command,operation='inspect'),sequence)
    common={'operation','protocol_version','sequence','reason'}
    if op=='evaluate_station':
        if set(command)-common-{'target_xy_m','inspected_image_sha256','task_horizon_q'}:raise ProtocolError('unknown station fields')
        xy=command.get('target_xy_m')
        if not isinstance(xy,list) or len(xy)!=2 or not all(type(x) in (int,float) and math.isfinite(x) for x in xy):
            raise ProtocolError('finite XY target required')
        h=command.get('inspected_image_sha256')
        if not isinstance(h,str) or len(h)!=64 or any(c not in '0123456789abcdef' for c in h):
            raise ProtocolError('current wrist image SHA256 required')
        if 'task_horizon_q' in command:
            value=command['task_horizon_q']
            if type(value) not in (int,float) or not math.isfinite(value):
                raise ProtocolError('finite native-coordinate task horizon required')
    else:
        if set(command)-common-{'evaluated_command'}:raise ProtocolError('unknown station fields')
        n=command.get('evaluated_command')
        if type(n) is not int or not 0<=n<sequence:raise ProtocolError('previous evaluated station sequence required')
    if not isinstance(command.get('reason'),str) or not command['reason'].strip():raise ProtocolError('Agent reason required')
    return command


def publish(directory,sequence,command):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    value=dict(command,protocol_version=VERSION,sequence=sequence);validate(value,sequence)
    if directory.name=='agent_commands':
        episode=directory.parent
        if (episode/'PROCESS_COMPLETION.json').exists()or(episode/'EPISODE_RESULT.json').exists():
            raise ProtocolError('EPISODE_ENDED_NO_COMMAND_CAN_EXECUTE')
        waiting=episode/'WAITING_FOR_AGENT.json'
        if not waiting.exists():raise ProtocolError('LIVE_AGENT_DECISION_WINDOW_REQUIRED')
        window=json.loads(waiting.read_text())
        deadline=window.get('deadline_unix_s',waiting.stat().st_mtime+float(window['timeout_s']))
        if window.get('sequence')!=sequence or time.time()>=deadline:
            raise ProtocolError('STALE_OR_CLOSED_AGENT_DECISION_WINDOW')
    with (directory/'.publisher.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        state_path=directory.parent/'AGENT_SESSION_STATUS.json'
        if directory.name=='agent_commands' and state_path.exists():
            state=json.loads(state_path.read_text())
            if state.get('state')!='WAITING' or state.get('sequence')!=sequence:
                raise ProtocolError('AGENT_SESSION_BUSY_OR_CLOSED:'+str(state.get('state')))
            if not state.get('unloaded') or time.time()>=state.get('deadline_unix_s',0):
                raise ProtocolError('STALE_OR_LOADED_AGENT_DECISION_WINDOW')
        result=atomic_publish(directory,sequence,value)
    return dict(result,protocol_version=VERSION,sequence=sequence,acknowledgment='pending',
        extension='V81_STATION',publication_is_not_execution=True)


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
