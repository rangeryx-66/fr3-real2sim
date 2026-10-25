"""Simulation-only instance admission; immutable incidents and shared ownership.

No reset/archive/delete operation exists here. Legacy entry points retain their
deny rule. An explicitly reviewed, destroyed simulated instance can be scoped
as history for a NEW World only; global/unknown/hardware holds always deny.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import uuid
from contextlib import contextmanager
from agent_command_protocol_v12 import atomic_json

DOMAIN=Path(__file__).with_name('ISOLATION_DOMAIN.json')
SCHEMA='isaac-instance-isolation/v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    value=json.loads(Path(path).read_text())
    if not isinstance(value,dict):raise ValueError('OBJECT_RECORD_REQUIRED:'+str(path))
    return value


def identity(pid):
    # Linux start tick distinguishes PID reuse. An unreadable owner is not dead.
    p=Path('/proc')/str(pid)
    if not p.exists():return None
    return dict(pid=int(pid),start_tick=p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19])


def domain_check(env):
    domain=read(DOMAIN);cfg=env['simulation_isolation']
    standard={'python','sam_python','sam_pythonpath','sam_checkpoint','sam_checkpoint_sha256',
        'sam_source_sha256','ffmpeg_dir','bundle','output_root','actor_lock','physics_gpu',
        'perception_gpu','simulation_isolation'}
    if set(env)-standard or cfg.get('schema')!=SCHEMA:
        raise ValueError('EXPLICIT_SIMULATION_ONLY_PROFILE_REQUIRED')
    if domain['execution']!='isaac_sim_only' or env['actor_lock']!=domain['actor_lock'] or env['python']!=domain['python']:
        raise ValueError('CANONICAL_SIMULATION_RESOURCE_DOMAIN_REQUIRED')
    if not Path(env['output_root']).resolve().is_relative_to(Path(domain['output_parent']).resolve()):
        raise ValueError('NEW_ISOLATED_OUTPUT_PARENT_REQUIRED')
    return domain


def global_and_history(env):
    """Revalidated under the shared lock and inside the child before physics."""
    domain=domain_check(env);lock=Path(domain['actor_lock'])
    for suffix in ('.global_stop.json','.hardware_stop.json'):
        if Path(str(lock)+suffix).exists():raise RuntimeError('GLOBAL_SAFETY_STOP:'+suffix)
    legacy=Path(str(lock)+'.safety_hold.json')
    reviews=env['simulation_isolation'].get('destroyed_instances',[])
    if len(reviews)>1:raise ValueError('ONE_EXPLICIT_LEGACY_INCIDENT_REVIEW_ONLY')
    if not legacy.exists():
        if reviews:raise RuntimeError('HISTORICAL_INCIDENT_EVIDENCE_MISSING')
        return []
    if len(reviews)!=1:raise RuntimeError('UNREVIEWED_GLOBAL_OR_LEGACY_HOLD')
    review=reviews[0]
    if str(legacy)!=review['hold_path'] or sha(legacy)!=review['hold_sha256']:
        raise RuntimeError('HISTORICAL_HOLD_CHANGED_OR_GLOBAL_STOP')
    hold=read(legacy)
    if hold.get('scope') not in (None,'INSTANCE') or hold.get('status')!='HARD_STOP_UNPARKED':
        raise RuntimeError('GLOBAL_OR_UNKNOWN_HOLD_CANNOT_BE_RESCOPED')
    old=Path(review['episode']).resolve()
    if hold.get('episode_id')!=old.name:raise ValueError('INCIDENT_EPISODE_MISMATCH')
    evidence=Path(hold['evidence'])
    if sha(evidence)!=review['audit_sha256']:raise ValueError('INCIDENT_AUDIT_CHANGED')
    audit=read(evidence)
    if not (audit.get('simulation_closed') is True and audit.get('safe_park_verified') is False
            and audit.get('status')=='HARD_STOP_UNPARKED' and audit.get('episode_id')==old.name):
        raise RuntimeError('DESTROYED_SIMULATION_EVIDENCE_REQUIRED')
    process=read(old/'PROCESS.json');completion=read(old/'PROCESS_COMPLETION.json')
    if sha(old/'PROCESS.json')!=review['process_sha256'] or sha(old/'PROCESS_COMPLETION.json')!=review['completion_sha256']:
        raise ValueError('INCIDENT_PROCESS_PROVENANCE_CHANGED')
    expected=[domain['python'],'-u','-m','agentic_interaction_v81.entry','--job',str(old/'job.json')]
    if process.get('command')!=expected or sha(old/'job.json')!=process['job_sha256']:
        raise RuntimeError('NON_NATIVE_OR_HARDWARE_ENTRY_NOT_ADMISSIBLE')
    if completion.get('actor_exited') is not True or identity(process['pid']) is not None:
        raise RuntimeError('OLD_CONTROLLER_NOT_PROVEN_TERMINATED')
    for pid in review['owned_pids']:
        if identity(pid) is not None:raise RuntimeError('OLD_OWNER_PID_EXISTS_OR_REUSED')
    for row in review['evidence_files']:
        f=(old/row['file']).resolve()
        if not f.is_relative_to(old) or sha(f)!=row['sha256']:
            raise ValueError('ORIGINAL_INCIDENT_EVIDENCE_CHANGED')
    return [dict(episode_id=old.name,status='NOT_PASS_SAFE_PARK_UNCONFIRMED',
        hold_path=str(legacy),hold_sha256=sha(legacy),scope='DESTROYED_SIMULATION_INSTANCE_ONLY',
        old_instance_resume_allowed=False,controller_stop_completed_verified=False,
        audit_sha256=sha(evidence),evidence_preserved=True)]


def no_live_owners(env):
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if p.stat().st_uid!=os.getuid():continue
            args=p.joinpath('cmdline').read_bytes().split(b'\0')
        except FileNotFoundError:continue
        if any(a in (b'agentic_interaction_v81.entry',b'agentic_interaction_v8.entry') for a in args):
            raise RuntimeError('LIVE_SIMULATION_CONTROLLER:'+p.name)
    result=subprocess.run(['nvidia-smi','--query-compute-apps=pid,process_name','--format=csv,noheader'],
        capture_output=True,text=True,check=True)
    for line in result.stdout.splitlines():
        if env['python'] in line:raise RuntimeError('OWNED_SIMULATION_GPU_PROCESS_REMAINS')


@contextmanager
def reserve(env,episode_id):
    """The SAME global resource mutex plus an independent UUID lease."""
    domain_check(env)
    if not episode_id.replace('_','').replace('-','').isalnum():
        raise ValueError('INVALID_NEW_EPISODE_ID')
    lock=Path(env['actor_lock'])
    with lock.open('a') as resource:
        fcntl.flock(resource,fcntl.LOCK_EX|fcntl.LOCK_NB)
        history=global_and_history(env);no_live_owners(env)
        if episode_id in {r['episode_id'] for r in history}:raise ValueError('OLD_INSTANCE_ID_CANNOT_RESUME')
        out=Path(env['output_root'])/episode_id
        if out.exists():raise ValueError('EPISODE_ALREADY_EXISTS_PRESERVE_RESULT')
        uid=str(uuid.uuid4());folder=Path(env['output_root']).parent/'instances'/uid
        folder.mkdir(parents=True,exist_ok=False)
        with (folder/'instance.lock').open('x') as lease:
            fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
            value=dict(schema=SCHEMA,uuid=uid,episode_id=episode_id,output=str(out),
                directory=str(folder),launcher=identity(os.getpid()),execution='isaac_sim_only',
                shared_actor_lock=env['actor_lock'],history=history,status='RESERVED',
                no_old_stage_or_controller_state=True)
            atomic_json(folder/'INSTANCE.json',value)
            instance=dict(value,resource=resource,environment=env,
                safety_registry=str(folder/'instance.safety_hold.json'))
            try:yield instance
            finally:
                value['status']='OWNER_CLOSED'
                if (out/'PROCESS_COMPLETION.json').exists():
                    value['process_completion']=read(out/'PROCESS_COMPLETION.json')
                atomic_json(folder/'INSTANCE.json',value)


def check_child(job,verify_history=True):
    """Direct child entry cannot substitute for ownership through the launcher."""
    folder=Path(job['instance_directory']);manifest=read(folder/'INSTANCE.json')
    if (manifest['uuid']!=job['instance_uuid'] or str(uuid.UUID(job['instance_uuid']))!=job['instance_uuid']
            or manifest['episode_id']!=job['episode_id'] or manifest['output']!=job['output']
            or manifest['status']!='RESERVED' or not manifest['launcher']
            or manifest['launcher']!=identity(os.getppid())):
        raise RuntimeError('INSTANCE_LAUNCH_OWNERSHIP_UNVERIFIED')
    for path in (Path(manifest['shared_actor_lock']),folder/'instance.lock'):
        with path.open('a') as f:
            try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:continue
            raise RuntimeError('REQUIRED_OWNER_LOCK_NOT_HELD')
    from .terminal_safety import require_clearance
    require_clearance(folder/'instance.safety_hold.json')
    if verify_history:global_and_history(job['instance_environment'])
    return manifest


def native_unloaded_check(r):
    """Reuses the original measured dwell and per-step supervisor, no new limits."""
    from .terminal_safety import persist
    from .policy_checks import settle_station
    from dataclasses import asdict
    import numpy as np
    folder=Path(r.job['instance_directory'])
    certificate=dict(instance_uuid=r.job['instance_uuid'],episode_id=r.job['episode_id'],passed=False)
    try:
        # Full incident provenance was checked before the child's no-history
        # audit hook. Online safety checks only use this fresh instance lease.
        check_child(r.job,verify_history=False)
        if not type(r.world).__module__.startswith('isaacsim.'):
            raise RuntimeError('ISAAC_WORLD_REQUIRED_NO_HARDWARE_ADMISSION')
        if r.supervisor.latched is not None or r.grasp_reference is not None:
            raise RuntimeError('FRESH_UNLOADED_CONTROLLER_REQUIRED')
        r.g['phase']='SETTLE';r.world.play()
        r.hold(.35)  # original hold/force/collision/velocity supervisor every step
        boundary=settle_station(r);boundary.check()
        if not r.rows or not all(x['wrist_ft']['valid'] for x in r.rows):
            raise RuntimeError('VALID_INITIAL_WRIST_FT_REQUIRED')
        q=np.asarray(r.robot.get_joint_positions())
        if not np.isfinite(q).all():raise RuntimeError('FINITE_INITIAL_JOINT_STATE_REQUIRED')
        ok,why=r.collision.check(r.model.poses(q[r.arm],r.base,finger_q=q[r.fingers]),r.moving(),False)
        if not ok:raise RuntimeError('INITIAL_NATIVE_COLLISION:'+str(why))
        certificate.update(passed=True,boundary=asdict(boundary),
            native_steps=r.tick,force_history_settled=bool(r.supervisor.force.settled()),
            pad_N=r.filtered().tolist(),FT_valid=True,original_supervisor_active=True,
            old_controller_state_loaded=False,selected_object_coordinate=r.actual())
    except BaseException as error:
        certificate['error']=repr(error)
        atomic_json(folder/'UNLOADED_ADMISSION.json',certificate)
        atomic_json(r.output/'UNLOADED_ADMISSION.json',certificate)
        persist(r,dict(safe_exit=dict(safe=False,reason='INSTANCE_UNLOADED_ADMISSION_FAILED')),pause=True)
        r.record.flush();r.record.queue.join();r.close()
        raise
    atomic_json(folder/'UNLOADED_ADMISSION.json',certificate)
    atomic_json(r.output/'UNLOADED_ADMISSION.json',certificate)
    r.world.pause()
    return certificate

