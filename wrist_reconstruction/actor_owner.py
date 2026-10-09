"""One actor per object across new outputs and the legacy continuation entry."""
import fcntl,json,os
from pathlib import Path


def claim(object_id,job,root):
    lock=Path('/tmp')/('piper_wrist_actor_'+str(object_id)+'.lock')
    owner=lock.open('a+')
    try:fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        owner.close();raise RuntimeError('ACTOR_ALREADY_OWNED:'+str(object_id))
    # Legacy actors may not yet use the lease. Inspect their actual job and
    # process state before any output/job file can be overwritten.
    proc=Path('/proc')
    if proc.exists():
        for process in proc.iterdir():
            if not process.name.isdigit() or int(process.name)==os.getpid():continue
            try:
                argv=(process/'cmdline').read_bytes().decode().split('\0')
                if not any('run_wrist_reconstruction_episode.py' in a for a in argv):continue
                if (process/'stat').read_text().split(') ',1)[1].split()[0]=='Z':continue
                path=Path(argv[argv.index('--job')+1]);doc=json.loads(path.read_text())
                obj=doc.get('object_id',doc.get('asset_id',doc.get('episode_id','')))
                if str(object_id) in str(obj) or str(object_id) in path.name or str(object_id) in str(path.parent):
                    owner.close();raise RuntimeError(f'ACTOR_ALREADY_LIVE:{object_id}:pid={process.name}:job={path}')
            except (OSError,ValueError,IndexError,KeyError):continue
    owner.seek(0);owner.truncate();owner.write(json.dumps({'object':str(object_id),'job':str(job),'supervisor_pid':os.getpid()}));owner.flush()
    return owner
