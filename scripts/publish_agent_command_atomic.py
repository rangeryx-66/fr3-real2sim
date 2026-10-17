"""Atomically publish an Agent file command; no robot/controller dependency.

Upload input to a staging path OUTSIDE agent_commands, then invoke this on the
same host/filesystem. Never SCP directly to a visible NNN.json command name.
This transport repair is offline-verified after the V1.1 episode; no additional
physical episode is implied by its tests.
"""
import argparse,json,os,tempfile
from pathlib import Path


def publish(command_directory,sequence,command):
    directory=Path(command_directory);directory.mkdir(parents=True,exist_ok=True)
    encoded=json.dumps(command,ensure_ascii=False,indent=2).encode('utf-8')
    # Verify serialization before making any consumer-visible command file.
    json.loads(encoded);final=directory/('%03d.json'%sequence)
    if final.exists():raise FileExistsError('Existing command is preserved: '+str(final))
    fd,name=tempfile.mkstemp(prefix='.%03d-'%sequence,suffix='.pending',dir=directory)
    temporary=Path(name)
    try:
        with os.fdopen(fd,'wb') as stream:
            stream.write(encoded);stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,final)
        return dict(published=str(final),bytes=len(encoded),atomic_same_filesystem_rename=True)
    finally:
        temporary.unlink(missing_ok=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,required=True);p.add_argument('--sequence',type=int,required=True);p.add_argument('--input',type=Path,required=True);a=p.parse_args()
    print(json.dumps(publish(a.directory,a.sequence,json.loads(a.input.read_text()))))
