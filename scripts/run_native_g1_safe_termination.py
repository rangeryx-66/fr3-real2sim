"""Pinned V1 owner + one explicit V1.1 safe-exit operation dependency."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_contact_recovery import source as frozen_source


def source(job):
    text=frozen_source(job)
    old='from native_g1_contact_recovery import run as run_contact_recovery'
    if text.count(old)!=1:raise ValueError('Pinned explicit owner import changed; scene not initialized')
    text=text.replace(old,'from native_g1_safe_termination import run as run_contact_recovery')
    return text.replace("classification='CONTACT_FEEDBACK_RECOVERY_V1 /", "classification='CONTACT_FEEDBACK_RECOVERY_V1.1_SAFE_PARK /")


if __name__=='__main__':
    job=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());text=source(job)
    compile(text,__file__+'::explicit_safe_exit_owner','exec')
    out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    (out/'ACTUAL_RECOVERY_EXECUTABLE.py').write_text(text)
    hashes={}
    for name in job['activation_source_files']:
        path=Path(__file__).resolve().parents[1]/name;hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest()
        (out/(path.name+'.snapshot')).write_bytes(path.read_bytes())
    (out/'RECOVERY_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=hashes,generated_sha256=hashlib.sha256(text.encode()).hexdigest(),controllers_unchanged=True,exit_version='V1.1',maximum_grasps=2),indent=2))
    exec(compile(text,__file__+'::explicit_safe_exit_owner','exec'),globals())
