"""Incremental wrist RGB-D with the frozen explicitly approved physical skill."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_wrist_short import wrist_source


def source(job):
    text=wrist_source(job);hook='from native_g1_wrist_input import prepare,refine'
    if text.count(hook)!=1:raise ValueError('Pinned observation boundary changed; no scene initialized')
    return text.replace(hook,'from native_g1_agent_incremental_physical import prepare\n    from native_g1_wrist_input import refine')


if __name__=='__main__':
    job=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());text=source(job)
    compile(text,__file__+'::incremental_frozen_physical','exec')
    out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    (out/'ACTUAL_AGENT_EXECUTABLE.py').write_text(text);hashes={}
    for name in job['activation_source_files']:
        path=Path(__file__).resolve().parents[1]/name
        hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest()
        (out/(path.name+'.snapshot')).write_bytes(path.read_bytes())
    (out/'AGENT_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=hashes,
        generated_sha256=hashlib.sha256(text.encode()).hexdigest(),
        explicit_Agent_execute_required=True,incremental_geometry=True,
        controller_and_score_changes=False),indent=2))
    exec(compile(text,__file__+'::incremental_frozen_physical','exec'),globals())
