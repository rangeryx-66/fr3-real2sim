"""Pinned observation/planning-only activation; no execute/grasp operation."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_wrist_short import wrist_source

if __name__=='__main__':
    job=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
    text=wrist_source(job);hook='from native_g1_wrist_input import prepare,refine'
    if text.count(hook)!=1:raise ValueError('Pinned observation entry hook changed; no actor initialized')
    text=text.replace(hook,'from native_g1_agent_incremental_activation import prepare\n    from native_g1_wrist_input import refine')
    compile(text,__file__+'::observation_only','exec')
    out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    (out/'ACTUAL_AGENT_EXECUTABLE.py').write_text(text)
    sources={}
    for name in job['activation_source_files']:
        p=Path(__file__).resolve().parents[1]/name
        sources[name]=hashlib.sha256(p.read_bytes()).hexdigest()
        (out/(p.name+'.snapshot')).write_bytes(p.read_bytes())
    (out/'AGENT_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=sources,
        generated_sha256=hashlib.sha256(text.encode()).hexdigest(),
        observation_only=True,controller_and_score_changes=False),indent=2))
    exec(compile(text,__file__+'::observation_only','exec'),globals())
