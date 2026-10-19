"""One observation-and-planning-only runner, preserving frozen control sources."""
import hashlib
import json
import sys
from pathlib import Path
from run_native_g1_wrist_short import wrist_source


if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
    text=wrist_source(spec).replace('from native_g1_wrist_input import prepare,refine',
        'from native_g1_agent_incremental import prepare\n    from native_g1_wrist_input import refine')
    compile(text,__file__+'::incremental_observation_only','exec')
    out=Path(spec['output']);out.mkdir(parents=True,exist_ok=True)
    (out/'ACTUAL_AGENT_EXECUTABLE.py').write_text(text)
    sources={}
    for name in ['run_native_g1_agent_incremental.py','native_g1_agent_incremental.py',
                 'native_g1_incremental_geometry.py','native_g1_metric_rgbd_geometry.py',
                 'native_g1_agent_observation.py','native_g1_agent_inspection.py',
                 'infer_sam3_agent_observed.py','native_g1_wrist_input.py',
                 'native_g1_wrist_views.py','native_g1_multiview_input.py']:
        p=Path(__file__).with_name(name)
        sources[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest()
        (out/(name+'.snapshot')).write_bytes(p.read_bytes())
    (out/'AGENT_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=sources,
        generated_sha256=hashlib.sha256(text.encode()).hexdigest(),
        control_and_scoring_changes=False,physical_grasp_execution=False),indent=2))
    exec(compile(text,__file__+'::incremental_observation_only','exec'),globals())
