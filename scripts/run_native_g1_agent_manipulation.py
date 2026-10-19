"""Thin approved handoff to the frozen native-G1 short manipulation runner."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_wrist_short import wrist_source

def source(spec):
    text=wrist_source(spec)
    old='from native_g1_wrist_input import prepare,refine'
    if text.count(old)!=1:raise ValueError('Pinned observation boundary changed; no scene started')
    return text.replace(old,'from native_g1_agent_execution import prepare\n    from native_g1_wrist_input import refine')

if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());text=source(spec)
    compile(text,__file__+'::approved_frozen_skill','exec')
    out=Path(spec['output']);out.mkdir(parents=True,exist_ok=True);(out/'ACTUAL_AGENT_EXECUTABLE.py').write_text(text)
    files=['run_native_g1_agent_manipulation.py','native_g1_agent_execution.py','native_g1_agent_observation.py','native_g1_agent_inspection.py','infer_sam3_agent_observed.py','native_g1_wrist_input.py','native_g1_wrist_views.py','native_g1_multiview_input.py','native_g1_metric_rgbd_geometry.py','native_g1_visual_input.py','precontact_trajectory.py','shared_door_execution.py','xarm7_native_execution.py']
    hashes={}
    for name in files:
        p=Path(__file__).with_name(name);hashes[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest();(out/(name+'.snapshot')).write_bytes(p.read_bytes())
    (out/'AGENT_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=hashes,generated_sha256=hashlib.sha256(text.encode()).hexdigest(),control_scoring_physics_changes=False),indent=2))
    exec(compile(text,__file__+'::approved_frozen_skill','exec'),globals())
