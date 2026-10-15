"""Thin sensor-source entry point; frozen closure/opening code unchanged."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_rgbd_short import visual_source

def wrist_source(spec):
    text=visual_source(spec)
    text=text.replace('from native_g1_visual_input import prepare,refine','from native_g1_wrist_input import prepare,refine')
    return text

if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());text=wrist_source(spec)
    compile(text,__file__+'::rigid_wrist_sensor_frozen_control','exec')
    out=Path(spec['output']);out.mkdir(parents=True,exist_ok=True)
    (out/'ACTUAL_WRIST_EXECUTABLE.py').write_text(text)
    sources={}
    for name in ['run_native_g1_wrist_short.py','native_g1_wrist_input.py','native_g1_wrist_views.py','native_g1_multiview_input.py','native_g1_visual_input.py','precontact_trajectory.py','shared_door_execution.py','xarm7_native_execution.py']:
        p=Path(__file__).with_name(name);sources[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest()
        (out/(name+'.snapshot')).write_bytes(p.read_bytes())
    (out/'WRIST_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=sources,generated_sha256=hashlib.sha256(text.encode()).hexdigest(),no_control_or_scoring_changes=True),indent=2))
    exec(compile(text,__file__+'::rigid_wrist_sensor_frozen_control','exec'),globals())
