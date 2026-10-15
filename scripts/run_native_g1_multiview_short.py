"""Thin multi-view source adapter; no closure or opening-control changes."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_rgbd_short import visual_source

def multiview_source(spec):
    text=visual_source(spec)
    return text.replace('from native_g1_visual_input import prepare,refine',
                        'from native_g1_multiview_input import prepare,refine')

if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
    text=multiview_source(spec);compile(text,__file__+'::unchanged_physical_skill','exec')
    out=Path(spec['output']);out.mkdir(parents=True,exist_ok=True)
    (out/'ACTUAL_VISUAL_EXECUTABLE.py').write_text(text)
    sources=[Path(__file__),Path(__file__).with_name('native_g1_multiview_input.py'),Path(__file__).with_name('native_g1_metric_rgbd_geometry.py')]
    records=[]
    for p in sources:
        (out/p.name).write_bytes(p.read_bytes());records.append(dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    (out/'MULTIVIEW_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=records,generated_sha256=hashlib.sha256(text.encode()).hexdigest(),new_execution_gates=False),indent=2))
    exec(compile(text,__file__+'::unchanged_physical_skill','exec'),globals())
