"""Thin RGB-D source adapter around the frozen one-second-hold/short runner."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_short_open import short_source

def visual_source(spec):
    text=short_source(spec)
    text=text.replace('text=adapt_baseline(text,spec)',"text=adapt_baseline(text,spec)\nfrom native_g1_visual_input import adapt_setup\ntext=adapt_setup(text)")
    anchor="    exec(compile(initial,str(baseline)+'::unchanged_initial_sequence','exec'),globals())"
    insert="""    from native_g1_visual_input import prepare,refine
    prepare(globals())
    initial=initial.replace("move(qpre,.8);phase='APPROACH'", "move(qpre,.8);refine(globals());phase='APPROACH'")
"""
    if text.count(anchor)!=1:raise ValueError('PRELAUNCH_VISUAL_SOURCE_HOOK')
    text=text.replace(anchor,insert+anchor)
    text=text.replace("classification='XARM7_NATIVE_G1 / ORACLE_HANDLE / KNOWN_MODEL / SHORT_FORCE_TRANSMISSION'", "classification='XARM7_NATIVE_G1 / RGBD_SAM3_HANDLE / KNOWN_HINGE_STATE_COLLISION / SHORT_FORCE_TRANSMISSION'")
    text=text.replace("selected_plan_unchanged=True", "selected_plan_unchanged=False,GT_grasp_fallback=False,hold_s=1.,capture_shadow_only=spec.get('capture_shadow_only',False),GT_dependencies=['registered station','known hinge/dynamic state','robot calibration','unchanged scene collisions and native contact IDs']")
    return text

if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());text=visual_source(spec);compile(text,__file__+'::frozen_controller_visual_input','exec')
    output=Path(spec['output']);output.mkdir(parents=True,exist_ok=True)
    (output/'ACTUAL_VISUAL_EXECUTABLE.py').write_text(text)
    adapter=Path(__file__).with_name('native_g1_visual_input.py');(output/'native_g1_visual_input_snapshot.py').write_text(adapter.read_text())
    (output/'visual_adapter_provenance.json').write_text(json.dumps(dict(path=str(adapter.resolve()),sha256=hashlib.sha256(adapter.read_bytes()).hexdigest(),estimator_and_ranking_source='RGB-D plus declared priors, not GT handle meshes'),indent=2))
    (output/'VISUAL_EXECUTABLE_HASH.json').write_text(json.dumps(dict(runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),generated_sha256=hashlib.sha256(text.encode()).hexdigest(),frozen_closure_and_opening_unchanged=True,new_quality_gates=False),indent=2))
    exec(compile(text,__file__+'::frozen_controller_visual_input','exec'),globals())
