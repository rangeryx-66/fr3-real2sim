"""One short diagnostic using the unchanged native grasp/Pink/shared controller.
Only the task endpoint and reporting labels differ; frozen sources stay intact.
"""
from pathlib import Path
import sys,json,hashlib
from xarm7_native_execution import executable

def short_source(spec):
    text=executable(spec)
    text=text.replace("goal=float(whole['goal_state'])","goal=float(np.deg2rad(spec['short_target_deg']))")
    anchor="    grasp_relative=np.linalg.inv(moving())@tcp();reference=grasp_relative.copy();ref=tcp().copy()"
    # Match the validated grasp-only prefix's extra 0.5 s hold. Original
    # initial sequence already contains its own 0.5 s post-verification hold.
    extra="""    phase='VALIDATED_PREFIX_EXTRA_HOLD';hold(.5)
    opening_start_angle=actual();goal=opening_start_angle+float(np.deg2rad(spec['short_target_deg']))
    try:telemetry_events.append({'event':'SHORT_OPENING_START','t':tick*dt,'angle_deg':float(np.rad2deg(opening_start_angle)),'target_deg':float(np.rad2deg(goal)),'filtered_pad_loads_n':filtered().tolist(),'hold_prefix_extra_s':.5})
    except Exception as error:diagnostic_issue('short_open_start_observer',error)
"""
    if text.count(anchor)!=1:raise ValueError('PRELAUNCH_SHORT_OPEN_ANCHOR')
    text=text.replace(anchor,extra+anchor)
    endpoint="    opening=False;qvelocity=np.zeros(7);phase='FINAL_HOLD' if success else 'TRANSMISSION_FAILURE_SAFE_HOLD'"
    text=text.replace(endpoint,"    original_shared_return_status=status\n    if success:status='SHORT_OPENING_TARGET_REACHED'\n"+endpoint)
    marker="diagnostic_write(a.output/'report.json',json.dumps(result,indent=2))"
    report="""result.update(classification='XARM7_NATIVE_G1 / ORACLE_HANDLE / KNOWN_MODEL / SHORT_FORCE_TRANSMISSION',target_deg=float(np.rad2deg(goal)),requested_increment_deg=float(spec['short_target_deg']),full_30_degree_task_success_claim=False,original_shared_return_status=globals().get('original_shared_return_status'),opening_start_angle_deg=float(np.rad2deg(opening_start_angle)) if 'opening_start_angle' in globals() else None,selected_plan_unchanged=True)
"""
    return text.replace(marker,report+marker)

if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
    text=short_source(spec);compile(text,__file__+'::unchanged_short_opening','exec')
    output=Path(spec['output']);output.mkdir(parents=True,exist_ok=True)
    (output/'ACTUAL_SHORT_OPEN_EXECUTABLE.py').write_text(text)
    (output/'SHORT_EXECUTABLE_HASH.json').write_text(json.dumps({'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'generated_sha256':hashlib.sha256(text.encode()).hexdigest(),'controller_changed':False,'new_runtime_gates':False}))
    exec(compile(text,__file__+'::unchanged_short_opening','exec'),globals())
