"""Single shared closure-continuity repair; all physical protections unchanged.

During unilateral centering retain the last position command instead of copying
back the contact-deflected measured aperture and reopening/unloading the jaws.
The frozen closure file, gains, force limits, thresholds and timing are untouched.
"""
from pathlib import Path
shared=Path(__file__).with_name('run_revolute10_fixed.py')
code=shared.read_text().replace("'classification':'KNOWN_MODEL_DIAGNOSTIC'", "'classification':'ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT / CLOSURE_CONTINUITY_RETEST'")
code=code.replace("original_step=step;grasp_relative=None", """import inspect
closure_source=inspect.getsource(JawCenteredClosure)
closure_source=closure_source.replace('self.opening=float(opening) # pause further squeeze',
                                    'self.opening=min(self.opening,float(opening)) # retain closing command while centering')
exec(compile(closure_source,'JawCenteredClosure::retain_position_command','exec'),globals())
original_step=step;grasp_relative=None""",1)
code=code.replace('AUTHORED_TARGET_REACHED_SINGLE_GRASP','ORACLE_HANDLE_30_DEG_SUCCESS')
code=code.replace("phase='FINAL_HOLD';hold(.5)","phase='FINAL_HOLD'",1)
code=code.replace('\napp.close()',"""
diagnostic_write(a.output/'scene_preserved_after_result.json',json.dumps({
    'pid':os.getpid(),'world_paused':True,'status':status,
    'retained_until_explicit_owner_cleanup':True},indent=2))
while app.is_running():
    if (a.output/'close_completed_scene').exists():
        app.close();break
    app.update();time.sleep(.1)
""",1)
exec(compile(code,str(shared)+'::closure_continuity_development','exec'),globals())
