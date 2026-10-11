"""Reuse the frozen physical runner. Retain paused scenes until owner cleanup.

Only the authored task goal supplied by the plan differs. No controller, contact,
force, velocity, collision or joint-margin checks are changed.
"""
from pathlib import Path

shared=Path(__file__).with_name('run_revolute10_fixed.py')
code=shared.read_text()
code=code.replace('AUTHORED_TARGET_REACHED_SINGLE_GRASP','BENCHMARK_A_30_DEG_SUCCESS')
code=code.replace('\napp.close()',"""
diagnostic_write(a.output/'scene_preserved_after_result.json',json.dumps({
    'pid':os.getpid(),'world_paused':True,'status':status,
    'retained_until_explicit_owner_cleanup':True},indent=2))
while app.is_running():
    if (a.output/'close_completed_scene').exists():
        app.close();break
    app.update();time.sleep(.1)
""",1)
exec(compile(code,str(shared)+'::benchmark_scene_owner','exec'),globals())
