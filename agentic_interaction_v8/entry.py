"""Small explicit production-owner wiring. No frozen controller edits."""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path)
    args=p.parse_args();job=json.loads(args.job.read_text())
    if job.get('package_version')!='8.0.0':raise ValueError('V8_JOB_REQUIRED')
    from .cli import audit,FREEZE
    audit()
    excluded=json.loads(FREEZE.read_text())['execution_directory_snapshot']
    forbidden=tuple(job['v8_forbidden_history_roots'])
    def prevent_legacy_import(event,args):
        if event=='open' and isinstance(args[0],(str,bytes)):
            path=str(args[0])
            if path.startswith(forbidden):raise RuntimeError('UNDECLARED_HISTORICAL_IO:'+path)
        if event=='exec':
            filename=Path(args[0].co_filename)
            if filename.is_absolute() and filename.is_relative_to(ROOT):
                if str(filename.relative_to(ROOT)) in excluded:
                    raise RuntimeError('UNFROZEN_LEGACY_EXECUTION:'+str(filename))
    sys.addaudithook(prevent_legacy_import)
    # IO locator is bound before any planning module imports OUT by value.
    # All geometry/controller classes/functions retain original source bytes.
    import xarm7_native_gripper
    xarm7_native_gripper.OUT=Path(job['v8_robot_io'])
    from run_efficient_probing_v2 import main as run
    from run_angle_resistance_v5 import SOURCES
    common=SOURCES+('scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py')
    if job['articulation_v7']['joint_type']=='revolute':
        from interaction_runtime.probing_v7 import RuntimeV7
        from interaction_runtime.registered_observation_v71 import serve_registered
        return run(RuntimeV7,serve_registered,extra_sources=common)
    from interaction_runtime.prismatic_v751 import PrismaticRuntimeV751
    from interaction_runtime.prismatic_radial_session_v751 import serve_radial_v751
    return run(PrismaticRuntimeV751,serve_radial_v751,extra_sources=common+(
        'scripts/native_g1_radial_families_v751.py','scripts/prismatic_visual_v73.py',
        'scripts/prismatic_quality_v73.py','scripts/update_prismatic_v73_model.py'))


if __name__=='__main__':sys.exit(main())
