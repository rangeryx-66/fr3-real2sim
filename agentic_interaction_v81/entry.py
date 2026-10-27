"""Explicit V8.1 development actor; frozen low-level execution is reused."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path);a=p.parse_args()
    job=json.loads(a.job.read_text())
    if job.get('instance_uuid'):
        from .instance_admission import check_child
        check_child(job)
    if job.get('v81_phase') not in ('engineering','independent_repetition'):
        raise ValueError('EXPLICIT_V81_PHASE_REQUIRED')
    if job.get('v81_phase')=='independent_repetition' and not job.get('v81_frozen_release_commit'):
        raise ValueError('ENGINEERING_READINESS_AND_FROZEN_RELEASE_REQUIRED')
    from .preflight import baseline_audit
    baseline_audit()
    from agentic_interaction_v8.cli import FREEZE
    excluded=json.loads(FREEZE.read_text())['execution_directory_snapshot']
    forbidden=tuple(job['v8_forbidden_history_roots'])
    def prevent_history(event,args):
        if event=='open' and isinstance(args[0],(str,bytes)) and str(args[0]).startswith(forbidden):
            raise RuntimeError('UNDECLARED_HISTORICAL_IO:'+str(args[0]))
        if event=='exec':
            name=Path(args[0].co_filename)
            if name.is_absolute() and name.is_relative_to(ROOT) and str(name.relative_to(ROOT)) in excluded:
                raise RuntimeError('UNFROZEN_LEGACY_EXECUTION:'+str(name))
    sys.addaudithook(prevent_history)
    import xarm7_native_gripper
    xarm7_native_gripper.OUT=Path(job['v8_robot_io'])
    from .runtime import runtime_class
    if job.get('v87_continuous_recovery'):
        from agentic_interaction_v87.runtime import runtime_class as govern
        existing_runtime_class=runtime_class
        runtime_class=lambda parent:govern(existing_runtime_class(parent))
    if job.get('v88_coordination_policy'):
        from agentic_interaction_v88.runtime import runtime_class as coordinate
        governed_runtime_class=runtime_class
        runtime_class=lambda parent:coordinate(governed_runtime_class(parent))
    from .session import serve
    if job.get('v84_abc'):
        from agentic_interaction_v84.session import serve
    if 'v81_contact_comparison' in job:
        if job['v81_phase']!='engineering':raise ValueError('ENGINEERING_COMPARISON_CANNOT_RUN_AS_REPETITION')
        from .contact_fixture import serve
    from interaction_runtime.probing_v7 import RuntimeV7
    from interaction_runtime.prismatic_v751 import PrismaticRuntimeV751
    from run_efficient_probing_v2 import main as run
    from agentic_interaction_v8.entry import finish_before_shutdown
    from agentic_interaction_v8.resources import digest,dump
    def sources():
        files={}
        for module in list(sys.modules.values()):
            name=getattr(module,'__file__',None)
            if name:
                f=Path(name)
                if f.suffix=='.py' and f.is_file() and f.is_relative_to(ROOT):
                    files[str(f.relative_to(ROOT))]=digest(f)
        dump(Path(job['output'])/'V81_LOADED_SOURCES.json',dict(sources=files,
            baseline='agentic-interaction-v8.0.0',phase=job['v81_phase']))
    from functools import partial
    owner=PrismaticRuntimeV751 if job['articulation_v7']['joint_type']=='prismatic' else RuntimeV7
    return run(runtime_class(owner),partial(finish_before_shutdown,serve=serve,finalize=sources),
        extra_sources=tuple(str(f.relative_to(ROOT)) for folder in ('agentic_interaction_v81','agentic_interaction_v84','agentic_interaction_v87','agentic_interaction_v88') for f in sorted((ROOT/folder).glob('*.py')))+(
        'scripts/xarm_ft_v7_scene.py','scripts/xarm_ft_v7_estimator.py','scripts/update_probing_v7_model.py',
        'scripts/update_prismatic_v73_model.py','scripts/xarm_ft_v31_estimator.py'))


if __name__=='__main__':sys.exit(main())
