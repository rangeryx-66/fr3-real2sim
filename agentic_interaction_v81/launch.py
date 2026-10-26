"""Versioned V8.1 engineering launcher; no full benchmark is enabled."""
import argparse,fcntl,json,os,subprocess,sys,time
from contextlib import nullcontext
from pathlib import Path
from agentic_interaction_v8.cli import prepare
from agentic_interaction_v8.resources import digest,dump
from .preflight import baseline_audit

ROOT=Path(__file__).resolve().parents[1]


def launch(specfile,profile):
    env=json.loads(Path(profile).read_text())
    if 'simulation_isolation' not in env:return _launch(specfile,profile)
    from .instance_admission import reserve
    spec=json.loads(Path(specfile).read_text())
    with reserve(env,spec['episode_id']) as instance:
        return _launch(specfile,profile,instance)


def _launch(specfile,profile,instance=None):
    baseline_audit()
    from .terminal_safety import registry_path,require_clearance,process_outcome,preserve_unverified_process_hold
    safety_registry=(Path(instance['safety_registry']) if instance else
        registry_path(json.loads(Path(profile).read_text())))
    require_clearance(safety_registry)
    spec=json.loads(Path(specfile).read_text())
    allowed={'schema','episode_id','phase','scene','physics_file','physics_sha256','identification_case',
             'wall_budget_s','sim_budget_s','acquisition_budget','agent_wait_s','frozen_release_commit','contact_comparison','debug_budget_profile','path_rejection_fixture',
             'integration_profile','preload_study','orchestration_authorization','controlled_interruption_fixture','graspgenx'}
    if set(spec)-allowed or spec.get('schema')!='agentic-interaction/v81':raise ValueError('STRICT_V81_SPEC_REQUIRED')
    phase=spec.get('phase')
    from .integration import apply as integrated_profile
    integrated_profile(spec,{})
    if 'path_rejection_fixture' in spec and (phase!='engineering' or spec['path_rejection_fixture'] is not True):
        raise ValueError('EXPLICIT_ENGINEERING_REJECTION_FIXTURE_ONLY')
    if phase not in ('engineering','independent_repetition'):raise ValueError('V81_DEVELOPMENT_PHASE_REQUIRED')
    comparison=spec.get('contact_comparison')
    if comparison is not None:
        if phase!='engineering':raise ValueError('MATCHED_COMPARISON_CANNOT_BE_AN_INDEPENDENT_REPETITION')
        from .contact_fixture import validate
        validate(comparison)
        if comparison['kind']=='prismatic_preload_extension_v81' and not spec.get('preload_study'):
            raise ValueError('CONTROLLED_PRELOAD_STUDY_REGISTRATION_REQUIRED')
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if phase=='independent_repetition':
        gate=ROOT/'docs/experiments/skill_v81/ENGINEERING_READINESS.json'
        if not gate.exists():raise ValueError('ENGINEERING_READINESS_NOT_ESTABLISHED')
        readiness=json.loads(gate.read_text())
        if not readiness.get('revolute_complete') or not readiness.get('prismatic_complete'):
            raise ValueError('BOTH_ENGINEERING_LIFECYCLES_REQUIRED_BEFORE_REPETITIONS')
        if not spec.get('frozen_release_commit') or spec['frozen_release_commit']!=commit:
            raise ValueError('REPETITION_SOURCE_NOT_FROZEN_RELEASE')
    budget=int(spec.get('acquisition_budget',6))
    if not 1<=budget<=6:raise ValueError('FINITE_V81_ACQUISITION_BUDGET_REQUIRED')
    # V8 prepare remains intact as the exact resource/IO/plant-commitment layer.
    # Its three-acquisition smoke ceiling is a compatibility input only. The
    # separately authorized V8.1 runtime budget is explicit in the new record.
    from .debug_budget import apply as debug_budget
    debug_budget(spec,{})  # reject inconsistent resources before creating output
    compat={k:v for k,v in spec.items() if k not in ('phase','frozen_release_commit','contact_comparison','debug_budget_profile','path_rejection_fixture','integration_profile','preload_study','orchestration_authorization','controlled_interruption_fixture','graspgenx')}
    compat.update(schema='agentic-interaction/v8',purpose='engineering',acquisition_budget=min(budget,3))
    compat_path=Path(specfile).with_name(Path(specfile).stem+'_V8_RESOURCE_COMPAT.json')
    dump(compat_path,compat);out,p=prepare(compat_path,profile)
    job=json.loads((out/'job.json').read_text())
    job['terminal_safety_registry']=str(safety_registry)
    if instance:
        job.update(instance_uuid=instance['uuid'],instance_directory=instance['directory'],
            instance_environment=instance['environment'])
    debug_budget(spec,job)
    integrated_profile(spec,job)
    job.update(v81_phase=phase,v81_frozen_release_commit=spec.get('frozen_release_commit'),
        runtime_acquisition_budget=budget,maximum_agent_operations=100,research_reference='agentic-interaction-v8.0.0')
    from .orchestration_extension import FLAG, KEY_ENV, wait_actor
    ledger_key=None
    if job.get(FLAG):
        import secrets
        ledger_key=secrets.token_hex(32)
        job.update(v83_orchestration_source_commit=commit,
                   v83_orchestration_session_nonce=secrets.token_hex(16))
    if spec.get('path_rejection_fixture'):job['v81_path_rejection_fixture']=True
    if comparison is not None:
        if job['articulation_v7']['joint_type']!='prismatic':raise ValueError('MATCHED_COMPARISON_REQUIRES_PRISMATIC')
        job.update(v81_contact_comparison=comparison,
                   native_g1_preload_request_n=float(comparison['requested_preload_n']))
    # Domain is an explicit kinematic prior, not a changed physical parameter or
    # truth-selected fit coefficient. Prevent the old 45-degree domain from
    # producing NaNs while the actual cabinet progresses towards 90 degrees.
    limits=job['articulation_v7']['limits_native'];old_domain=job['estimation_v7']['domain_native']
    job['estimation_v7']['domain_native']=[min(old_domain[0],limits[0]),max(old_domain[1],limits[1])]
    dump(out/'job.json',job)
    sources={str(f.relative_to(ROOT)):digest(f) for folder in ('agentic_interaction_v81','agentic_interaction_v84','agentic_interaction_v87') for f in sorted((ROOT/folder).glob('*.py'))}
    sources['agentic_interaction_v81/PRELOAD_EVIDENCE_POLICY.json']=digest(ROOT/'agentic_interaction_v81/PRELOAD_EVIDENCE_POLICY.json')
    dump(out/'V81_PRELAUNCH.json',dict(spec=spec,spec_sha256=digest(specfile),job_sha256=digest(out/'job.json'),
        source_commit=commit,source_sha256=sources,baseline_audit=baseline_audit(),
        effective_resources={k:job[k]for k in ('episode_wall_budget_s','episode_sim_budget_s','agent_wait_timeout_s','safe_exit_wall_reserve_s','safe_exit_sim_reserve_s')},
        simulation_mount='existing-authority XY carriage adapted to xArm; not wheeled hardware',
        estimator_domain_prior_only=dict(previous=old_domain,current=job['estimation_v7']['domain_native']),
        original_controller_and_safety_sources_unchanged=True))
    with (nullcontext(instance['resource']) if instance else Path(p['actor_lock']).open('a')) as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require_clearance(safety_registry)  # recheck after lock; prior actor may have faulted
        if instance:
            from .instance_admission import global_and_history
            global_and_history(instance['environment'])
        free={int(x.split(',')[0]):int(x.split(',')[1]) for x in subprocess.check_output(
            ['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()}
        if free[int(p['physics_gpu'])]<12000:raise ValueError('INSUFFICIENT_SHARED_GPU_CAPACITY')
        env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',
            OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),
            PATH=p['ffmpeg_dir']+':'+os.environ['PATH'])
        if instance:
            cache=Path(instance['directory'])/'cache';cache.mkdir()
            env.update(TMPDIR=str(cache),XDG_CACHE_HOME=str(cache),
                       MPLCONFIGDIR=str(cache/'matplotlib'))
        if ledger_key is not None:env[KEY_ENV]=ledger_key
        command=[p['python'],'-u','-m','agentic_interaction_v81.entry','--job',str(out/'job.json')]
        began=time.perf_counter()
        with (out/'actor.log').open('wb') as log:
            actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            dump(out/'PROCESS.json',dict(pid=actor.pid,command=command,checkout=str(ROOT),free_VRAM_MiB=free,
                source_commit=commit,job_sha256=digest(out/'job.json')))
            try:rc=wait_actor(actor,job,out,ledger_key)
            except subprocess.TimeoutExpired:
                (out/'STOP_AND_EXPORT').touch()
                try:rc=actor.wait(timeout=90)
                except subprocess.TimeoutExpired:
                    actor.terminate()
                    try:rc=actor.wait(timeout=30)
                    except subprocess.TimeoutExpired:actor.kill();rc=actor.wait(timeout=10)
        physical=(preserve_unverified_process_hold(out,rc,safety_registry) if job.get('v84_abc')
            else process_outcome(out,rc))
        dump(out/'PROCESS_COMPLETION.json',dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True,
            physical_outcome=physical))
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--spec',required=True,type=Path)
    p.add_argument('--environment',required=True,type=Path);a=p.parse_args();print(launch(a.spec,a.environment))
