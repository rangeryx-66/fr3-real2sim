"""V8 package CLI: reproducible IO, provenance, lifecycle and Agent transport."""
import argparse
import fcntl
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from .resources import digest, dump, export_bundle, materialize, verify_bundle

ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT/'agentic_interaction_v8/BEHAVIOR.json'


def audit(bundle=None):
    m = json.loads(FREEZE.read_text()); failures = []
    for name, expected in m['source_sha256'].items():
        if not (ROOT/name).is_file() or digest(ROOT/name) != expected: failures.append(name)
    for name,row in m['execution_directory_snapshot'].items():
        if digest(ROOT/row['snapshot']) != row['sha256']: failures.append('SNAPSHOT:'+name)
    for row in m['historical_execution_profiles'].values():
        if digest(ROOT/row['registration']) != row['registration_sha256']: failures.append(row['registration'])
        for x in row['differing_sources'].values():
            if digest(ROOT/x['snapshot']) != x['sha256']: failures.append(x['snapshot'])
    source = json.loads((ROOT/'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_FREEZE.json').read_text())['source_sha256']
    for name, expected in source.items():
        if digest(ROOT/name) != expected: failures.append('REVOLUTE:'+name)
    result = dict(version='8.0.0', physical_source_files=len(m['source_sha256']),
                  frozen_revolute_source_files=len(source), failures=failures,
                  historical_execution_profiles=m['historical_execution_profiles'])
    if bundle:
        b = verify_bundle(bundle); ext = json.loads((ROOT/'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_EXTERNAL_FREEZE.json').read_text())['external_robot_camera_closure_uncertainty_sha256']
        for name, expected in ext.items():
            if digest(Path(bundle)/b['frozen_external_to_bundle'][name]) != expected: failures.append(name)
        result['frozen_external_files'] = len(ext); result['bundle_sha256'] = digest(Path(bundle)/'BUNDLE.json')
    if failures: raise ValueError('FROZEN_BEHAVIOR_DRIFT:'+repr(failures))
    return result


def environment(profile):
    """Executable paths are declared, never inherited from a development chat."""
    p = json.loads(Path(profile).read_text())
    for k in ['python','sam_python','sam_checkpoint','ffmpeg_dir','bundle','output_root','actor_lock']:
        if k not in p or not Path(p[k]).is_absolute(): raise ValueError('EXPLICIT_ABSOLUTE_PATH_REQUIRED:'+k)
    for k in ['python','sam_python','sam_checkpoint']:
        if not Path(p[k]).is_file(): raise ValueError('MISSING_ENVIRONMENT_RESOURCE:'+k)
    if digest(p['sam_checkpoint']) != p['sam_checkpoint_sha256']: raise ValueError('SAM_CHECKPOINT_DRIFT')
    for name, expected in p['sam_source_sha256'].items():
        if digest(name) != expected: raise ValueError('SAM_IMPLEMENTATION_DRIFT:'+name)
    if not p['sam_source_sha256']: raise ValueError('SAM_SOURCE_COMMITMENT_REQUIRED')
    return p


def prepare(specfile, profile):
    spec = json.loads(Path(specfile).read_text()); p = environment(profile); frozen = audit(p['bundle'])
    allowed = {'schema','episode_id','scene','purpose','physics_file','physics_sha256','identification_case',
               'wall_budget_s','sim_budget_s','acquisition_budget','agent_wait_s'}
    if set(spec)-allowed: raise ValueError('UNKNOWN_JOB_FIELDS:'+repr(set(spec)-allowed))
    if spec['schema'] != 'agentic-interaction/v8' or spec['purpose'] not in ('reproducibility_smoke','engineering'):
        raise ValueError('V8_DOES_NOT_AUTHORIZE_PAPER_BENCHMARK')
    if not spec['episode_id'].replace('_','').replace('-','').isalnum(): raise ValueError('INVALID_EPISODE_ID')
    if digest(spec['physics_file']) != spec['physics_sha256']: raise ValueError('PLANT_COMMITMENT_MISMATCH')
    if not (600<=float(spec.get('wall_budget_s',2400))<=7200 and 180<=float(spec.get('sim_budget_s',1400))<=3600
            and 1<=int(spec.get('acquisition_budget',2))<=3 and 30<=float(spec.get('agent_wait_s',360))<=600):
        raise ValueError('FINITE_SMOKE_BUDGET_WITH_EXIT_RESERVE_REQUIRED')
    bundle = Path(p['bundle']); m = verify_bundle(bundle); scene = m['scenes'][spec['scene']]
    out = Path(p['output_root'])/spec['episode_id']
    if out.exists(): raise ValueError('EPISODE_ALREADY_EXISTS_PRESERVE_RESULT')
    out.mkdir(parents=True)
    materialize(bundle, out/'robot_io')
    # USD import caches are private, fresh, per episode, never shared historic outputs.
    import shutil
    shutil.copytree(bundle/scene['asset'], out/'asset')
    shutil.copytree(bundle/scene['station'], out/'station')
    job = json.loads((bundle/scene['template']).read_text())
    job.update(output=str(out),asset_root=str(out/'asset'),source=str(out/'station'),
               episode_id=spec['episode_id'],private_physics_configuration=str(Path(spec['physics_file']).resolve()),
               identification_case=int(spec.get('identification_case',0)),
               episode_wall_budget_s=float(spec.get('wall_budget_s',2400)),
               episode_sim_budget_s=float(spec.get('sim_budget_s',1400)),
               runtime_acquisition_budget=int(spec.get('acquisition_budget',2)),
               agent_wait_timeout_s=float(spec.get('agent_wait_s',360)),
               safe_exit_wall_reserve_s=300,safe_exit_sim_reserve_s=120,
               package_version='8.0.0',package_purpose=spec['purpose'],
               gpu=int(p['physics_gpu']),visual_gpu=int(p['perception_gpu']),
               wrist_experiment={'sam3':{'python':p['sam_python'],'pythonpath':p['sam_pythonpath'],'checkpoint':p['sam_checkpoint']}},
               v8_robot_io=str(out/'robot_io'))
    for key, rel in m['resources'].items():
        job[key] = json.loads((bundle/rel).read_text()) if key=='ft_calibration' else str(bundle/rel)
    kind = job['articulation_v7']['joint_type']
    if kind not in ('revolute','prismatic'): raise ValueError('UNSUPPORTED_ARTICULATION')
    # Compatibility fields are explicit; original versioned owner guards stay active.
    if kind=='prismatic':
        job.update(run_classification='PRISMATIC_ENGINEERING',engineering_revision='V7.5.1',prismatic_version='V7.5',
                   native_g1_preload_request_n=1.,prismatic_integrated_ik_feedback=False,prismatic_contact_retention=False)
    for name in ['passive_geometry_cameras','passive_rgbd','passive_dataset_output','passive_evaluation_output','saved_grasp_poses','preload_fixture','benchmark_registration']:
        if name in job: raise ValueError('FORBIDDEN_SMOKE_INPUT:'+name)
    dump(out/'job.json',job)
    dump(out/'V8_PRELAUNCH.json',dict(spec=spec,job_sha256=digest(out/'job.json'),frozen=frozen,
        robot_io_equivalence=json.loads((out/'robot_io/IO_EQUIVALENCE.json').read_text()),
        sources={str(x.relative_to(ROOT)):digest(x) for x in sorted((ROOT/'agentic_interaction_v8').glob('*.py'))},
        independent_private_plant=True,passive_cameras_available_to_agent=False,
        original_station_assumption=True,baseline_initial_q_is_registered_station_not_saved_grasp=True))
    return out,p


def launch(specfile, profile):
    out,p = prepare(specfile,profile)
    lockpath=Path(p['actor_lock']);lockpath.parent.mkdir(parents=True,exist_ok=True)
    with lockpath.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        free={int(x.split(',')[0]):int(x.split(',')[1]) for x in subprocess.check_output(
            ['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()}
        if free[int(p['physics_gpu'])]<12000:raise ValueError('INSUFFICIENT_SHARED_GPU_CAPACITY')
        env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',
                 OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MPLCONFIGDIR=str(out/'plot_cache'),
                 PATH=p['ffmpeg_dir']+':'+os.environ['PATH'])
        command=[p['python'],'-u','-m','agentic_interaction_v8.entry','--job',str(out/'job.json')]
        began=time.perf_counter()
        with (out/'actor.log').open('wb') as log:
            actor=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            dump(out/'PROCESS.json',dict(pid=actor.pid,command=command,free_VRAM_MiB=free,
                 checkout=str(ROOT),fresh_assets=True,job_sha256=digest(out/'job.json')))
            try:rc=actor.wait(timeout=float(json.loads((out/'job.json').read_text())['episode_wall_budget_s'])+180)
            except subprocess.TimeoutExpired:
                (out/'STOP_AND_EXPORT').touch()
                try:rc=actor.wait(timeout=90)
                except subprocess.TimeoutExpired:
                    actor.terminate()
                    try:rc=actor.wait(timeout=30)
                    except subprocess.TimeoutExpired:actor.kill();rc=actor.wait(timeout=10)
        dump(out/'PROCESS_COMPLETION.json',dict(returncode=rc,wall_s=time.perf_counter()-began,actor_exited=True))
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='op',required=True)
    a=s.add_parser('audit');a.add_argument('--bundle',type=Path)
    a=s.add_parser('export');a.add_argument('--descriptor',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    a=s.add_parser('launch');a.add_argument('--job',type=Path,required=True);a.add_argument('--environment',type=Path,required=True)
    a=s.add_parser('command');a.add_argument('--episode',type=Path,required=True);a.add_argument('--sequence',type=int,required=True);a.add_argument('--json',type=Path,required=True)
    a=s.add_parser('status');a.add_argument('--episode',type=Path,required=True)
    args=p.parse_args()
    if args.op=='audit':print(json.dumps(audit(args.bundle),indent=2))
    elif args.op=='export':
        m=export_bundle(ROOT,json.loads(args.descriptor.read_text()),args.output);print(json.dumps(dict(files=len(m['files']),bundle_sha256=digest(args.output/'BUNDLE.json'))))
    elif args.op=='launch':print(launch(args.job,args.environment))
    elif args.op=='command':
        sys.path.insert(0,str(ROOT/'scripts'))
        from agent_command_protocol_v12 import publish
        print(publish(args.episode/'agent_commands',args.sequence,json.loads(args.json.read_text())))
    elif args.op=='status':
        for name in ['PROCESS_COMPLETION.json','EPISODE_RESULT.json','WAITING_FOR_AGENT.json','DECISION_PACKET.json']:
            f=args.episode/name
            if f.exists():print(name+'\n'+f.read_text())


if __name__=='__main__':main()
