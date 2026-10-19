"""Single conditional V1.2 episode; immutable V1.1 inputs/controllers preserved."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/agent_pull_push_v12_20261018'
FILES = ['scripts/'+name for name in (
    'agent_command_protocol_v12.py', 'identification_packet_v12.py',
    'native_g1_pull_push_v12.py', 'run_native_g1_pull_push_v12.py',
    'verify_pull_push_protocol_v12.py', 'verify_pull_push_packets_v12.py',
    'verify_pull_push_panel_v12.py', 'prepare_pull_push_v12.py',
    'publish_agent_command_atomic.py')]


def prepare(launch=False):
    OUT.mkdir(exist_ok=True)
    old = ROOT/'results/xarm_cabinet_torque_v11_20261017'
    freeze = json.loads((old/'FROZEN.json').read_text())
    names = list(dict.fromkeys(list(freeze['source_sha256'])+FILES))
    hashes = {n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names}
    for n, expected in freeze['source_sha256'].items():
        if hashes[n] != expected: raise ValueError('Frozen source changed: '+n)
    from verify_pull_push_protocol_v12 import verify
    protocol = verify(); (OUT/'PROTOCOL_PREFLIGHT.json').write_text(json.dumps(protocol,indent=2))
    import native_g1_pull_push_v12 as implementation
    import native_g1_contact_recovery as recovery
    import inspect
    from agent_command_protocol_v12 import wire_owner, OPERATIONS
    owner = wire_owner(inspect.getsource(recovery.run))
    compile(owner,'actual_protocolized_owner','exec')
    compile(implementation.generated_owner_source(),'actual_hybrid_interface','exec')
    # Hybrid's existing explicit additions remain in its original source.
    hybrid = implementation.generated_owner_source()
    for op in OPERATIONS:
        if "op=='"+op+"'" not in owner+hybrid:
            raise ValueError('Missing explicit operation: '+op)
    (OUT/'PROTOCOLIZED_AGENT_OWNER.py').write_text(owner)
    job = json.loads((old/'job.json').read_text())
    job.update(output=str(OUT/'episode'),episode_id='agent_pull_push_v12_45936',
        private_physics_configuration=str(OUT/'HIDDEN_CONFIGURATION.json'),
        activation_source_files=names, expected_runner_sha256=hashes['scripts/run_native_g1_pull_push_v12.py'])
    registration = dict(reference='587a269',asset='45936',normal_closed_start=True,scene_budget=1,
        planned_handle_acquisitions=2,panel_push_attempts=1,recovery_acquisitions=0,Agent_view_adjustments=4,
        initial_pull_target='Agent choice toward10..15deg within existing protections',
        panel_target='fresh wrist RGB-D, Agent choice; original native tip-contact/whole-route validation',
        optional_post_push_regrasp='fresh visual geometry, unchanged candidate ranking; no saved grasp',
        physics_seed=181845936,physics_family='same V1.1 constant prior family',
        unknown_dynamics_to_Agent_and_estimator=True,
        control_physics_safety_change=False,protocol_error='ACK, zero action, next sequence; scene remains usable',
        packet_policy='atomic raw readiness, dependency-ordered passive estimation, explicit incomplete result',
        raw_evidence_private=True)
    frozen = dict(source_sha256=hashes,registration=registration)
    if (OUT/'FROZEN.json').exists() and json.loads((OUT/'FROZEN.json').read_text()) != frozen:
        raise ValueError('Already frozen version differs; preserve registered implementation')
    (OUT/'FROZEN.json').write_text(json.dumps(frozen,indent=2))
    hidden=OUT/'HIDDEN_CONFIGURATION.json'
    if not hidden.exists():
        rng=np.random.default_rng(181845936)
        case=dict(case=0,asset='45936',seed=181845936,mass_scale=float(rng.uniform(.75,1.25)),
            density_gradient=rng.uniform(-.3,.3,3).tolist(),tau_c=float(rng.uniform(.018,.026)),
            b=float(rng.uniform(.1,.35)),spring=False)
        case['law']=dict(family='constant',base_nm=case['tau_c'],amplitude_nm=0.,center_rad=0.,width_rad=1.)
        hidden.write_text(json.dumps([case],indent=2));hidden.chmod(0o600)
    gpus=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in subprocess.check_output(
        ['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
    job.update(gpu=gpus[0][1],visual_gpu=gpus[1][1])
    job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=4)).isoformat()
    (OUT/'job.json').write_text(json.dumps(job,indent=2))
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',
        MPLCONFIGDIR='/tmp/agent_pull_push_v12',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts/run_native_g1_pull_push_v12.py'),'--job',str(OUT/'job.json')]
    result=subprocess.run(command+['--preflight'],env=env,cwd=ROOT,capture_output=True,text=True)
    (OUT/'GENERATED_PREFLIGHT.log').write_text(result.stdout+result.stderr)
    if result.returncode: raise ValueError('Generated executable preflight failed')
    if not launch:return dict(prepared=True,actor_launched=False,source_files=len(names),GPUs=gpus[:2])
    # This is the user-requested pre-episode infrastructure/route condition,
    # not a new runtime grasp-quality or manipulation gate.
    panel=json.loads((OUT/'offline/full_panel/FULL_PANEL_PREFLIGHT.json').read_text())
    if not panel['complete_route_valid']: raise ValueError('No complete existing-guard panel route; do not launch')
    if (OUT/'PROCESS.json').exists(): raise ValueError('Registered actor already launched; no automatic physical retry')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if proc.stat().st_uid!=os.getuid():continue
            argv=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
        except OSError:continue
        if proc.name!=str(os.getpid()) and 'scripts/run_native_g1_' in argv and ' --job ' in argv:
            raise RuntimeError('Owned actor already running: '+proc.name)
    if gpus[0][0]<12000 or gpus[1][0]<7000:raise RuntimeError('Insufficient current free VRAM; GPU exclusivity not required')
    with (OUT/'actor.log').open('wb') as log:
        actor=subprocess.Popen(command,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (OUT/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=command,GPUs=gpus[:2],frozen_sources=hashes),indent=2))
    return dict(pid=actor.pid,GPUs=gpus[:2])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--launch',action='store_true');a=p.parse_args()
    print(json.dumps(prepare(a.launch)))
