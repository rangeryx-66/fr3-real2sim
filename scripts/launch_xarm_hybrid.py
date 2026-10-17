"""One registered state-dependent episode, two planned acquisitions, no retries."""
import argparse, datetime, hashlib, json, os, subprocess
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/xarm_hybrid_v0_20261017'
SEED=171746427
FILES=['scripts/hybrid_panel_preflight.py','scripts/hybrid_routes_preflight.py',
       'scripts/native_g1_hybrid_contact.py','scripts/run_native_g1_hybrid.py','scripts/launch_xarm_hybrid.py',
       'scripts/verify_hybrid_contact_interfaces.py']


def main():
    p=argparse.ArgumentParser();p.add_argument('--prepare-only',action='store_true');a=p.parse_args();OUT.mkdir(parents=True,exist_ok=True)
    # Same passive family and sampling ranges, independently seeded once.
    hidden=OUT/'HIDDEN_CONFIGURATION.json'
    if not hidden.exists():
        rng=np.random.default_rng(SEED)
        case=dict(case=0,asset='46427',seed=SEED,mass_scale=float(rng.uniform(.75,1.25)),
            density_gradient=rng.uniform(-.3,.3,3).tolist(),tau_c=float(rng.uniform(.018,.026)),b=float(rng.uniform(.1,.35)),spring=False)
        case['law']=dict(family='localized_passive_friction',base_nm=case['tau_c'],amplitude_nm=float(rng.uniform(.008,.016)),
            center_rad=float(np.deg2rad(rng.uniform(8,14))),width_rad=float(np.deg2rad(rng.uniform(3,5))))
        hidden.write_text(json.dumps([case],indent=2));hidden.chmod(0o600)
    prior=json.loads((ROOT/'results/xarm_wrist_ft_v1_20261017/FROZEN.json').read_text())
    names=list(prior['source_sha256'])+FILES
    hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in dict.fromkeys(names)}
    registration=dict(reference='bd501b9',frozen_recovery='136b6e0',sensor='d6e690f',asset='46427',seed=SEED,
        maximum_valid_episodes=1,maximum_planned_handle_acquisitions=2,maximum_unexpected_recovery_acquisitions=0,
        initial_pull_target_deg=10.,panel_push_target='Agent chooses smaller nonzero state from actual achieved angle',
        final_pull_target_deg=15.,exploration_targets_not_success_guarantees=True,
        original_controller_and_physics_protections_unchanged=True,
        panel_wrench_envelope=dict(force_n=2.,moment='2 N times actual sensor-to-contact lever length',
            window_s=.05,sustained_windows=3,emergency_equivalent_n=10.,
            residual_tool_inertia_included_conservatively_in_guard=True,
            full_tool_inertia_compensation_in_passive_estimator=True,
            not_manufacturer_safety_limit=True),
        raw_evidence_private=True,unloaded_decision_boundaries=True,mode_sequence=['HANDLE_PULL','UNLOADED_REPOSITION','PANEL_PUSH','UNLOADED_REPOSITION','HANDLE_REGRASP','SAFE_EXIT'])
    frozen=OUT/'FROZEN.json';value=dict(source_sha256=hashes,registration=registration)
    if frozen.exists() and json.loads(frozen.read_text())!=value:raise ValueError('Frozen hybrid implementation changed; do not launch')
    frozen.write_text(json.dumps(value,indent=2))
    from verify_hybrid_contact_interfaces import verify
    (OUT/'INTERFACE_PREFLIGHT.json').write_text(json.dumps(verify(),indent=2))
    if a.prepare_only:print(json.dumps(registration,indent=2));return
    process=OUT/'PROCESS.json'
    if process.exists():raise ValueError('Existing launch record; inspect it, no automatic physical retry')
    # Serialize only owned Isaac actors; partially occupied GPUs are allowed.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if proc.stat().st_uid!=os.getuid():continue
            argv=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
        except OSError:continue
        if int(proc.name)!=os.getpid() and ('scripts/run_native_g1_' in argv and ' --job ' in argv):
            raise RuntimeError('Owned actor already running: '+proc.name)
    gpus=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
    if gpus[0][0]<12000 or gpus[1][0]<7000:raise RuntimeError('Insufficient available memory now; no exclusivity requirement')
    job=json.loads((ROOT/'results/xarm_wrist_ft_id_v0_20261017/case_02/job.json').read_text())
    job.update(output=str(OUT/'episode'),episode_id='hybrid_v0_46427',gpu=gpus[0][1],visual_gpu=gpus[1][1],identification_case=0,
        private_physics_configuration=str(hidden),initial_information_block=[],information_action_menu={},
        expected_runner_sha256=hashes['scripts/run_native_g1_hybrid.py'],
        deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=4)).isoformat())
    (OUT/'job.json').write_text(json.dumps(job,indent=2))
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/hybrid_v0')
    env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
    command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts/run_native_g1_hybrid.py'),'--job',str(OUT/'job.json')]
    pre=subprocess.run(command+['--preflight'],env=env,cwd=ROOT,capture_output=True,text=True)
    (OUT/'PREFLIGHT.log').write_text(pre.stdout+pre.stderr)
    if pre.returncode:raise RuntimeError('Generated owner preflight failed before Isaac')
    with (OUT/'actor.log').open('wb') as log:actor=subprocess.Popen(command,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    process.write_text(json.dumps(dict(pid=actor.pid,command=command,gpu=job['gpu'],visual_gpu=job['visual_gpu'],frozen_source=hashes),indent=2))
    print(json.dumps(dict(pid=actor.pid,gpu=job['gpu'],visual_gpu=job['visual_gpu'])))


if __name__=='__main__':main()
