"""One independently seeded scene; no automatic physical retry, shared GPU use."""
import argparse,datetime,hashlib,json,os,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/xarm_large_range_torque_v1_20261017'
FILES=['scripts/interaction_reference_handover.py','scripts/native_g1_large_range_contact.py','scripts/run_native_g1_large_range_torque.py','scripts/torque_components_v1.py','scripts/run_torque_components_v1.py','scripts/verify_large_range_torque.py','scripts/prepare_large_range_torque.py','scripts/launch_large_range_torque_v1.py']
SEED=171947686

def main():
    p=argparse.ArgumentParser();p.add_argument('--prepare-only',action='store_true');a=p.parse_args();OUT.mkdir(exist_ok=True)
    prior=json.loads((ROOT/'results/xarm_hybrid_v0_20261017/FROZEN.json').read_text())
    hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in list(prior['source_sha256'])+FILES}
    registration=dict(reference='eeda9fa',asset='47686',prepared_height_m=.70,source_height_basis='source metadata dimension 80x35x70cm; no outcome-based scale tuning',
        world_axis=[-1.,0.,0.],known_hinge_state=True,maximum_continuous_scenes=1,maximum_valid_physical_episodes=1,maximum_planned_handle_acquisitions=2,
        maximum_unexpected_recovery_grasps=0,Agent_view_adjustments=4,exploratory_targets_deg=[5,15,20,30],
        original_loaded_TCP_limit_m_s=.005,original_safety_protections=True,frozen_baselines_unchanged=True,
        action_menu=dict(pull_speeds_m_s=[.00075,.0015,.0025],reference_accelerations_m_s2=[.002,.004],maximum_segments_per_acquisition=4,
            directions='both allowed after current-state feasibility; Agent explicitly approves finite actions before loaded execution'),
        physics=dict(seed=SEED,family='constant Coulomb/viscous; coherent density-gradient mass/COM/inertia',mass_scale_range=[.75,1.25],density_gradient_range=[-.3,.3],Coulomb_prior_Nm=[.018,.026],damping_prior_Nm_s_rad=[.1,.35],passive_spring=False),
        model='I*qdd+b*qd+c*sign(qd)+gs*sin(q)+gc*cos(q)',heldout='complete designated VALIDATION segments; existing7..9deg interior split; segment-local derivatives',
        raw_evidence_private=True,hidden_truth_for_posthoc_only=True,no_GT_grasp_or_contact_scoring=True)
    hidden=OUT/'HIDDEN_CONFIGURATION.json'
    if not hidden.exists():
        rng=np.random.default_rng(SEED);c=dict(case=0,asset='47686',seed=SEED,mass_scale=float(rng.uniform(.75,1.25)),density_gradient=rng.uniform(-.3,.3,3).tolist(),tau_c=float(rng.uniform(.018,.026)),b=float(rng.uniform(.1,.35)),spring=False)
        c['law']=dict(family='constant',base_nm=c['tau_c'],amplitude_nm=0.,center_rad=0.,width_rad=1.)
        hidden.write_text(json.dumps([c],indent=2));hidden.chmod(0o600)
    frozen=OUT/'FROZEN.json';value=dict(source_sha256=hashes,registration=registration)
    if frozen.exists() and json.loads(frozen.read_text())!=value:raise ValueError('Source/config differs from registered freeze; inspect before launch')
    frozen.write_text(json.dumps(value,indent=2))
    if a.prepare_only:print(json.dumps(registration,indent=2));return
    if (OUT/'PROCESS.json').exists():raise ValueError('Prior launch record exists; no silent retry')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if proc.stat().st_uid!=os.getuid():continue
            argv=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
        except OSError:continue
        if int(proc.name)!=os.getpid() and ('scripts/run_native_g1_' in argv and ' --job ' in argv):raise RuntimeError('Owned actor running: '+proc.name)
    gpus=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
    if gpus[0][0]<12000 or gpus[1][0]<7000:raise RuntimeError('Insufficient free capacity now; no GPU exclusivity requirement')
    job=json.loads((OUT/'INPUT_JOB.json').read_text());job.update(output=str(OUT/'episode'),gpu=gpus[0][1],visual_gpu=gpus[1][1],identification_case=0,private_physics_configuration=str(hidden),initial_information_block=[],information_action_menu={},expected_runner_sha256=hashes['scripts/run_native_g1_large_range_torque.py'],deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=4)).isoformat())
    (OUT/'job.json').write_text(json.dumps(job,indent=2));env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/large_range_torque_v1');env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
    command=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts/run_native_g1_large_range_torque.py'),'--job',str(OUT/'job.json')]
    pre=subprocess.run(command+['--preflight'],env=env,cwd=ROOT,capture_output=True,text=True);(OUT/'PREFLIGHT.log').write_text(pre.stdout+pre.stderr)
    if pre.returncode:raise RuntimeError('Generated executable preflight failed before Isaac')
    with (OUT/'actor.log').open('wb') as log:actor=subprocess.Popen(command,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (OUT/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=command,gpu=job['gpu'],visual_gpu=job['visual_gpu'],frozen_source=hashes),indent=2));print(json.dumps(dict(pid=actor.pid,gpu=job['gpu'],visual_gpu=job['visual_gpu'])))

if __name__=='__main__':main()
