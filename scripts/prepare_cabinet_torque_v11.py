"""One registered 45936 scene, unchanged station/assets, no saved grasp/view."""
import argparse,datetime,hashlib,json,os,subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/xarm_cabinet_torque_v11_20261017'
FILES=['scripts/native_g1_cabinet_torque_contact.py','scripts/cabinet_torque_v11.py','scripts/run_cabinet_torque_v11.py','scripts/run_native_g1_cabinet_torque_v11.py','scripts/prepare_cabinet_torque_v11.py','scripts/verify_cabinet_torque_v11.py']

def main():
    p=argparse.ArgumentParser();p.add_argument('--launch',action='store_true');a=p.parse_args();OUT.mkdir(exist_ok=True)
    job=json.loads((ROOT/'results/native_g1_safe_termination_v11_20261016/45936/job.json').read_text())
    old=json.loads((ROOT/'results/xarm_large_range_torque_v1_20261017/job.json').read_text())
    job.update(output=str(OUT/'episode'),episode_id='cabinet_torque_v11_45936',identification_asset='45936',identification_case=0,private_physics_configuration=str(OUT/'HIDDEN_CONFIGURATION.json'),ft_calibration=old['ft_calibration'],initial_information_block=[],information_action_menu={},agent_wait_timeout_s=900)
    freeze=json.loads((ROOT/'results/xarm_large_range_torque_v1_20261017/FROZEN.json').read_text())
    names=list(dict.fromkeys(list(freeze['source_sha256'])+FILES))
    hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names}
    for n,sha in freeze['source_sha256'].items():
        if hashes[n]!=sha:raise ValueError('Frozen reference changed: '+n)
    registration=dict(reference='6aece1c',asset='45936',scene_budget=1,initial_acquisitions=1,recovery_acquisitions=1,view_adjustments=4,normal_closed_start=True,
        targets_deg=[10,15,20],conditional_extension_deg=30,action_menu='Agent-preapproved finite blocks, at most four segments, both directions, speeds .00075/.0015/.0025m/s and accelerations .002/.004m/s2; no waiting loaded',
        unchanged='SAM3/camera/grasp ranking/closure/one-second hold/Pink/native G1/robot gains/forces/collisions/5mm/s/joint margins/parking',
        physics=dict(seed=171945936,family='constant',mass_scale=[.75,1.25],density_gradient=[-.3,.3],Coulomb_Nm=[.018,.026],damping_Nmsrad=[.1,.35],spring=False),
        model='I*qdd+b*qd+C*sign(qd); vertical hinge gravity negligible',validation='complete VALIDATION segment and preserved7..9deg interior split; segment-local derivatives before fitting',
        known_dependencies=['hinge axis/state','registered station','scene safety colliders','simulated wrist calibration','approximate actuators','idealized virtual wrist load cell'],online_GT_handle=False,private_evidence=True)
    value=dict(source_sha256=hashes,registration=registration)
    f=OUT/'FROZEN.json'
    if f.exists() and json.loads(f.read_text())!=value:raise ValueError('Registered sources differ; inspect before launch')
    f.write_text(json.dumps(value,indent=2))
    hidden=OUT/'HIDDEN_CONFIGURATION.json'
    if not hidden.exists():
        rng=np.random.default_rng(171945936);case=dict(case=0,asset='45936',seed=171945936,mass_scale=float(rng.uniform(.75,1.25)),density_gradient=rng.uniform(-.3,.3,3).tolist(),tau_c=float(rng.uniform(.018,.026)),b=float(rng.uniform(.1,.35)),spring=False)
        case['law']=dict(family='constant',base_nm=case['tau_c'],amplitude_nm=0.,center_rad=0.,width_rad=1.)
        hidden.write_text(json.dumps([case],indent=2));hidden.chmod(0o600)
    job['activation_source_files']=names;job['expected_runner_sha256']=hashes['scripts/run_native_g1_cabinet_torque_v11.py']
    job['deadline_shanghai']=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=4)).isoformat()
    gpus=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
    job.update(gpu=gpus[0][1],visual_gpu=gpus[1][1]);(OUT/'job.json').write_text(json.dumps(job,indent=2))
    env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/cabinet_torque_v11',PATH='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ.get('PATH',''))
    python='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python';command=[python,'-u',str(ROOT/'scripts/run_native_g1_cabinet_torque_v11.py'),'--job',str(OUT/'job.json')]
    pre=subprocess.run(command+['--preflight'],env=env,cwd=ROOT,capture_output=True,text=True);(OUT/'GENERATED_PREFLIGHT.log').write_text(pre.stdout+pre.stderr)
    if pre.returncode:raise RuntimeError('Generated executable invalid before Isaac')
    if not a.launch:print(json.dumps(dict(prepared=True,actor_launched=False,source_files=len(names),GPUs=gpus[:2])));return
    if (OUT/'PROCESS.json').exists():raise ValueError('Launch already registered; no automatic physical retry')
    if not json.loads((OUT/'OFFLINE_PREFLIGHT.json').read_text())['initial_view_valid']:raise ValueError('Initial route invalid')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if proc.stat().st_uid!=os.getuid():continue
            argv=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
        except OSError:continue
        if proc.name!=str(os.getpid()) and 'scripts/run_native_g1_' in argv and ' --job ' in argv:raise RuntimeError('Owned actor already running: '+proc.name)
    if gpus[0][0]<12000 or gpus[1][0]<7000:raise RuntimeError('Insufficient available VRAM now, exclusivity unnecessary')
    with (OUT/'actor.log').open('wb') as log:actor=subprocess.Popen(command,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (OUT/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=command,GPUs=gpus[:2],frozen_sources=hashes),indent=2));print(json.dumps(dict(pid=actor.pid,GPUs=gpus[:2])))

if __name__=='__main__':main()
