"""Sequential registered cases; reuse completed results, no physical retries."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,subprocess
from xarm_ft_id_scene import register,PRIORS,SEEDS
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/xarm_wrist_ft_id_v0_20261017'
p=argparse.ArgumentParser();p.add_argument('--case',type=int,choices=[0,1,2]);p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
OUT.mkdir(parents=True,exist_ok=True)
if not (OUT/'HIDDEN_CONFIGURATIONS.json').exists():register(OUT)
source_files=list(json.loads((ROOT/'docs/experiments/native_g1_safe_termination_v11_20261016/FROZEN.json').read_text())['source_sha256'])
new_files=['scripts/xarm_wrist_ft.py','scripts/xarm_ft_id_scene.py','scripts/xarm_ft_id_estimator.py','scripts/native_g1_ft_identification.py','scripts/run_xarm_ft_estimation.py','scripts/run_native_g1_ft_identification.py','scripts/launch_xarm_ft_identification.py']
hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in source_files+new_files}
frozen=OUT/'IDENTIFICATION_FROZEN.json'
registration=dict(cases=[dict(case=i,asset='46427',seed=s,order=i+1,maximum_grasps=2) for i,s in enumerate(SEEDS)],priors=PRIORS,
    maximum_continuous_identification_episodes=3,maximum_recovery_regrasps_per_episode=1,reference='136b6e0',
    directions='opening and contact-driven closing, Agent-selected <=5 segments <=10deg',
    holdout='complete VALIDATION closing segment, excluded from fitting',tool='idealized fixed-interface FT, native G1 unchanged',
    sensor_empirical_unloaded_motion_residual_rms=dict(force_n=.01209,torque_nm=.003625),
    FT_passive=True,truth_path_private=True,raw_evidence_private=True)
if frozen.exists():
    if json.loads(frozen.read_text())['source_sha256']!=hashes:raise ValueError('Frozen batch source changed before launch')
else:frozen.write_text(json.dumps(dict(source_sha256=hashes,registration=registration),indent=2))
if a.prepare_only:print(json.dumps(registration,indent=2));raise SystemExit(0)
if a.case is None:raise ValueError('Registered case required')
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
    except OSError:continue
    if 'scripts/run_native_g1_ft_identification.py --job' in cmd and 'python' in cmd and int(proc.name)!=os.getpid():raise ValueError('Our prior identification actor is still running: '+proc.name)
case=OUT/('case_%02d'%a.case);case.mkdir(exist_ok=True)
if (case/'PROCESS.json').exists():raise ValueError('Case already launched; inspect saved scene/evidence; no automatic physical retry')
job=json.loads((ROOT/'results/native_g1_contact_recovery_v1_20261016/46427/job.json').read_text())
gpu_lines=subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits']).decode().splitlines()
gpus=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in gpu_lines],reverse=True)
gpu=gpus[0][1];visual_gpu=gpus[1][1]
if gpus[0][0]<12000 or gpus[1][0]<7000:raise ValueError('Insufficient currently free capacity; no exclusivity requirement')
job.update(output=str(case/'episode'),episode_id='ft_id_v0_%02d'%a.case,gpu=gpu,visual_gpu=visual_gpu,
    identification_case=a.case,identification_asset='46427',private_physics_configuration=str(OUT/'HIDDEN_CONFIGURATIONS.json'),
    ft_calibration=json.loads((OUT/'sensor_smoke_00/SENSOR_ANALYSIS.json').read_text()),
    activation_source_files=source_files,expected_runner_sha256=hashes['scripts/run_native_g1_ft_identification.py'],
    deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=3)).isoformat())
(case/'job.json').write_text(json.dumps(job,indent=2));env=os.environ.copy();env.update(PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/xarm_ft_id_'+str(a.case))
env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts/run_native_g1_ft_identification.py'),'--job',str(case/'job.json')]
pre=subprocess.run(cmd+['--preflight'],env=env,cwd=ROOT,capture_output=True,text=True);(case/'PREFLIGHT.log').write_text(pre.stdout+pre.stderr)
if pre.returncode:raise ValueError('Pre-initialization source preflight failed; no physical attempt')
with (case/'actor.log').open('wb') as log:actor=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(case/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=cmd,gpu=gpu,visual_gpu=visual_gpu,source_hashes=hashes),indent=2));print(actor.pid)
