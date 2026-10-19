"""Exactly two preregistered scenes; only own actors serialized; shared GPU allowed."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,subprocess
from xarm_ft_v1_scene import register,PRIORS,SEEDS
from xarm_ft_v1_estimator import CONFIG
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/xarm_wrist_ft_v1_20261017'
p=argparse.ArgumentParser();p.add_argument('--prepare-only',action='store_true');p.add_argument('--case',type=int,choices=[0,1]);a=p.parse_args();OUT.mkdir(parents=True,exist_ok=True)
if not (OUT/'HIDDEN_CONFIGURATIONS.json').exists():register(OUT)
files=list(json.loads((ROOT/'results/xarm_wrist_ft_id_v0_20261017/IDENTIFICATION_FROZEN.json').read_text())['source_sha256'])
files+=['scripts/xarm_ft_known_joint_prior.py','scripts/xarm_ft_v1_scene.py','scripts/xarm_ft_v1_estimator.py','scripts/native_g1_ft_multistate.py','scripts/run_xarm_ft_v1_estimation.py','scripts/run_native_g1_ft_v1.py','scripts/launch_xarm_ft_v1.py']
hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in files}
initial=[dict(target_deg=3.,speed_scale=.45,profile='constant',role='FIT_INITIAL_OPEN',hold=True),dict(target_deg=1.,speed_scale=.65,profile='smooth_variation',role='VALIDATION_INITIAL_CLOSE',hold=True),dict(target_deg=6.,speed_scale=1.,profile='smooth_variation',role='FIT_INITIAL_REOPEN',hold=True)]
menu={
 'expand10_revisit':[dict(target_deg=10.,speed_scale=1.,profile='smooth_variation',role='FIT_EXPAND10',hold=True),dict(target_deg=6.,speed_scale=.4,profile='constant',role='FIT_RETURN6',hold=True),dict(target_deg=10.,speed_scale=1.,profile='constant',role='VALIDATION_FAST_REOPEN10',hold=True)],
 'expand15_20':[dict(target_deg=15.,speed_scale=1.,profile='smooth_variation',role='FIT_EXPAND15',hold=True),dict(target_deg=10.,speed_scale=.45,profile='constant',role='FIT_CLOSE10',hold=True),dict(target_deg=20.,speed_scale=1.,profile='smooth_variation',role='FIT_EXPAND20',hold=True),dict(target_deg=15.,speed_scale=.7,profile='constant',role='VALIDATION_CLOSE15',hold=True)],
 'stop':{'stop':True}}
# These are reference time-law parameters, not actuator/IK gains or protections.
# Familiar first block stays at the demonstrated <=1.5mm/s reference. Later
# optional excitation uses <=3.5mm/s, below the unchanged measured 5mm/s guard.
for name in ['expand10_revisit','expand15_20']:
 for action in menu[name]:
  if action['profile']=='smooth_variation':
   action.update(reference_speed_m_s=.003 if name=='expand10_revisit' else .0035,reference_acceleration_m_s2=.006)
registration=dict(reference='d6e690f',frozen_manipulation='136b6e0',cases=[dict(case=i,asset='46427',seed=s,family='constant' if i==0 else 'state_dependent',maximum_episodes=1,maximum_grasps=2) for i,s in enumerate(SEEDS)],
    parameter_priors=PRIORS,estimation=CONFIG,initial_block=initial,finite_action_menu=menu,maximum_blocks=3,maximum_segments_per_block=4,
    Agent_selects_from_measured_feedback=True,not_fixed_permanent_object_policy=True,raw_private=True,hidden_profiles_not_in_worker_input=True,
    native_passive_law='native joint friction properties updated from a private fixed passive c(q); no joint target/torque actuation')
frozen=OUT/'FROZEN.json'
if frozen.exists():
 if json.loads(frozen.read_text())['source_sha256']!=hashes:raise ValueError('Frozen V1 implementation changed')
else:frozen.write_text(json.dumps(dict(source_sha256=hashes,registration=registration),indent=2))
if a.prepare_only:print(json.dumps(registration,indent=2));raise SystemExit(0)
if a.case is None:raise ValueError('Registered case required')
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:argv=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
 except OSError:continue
 if 'scripts/run_native_g1_ft_v1.py --job' in argv and int(proc.name)!=os.getpid():raise ValueError('Own V1 actor still active: '+proc.name)
case=OUT/('case_%02d'%a.case);case.mkdir(exist_ok=True)
if (case/'PROCESS.json').exists():raise ValueError('Already launched; preserve and inspect, no automatic physical retry')
job=json.loads((ROOT/'results/xarm_wrist_ft_id_v0_20261017/case_02/job.json').read_text())
gpus=sorted([(int(s.split(',')[1]),int(s.split(',')[0])) for s in subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True).splitlines()],reverse=True)
if gpus[0][0]<12000 or gpus[1][0]<7000:raise ValueError('No sufficient free capacity now; occupied GPUs otherwise permitted')
job.update(output=str(case/'episode'),episode_id='ft_v1_%02d'%a.case,gpu=gpus[0][1],visual_gpu=gpus[1][1],identification_case=a.case,
 private_physics_configuration=str(OUT/'HIDDEN_CONFIGURATIONS.json'),initial_information_block=initial,information_action_menu=menu,
 expected_runner_sha256=hashes['scripts/run_native_g1_ft_v1.py'],deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=4)).isoformat())
(case/'job.json').write_text(json.dumps(job,indent=2));env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MPLCONFIGDIR='/tmp/ft_v1_'+str(a.case));env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u',str(ROOT/'scripts/run_native_g1_ft_v1.py'),'--job',str(case/'job.json')]
pre=subprocess.run(cmd+['--preflight'],env=env,cwd=ROOT,capture_output=True,text=True);(case/'PREFLIGHT.log').write_text(pre.stdout+pre.stderr)
if pre.returncode:raise RuntimeError('V1 source preflight failed before Isaac/contact; see log')
with (case/'actor.log').open('wb') as log:actor=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(case/'PROCESS.json').write_text(json.dumps(dict(pid=actor.pid,command=cmd,source_hashes=hashes,gpu=job['gpu'],visual_gpu=job['visual_gpu']),indent=2));print(actor.pid)
