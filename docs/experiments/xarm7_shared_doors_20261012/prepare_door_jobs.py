from pathlib import Path
import json,hashlib,copy,inspect
from datetime import datetime,timedelta
from zoneinfo import ZoneInfo
from xarm7_runtime import configuration
from xarm7_door_adapter import executable,adapt_baseline,adapt_xarm_loader
import piper_mobile_execute as original
import interactive_twin.plant as plant
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/xarm7_shared_doors_20261012';old=R/'results/door_transfer4_20261012';c=configuration();common=json.loads((old/'45134/job.json').read_text());registry=[]
for asset in ['45134','45443']:
 folder=O/asset;folder.mkdir(exist_ok=True);job=json.loads((old/asset/'job.json').read_text());whole_path=R/'results/cross_robot_20261012/offline/xarm7'/asset/'own_plan/candidate.json';whole=json.loads(whole_path.read_text());source=folder/'source';source.mkdir(exist_ok=True);src=json.loads((Path(job['source'])/'report.json').read_text());src['robot_base_pose']=whole['base'];(source/'report.json').write_text(json.dumps(src,indent=2));grasp={'trial_candidates':[dict(whole,T=whole['T_grasp'])]};(folder/'grasp_plan.json').write_text(json.dumps(grasp,indent=2))
 for field in ['wrist_experiment','timing_experiment','pink_motion_scale']:
  if field in common:job[field]=copy.deepcopy(common[field])
 job.update(source=str(source),output=str(folder/'physical_01'),plan=str(folder/'grasp_plan.json'),whole_task_plan=str(whole_path),robot_start_q=c['home_q'],gpu=2,deadline_shanghai=(datetime.now(ZoneInfo('Asia/Shanghai'))+timedelta(hours=2)).isoformat(),shared_policy=str(R/'scripts/shared_door_execution.py'),expected_runner_sha256=hashlib.sha256((R/'scripts/run_xarm7_shared_door.py').read_bytes()).hexdigest())
 job['expected_active_closure_sha256']=hashlib.sha256(Path(job['active_closure_source']).read_bytes()).hexdigest();job['expected_trajectory_helper_sha256']=hashlib.sha256((R/'scripts/precontact_trajectory.py').read_bytes()).hexdigest()
 (folder/'job.json').write_text(json.dumps(job,indent=2));code=executable(job);compile(code,'composed_'+asset,'exec');(folder/'PRELAUNCH_EXECUTABLE.py').write_text(code)
 # Expand both setup adapters before Isaac; verify the imported filename and
 # native seven-joint group are literal in the actual loader.
 loader=original.__dict__['ROOT'].joinpath('src/r1a7_articulated_sim_server.py').read_text().split('commands = queue.Queue(); state = {}; lock = threading.Lock()')[0]
 # Replay only the static replacements made by original.bootstrap.
 replacements=[('scripts/prepare_r1a7_description.py','scripts/prepare_piper_description.py'),('nargs=7','nargs=6'),('config/r1a7_dex1.urdf','config/piper_sim.urdf'),('/World/R1A7','/World/Piper'),("names.index(f'J{i}') for i in range(1, 8)","names.index(f'joint{i}') for i in range(1, 7)"),('kp[arm] = [500., 500., 400., 400., 250., 150., 150.]','kp[arm] = 10000.'),('kd[arm] = [40., 40., 32., 32., 22., 15., 15.]','kd[arm] = 400.')]
 for a,b in replacements:loader=loader.replace(a,b)
 loader=adapt_xarm_loader(plant.adapt_loader_source(loader,job),c,folder)
 compile(loader,'loader_static_'+asset,'exec');assert c['sim_urdf'] in loader;assert "range(1, 8)" in loader;assert "kp[arm] = 10000." not in loader
 (folder/'PRELAUNCH_LOADER_STATIC.py').write_text(loader)
 registry.append({'asset':asset,'job':str(folder/'job.json'),'automatic_plan':str(whole_path),'plan_sha256':hashlib.sha256(whole_path.read_bytes()).hexdigest(),'base':whole['base'],'registered_handle':whole.get('grasp_geometry_selection'),'compiled_source_sha256':hashlib.sha256(code.encode()).hexdigest(),'shared_controller':'unchanged frozen Pink, closure and contact feedback; dimensions seven'})
(O/'REGISTERED_CASES.json').write_text(json.dumps(registry,indent=2));files=[R/'scripts/xarm7_runtime.py',R/'scripts/xarm7_door_adapter.py',R/'scripts/run_xarm7_shared_door.py',R/'scripts/cross_robot_adapter.py',R/'scripts/shared_door_execution.py',R/'scripts/precontact_trajectory.py',Path(common['frozen_execution_runner']),Path(common['active_closure_source']),Path(c['sim_urdf'])];freeze={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};(O/'CONTACT_BATCH_SOURCE_FREEZE.json').write_text(json.dumps({'sources':freeze,'predeclared_objects':['45134','45443'],'no_parameter_changes_between_cases':True,'actuator_effective_solver':'PGS','removed_ignored_setting':'TGS external-force-every-iteration flag disabled because PGS ignores it','known_diagnostic':'native joint velocities need not equal full-step position differences under constraints; both retained, existing speed protection unchanged'},indent=2));print('Both cases compiled offline; batch frozen')
