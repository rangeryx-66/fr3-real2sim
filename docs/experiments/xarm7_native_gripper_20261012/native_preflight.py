import json,hashlib,copy,inspect,ast
from pathlib import Path
import numpy as np
from xarm7_native_gripper import ROOT,OUT,configuration,GRIPPER_JOINTS
from xarm7_native_execution import executable,adapt_baseline,adapt_loader
from xarm7_native_planning import NativeModel
c=configuration();sources=[ROOT/'scripts'/n for n in ['xarm7_native_gripper.py','xarm7_native_planning.py','xarm7_native_execution.py','run_xarm7_native_door.py','xarm7_runtime.py','xarm7_door_adapter.py','cross_robot_adapter.py','plan_shared_door_skill.py','shared_door_execution.py','precontact_trajectory.py']]
for asset in json.loads((OUT/'REGISTERED_CASES.json').read_text())['asset_ids']:
 d=OUT/asset;p=d/'planning/candidate.json'
 if (d/'physical_01/actor_start_provenance.json').exists():
  print(asset,'already launched; inputs preserved');continue
 if not p.exists():p=d/'planning_corrected_input/candidate.json'
 if not p.exists():continue
 plan=json.loads(p.read_text());j=json.loads((d/'planning_job.json').read_text());j['whole_task_plan']=str(p);source=json.loads((d/'source/report.json').read_text());source['robot_base_pose']=plan['base'];(d/'source/report.json').write_text(json.dumps(source,indent=2))
 j['expected_runner_sha256']=hashlib.sha256((ROOT/'scripts/run_xarm7_native_door.py').read_bytes()).hexdigest();j['skill_inputs']['robot_gripper']={'urdf':c['sim_urdf'],'native_gripper':'UFACTORY xArm G1','controller':'finite position hold','simulation_force_feedback_not_hardware':True,'base_height_m':-.1}
 (d/'job.json').write_text(json.dumps(j,indent=2));(d/'grasp_plan.json').write_text(json.dumps({'trial_candidates':[{'base':plan['base'],'T':plan['T_grasp'],'approach':plan['approach'],'preplan':plan['preplan']}]}))
 text=executable(j);compile(text,str(d/'PRELAUNCH_EXECUTABLE.py'),'exec');(d/'PRELAUNCH_EXECUTABLE.py').write_text(text)
 baseline=adapt_baseline(Path(j['baseline_runner']).read_text(),j);compile(baseline,'native baseline','exec');(d/'PRELAUNCH_BASELINE.py').write_text(baseline)
 # Resolve the loader from actual known source, then inspect its native-only
 # mappings before scene creation; no object contact or physics is performed.
 from types import SimpleNamespace
 import xarm7_door_adapter as frozen
 class LoaderReady(Exception):pass
 def capture_loader(source,config,out):
  loader=adapt_loader(source,config,out);compile(loader,'native loader','exec');(d/'PRELAUNCH_LOADER_SHAPE_CHECK.py').write_text(loader)
  raise LoaderReady()
 ns=dict(frozen.__dict__,configuration=configuration,adapt_xarm_loader=capture_loader)
 exec(compile(inspect.getsource(frozen.bootstrap_job),'prelaunch actual loader','exec'),ns)
 args=SimpleNamespace(source=d/'source',asset_root=d/'asset',output=d/'prelaunch_only',gpu=2,ownership=None)
 args.output.mkdir(exist_ok=True)
 try:ns['bootstrap_job'](args,plan['base'],j)
 except LoaderReady:pass

 print(asset,{'entry_compiles':True,'baseline_compiles':True,'native_width_command_in_step':"controller.command_width(jaw_width_command)" in baseline,'native_model':"from xarm7_native_execution import robot_model as Model" in baseline,'coupled_finger_names':all(n in c['gripper_joint_names'] for n in GRIPPER_JOINTS),'gt_grasp_template':False})
freeze={'sources':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},'config_sha256':hashlib.sha256((OUT/'NATIVE_CONFIG.json').read_bytes()).hexdigest(),'model_sha256':hashlib.sha256(Path(c['sim_urdf']).read_bytes()).hexdigest(),'max_contact_attempts_each':1,'asset_ids':['45134','45443','45249'],'no_between_case_tuning':True}
(OUT/'CONTACT_BATCH_FINAL_INPUTS_FREEZE.json').write_text(json.dumps(freeze,indent=2))
