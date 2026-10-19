from pathlib import Path
import json,hashlib,shutil,datetime
import numpy as np
from xarm7_native_gripper import ROOT,OUT,configuration
O=ROOT/'results/xarm7_native_transfer_repair_20261012'
for a in ['45134','45443','45249']:
 d=O/a;d.mkdir(exist_ok=True);j=json.loads((OUT/a/'job.json').read_text());(d/'source').mkdir(exist_ok=True);shutil.copy2(OUT/a/'source/report.json',d/'source/report.json')
 j.update(source=str(d/'source'),output=str(d/'physical_01'),plan=str(d/'grasp_plan.json'),whole_task_plan=str(d/'planning/candidate.json'),gpu=4,episode_id='native_transfer_repair_'+a,deadline_shanghai=(datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))+datetime.timedelta(hours=8)).isoformat())
 if (OUT/a/'physical_01/cooked_initial.json').exists():j['collision_export']=str(OUT/a/'physical_01/cooked_initial.json')
 j['expected_runner_sha256']=hashlib.sha256((ROOT/'scripts/run_xarm7_native_transfer_repair.py').read_bytes()).hexdigest();j['classification']='XARM7 / NATIVE_G1 / ORACLE_HANDLE / KNOWN_MODEL / BOUNDED_TRANSFER_REPAIR / APPROXIMATE_ACTUATOR_SIMULATION'
 j['skill_inputs']['robot_gripper']={'urdf':configuration()['sim_urdf'],'source':'official native G1','mode':'finite position hold'}
 j['skill_inputs']['articulation_model']={'type':'known_revolute','source':'case asset manifest/URDF'}
 (d/'planning_job.json').write_text(json.dumps(j,indent=2))
reg={'reference_commit':'fe639b6','offline_cases':['45134','45443','45249'],'physical_cases':['45443','45249'],'physical_attempts_each_max':1,'45134_new_physical_attempts':0,'45249_physical_contingent_on_credible_offline_improvement':True,'new_manipulation_change':'shared native pad-normal opening support score only','controller_gains_grip_effort_physics_guards':'unchanged','actors':'strictly sequential','public_upload_policy':'code and compact reports only; raw data/video local and server','source_hashes':{str(ROOT/'scripts'/n):hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest() for n in ['xarm7_native_gripper.py','xarm7_native_planning.py','xarm7_native_execution.py','xarm7_door_adapter.py','xarm7_runtime.py','cross_robot_adapter.py','shared_door_execution.py','precontact_trajectory.py','plan_shared_door_skill.py']}}
(O/'ROUND_REGISTRATION.json').write_text(json.dumps(reg,indent=2))
print('registered and prepared without physics')
