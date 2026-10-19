from pathlib import Path
import json,hashlib
from xarm7_native_gripper import ROOT
from xarm7_native_execution import executable,adapt_baseline
O=ROOT/'results/native_g1_heldout_20261013';reg=json.loads((O/'REGISTERED_OBJECTS.json').read_text());assert (O/'ALL_PLANNING_COMPLETE.json').exists()
hashes=reg['frozen_source_sha256']|json.loads((O/'EXTRA_FROZEN_INPUT_HASHES.json').read_text());changed=[p for p,h in hashes.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h];assert not changed,changed
results=[];casehashes={}
for a in reg['asset_ids']:
 d=O/a;p=d/'planning/candidate.json'
 if not p.exists():results.append({'asset_id':a,'execution_ready':False,'reason':'NO_PLANNED_CANDIDATE','physical_result':None});continue
 plan=json.loads(p.read_text());j=json.loads((d/'planning_job.json').read_text());s=json.loads((d/'source/report.json').read_text());s['robot_base_pose']=plan['base'];(d/'source/report.json').write_text(json.dumps(s,indent=2))
 (d/'job.json').write_text(json.dumps(j,indent=2));(d/'grasp_plan.json').write_text(json.dumps({'trial_candidates':[{'base':plan['base'],'T':plan['T_grasp'],'approach':plan['approach'],'preplan':plan['preplan']}]}))
 text=executable(j);compile(text,str(d/'PRELAUNCH_EXECUTABLE.py'),'exec');(d/'PRELAUNCH_EXECUTABLE.py').write_text(text);baseline=adapt_baseline(Path(j['baseline_runner']).read_text(),j);compile(baseline,'frozen native baseline','exec')
 evidence={'asset_id':a,'execution_ready':True,'planned_range_deg':float(__import__('numpy').rad2deg(plan['planned_reached_state'])),'full_range_not_used_as_new_execution_gate':True,'entry_runner_sha256':j['expected_runner_sha256'],'generated_executable_sha256':hashlib.sha256(text.encode()).hexdigest(),'native_position_command_present':'controller.command_width(jaw_width_command)' in baseline,'camera_encoder_preflight_present':'RECORDING_READY.json' in text,'source_identity_check_present':'PRELAUNCH_SOURCE_MISMATCH_NO_ISAAC_STARTED' in text,'frozen_controller_changed':False}
 (d/'PREFLIGHT.json').write_text(json.dumps(evidence,indent=2));results.append(evidence)
 casehashes[a]={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [d/'job.json',d/'planning/candidate.json',d/'source/report.json',d/'grasp_plan.json',d/'asset/manifest.json',d/'authored_offline_export.json']}
freeze={'registered_manifest_sha256':hashlib.sha256((O/'REGISTERED_OBJECTS.json').read_bytes()).hexdigest(),'source_hashes':hashes,'case_input_hashes':casehashes,'physical_cases':reg['physical_evaluation_order'],'reserved_cases':reg['reserved_physical_cases'],'results':results,'no_algorithm_changes':True,'no_retries':True,'maximum_attempts_each':1}
with (O/'PHYSICAL_BATCH_FREEZE.json').open('x') as f:json.dump(freeze,f,indent=2)
print(json.dumps(results,indent=2))
