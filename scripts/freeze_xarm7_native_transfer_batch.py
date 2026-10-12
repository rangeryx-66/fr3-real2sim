import json,hashlib,inspect
from pathlib import Path
from types import SimpleNamespace
from xarm7_native_gripper import ROOT,configuration
from xarm7_native_execution import executable,adapt_baseline,adapt_loader
import xarm7_door_adapter as frozen
O=ROOT/'results/xarm7_native_transfer_repair_20261012';reg=json.loads((O/'ROUND_REGISTRATION.json').read_text());comparison=json.loads((O/'OFFLINE_COMPARISON.json').read_text())
for a in ['45443','45249']:
 d=O/a;p=d/'planning/candidate.json'
 if not p.exists():continue
 plan=json.loads(p.read_text());j=json.loads((d/'planning_job.json').read_text());source=json.loads((d/'source/report.json').read_text());source['robot_base_pose']=plan['base'];(d/'source/report.json').write_text(json.dumps(source,indent=2));(d/'job.json').write_text(json.dumps(j,indent=2));(d/'grasp_plan.json').write_text(json.dumps({'trial_candidates':[{'base':plan['base'],'T':plan['T_grasp'],'approach':plan['approach'],'preplan':plan['preplan']}]}))
 code=executable(j);compile(code,'actual frozen native executable','exec');(d/'PRELAUNCH_EXECUTABLE.py').write_text(code)
 class LoaderReady(Exception):pass
 def capture_loader(source,c,out):
  loader=adapt_loader(source,c,out);compile(loader,'actual frozen native loader','exec');(d/'PRELAUNCH_LOADER.py').write_text(loader);raise LoaderReady()
 ns=dict(frozen.__dict__,configuration=configuration,adapt_xarm_loader=capture_loader);exec(compile(inspect.getsource(frozen.bootstrap_job),'frozen actual loader capture','exec'),ns)
 args=SimpleNamespace(source=d/'source',asset_root=Path(j['asset_root']),output=d/'prelaunch_only',gpu=j['gpu'],ownership=None);args.output.mkdir(exist_ok=True)
 try:ns['bootstrap_job'](args,plan['base'],j)
 except LoaderReady:pass
c=comparison.get('45249',{});eligible=bool('new_shared_plan' in c and c['new_shared_plan']['normal_channel']>c['frozen_plan']['normal_channel'] and c['new_shared_plan']['planned_reached_deg']>=30-1e-9 and all(x['ok'] for x in c['new_shared_plan']['path_checks']))
(O/'45249_OFFLINE_DECISION.json').write_text(json.dumps({'eligible_for_one_physical_attempt':eligible,'reason':'shared direct-normal opening channel improves over frozen candidate, original collision/IK path remains feasible; an engineering hypothesis, not certification','evidence':{k:{m:v[m] for m in ['normal_channel','new_score','legacy_score','planned_reached_deg','pitch','roll']} for k,v in c.items()},'new_runtime_gates':False},indent=2))
files=[Path(p) for p in reg['source_hashes']]+[ROOT/'scripts/xarm7_native_transfer_contact.py',ROOT/'scripts/run_xarm7_native_transfer_repair.py']
freeze={'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},'original_controller_source_changes':[p for p,h in reg['source_hashes'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h],'native_config_sha256':hashlib.sha256((ROOT/'results/xarm7_native_gripper_20261012/NATIVE_CONFIG.json').read_bytes()).hexdigest(),'gpu':4,'physical_cases':['45443']+(['45249'] if eligible else []),'attempts_each_max':1,'retries':0,'code_and_compact_report_publication_only':True}
(O/'PHYSICAL_BATCH_FREEZE.json').write_text(json.dumps(freeze,indent=2));print(freeze['physical_cases'],freeze['original_controller_source_changes'])
