from pathlib import Path
import json,subprocess,os,concurrent.futures,hashlib,xml.etree.ElementTree as ET
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/door_transfer4_20261012';python='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
def plan(i):
 case=out/i;job=json.loads((case/'planning_job.json').read_text());folder=case/'planning';folder.mkdir(exist_ok=True)
 with (case/'planning.log').open('w') as log:code=subprocess.run([python,'-u',str(ROOT/'scripts/plan_shared_door_skill.py'),'--job',str(case/'planning_job.json'),'--output',str(folder),'--wall-s','300'],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT).returncode
 if not (folder/'candidate.json').exists():
  (case/'planning_summary.json').write_text(json.dumps({'planning':False,'planning_process_returncode':code,'blocker':'BOUNDED_SHARED_APPROACH_IK_SEARCH_NO_PLAN','physical_attempts':0},indent=2));return
 c=json.loads((folder/'candidate.json').read_text());(case/'whole_plan.json').write_text(json.dumps(c,indent=2));(case/'grasp_plan.json').write_text(json.dumps({'trial_candidates':[{'base':c['base'],'T':c['T_grasp'],'preplan':c['preplan'],'approach':c['approach']}]},indent=2));source=case/'source/report.json';s=json.loads(source.read_text());s['robot_base_pose']=c['base'];source.write_text(json.dumps(s,indent=2));(case/'job.json').write_text(json.dumps(job,indent=2));summary=json.loads((folder/'planning_summary.json').read_text());summary.update(planning=True,physical_attempts=0,shared_selection_utility=c['shared_selection_utility']);(case/'planning_summary.json').write_text(json.dumps(summary,indent=2));print('PLAN',i,summary,flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:list(pool.map(plan,['45176','45134','45443']))
for i in ['45415','45176','45134','45443']:
 case=out/i
 if not (case/'job.json').exists():continue
 job=json.loads((case/'job.json').read_text());job.update(expected_runner_sha256=sha(ROOT/'scripts/run_shared_door_skill.py'),shared_policy_sha256=sha(ROOT/'scripts/shared_door_execution.py'));joint=ET.parse(Path(job['asset_root'])/'urdf'/f'{i}.urdf').getroot().find(f"joint[@name='{job['skill_inputs']['articulation_model']['joint']}']");job['skill_inputs']['articulation_model'].update(urdf_origin=dict(joint.find('origin').attrib),origin_frame='parent',axis_frame='joint');job['shared_execution']={'progress_window_s':.5,'nonprogress_windows':3,'probe':'first_saved_arc_step','centering':'verified JawCenteredClosure target only; original displacement/speed bounds','grasp_reference_reset':False};(case/'job.json').write_text(json.dumps(job,indent=2))
print('ALL_OFFLINE_PLANS_COMPLETE',flush=True)
