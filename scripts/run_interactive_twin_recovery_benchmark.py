"""Frozen 12-scene mobile recovery, then DEV oracle physics decomposition."""
import argparse
import concurrent.futures
import copy
import csv
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import threading
import multiprocessing
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from run_interactive_twin_benchmark import Benchmark,read,write,sha,resolve,load_config


def summary_safe(value):
 """Missing native failure metrics are null; raw reports remain untouched."""
 if isinstance(value,float) and not math.isfinite(value):return None
 if isinstance(value,dict):return {k:summary_safe(v) for k,v in value.items()}
 if isinstance(value,list):return [summary_safe(v) for v in value]
 return value


class RecoveryBenchmark(Benchmark):
 def __init__(self,config):
  self.experiment=config
  original=read(resolve(config['original_config']))
  original['output']=config['output'];original['runtime']['deadline_shanghai']=config['deadline_shanghai']
  super().__init__(original)
  self.original=resolve(config['original_output'])
  self.manifest=read(self.original/'frozen_benchmark_manifest.json')
  self.calibration=read(self.original/'robot_calibration/robot_calibration.json')
  self.sensitivity=read(self.original/'sensitivity/sensitivity_report.json')
  self.table_lock=threading.Lock()
  for key in ('http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','ALL_PROXY','all_proxy'):
   self.env.pop(key,None)

 def _code_identity(self):
  result=super()._code_identity()
  for p in sorted([*ROOT.glob('interactive_twin_recovery/*.py'),ROOT/'scripts/run_interactive_twin_recovery_benchmark.py',ROOT/'scripts/run_interactive_twin_recovery_episode.py']):
   result[str(p.relative_to(ROOT))]=sha(p)
  return result

 def verified_code_history(self,previous,current):
  path=self.output/'implementation_amendments.json'
  amendments=read(path).get('amendments',[]) if path.exists() else []
  for name in set(previous)|set(current):
   before,after=previous.get(name),current.get(name)
   if before==after:continue
   if not (name.startswith('interactive_twin_recovery/') or name in
           ('scripts/run_interactive_twin_recovery_benchmark.py','scripts/run_interactive_twin_recovery_episode.py')):
    return False
   reachable={before}
   for _ in amendments:
    for row in amendments:
     if row.get('file')==name and row.get('from') in reachable:reachable.add(row.get('to'))
   if after not in reachable:return False
  return True

 def freeze_recovery(self):
  episodes=[e for e in self.manifest['episodes'] if e['split']=='TEST']
  if len(episodes)!=12:raise RuntimeError('EXACT_FROZEN_12_EPISODES_REQUIRED')
  inputs={}
  for episode in episodes:
   out=self.original/'episodes'/episode['episode_id']
   for p in (out/'planning/cooked_initial.json',out/'planning/plan.json',out/'initial_visual_handle_world.json',out/'source/report.json',out/'asset/manifest.json'):
    inputs[str(p)]=sha(p)
  frozen={'schema':'mobile-oracle-experiment-v1','original_manifest_sha256':sha(self.original/'frozen_benchmark_manifest.json'),
          'episodes':episodes,'config':self.experiment,'input_sha256':inputs,'code_sha256':self._code_identity(),
          'frozen_baselines':['f5dcc6f','cd616606415bae9176f7545dd3e071f5e658fcbe'],
          'scope':'SIM_TO_SIM_BLIND_SYSID; kinematic simulated SE2 reposition, not real navigation',
          'selection_changed':False,'excluded_failures':False}
  path=self.output/'frozen_manifest.json'
  if path.exists():
   previous=read(path)
   if {k:v for k,v in previous.items() if k!='code_sha256'}!={k:v for k,v in frozen.items() if k!='code_sha256'}:
    raise RuntimeError('FROZEN_RECOVERY_EXPERIMENT_CHANGED')
   if not self.verified_code_history(previous['code_sha256'],frozen['code_sha256']):
    raise RuntimeError('UNDOCUMENTED_RECOVERY_CODE_CHANGE')
  else:write(path,frozen)
  return episodes

 def run(self,job):
  out=Path(job['output']);report=out/'report.json';identity=self._job_identity(job)
  if report.exists():
   previous=read(out/'cache_identity.json') if (out/'cache_identity.json').exists() else {}
   if (previous.get('job')!=identity['job'] or previous.get('input_sha256')!=identity['input_sha256']
       or not self.verified_code_history(previous.get('code_sha256',{}),identity['code_sha256'])):
    raise RuntimeError('STALE_NATIVE_RESULT:'+str(out))
   return read(report)
  if self.expired():return {'status':'CUTOFF_05_00','success':False}
  out.mkdir(parents=True,exist_ok=True);write(out/'job_private.json',job);write(out/'cache_identity.json',identity)
  entry='run_interactive_twin_recovery_episode.py' if job.get('mobile_platform') else 'run_interactive_twin_episode.py'
  command=[self.python,str(ROOT/'scripts'/entry),'--job',str(out/'job_private.json')]
  started=time.time()
  with (out/'process.log').open('w') as log:
   p=subprocess.Popen(command,cwd=ROOT,env=self.env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   write(out/'process.json',{'pid':p.pid,'command':command,'start_time':started})
   timeout=max(.01,min(job['wall_clock_budget_s']+35,self.deadline-time.time()))
   try:p.wait(timeout=max(.01,timeout-30))
   except subprocess.TimeoutExpired:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=max(.01,min(30,self.deadline-time.time())))
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
   write(out/'process.json',{'pid':p.pid,'command':command,'start_time':started,'end_time':time.time(),'exit_code':p.returncode})
  if not report.exists():write(report,{'status':'PROCESS_FAILED','success':False,'exit_code':p.returncode})
  return read(report)

 def trial(self,episode,plan,source,out,gpu,route=None):
  old=self.original/'episodes'/episode['episode_id']
  job=self.job(str(out.relative_to(self.output)),source=source,asset=old/'asset',plan=plan,gpu=gpu,
               mode='kinematics',episode_id=episode['episode_id'],seed=episode['seed'],
               initial_articulation_rad=episode['initialization_only']['joint_position_rad'],
               plant=self.reference_parameters(episode),continue_manipulation=True,mobile_platform=True)
  job['wall_clock_budget_s']=self.experiment['mobile']['episode_wall_s']
  if route:job['mobile_route']={**route,'deadline_unix_s':self.deadline}
  attempts=[]
  started=time.monotonic();historical_seconds=0.
  for index in range(min(12,len(read(plan)['trial_candidates']))):
   folder=out/f'candidate_{index:02d}'
   if (folder/'report.json').exists():
    previous=read(folder/'job_private.json');r=self.run(previous)
    timing=read(folder/'process.json')
    end=timing.get('end_time',(folder/'report.json').stat().st_mtime)
    historical_seconds+=max(0.,end-timing['start_time'])
    attempts.append({'candidate':index,'path':str(folder),'report':summary_safe(r)})
    if r.get('bilateral_hold_established'):break
    continue
   if (folder/'process.json').exists():
    pid=read(folder/'process.json')['pid']
    try:os.kill(pid,0)
    except ProcessLookupError:pass
    else:raise RuntimeError('EXISTING_NATIVE_PROCESS_STILL_RUNNING:'+str(pid))
   remaining=self.experiment['mobile']['configuration_execution_wall_s']-historical_seconds-(time.monotonic()-started)
   if remaining<=35:break
   candidate={**job,'candidate':index,'output':str(out/f'candidate_{index:02d}')}
   candidate['wall_clock_budget_s']=min(candidate['wall_clock_budget_s'],remaining-35)
   r=self.run(candidate);attempts.append({'candidate':index,'path':candidate['output'],'report':summary_safe(r)})
   if r.get('bilateral_hold_established') or self.expired():break
  return attempts

 def mobile_episode(self,episode,gpu):
  from interaction_identification.contact_probe import robot_only_model
  from interactive_twin.planning import plan_grasps
  from interactive_twin_recovery.mobile import scene_at,eligible,recover
  eid=episode['episode_id'];out=self.output/'mobile'/eid;old=self.original/'episodes'/eid
  out.mkdir(parents=True,exist_ok=True)
  summary_path=out/'summary.json'
  if summary_path.exists():return read(summary_path)
  summary={'episode_id':eid,'asset_id':episode['asset_id'],'fixed_base':None,'mobile_base':None,'recovery_triggered':False,
           'fixed_attempts':[],'mobile_attempts':[],'failure_reason':None}
  try:
   if self.expired():raise RuntimeError('CUTOFF_05_00')
   initial=read(old/'source/report.json')['robot_base_pose'];summary['initial_base']=initial
   visual=read(old/'initial_visual_handle_world.json');export=read(old/'planning/cooked_initial.json')
   model=robot_only_model(ROOT/'config/piper.urdf');scene=scene_at(ROOT,export,model,initial,initial)
   fixed=read(out/'fixed_plan.json') if (out/'fixed_plan.json').exists() else plan_grasps(model,scene,scene.moving_reference,visual,initial,seed=episode['seed'],budget=12,
                     wall_clock_s=self.experiment['mobile']['fixed_plan_wall_s'],candidate_grid=read(old/'planning/plan.json')['candidate_grid'])
   write(out/'fixed_plan.json',fixed)
   summary['fixed_base']={'grasp_feasible':bool(fixed['trial_candidates']),'candidate_count':len(fixed['trial_candidates']),
                          'failure_funnel':dict(__import__('collections').Counter(r['status'] for r in fixed['rows']))}
   if fixed['trial_candidates']:
    summary['fixed_attempts']=self.trial(episode,out/'fixed_plan.json',old/'source',out/'fixed_execution',gpu)
    summary['failure_reason']=summary['fixed_attempts'][-1]['report']['status'] if summary['fixed_attempts'] else 'CONFIGURATION_EXECUTION_BUDGET'
    # Contact/fit/slip failure never triggers a reposition.
    summary['mobile_base']={'triggered':False,'reason':'FIXED_ARM_FEASIBLE_NO_REPOSITION'}
    return summary
   if not eligible(fixed):
    summary['failure_reason']='NOT_REACHABILITY_RECOVERY_ELIGIBLE';return summary
   summary['recovery_triggered']=True
   recovery=read(out/'base_search.json') if (out/'base_search.json').exists() else recover(ROOT,export,fixed,visual,initial,self.experiment['mobile']['search'],seed=episode['seed'],deadline=self.deadline)
   write(out/'base_search.json',recovery)
   if not recovery['selected']:
    summary['mobile_base']={'grasp_feasible':False,'reason':recovery['status']}
    summary['failure_reason']=recovery['status'];return summary
   selected=recovery['selected'];write(out/'mobile_plan.json',selected['plan'])
   source=copy.deepcopy(read(old/'source/report.json'));source['robot_base_pose']=selected['base']
   write(out/'mobile_source/report.json',source)
   path={**selected['route'],'initial_base':initial}
   summary['mobile_base']={'grasp_feasible':True,'selected_pose':selected['base'],'travel_m':path['translation_m'],
                           'yaw_travel_deg':path['rotation_deg'],'planned_minimum_margin_rad':selected['score'][0],
                           'candidate_count':len(selected['plan']['trial_candidates'])}
   summary['mobile_attempts']=self.trial(episode,out/'mobile_plan.json',out/'mobile_source',out/'mobile_execution',gpu,path)
   summary['failure_reason']=summary['mobile_attempts'][-1]['report']['status'] if summary['mobile_attempts'] else 'CUTOFF_05_00'
   return summary
  except Exception as error:
   summary['failure_reason']=str(error);summary['error_type']=type(error).__name__;return summary
  finally:write(summary_path,summary_safe(summary))

 def mobile_stage(self,episodes):
  groups={}
  for e in episodes:groups.setdefault(e['asset_id'],[]).append(e)
  # SciPy's FK callbacks hold the Python GIL. Independent processes preserve
  # each episode's original wall budget instead of timing out queued threads.
  with concurrent.futures.ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('spawn')) as pool:
   futures=[pool.submit(_mobile_lane,self.experiment,gpu,group) for gpu,group in zip(self.gpus,groups.values())]
   for future in concurrent.futures.as_completed(futures):
    future.result();self.mobile_table(episodes)
  return self.mobile_table(episodes)

 def mobile_table(self,episodes):
  with self.table_lock:return self._mobile_table(episodes)

 def _mobile_table(self,episodes):
  rows=[]
  for e in episodes:
   p=self.output/'mobile'/e['episode_id']/'summary.json';s=read(p) if p.exists() else {}
   fixed=s.get('fixed_attempts',[]);mobile=s.get('mobile_attempts',[])
   fr=fixed[-1]['report'] if fixed else {};mr=mobile[-1]['report'] if mobile else fr
   def identified(r):return (r.get('estimate') or {}).get('joint_type')=='revolute' and r.get('accepted_estimate_count',0)>0
   row={'asset':e['asset_id'],'config':e['episode_id'],'initial_base':s.get('initial_base'),
        'fixed_grasp_feasible':(s.get('fixed_base') or {}).get('grasp_feasible',False),
        'fixed_grasp_success':fr.get('bilateral_hold_established',False),'fixed_ID_success':identified(fr),'fixed_5deg_success':fr.get('success',False),
        'mobile_selected_pose':(s.get('mobile_base') or {}).get('selected_pose'),
        'mobile_travel_m':(s.get('mobile_base') or {}).get('travel_m',0.),
        'mobile_grasp_feasible':bool((s.get('mobile_base') or {}).get('grasp_feasible',False) or (s.get('fixed_base') or {}).get('grasp_feasible',False)),
        'mobile_grasp_success':mr.get('bilateral_hold_established',False),'mobile_ID_success':identified(mr),'mobile_5deg_success':mr.get('success',False),
        'axis_error':(mr.get('evaluation') or {}).get('axis_angular_error_deg'),
        'minimum_joint_margin_rad':mr.get('minimum_joint_margin_rad'),
        'recovered_grasp_feasible':bool(not (s.get('fixed_base') or {}).get('grasp_feasible',False) and (s.get('mobile_base') or {}).get('grasp_feasible',False)),
        'recovered_manipulation':bool(not fr.get('success',False) and mobile and mr.get('success',False)),
        'failure_reason':s.get('failure_reason','NOT_RUN')}
   rows.append(row)
  path=self.output/'cross_object_mobile.csv';tmp=path.with_suffix('.tmp')
  with tmp.open('w') as f:
   writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
  tmp.replace(path)
  summary={'denominator':12,'completed':sum(r['failure_reason']!='NOT_RUN' for r in rows),
           **{key:sum(bool(r[key]) for r in rows) for key in ('fixed_grasp_feasible','fixed_ID_success','fixed_5deg_success','mobile_grasp_feasible','mobile_ID_success','mobile_5deg_success','recovered_grasp_feasible','recovered_manipulation')},'rows':rows}
  write(self.output/'mobile_summary.json',summary);return summary


def _mobile_lane(config,gpu,episodes):
 import faulthandler
 faulthandler.enable()
 bench=RecoveryBenchmark(config)
 return [bench.mobile_episode(e,gpu) for e in episodes]


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True)
 p.add_argument('--stage',choices=['mobile','oracle','prior','full','summary'],default='full');a=p.parse_args()
 bench=RecoveryBenchmark(load_config(a.config));episodes=bench.freeze_recovery()
 if a.stage in ('mobile','full'):bench.mobile_stage(episodes)
 if a.stage in ('oracle','prior','full'):
  if not (bench.output/'mobile_summary.json').exists() or read(bench.output/'mobile_summary.json')['completed']!=12:
   raise RuntimeError('RUN_ALL_12_MOBILE_EPISODES_BEFORE_PHYSICS')
  from interactive_twin_recovery.oracle import run_decomposition,run_prior
  if a.stage in ('oracle','full'):run_decomposition(bench)
  if a.stage in ('prior','full'):run_prior(bench)
 if a.stage=='summary':bench.mobile_table(episodes)


if __name__=='__main__':main()
