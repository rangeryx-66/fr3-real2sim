"""Bounded multi-asset SIM_TO_SIM_BLIND_SYSID benchmark; existing baselines are read-only."""
import argparse,concurrent.futures,copy,hashlib,json,os,signal,subprocess,sys,time
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def read(path):return json.loads(Path(path).read_text())
def write(path,value):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2,allow_nan=False));tmp.replace(path)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def resolve(path):
 p=Path(path);return p.resolve() if p.is_absolute() else (ROOT/p).resolve()

def load_config(path):
 text=Path(path).read_text()
 try:return json.loads(text)
 except ValueError:
  import yaml
  return yaml.safe_load(text)

class Benchmark:
 def __init__(self,config):
  self.config=config;self.output=resolve(config['output']);self.output.mkdir(parents=True,exist_ok=True)
  self.deadline=datetime.fromisoformat(config['runtime']['deadline_shanghai']).timestamp()
  self.env=dict(os.environ);self.env.pop('PYTHONPATH',None);self.env.pop('CUDA_VISIBLE_DEVICES',None)
  self.python=config['runtime']['isaac_python'];self.gpus=config['runtime']['gpus'];self.manifest=None
  self.calibration=None;self.sensitivity=None
 def expired(self):return time.time()>=self.deadline
 def freeze(self):
  from interactive_twin.manifest import build_manifest,save_frozen_manifest,verify_frozen_manifest
  file=self.output/'frozen_benchmark_manifest.json'
  if file.exists():
   if read(self.output/'frozen_config.json')!=self.config:raise RuntimeError('FROZEN_TEST_CONFIG_CHANGED_USE_NEW_VERSION_AND_RERUN_ALL_TEST_ASSETS')
   self.manifest=verify_frozen_manifest(file,verify_inputs=True);return self.manifest
  gate=self.output/'dev_gate.json'
  if not gate.exists() or not read(gate).get('recorded'):raise RuntimeError('DEV_MUST_COMPLETE_OR_RECORD_BLOCKER_BEFORE_TEST_FREEZE')
  # Parameters/selection freeze before TEST. DEV diagnostics remain separate.
  inputs=[*ROOT.glob('interactive_twin/*.py'),ROOT/'scripts/run_interactive_twin_episode.py',ROOT/'scripts/run_interactive_twin_benchmark.py',*[ROOT/'interaction_identification'/name for name in ('act2see_loop.py','fitting.py','contact_probe.py')]]
  self.manifest=build_manifest([resolve(p) for p in self.config['prepared_roots']],ranking_paths=[resolve(p) for p in self.config['ranking_paths']],source_roots=self.config['source_roots'],policy=self.config['policy'],frozen_algorithm_files=inputs)
  save_frozen_manifest(self.manifest,file);write(self.output/'frozen_config.json',self.config)
  if not self.manifest['selection_complete']:raise RuntimeError('INSUFFICIENT_DATASET_ASSETS_SEE_FROZEN_INVENTORY')
  return self.manifest
 def job(self,name,*,source=None,asset=None,plan=None,gpu=1,mode='physics_reference',**extra):
  dev=self.config['dev'];asset=resolve(asset or dev['asset_root']);out=self.output/name
  job={'episode_id':name,'mode':mode,'role':'reference','source':str(resolve(source or dev['source'])),'asset_root':str(asset),'plan':str(resolve(plan or dev['plan'])),'output':str(out),
   'gpu':gpu,'deadline_shanghai':self.config['runtime']['deadline_shanghai'],'wall_clock_budget_s':self.config['runtime']['episode_wall_clock_s'],
   'frozen_proxy_sha256':sha(asset/'manifest.json'),'continue_manipulation':False,'robot_calibration_id':self.calibration['calibration_id'] if self.calibration else 'PENDING_ROBOT_ONLY_CALIBRATION',**extra}
  episode=next((e for e in (self.manifest or {}).get('episodes',[]) if e['episode_id']==job['episode_id']),None)
  if episode and episode['split']=='TEST':
   job.update(budget_asset_id=episode['asset_id'],asset_wall_clock_budget_s=episode['budgets']['wall_clock_seconds'],asset_sysid_simulation_budget=episode['budgets']['sysid_simulations'],episode_sysid_simulation_budget=self.config['physics'].get('per_configuration_simulation_budget',16))
  return job
 def _code_identity(self):
  files=sorted([*ROOT.glob('interactive_twin/*.py'),ROOT/'scripts/run_interactive_twin_episode.py',ROOT/'scripts/run_interactive_twin_benchmark.py',ROOT/'src/r1a7_articulated_sim_server.py',*[ROOT/'interaction_identification'/name for name in ('act2see_loop.py','fitting.py','contact_probe.py')]])
  return {str(p.relative_to(ROOT)):sha(p) for p in files}
 def _job_identity(self,job):
  inputs={}
  for key in ('plan','initial_visual','replay_commands','initial_estimate','initial_estimate_memory','reference_safety_memory'):
   if key=='plan' and job['mode']=='plan':continue
   path=Path(job[key]) if job.get(key) else None
   if path and path.is_file():inputs[key]=sha(path)
   if key=='reference_safety_memory' and path and (path.parent/'adopted_estimate.json').is_file():inputs['adopted_estimate']=sha(path.parent/'adopted_estimate.json')
  for key,path in [('scene_prior',Path(job['source'])/'report.json'),('asset_manifest',Path(job['asset_root'])/'manifest.json')]:
   if path.is_file():inputs[key]=sha(path)
  if job.get('import_spec'):
   spec=job['import_spec'];run=resolve(spec['run_dir'])
   for name in ('report.json','command_tape.json','structured_memory.json'):
    path=run/name
    if path.is_file():inputs['import_'+name]=sha(path)
   original_job=self._source_job_path(spec)
   if original_job.is_file():inputs['import_job']=sha(original_job)
   for name in ('observations.json','experiment_code_hashes.json','native_robot_dofs.json'):
    path=run/name
    if path.is_file():inputs['import_'+name]=sha(path)
  payload={'job':job,'input_sha256':inputs,'code_sha256':self._code_identity()}
  return {**payload,'sha256':hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest()}
 def _budget(self,job,action,elapsed_s=0.):
  asset_id=job.get('budget_asset_id')
  if asset_id is None:return {'remaining_wall_s':float('inf'),'remaining_sysid_runs':float('inf')}
  import fcntl
  folder=self.output/'asset_budgets';folder.mkdir(parents=True,exist_ok=True);path=folder/(str(asset_id)+'.json')
  with (folder/(str(asset_id)+'.lock')).open('a+') as lock:
   fcntl.flock(lock.fileno(),fcntl.LOCK_EX)
   state=read(path) if path.exists() else {'asset_id':str(asset_id),'active_wall_seconds':0.,'physical_launches':0,'sysid_launches':0,'episode_sysid_launches':{},'jobs':{}}
   wall_limit=float(job['asset_wall_clock_budget_s']);count_limit=int(job['asset_sysid_simulation_budget']);episode_limit=int(job.get('episode_sysid_simulation_budget',16));eid=job['episode_id'];name=str(job['output']);sysid=job['mode'] in ('physics_reference','sensitivity_reference','replay','updated_interaction')
   remaining=max(0.,wall_limit-state['active_wall_seconds']);count=count_limit-state['sysid_launches'];episode_remaining=episode_limit-state['episode_sysid_launches'].get(eid,0)
   if action=='reserve':
    if remaining<=0:raise RuntimeError('ASSET_ACTIVE_WALL_CLOCK_BUDGET_EXHAUSTED')
    if sysid and (count<=0 or episode_remaining<=0):raise RuntimeError('ASSET_OR_CONFIGURATION_SYSID_BUDGET_EXHAUSTED')
    if any(row['status']=='RUNNING' for row in state['jobs'].values()):raise RuntimeError('ASSET_ALREADY_HAS_RUNNING_JOB_RECONCILE_BUDGET_BEFORE_RESTART')
    state['physical_launches']+=1
    if sysid:state['sysid_launches']+=1;state['episode_sysid_launches'][eid]=state['episode_sysid_launches'].get(eid,0)+1
    state['jobs'][name]={'status':'RUNNING','mode':job['mode'],'episode_id':eid,'started_at_unix_s':time.time(),'sysid':sysid}
   elif action=='finish':
    if name not in state['jobs'] or state['jobs'][name]['status']!='RUNNING':raise RuntimeError('BUDGET_RESERVATION_MISSING')
    state['active_wall_seconds']+=float(elapsed_s);state['jobs'][name].update(status='FINISHED',active_wall_seconds=float(elapsed_s))
   state.update(wall_clock_limit_s=wall_limit,sysid_limit=count_limit,per_configuration_sysid_limit=episode_limit,budget_scope='all configurations of this asset; active rollout time excludes queue time')
   write(path,state)
   return {'remaining_wall_s':remaining,'remaining_sysid_runs':min(count,episode_remaining)}
 def _source_job_path(self,spec):
  source=resolve(spec['run_dir']);path=resolve(spec['job_file']) if spec.get('job_file') else source/'job_private.json'
  adjacent=source.parent/(source.name+'_job.json')
  return adjacent if not path.is_file() and adjacent.is_file() else path
 def _refresh_observable_import(self,spec,destination,analysis_episode_id=None):
  """Re-project immutable raw observations with the current observable boundary."""
  import numpy as np
  from interactive_twin.execution import save_observable_logs
  source=resolve(spec['run_dir']);destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
  paths={name:source/name for name in ('observations.json','command_tape.json','report.json')}
  if not all(path.is_file() for path in paths.values()):raise RuntimeError('RAW_IMPORT_ARTIFACTS_PENDING')
  job_path=self._source_job_path(spec)
  if not job_path.is_file():raise RuntimeError('EXPLICIT_IMPORT_REQUIRES_ORIGINAL_JOB_FILE')
  rows=read(paths['observations.json']);commands=read(paths['command_tape.json']);report=read(paths['report.json']);job=read(job_path)
  original_calibration=job.get('robot_calibration_id');original_episode_id=job.get('episode_id');arm=spec.get('arm_indices',job.get('arm_indices'));mapping_source='explicit logged arm indices'
  native_mapping=source/'native_robot_dofs.json'
  if arm is None and native_mapping.is_file():
   mapping=read(native_mapping);arm=mapping['arm_indices'];mapping_source='native_robot_dofs.json verified named joints'
   if [mapping['joint_names'][i] for i in arm]!=['joint'+str(i) for i in range(1,7)]:raise RuntimeError('NATIVE_ARM_DOF_NAMES_MISMATCH')
  if analysis_episode_id is not None:job['episode_id']=analysis_episode_id
  if arm is None:
   # Recover DOF order from a previously exported encoder observation; never
   # assume native articulation DOF ordering from the URDF declaration order.
   mapping_source='unique raw q column correspondence to original quantized SDK-style q log'
   prior=resolve(spec.get('observable_dir',str(source/'observable')));arm=None
   for probe in ('P1','P2','P3','ROBOT_CALIBRATION','EXPLORATORY'):
    file=prior/(probe+'.json')
    if not file.is_file():continue
    log=read(file);indices=[i for i,row in enumerate(rows) if row['phase']==probe]
    if len(indices)!=len(log.get('signals',{}).get('q_rad',[])) or not indices:continue
    sample=np.linspace(0,len(indices)-1,min(64,len(indices)),dtype=int)
    raw=np.asarray([rows[indices[i]]['q'] for i in sample],float);quantum=np.deg2rad(.001);raw=np.round(raw/quantum)*quantum
    observed=np.asarray(log['signals']['q_rad'])[sample]
    matches=[[i for i in range(raw.shape[1]) if np.allclose(raw[:,i],observed[:,k],rtol=0,atol=1e-12)] for k in range(observed.shape[1])]
    if len(matches)==6 and all(len(match)==1 for match in matches) and len({m[0] for m in matches})==6:arm=[m[0] for m in matches];break
   if arm is None:raise RuntimeError('IMPORTED_ARM_DOF_MAPPING_UNVERIFIED_REQUIRE_EXPLICIT_ARM_INDICES')
  if len(arm)!=6 or len(set(arm))!=6 or not all(isinstance(i,int) and 0<=i<len(rows[0]['q']) for i in arm):raise RuntimeError('INVALID_ARM_DOF_MAPPING')
  if self.calibration:
   for key,default in [('robot_model_id','f5dcc6f-frozen-piper'),('controller_id','cd61660-constrained-probe+physics-protocol-v1')]:
    if job.get(key,default)!=self.calibration[key]:raise RuntimeError('IMPORTED_OBSERVATION_CALIBRATION_MODEL_MISMATCH:'+key)
   job['robot_calibration_id']=self.calibration['calibration_id']
  save_observable_logs(destination,rows,commands,arm,job,report)
  audit={'method':'current observable projection from immutable raw observations and actual command tape','source':str(source),'source_sha256':{name:sha(path) for name,path in paths.items()},'original_job_sha256':sha(job_path),'original_episode_id':original_episode_id,'analysis_episode_id':job.get('episode_id'),'projection_source_sha256':sha(ROOT/'interactive_twin/execution.py'),'projection_version':'cartesian-input-observable-v2','arm_indices':arm,'arm_mapping_evidence':mapping_source,'original_calibration_id':original_calibration,'analysis_calibration_id':job.get('robot_calibration_id'),'raw_files_changed':False,'measurements_regenerated':False,'quantization':'original published PiPER q/qdot units; unchanged measurement samples'}
  write(destination/'observable/import_audit.json',audit)
  return audit
 def _import_run(self,spec,job,out,identity):
  import shutil
  source=resolve(spec['run_dir']);report_path=source/'report.json'
  if not report_path.exists():return {'status':'BLOCKED_IMPORT_PENDING','success':False,'source':str(source)}
  job_file=self._source_job_path(spec)
  if not job_file.is_file():raise RuntimeError('EXPLICIT_IMPORT_REQUIRES_ORIGINAL_JOB_FILE')
  original_job=read(job_file);report=read(report_path)
  for key in ('asset_root','source','plan'):
   if key in original_job and key in job and resolve(original_job[key])!=resolve(job[key]):raise RuntimeError('IMPORTED_JOB_INPUT_MISMATCH:'+key)
  if original_job.get('plant',{})!=job.get('plant',{}):raise RuntimeError('IMPORTED_PLANT_CONFIGURATION_MISMATCH')
  out.mkdir(parents=True,exist_ok=True)
  # Artifacts are copied/referenced read-only. This is an explicit import of a
  # versioned DEV experiment, not automatic cache reuse under new code.
  copies=['estimated_articulation.json','structured_memory.json','adopted_estimate.json','initial_scene_private.json','plant_setup_private.json','command_tape.json','native_robot_dofs.json','robot_control_check_initial.json','robot_control_check_grasp.json']
  provenance={'source':str(source),'original_job_file':str(job_file),'original_job_sha256':sha(job_file),'original_episode_id':report.get('episode_id'),'explicit_import':True,'files':{},'source_code_version':spec.get('source_code_version','DEV_PRE_FREEZE_VERSION_SEE_ORIGINAL_RUN')}
  for name in copies:
   path=source/name
   if path.exists():shutil.copy2(path,out/name);provenance['files'][name]={'source':str(path),'sha256':sha(path)}
  self._refresh_observable_import(spec,out,analysis_episode_id=job['episode_id'])
  for name in ('contact_baseline.mp4','final_close.png','observations.json','physics_steps.json'):
   if (source/name).exists():
    provenance['files'][name]={'source':str(source/name),'sha256':sha(source/name)}
    if name in ('contact_baseline.mp4','final_close.png') and not (out/name).exists():(out/name).symlink_to(source/name)
  provenance['files']['report.json']={'source':str(report_path),'sha256':sha(report_path)}
  write(out/'import_provenance.json',provenance);write(out/'cache_identity.json',identity);write(out/'job_private.json',job);write(out/'report.json',report)
  return report
 def run(self,job):
  out=Path(job['output']);file=out/'report.json';identity=self._job_identity(job)
  if file.exists():
   cache=out/'cache_identity.json'
   if not cache.exists() or read(cache)['sha256']!=identity['sha256']:raise RuntimeError('STALE_OR_UNVERIFIED_RUN_CACHE_USE_NEW_OUTPUT_OR_EXPLICIT_VERSIONED_IMPORT:'+str(out))
   return read(file)
  if self.expired():return {'status':'CUTOFF_05_00','success':False}
  if job.get('import_spec'):return self._import_run(job['import_spec'],job,out,identity)
  try:budget=self._budget(job,'reserve')
  except RuntimeError as error:return {'status':str(error),'success':False,'budget_blocked':True}
  out.mkdir(parents=True,exist_ok=True);write(out/'job_private.json',job);write(out/'cache_identity.json',identity)
  command=[self.python,str(ROOT/'scripts/run_interactive_twin_episode.py'),'--job',str(out/'job_private.json')];started=time.monotonic()
  try:
   with (out/'process.log').open('w') as log:
    p=subprocess.Popen(command,cwd=ROOT,env=self.env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    write(out/'process.json',{'pid':p.pid,'command':command,'start_time':time.time(),'cache_identity_sha256':identity['sha256']})
    hard_timeout=max(.01,min(job['wall_clock_budget_s']+45,budget['remaining_wall_s'],self.deadline-time.time()));timeout=max(.01,hard_timeout-min(35.,hard_timeout/4))
    try:p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
     os.killpg(p.pid,signal.SIGINT)
     try:p.wait(timeout=max(.01,min(hard_timeout-(time.monotonic()-started),self.deadline-time.time())))
     except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    metadata=read(out/'process.json');metadata.update(exit_code=p.returncode,end_time=time.time());write(out/'process.json',metadata)
   if not file.exists():write(file,{'status':'PROCESS_FAILED','success':False,'exit_code':p.returncode,'episode_id':job['episode_id'],'log':str(out/'process.log')})
   return read(file)
  finally:self._budget(job,'finish',time.monotonic()-started)
 def capability(self):
  from interactive_twin.capability import write_capability_report
  return write_capability_report(self.output/'capability',sdk_root=self.config.get('sdk_root'))
 def _import_calibration(self):
  spec=self.config.get('imports',{}).get('robot_calibration')
  if not spec:return None
  path=resolve(spec['report'])
  if not path.exists():return {'status':'BLOCKED_IMPORT_PENDING','source':str(path)}
  result=read(path)
  if not result.get('no_contact_supervisor_certified') or result.get('repeats',0)<2:raise RuntimeError('IMPORTED_ROBOT_CALIBRATION_NOT_CERTIFIED')
  if result.get('noise_floor_config')!=self.config['noise_floors']:raise RuntimeError('IMPORTED_CALIBRATION_NOISE_FLOORS_DIFFER_FROM_CONFIG')
  self.calibration=result;write(self.output/'robot_calibration/robot_calibration.json',result);write(self.output/'robot_calibration/import_provenance.json',{'source':str(path),'sha256':sha(path),'explicit_import':True,'hardware_calibration':False,'source_logs':spec.get('source_runs',[])})
  return result
 def dev_episode(self):
  return {'episode_id':'dev_'+str(self.config['policy'].get('dev_asset_id','7320'))+'_00','asset_id':str(self.config['policy'].get('dev_asset_id','7320')),'split':'DEV','seed':int(self.config['policy']['seed']),'budgets':copy.deepcopy(self.config['policy']['budgets'])}
 def dev_setup(self):
  episode=self.dev_episode();out=self.output/'episodes'/episode['episode_id'];file=out/'episode_summary.json'
  calibration=self.calibrate()
  if not self.calibration:
   summary={'episode_id':episode['episode_id'],'asset_id':episode['asset_id'],'split':'DEV','stages':{k:{'status':'NOT_RUN'} for k in 'ABCDEFGHI'},'dev_gate_recorded':False,'not_in_unseen_denominator':True}
   summary['stages']['E']={'status':'BLOCKED','reason':calibration.get('status','ROBOT_CALIBRATION_UNAVAILABLE')};write(file,summary);return summary
  if file.exists() and read(file).get('dev_gate_recorded'):
   summary=read(file);self.run(summary['selected_job']);return summary
  spec=self.config.get('imports',{}).get('dev_kinematic',self.config.get('imports',{}).get('dev_reference'));plant=self.config['physics']['hidden_dev_conditions']['LOW']
  job=self.job(f"episodes/{episode['episode_id']}/dev_kinematics",gpu=self.gpus[0],mode='kinematics',continue_manipulation=True,plant=plant,episode_id=episode['episode_id'])
  if spec:job['import_spec']=spec
  report=self.run(job);summary={'episode_id':episode['episode_id'],'asset_id':episode['asset_id'],'split':'DEV','stages':{k:{'status':'NOT_RUN'} for k in 'ABCDEFGHI'},'dev_gate_recorded':True,'selected_job':job,'selected_report':str(Path(job['output'])/'report.json'),'not_in_unseen_denominator':True}
  summary['stages']['A']={'status':'SUCCESS','source':'frozen DEV prepared geometry','proxy_approximate':True,'geometry_improvement_measured':False}
  if report['status']=='BLOCKED_IMPORT_PENDING':
   summary.update(dev_gate_recorded=False);summary['stages']['B']={'status':'BLOCKED','reason':'BLOCKED_IMPORT_PENDING','source':report.get('source')};write(file,summary);return summary
  legal=report.get('bilateral_hold_established',False);identified=report.get('accepted_estimate_count',0)>0 and (report.get('estimate') or {}).get('joint_type')=='revolute'
  summary['stages']['B']={'status':'SUCCESS' if legal else 'FAILED','bilateral':legal,'reason':report['status'],'slip_observability':'partial_simulator_safety_only'}
  summary['stages']['C']={'status':'SUCCESS' if identified else 'FAILED','reason':report['status'],'attempt_history':report.get('attempt_history',[])}
  summary['stages']['D']={'status':'SUCCESS' if identified else 'UNOBSERVABLE','estimate':report.get('estimate'),'evaluation':report.get('evaluation')}
  summary['initial_dev_progress_deg']=report.get('evaluation',{}).get('actual_door_displacement_deg',report.get('evaluation',{}).get('actual_final_door_angle_deg'))
  summary['stages']['I']={'status':'KINEMATIC_ONLY_SUCCESS' if report.get('success') else 'NOT_RUN','reason':'initial D-stage interaction; physics has not been identified','actual_angle_deg':summary['initial_dev_progress_deg'],'physics_updated':False}
  if identified:summary=self.compile_kinematic_prior(episode,summary)
  write(file,summary);write(self.output/'dev_gate.json',{'episode_summary':str(file),'recorded':True,'full_loop_completed':False,'status':'DEV_INTERACTION_MEASURED' if identified else 'DEV_BLOCKED','reason':report['status'],'test_outcomes_used':False});return summary
 def dev_full(self):
  summary=self.dev_setup()
  if not summary.get('dev_gate_recorded'):return summary
  sensitivity=self.sensitivity_stage()
  if sensitivity.get('status')=='OBSERVABLE_RESPONSE' and summary['stages']['D']['status']=='SUCCESS':
   try:summary=self.fit_episode(self.dev_episode(),self.gpus[0])
   except Exception as error:
    out=self.output/'episodes'/self.dev_episode()['episode_id'];summary=read(out/'episode_summary.json');stage=next((k for k in 'EFGHI' if summary['stages'][k]['status']=='NOT_RUN'),'F');summary['stages'][stage]={'status':'FAILED','reason':str(error),'error_type':type(error).__name__};write(out/'episode_summary.json',summary)
  else:
   summary['stages']['E']={'status':'BLOCKED','reason':sensitivity.get('status','DEV_SENSITIVITY_UNAVAILABLE')};summary['stages']['F']={'status':'NOT_IDENTIFIED','accepted_parameters':None};summary['stages']['H']={'status':'BLOCKED','reason':'NO_VALIDATED_INDEPENDENT_HELDOUT_PREDICTION'};write(self.output/'episodes'/self.dev_episode()['episode_id']/'episode_summary.json',summary)
  write(self.output/'dev_gate.json',{'episode_summary':str(self.output/'episodes'/self.dev_episode()['episode_id']/'episode_summary.json'),'recorded':True,'full_loop_completed':all(summary['stages'][k]['status']=='SUCCESS' for k in 'ABCDEGI') and summary['stages']['F']['status']=='IDENTIFIABLE_ON_FROZEN_GRID' and summary['stages']['H']['status']=='EVALUATED','status':'DEV_RUN_COMPLETE_OR_EXPLICITLY_BLOCKED','sensitivity':sensitivity.get('status'),'test_outcomes_used':False})
  return summary
 def _import_sensitivity(self):
  spec=self.config.get('imports',{}).get('sensitivity')
  if not spec:return None
  if not self.calibration:
   calibration=self.calibrate()
   if not self.calibration:return {'status':'BLOCKED_ROBOT_CALIBRATION','detail':calibration}
  report_path=resolve(spec['report']) if spec.get('report') else None
  # Explicit precomputed report is accepted only with its provenance alongside.
  if report_path and report_path.exists():
   result=read(report_path)
   for key in ('robot_model_id','controller_id','robot_calibration_id'):
    expected=self.calibration['calibration_id'] if key=='robot_calibration_id' else self.calibration[key]
    if result.get(key)!=expected:raise RuntimeError('IMPORTED_SENSITIVITY_CALIBRATION_MISMATCH:'+key)
   self.sensitivity=result;write(self.output/'sensitivity/sensitivity_report.json',result);write(self.output/'sensitivity/import_provenance.json',{'source':str(report_path),'sha256':sha(report_path),'runs':spec.get('runs',[]),'explicit_import':True});return result
  from interactive_twin.sysid import NoiseScales,sensitivity_report
  logs={key:[] for key in self.config['physics']['hidden_dev_conditions']};pending=[];failures=[];audits=[]
  for row in spec.get('runs',[]):
   run=resolve(row['run_dir']);final=run/'report.json'
   if not final.exists():pending.append(row['run_dir']);continue
   target=self.output/'sensitivity/imported_observables'/f"{row['condition']}_{row['repeat']}"
   if not (run/'observations.json').exists() or not (run/'command_tape.json').exists():
    failures.append({'run':row['run_dir'],'status':read(final)['status'],'reason':'RAW_OBSERVATIONS_OR_COMMANDS_MISSING'});continue
   audit=self._refresh_observable_import(row,target);file=target/'observable/P2.json'
   if not file.exists():failures.append({'run':row['run_dir'],'status':read(final)['status'],'reason':'P2_NOT_REACHED'});continue
   log=read(file);entry=read(target/'observable/index.json').get('P2',{})
   if not log['provenance'].get('complete') or entry.get('terminal_step_status_uncertain'):
    failures.append({'run':row['run_dir'],'status':'P2_INCOMPLETE_OR_FAILURE_TIMING_UNCERTAIN'});continue
   logs[row['condition']].append(log);audits.append({**audit,'condition':row['condition'],'repeat':row['repeat']})
  if pending:return {'status':'BLOCKED_IMPORT_PENDING','pending':pending,'rerun_launched':False}
  if failures:result={'status':'SENSITIVITY_CENSORED_BY_SAFETY','failures':failures,'repeat_variability_measured':False}
  elif any(len(value)<2 for value in logs.values()):result={'status':'SENSITIVITY_REPEATS_MISSING','counts':{k:len(v) for k,v in logs.items()},'repeat_variability_measured':False}
  else:result=sensitivity_report(logs,NoiseScales(**self.calibration['noise_scales']))
  result['imported_runs']=audits;result['sensitivity_plot']=self.save_sensitivity_plot(logs,result);self.sensitivity=result;write(self.output/'sensitivity/sensitivity_report.json',result);return result
 def calibrate(self):
  from interactive_twin.calibration import calibrate_robot_response
  imported=self._import_calibration()
  if imported is not None:return imported
  file=self.output/'robot_calibration/robot_calibration.json'
  if file.exists():self.calibration=read(file);return self.calibration
  dev=self.config['dev'];plan=read(resolve(dev['plan']));q=plan['trial_candidates'][0]['q_grasp'];src=read(resolve(dev['source'])/'report.json');src['asset_installation'].update(x_m=3.,y_m=3.)
  source=self.output/'robot_calibration/source';write(source/'report.json',src)
  jobs=[self.job(f'robot_calibration/repeat_{i}',source=source,gpu=self.gpus[i%len(self.gpus)],mode='robot_calibration',plant={},robot_start_q=q,episode_id='robot_calibration') for i in range(2)]
  with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:reports=list(pool.map(self.run,jobs))
  if any(r['status']!='ROBOT_CALIBRATION_COMPLETE' for r in reports):
   result={'status':'ROBOT_RESPONSE_CALIBRATION_BLOCKED','reports':reports};write(self.output/'robot_calibration/status.json',result);return result
  logs=[read(Path(j['output'])/'observable/ROBOT_CALIBRATION.json') for j in jobs]
  self.calibration=calibrate_robot_response(logs,noise_floors=self.config['noise_floors'],output_dir=self.output/'robot_calibration');return self.calibration
 def sensitivity_stage(self):
  from interactive_twin.sysid import NoiseScales,sensitivity_report
  imported=self._import_sensitivity()
  if imported is not None:return imported
  file=self.output/'sensitivity/sensitivity_report.json'
  if file.exists():self.sensitivity=read(file);return self.sensitivity
  if not self.calibration:self.calibrate()
  if not self.calibration:return {'status':'BLOCKED_ROBOT_CALIBRATION'}
  dev_file=self.output/'episodes'/self.dev_episode()['episode_id']/'episode_summary.json'
  if not dev_file.exists():self.dev_setup()
  dev=read(dev_file)
  if dev['stages']['D']['status']!='SUCCESS':return {'status':'BLOCKED_DEV_KINEMATIC_IDENTIFICATION'}
  selected=dev['selected_job'];context=self.kinematic_context(selected,Path(selected['output']),self.output/'sensitivity/kinematic_context')
  conditions=self.config['physics']['hidden_dev_conditions'];base=self.job('sensitivity/LOW_0',source=selected['source'],asset=selected['asset_root'],plan=selected['plan'],gpu=self.gpus[0],mode='sensitivity_reference',plant=conditions['LOW'],episode_id='DEV_sensitivity',candidate=selected.get('candidate',0),initial_articulation_rad=selected.get('initial_articulation_rad',0.),initial_estimate=str(self.output/'sensitivity/kinematic_context/kinematic_fit_for_twins.json'),initial_estimate_memory=str(self.output/'sensitivity/kinematic_context/kinematic_memory_for_twins.json'),fixed_fixture=context['scene']['fixture'])
  report=self.run(base)
  if not (Path(base['output'])/'observable/P2.json').exists() or not read(Path(base['output'])/'observable/P2.json')['provenance'].get('complete'):
   result={'status':'BLOCKED_DEV_REFERENCE','report':report};write(file,result);return result
  commands=Path(base['output'])/'command_tape.json';fixed=read(Path(base['output'])/'initial_scene_private.json')['fixture']
  todo=[]
  for level,params in conditions.items():
   for repeat in range(2):
    if level=='LOW' and repeat==0:continue
    todo.append((level,repeat,params))
  alljobs=[base]
  for start in range(0,len(todo),len(self.gpus)):
   jobs=[self.job(f'sensitivity/{label}_{i}',source=base['source'],asset=base['asset_root'],plan=base['plan'],gpu=self.gpus[k],mode='replay',role='reference_sensitivity',plant=params,replay_commands=str(commands),reference_safety_memory=str(Path(base['output'])/'structured_memory.json'),fixed_fixture=fixed,episode_id='DEV_sensitivity',candidate=base.get('candidate',0),initial_articulation_rad=base.get('initial_articulation_rad',0.)) for k,(label,i,params) in enumerate(todo[start:start+len(self.gpus)])]
   with concurrent.futures.ThreadPoolExecutor(max_workers=len(jobs)) as pool:list(pool.map(self.run,jobs))
   alljobs+=jobs
  logs={name:[] for name in conditions};failures=[]
  for j in alljobs:
   folder=Path(j['output']);label=folder.name.rsplit('_',1)[0];r=read(folder/'report.json')
   path=folder/'observable/P2.json';index=read(folder/'observable/index.json') if (folder/'observable/index.json').exists() else {}
   if not path.exists() or not read(path)['provenance'].get('complete') or index.get('P2',{}).get('terminal_step_status_uncertain'):failures.append({'run':str(folder),'status':r['status'],'reason':'P2_INCOMPLETE_OR_UNCERTAIN'});continue
   logs[label].append(read(path))
  if failures:result={'status':'SENSITIVITY_CENSORED_BY_SAFETY','failures':failures,'repeat_variability_measured':False}
  else:result=sensitivity_report(logs,NoiseScales(**self.calibration['noise_scales']))
  result['sensitivity_plot']=self.save_sensitivity_plot(logs,result);write(file,result);self.sensitivity=result;return result
 def reference_parameters(self,episode):
  import random
  if episode.get('split')=='DEV':params=dict(self.config['physics']['hidden_dev_conditions']['LOW'])
  else:
   seed=int.from_bytes(hashlib.sha256((str(self.config['policy']['seed'])+':'+str(episode['asset_id'])).encode()).digest()[:8],'big')
   rng=random.Random(seed+1907);params={'tau_c':rng.uniform(.001,.009),'b':rng.uniform(.1,.8)}
  private=self.output/'reference_plants_private'/str(episode['asset_id'])/'hidden_parameters.json'
  if private.exists() and read(private)['parameters']!=params:raise RuntimeError('FROZEN_REFERENCE_PHYSICS_CHANGED')
  write(private,{'parameters':params,'seed_policy':'benchmark seed + asset id; constant across configurations and all physical reference stages','not_an_estimator_input':True})
  return params
 def save_sensitivity_plot(self,logs,result):
  try:
   from interactive_twin.reporting import plot_sensitivity_curves
   return plot_sensitivity_curves(logs,self.output/'sensitivity/sensitivity_curves.png',report=result)
  except (ImportError,ValueError,KeyError) as error:
   write(self.output/'sensitivity/plot_status.json',{'status':'PLOT_UNAVAILABLE','reason':str(error)});return None
 def prepare_episode(self,episode):
  from interactive_twin.planning import make_deployment_template,prepare_deployment
  out=self.output/'episodes'/episode['episode_id'];asset=out/'asset';out.mkdir(parents=True,exist_ok=True)
  if not (asset/'manifest.json').exists():
   rankings=[]
   for path in self.config['ranking_paths']:rankings+=read(resolve(path))['ranking']
   selected=episode['visual_handle'];rows=[r for r in rankings if str(r['asset_id'])==episode['asset_id'] and r['mesh']==selected['mesh'] and Path(r['asset_root']).resolve()==Path(episode['asset_root']).resolve()]
   if not rows:raise RuntimeError('FROZEN_HANDLE_RANKING_MISSING')
   write(out/'selected_visual_ranking.json',{'ranking':rows})
   subprocess.run([self.config['runtime']['analysis_python'],str(ROOT/'scripts/prepare_semantic_handle_proxy.py'),'--ranking',str(out/'selected_visual_ranking.json'),'--output',str(asset)],cwd=ROOT,env=self.env,check=True,stdout=(out/'proxy_preparation.log').open('w'),stderr=subprocess.STDOUT,timeout=180)
  dev=self.config['dev'];template=make_deployment_template(resolve(dev['asset_root']),resolve(dev['source'])/'report.json')
  deployment=prepare_deployment(episode,asset,template,out)
  return out,asset,deployment
 def kinematic_episode(self,episode,gpu):
  eid=episode['episode_id'];out=self.output/'episodes'/eid;summary={'episode_id':eid,'asset_id':episode['asset_id'],'split':episode['split'],'stages':{k:{'status':'NOT_RUN'} for k in 'ABCDEFGHI'}}
  file=out/'episode_summary.json'
  if file.exists():return read(file)
  try:
   if self.expired():raise RuntimeError('CUTOFF_05_00')
   out,asset,dep=self.prepare_episode(episode);summary['stages']['A']={'status':'SUCCESS','source':'prepared_visual_geometry','proxy_approximate':True,'geometry_improvement_measured':False}
   job=self.job(f'episodes/{eid}/planning',source=dep['source'],asset=asset,plan=out/'planning/plan.json',gpu=gpu,mode='plan',initial_visual=str(out/'initial_visual_handle_world.json'),episode_id=eid,seed=episode['seed'],initial_articulation_rad=episode['initialization_only']['joint_position_rad'],planning_budget_s=600,plant={})
   planning=self.run(job);plan=read(out/'planning/plan.json') if (out/'planning/plan.json').exists() else {'trial_candidates':[]}
   summary['planning_report']=planning
   if not plan['trial_candidates']:summary['stages']['B']={'status':'FAILED','reason':planning['status']};return summary
   # Bounded candidates are tested in preregistered order, never chosen by GT.
   attempts=[];chosen=None;start=time.monotonic()
   for index in range(min(12,len(plan['trial_candidates']))):
    if time.monotonic()-start>episode['budgets']['wall_clock_seconds'] or self.expired():break
    trial=self.job(f'episodes/{eid}/candidate_{index:02d}',source=dep['source'],asset=asset,plan=out/'planning/plan.json',gpu=gpu,mode='kinematics',candidate=index,episode_id=eid,initial_articulation_rad=episode['initialization_only']['joint_position_rad'],continue_manipulation=True,plant=self.reference_parameters(episode))
    r=self.run(trial);attempts.append({'path':trial['output'],'status':r['status'],'bilateral':r.get('bilateral_hold_established',False)})
    if r.get('bilateral_hold_established'):
     chosen=(trial,r);break # A safe established grasp is not reoptimized for fitting outcomes.
   summary['candidate_attempts']=attempts
   if chosen is None:summary['stages']['B']={'status':'FAILED','reason':attempts[-1]['status'] if attempts else 'ASSET_BUDGET_EXHAUSTED'};return summary
   trial,r=chosen;summary['selected_report']=str(Path(trial['output'])/'report.json');summary['selected_job']=trial
   summary['stages']['B']={'status':'SUCCESS','bilateral':True,'slip_observability':'partial_simulator_safety_only'}
   identified=r.get('estimate',{}).get('joint_type')=='revolute' and r.get('accepted_estimate_count',0)>0
   summary['stages']['C']={'status':'SUCCESS' if identified else 'FAILED','reason':r['status'],'attempt_history':r.get('attempt_history',[])}
   summary['stages']['D']={'status':'SUCCESS' if identified else 'UNOBSERVABLE','estimate':r.get('estimate'),'evaluation':r.get('evaluation')}
   summary['stages']['I']={'status':'KINEMATIC_ONLY_SUCCESS' if r.get('success') else 'FAILED','actual_angle_deg':r.get('evaluation',{}).get('actual_door_displacement_deg'),'reason':r['status'],'physics_updated':False}
   if identified:summary=self.compile_kinematic_prior(episode,summary)
   return summary
  except Exception as error:
   missing=next((k for k,v in summary['stages'].items() if v['status']=='NOT_RUN'),'A');summary['stages'][missing]={'status':'FAILED','reason':str(error)};return summary
  finally:write(file,summary)
 def _test_asset_groups(self):
  if self.manifest is None:self.freeze()
  groups={}
  for episode in self.manifest['episodes']:
   if episode['split']=='TEST':groups.setdefault(episode['asset_id'],[]).append(episode)
  return list(groups.values())
 def kinematics(self):
  if not (self.output/'dev_gate.json').exists():self.dev_setup()
  groups=self._test_asset_groups();lanes=[groups[i::len(self.gpus)] for i in range(len(self.gpus))]
  def worker(pair):
   gpu,lane=pair;results=[]
   for group in lane:
    for episode in group:results.append(self.kinematic_episode(episode,gpu))
   return results
  with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.gpus)) as pool:return list(pool.map(worker,zip(self.gpus,lanes)))
 def kinematic_context(self,selected,folder,out):
  from interactive_twin.twin import write_twins
  import numpy as np
  import xml.etree.ElementTree as ET
  folder=Path(folder);out=Path(out);out.mkdir(parents=True,exist_ok=True)
  scene=read(folder/'initial_scene_private.json');W=np.eye(4);W[:3,:3]=scene['asset_rotation'];W[:3,3]=scene['asset_xyz']
  memory=read(folder/'structured_memory.json')
  if memory.get('GT_inputs') is not False:raise RuntimeError('KINEMATIC_STAGE_MEMORY_MUST_BE_EE_ONLY')
  accepted=[record for record in memory['fit_history'] if record.get('accepted')]
  if not accepted:raise RuntimeError('NO_SAVED_PRE_PHYSICS_KINEMATIC_ESTIMATE')
  # The final accepted D-stage estimate is frozen before any physics probes.
  # Keep exactly its EE support; E-stage observations never replace/refine it.
  record=accepted[-1];fit=record['fit'];support=memory['supporting_observations'][:record['observation_count']]
  write(out/'kinematic_fit_for_twins.json',fit)
  write(out/'kinematic_memory_for_twins.json',{'source':'measured EE SE(3) before physics probes only','estimated_articulation':fit,'supporting_observations':support,'attempt_history':memory.get('attempt_history',[]),'fit_history':[record],'GT_inputs':False})
  asset_root=Path(selected['asset_root'])
  if selected.get('initial_articulation_rad',0)!=0:
   from interactive_twin.initial_state import bake_initial_articulation
   initial_bake=bake_initial_articulation(asset_root,out/'initial_twin_zero_frame',selected['initial_articulation_rad'])
   asset_root=Path(initial_bake['asset_root']);write(out/'initial_twin_zero_frame_audit.json',initial_bake)
   if initial_bake['effective_initial_articulation_rad']!=0:raise RuntimeError('TWIN_INITIAL_BAKE_NOT_ZERO')
  manifest=read(asset_root/'manifest.json');prior_urdf=asset_root/'urdf'/f"{manifest['asset_id']}.urdf"
  prior_xml=ET.parse(prior_urdf).getroot()
  # No scalar effective inertia is known. Freeze an opaque original-model
  # signature rather than claiming the previous numerical sentinel was 1 kg m2.
  inertia_source={'inertial_blocks':{link.get('name'):ET.tostring(link.find('inertial'),encoding='unicode') for link in prior_xml.findall('link') if link.find('inertial') is not None},
   'moving_geometry_bounds_prior':manifest.get('moving_source_bounds'),'scale_prior':manifest.get('scale_source_to_meters'),
   'native_loader_sha256':sha(ROOT/'src/r1a7_articulated_sim_server.py'),'mass_model':'geometry; unchanged initial loader model'}
  inertia_prior={'kind':'fixed_articulated_model','sha256':hashlib.sha256(json.dumps(inertia_source,sort_keys=True).encode()).hexdigest(),'scalar_J_eff':None,'frozen':True}
  write(out/'fixed_inertia_prior.json',{'prior':inertia_prior,'model_provenance':inertia_source,'scalar_effective_inertia_identified':False})
  initial_prior={**self.config['physics']['initial_prior'],'J_eff':inertia_prior}
  def compile_twins(path,result=None):
   identity={'asset_manifest_sha256':sha(asset_root/'manifest.json'),'fit':fit,'world':W.tolist(),'initial_physics_prior':initial_prior,'physics_estimate':result,'support':support,'operational_limits':self.config['physics']['operational_joint_limits_rad'],'compiler_sha256':sha(ROOT/'interactive_twin/twin.py')}
   if (path/'twin_versions.json').exists():
    if not (path/'compilation_identity.json').exists() or read(path/'compilation_identity.json')!=identity:raise RuntimeError('STALE_TWIN_ARTIFACT_USE_NEW_EXPERIMENT_VERSION')
    return read(path/'twin_versions.json')
   artifacts=write_twins(asset_root,path,fit,W,initial_physics_prior=initial_prior,physics_estimate=result,ee_poses=support,q_initial=0.,operational_joint_limits_rad=self.config['physics']['operational_joint_limits_rad'],attempt_history=memory.get('attempt_history',[]));write(path/'compilation_identity.json',identity);return artifacts
  return {'scene':scene,'memory':memory,'fit':fit,'support':support,'inertia_prior':inertia_prior,'compile_twins':compile_twins}
 def compile_kinematic_prior(self,episode,summary):
  if summary['stages']['D']['status']!='SUCCESS':return summary
  out=self.output/'episodes'/episode['episode_id']/'kinematic_update'
  selected=summary['selected_job'];context=self.kinematic_context(selected,Path(selected['output']),out)
  twins=context['compile_twins'](out/'twins',{'status':'NOT_IDENTIFIED','accepted_parameters':None,'J_eff_prior_fixed':context['inertia_prior']})
  summary['stages']['G']={'status':'SUCCESS','T2_physics_accepted':False,'versions':twins['versions'],'kinematics_independent_of_physics_identifiability':True,'physics_status':'NOT_IDENTIFIED'}
  summary['kinematic_prior_twins']=twins['versions']
  return summary
 def fit_episode(self,episode,gpu):
  from interactive_twin.sysid import NoiseScales,candidate_grid,fit_resistance,write_analysis,validate_log
  from interactive_twin.twin import write_twins
  import numpy as np
  import xml.etree.ElementTree as ET
  import random
  out=self.output/'episodes'/episode['episode_id'];summary=read(out/'episode_summary.json')
  if summary['stages']['D']['status']!='SUCCESS':return summary
  summary=self.compile_kinematic_prior(episode,summary)
  if not self.calibration or not self.sensitivity or self.sensitivity.get('status')!='OBSERVABLE_RESPONSE':
   summary['stages']['E']={'status':'BLOCKED','reason':'DEV_SENSITIVITY_OR_ROBOT_CALIBRATION_NOT_VALIDATED'};summary['stages']['F']={'status':'NOT_IDENTIFIED','accepted_parameters':None};summary['stages']['H']={'status':'BLOCKED','reason':'NO_INDEPENDENT_HELDOUT_ROLLOUT_WITH_VALIDATED_PHYSICS_PROTOCOL'};write(out/'episode_summary.json',summary);return summary
  selected=summary['selected_job'];context=self.kinematic_context(selected,Path(selected['output']),out)
  scene=context['scene'];memory=context['memory'];fit=context['fit'];support=context['support'];inertia_prior=context['inertia_prior'];compile_twins=context['compile_twins']
  reference={**selected,'mode':'physics_reference','role':'reference','output':str(out/'physics_reference'),'continue_manipulation':False,'robot_calibration_id':self.calibration['calibration_id'],'initial_estimate':str(out/'kinematic_fit_for_twins.json'),'initial_estimate_memory':str(out/'kinematic_memory_for_twins.json'),'fixed_fixture':scene['fixture']}
  # D's explicit import must never be reused as E's response: it has no P1-P4.
  reference.pop('import_spec',None)
  # Hidden parameters are consumed by scene assembly; only observable training
  # logs enter the estimator. The seeded plant is unchanged in final manipulation.
  reference['plant']=self.reference_parameters(episode)
  if episode.get('split')=='DEV' and self.config.get('imports',{}).get('dev_reference'):reference['import_spec']=self.config['imports']['dev_reference']
  r=self.run(reference);reference.pop('import_spec',None);folder=Path(reference['output']);reference_logs={}
  reference['reference_safety_memory']=str(folder/'structured_memory.json')
  if r['status']=='BLOCKED_IMPORT_PENDING':summary['stages']['E']={'status':'BLOCKED','reason':r['status'],'source':r.get('source')};write(out/'episode_summary.json',summary);return summary
  for probe in ('P1','P2','P3'):
   path=folder/'observable'/f'{probe}.json'
   if not path.exists():break
   log=read(path)
   if not log['provenance'].get('complete'):break
   validate_log(log);reference_logs[probe]=log
  if len(reference_logs)!=3:
   summary['stages']['E']={'status':'FAILED','reason':r['status'],'complete_training_probes':list(reference_logs)};write(out/'episode_summary.json',summary);return summary
  summary['stages']['E']={'status':'SUCCESS','training_probe_ids':['P1','P2','P3'],'reference_final_status':r['status']}
  write(out/'kinematic_stage_binding.json',{'source_stage':'D','source_run':str(Path(selected['output'])),'fit_sha256':sha(out/'kinematic_fit_for_twins.json'),'memory_sha256':sha(out/'kinematic_memory_for_twins.json'),'physics_reference':str(folder),'physics_stage_refitted_articulation':False,'T1_T2_use_completed_D_estimate':True})
  twins=compile_twins(out/'twins')
  tape=read(folder/'command_tape.json');first_heldout=next((i for i,frame in enumerate(tape) if frame['phase']=='P4'),len(tape))
  training_tape=tape[:first_heldout]
  if not training_tape or training_tape[-1]['phase']!='P3':raise RuntimeError('TRAINING_TAPE_MUST_END_AFTER_P3_BEFORE_P4')
  train_tape_file=out/'command_tape_train_only.json';write(train_tape_file,training_tape)
  candidate_budget=self.config['physics']['maximum_candidates'];simulation_budget=min(episode.get('budgets',{}).get('sysid_simulations',self.config['policy']['budgets']['sysid_simulations']),self.config['physics'].get('per_configuration_simulation_budget',16))
  # Reserve one reference, four held-out ablations and one final manipulation.
  if candidate_budget+6>simulation_budget:raise RuntimeError('FROZEN_SYSID_ROLLOUT_BUDGET_INSUFFICIENT')
  grid=candidate_grid(**{k+'_values':v for k,v in self.config['physics']['grid'].items()},J_eff_prior=inertia_prior,budget=candidate_budget)
  completed=[];failed=[];candidate_jobs=[];started=time.monotonic()
  def evaluate_candidate(item):
   index,candidate=item
   if self.expired() or time.monotonic()-started>episode.get('budgets',{}).get('wall_clock_seconds',3600):return candidate,None,{'status':'CUTOFF_OR_DEV_WALL_BUDGET'}
   cid=candidate['candidate_id'];asset=Path(twins['versions']['T1']['asset_root'])
   job={**reference,'mode':'replay','role':'twin','initial_articulation_rad':0.,'asset_root':str(asset),'frozen_proxy_sha256':sha(asset/'manifest.json'),'output':str(out/'physics_candidates'/cid),'replay_commands':str(train_tape_file),'fixed_fixture':scene['fixture'],'plant':{'tau_c':candidate['tau_c'],'b':candidate['b']}}
   # DEV independent plants can use idle GPUs before any TEST starts. TEST
   # keeps one serial lane per asset so its shared budget cannot be exceeded.
   job['gpu']=self.gpus[index%len(self.gpus)] if episode.get('split')=='DEV' else gpu
   return candidate,job,self.run(job)
  if episode.get('split')=='DEV':
   lanes=[list(enumerate(grid))[i::len(self.gpus)] for i in range(len(self.gpus))]
   def evaluate_lane(lane):return [evaluate_candidate(item) for item in lane]
   with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.gpus)) as pool:evaluated=[row for lane in pool.map(evaluate_lane,lanes) for row in lane]
   order={candidate['candidate_id']:i for i,candidate in enumerate(grid)};evaluated.sort(key=lambda row:order[row[0]['candidate_id']])
  else:evaluated=list(map(evaluate_candidate,enumerate(grid)))
  for candidate,job,prediction in evaluated:
   cid=candidate['candidate_id']
   if job is not None:candidate_jobs.append(job)
   if prediction['status']!='REPLAY_COMPLETE':failed.append({'candidate_id':cid,'status':prediction['status'],'scope':'training_only_no_P4_executed'});continue
   probes={p:read(Path(job['output'])/'observable'/f'{p}.json') for p in ('P1','P2','P3')}
   completed.append({**candidate,'probes':probes})
  noise=NoiseScales(**self.calibration['noise_scales'])
  if not completed:result={'status':'NO_SAFE_TWIN_ROLLOUT','accepted_parameters':None,'failed_candidates':failed,'J_eff_prior_fixed':inertia_prior,'parameter_intervals':{key:[min(self.config['physics']['grid'][key]),max(self.config['physics']['grid'][key])] for key in ('tau_c','b')},'interval_semantics':'entire frozen domain; no safe completed training rollout','expected_grid_ids':[row['candidate_id'] for row in grid]}
  else:
   result=fit_resistance(reference_logs,completed,noise,J_eff_prior=inertia_prior,budget=candidate_budget,sensitivity_evidence=self.sensitivity,expected_grid=grid,censored_candidates=failed);result['failed_candidates']=failed
  result['physics_rollouts_used']=sum((Path(job['output'])/'process.json').exists() for job in candidate_jobs)
  result['candidate_rollout_requests']=len(candidate_jobs)
  result['physics_rollout_count_semantics']='actual attempted candidate Isaac processes, excludes reference/heldout/final (see asset ledger)'
  result.update(J_eff_model='fixed original articulated-model handle; scalar effective inertia unknown, not fitted',training_tape_sha256=sha(train_tape_file),heldout_executed_during_candidate_selection=False)
  write_analysis(out,'physics_fit',result);summary['stages']['F']={'status':result['status'],'accepted_parameters':result.get('accepted_parameters')}
  final_twins=compile_twins(out/'twins_final',result)
  summary['stages']['G']={'status':'SUCCESS','T2_physics_accepted':result.get('accepted_parameters') is not None,'versions':final_twins['versions']}
  # Freeze the selected model before reading any P4 response. Each ablation then
  # independently evolves under the full reference tape; P4 never ranks candidates.
  selections=[('B0','T0',self.config['physics']['initial_prior']),('B1','T1',self.config['physics']['initial_prior'])]
  if result.get('accepted_parameters'):selections.append(('B2','T2',result['accepted_parameters']))
  heldout_jobs={}
  for label,version,params in selections:
   asset=Path(final_twins['versions'][version]['asset_root'])
   heldout_jobs[label]={**reference,'mode':'replay','role':'twin','initial_articulation_rad':0.,'asset_root':str(asset),'frozen_proxy_sha256':sha(asset/'manifest.json'),'output':str(out/'heldout'/label),'replay_commands':str(folder/'command_tape.json'),'fixed_fixture':scene['fixture'],'plant':params}
  heldout_jobs['Oracle']={**reference,'mode':'replay','role':'diagnostic_oracle','output':str(out/'heldout/Oracle'),'replay_commands':str(folder/'command_tape.json'),'fixed_fixture':scene['fixture']}
  write(out/'heldout_plan.json',{'fit_sha256':sha(out/'physics_fit.json'),'kinematics_sha256':sha(out/'kinematic_fit_for_twins.json'),'reference_log':str(folder/'observable/P4.json'),'jobs':heldout_jobs,'B3':'UNAVAILABLE_WITHOUT_INDEPENDENT_CURRENT_CALIBRATION','fitting_action_ids':['P1','P2','P3'],'heldout_action_id':'P4'})
  write(out/'episode_summary.json',summary);summary=self.heldout_episode(episode,gpu)
  if not self.expired():
   accepted_physics=result.get('accepted_parameters') is not None
   final_job={**reference,'mode':'updated_interaction','role':'reference','output':str(out/'final_updated_interaction'),'initial_estimate':str(out/'kinematic_fit_for_twins.json'),'initial_estimate_memory':str(out/'kinematic_memory_for_twins.json'),'continue_manipulation':True,'fixed_fixture':scene['fixture']}
   final_report=self.run(final_job)
   summary['stages']['I']={'status':('SUCCESS' if accepted_physics else 'KINEMATIC_ONLY_SUCCESS') if final_report.get('success') else 'FAILED','reason':final_report['status'],'physics_updated':accepted_physics,'physics_model_used_by_controller':False,'manipulation_policy':'unchanged constrained controller; physics update evaluated in heldout prediction','reference_plant_unchanged':True,'controller_uses_estimated_articulation':True,'actual_angle_deg':final_report.get('evaluation',{}).get('actual_door_displacement_deg',final_report.get('evaluation',{}).get('actual_final_door_angle_deg')),'angle_semantics':'posthoc displacement relative to initial state','report':str(Path(final_job['output'])/'report.json')}
  else:summary['stages']['I']={'status':'NOT_RUN','reason':'CUTOFF_05_00'}
  write(out/'episode_summary.json',summary);return summary
 def heldout_episode(self,episode,gpu):
  from interactive_twin.sysid import NoiseScales,heldout_comparison,write_analysis
  out=self.output/'episodes'/episode['episode_id'];summary=read(out/'episode_summary.json');file=out/'heldout_plan.json'
  if not file.exists():summary['stages']['H']={'status':'BLOCKED','reason':'NO_FROZEN_FIT_AND_HELDOUT_PLAN'};write(out/'episode_summary.json',summary);return summary
  plan=read(file)
  if sha(out/'physics_fit.json')!=plan['fit_sha256'] or sha(out/'kinematic_fit_for_twins.json')!=plan['kinematics_sha256']:raise RuntimeError('FROZEN_ESTIMATE_CHANGED_BEFORE_HELDOUT')
  if not self.calibration:raise RuntimeError('HELDOUT_REQUIRES_FROZEN_ROBOT_CALIBRATION')
  reference_path=Path(plan['reference_log'])
  if not reference_path.exists() or not read(reference_path)['provenance'].get('complete'):
   summary['stages']['H']={'status':'FAILED','reason':'REFERENCE_HELDOUT_ACTION_NOT_COMPLETED','fit_unchanged':True};write(out/'episode_summary.json',summary);return summary
  predictions={};failures={}
  for label,job in plan['jobs'].items():
   if self.expired():failures[label]='CUTOFF_05_00';continue
   job={**job,'gpu':gpu};report=self.run(job);prediction_path=Path(job['output'])/'observable/P4.json'
   if prediction_path.exists() and read(prediction_path)['provenance'].get('complete'):predictions[label]=read(prediction_path)
   else:failures[label]=report['status']
  hold=heldout_comparison(read(reference_path),predictions,NoiseScales(**self.calibration['noise_scales']))
  hold['methods']['B3']={'status':'UNAVAILABLE','reason':plan['B3']}
  for label,reason in failures.items():hold['methods'][label]={'status':'FAILED','reason':reason}
  try:
   from interactive_twin.reporting import plot_heldout_predictions
   hold['prediction_plot']=plot_heldout_predictions(read(reference_path),predictions,out/'heldout_predictions.png',comparison=hold)
  except (ImportError,ValueError,KeyError) as error:hold['prediction_plot_unavailable']=str(error)
  hold['model_selection_frozen_before_heldout']=True;hold['fit_sha256']=plan['fit_sha256'];hold['kinematics_sha256']=plan['kinematics_sha256']
  write_analysis(out,'heldout_comparison',hold);summary['stages']['H']={'status':'EVALUATED' if predictions else 'FAILED','B2_improvement':hold['B2_improvement'],'methods':{k:v['status'] for k,v in hold['methods'].items()},'training_excluded_P4':True}
  write(out/'episode_summary.json',summary);return summary
 def physics_fit(self):
  if self.manifest is None:self.freeze()
  if not self.calibration and (self.output/'robot_calibration/robot_calibration.json').exists():self.calibration=read(self.output/'robot_calibration/robot_calibration.json')
  if not self.sensitivity and (self.output/'sensitivity/sensitivity_report.json').exists():self.sensitivity=read(self.output/'sensitivity/sensitivity_report.json')
  groups=self._test_asset_groups();lanes=[groups[i::len(self.gpus)] for i in range(len(self.gpus))]
  def worker(pair):
   gpu,lane=pair
   for group in lane:
    for episode in group:
     if self.expired():return
     file=self.output/'episodes'/episode['episode_id']/'episode_summary.json'
     if not file.exists():continue
     try:self.fit_episode(episode,gpu)
     except Exception as error:
      summary=read(file);stage=next((s for s in 'EFGHI' if summary['stages'][s]['status']=='NOT_RUN'),'F');summary['stages'][stage]={'status':'FAILED','reason':str(error),'error_type':type(error).__name__};write(file,summary)
  with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.gpus)) as pool:list(pool.map(worker,zip(self.gpus,lanes)))
 def heldout(self):
  if self.manifest is None:self.freeze()
  if not self.calibration and (self.output/'robot_calibration/robot_calibration.json').exists():self.calibration=read(self.output/'robot_calibration/robot_calibration.json')
  for i,group in enumerate(self._test_asset_groups()):
   for episode in group:
    file=self.output/'episodes'/episode['episode_id']/'episode_summary.json'
    if not file.exists() or self.expired():continue
    try:self.heldout_episode(episode,self.gpus[i%len(self.gpus)])
    except Exception as error:
     summary=read(file);summary['stages']['H']={'status':'FAILED','reason':str(error),'error_type':type(error).__name__};write(file,summary)
 def summarize(self):
  from interactive_twin.reporting import build_benchmark_report
  if self.manifest is None:self.freeze()
  return build_benchmark_report(self.manifest,self.output/'episodes',self.output/'summary',make_plots=True)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'configs/interactive_twin.yaml');p.add_argument('--stage',choices=['capability','kinematics','sensitivity','physics-fit','heldout','full'],default='full');p.add_argument('--real-log',type=Path);a=p.parse_args();b=Benchmark(load_config(a.config))
 if b.config['mode']=='REAL_LOG_TO_SIM':
  if not a.real_log:raise ValueError('REAL_LOG_REQUIRED_NO_SIM_SURROGATE_RELABELING')
  from interactive_twin.real_log import run_real_log_bundle
  def native_replay(job):
   return b.run({**job,'gpu':b.gpus[0],'deadline_shanghai':b.config['runtime']['deadline_shanghai']})
  result=run_real_log_bundle(a.real_log,b.output,native_replay)
  print(json.dumps(result,indent=2));return
 if b.config['mode']!='SIM_TO_SIM_BLIND_SYSID':raise ValueError('UNKNOWN_BENCHMARK_MODE')
 if a.stage in ('capability','full'):b.capability()
 if a.stage=='capability':return
 if a.stage=='full':
  dev=b.dev_full()
  if not (b.output/'dev_gate.json').exists() or dev['stages']['E'].get('reason')=='BLOCKED_IMPORT_PENDING':return
 if a.stage=='sensitivity':
  b.dev_setup();result=b.sensitivity_stage();write(b.output/'sensitivity/stage_status.json',result);return
 if a.stage in ('kinematics','full'):b.kinematics()
 if a.stage in ('physics-fit','full'):b.physics_fit()
 if a.stage=='heldout':
  # Use only a previously frozen estimate; never re-fit on held-out actions.
  b.heldout()
 b.summarize()

if __name__=='__main__':main()
