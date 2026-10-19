"""Bounded REAL_LOG_TO_SIM adapter for already-recorded Cartesian-drive bundles.

No hardware is opened or commanded. Native rollouts are delegated to a callback;
observed q(t) and object state are never used as replay commands. The first
adapter deliberately requires a verified mapping to the frozen 240 Hz servo.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from interactive_twin.sysid import (NoiseScales, InvalidLog, validate_log, command_hash,
    candidate_grid, fit_resistance, heldout_comparison, noise_normalized_distance,
    sensitivity_report, write_analysis)
from interactive_twin.twin import write_twins, sanitized_estimate

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'interactive-twin-real-log-bundle-v1'
FIELDS = ['ee_reference_x_m','ee_reference_y_m','ee_reference_z_m','direction_x',
          'direction_y','direction_z','drive_active','gripper_effort',
          'finger_q1_command','finger_q2_command']
FRAME_KEYS = {'t','phase','compliant','arm_position','arm_velocity','finger_mode',
              'finger_position','finger_effort','retention_armed','cartesian_input'}
SERVO_FILES = ('interaction_identification/act2see_loop.py',
               'scripts/run_interactive_twin_episode.py', 'config/piper.urdf',
               'config/semantic_interaction.json')

class BundleBlocked(ValueError):
    """Missing or incompatible evidence, not a claim of physical experiment failure."""

def _read(path):
    return json.loads(Path(path).read_text())

def _write(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');temporary.replace(path)

def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def _resolve(base,path):
    path=Path(path);return path.resolve() if path.is_absolute() else (base/path).resolve()

def current_servo_hashes():
    return {name:_sha(ROOT/name) for name in SERVO_FILES}

def _file(base, value, label):
    if not isinstance(value,str) or not value:raise BundleBlocked('MISSING_'+label)
    path=_resolve(base,value)
    if not path.is_file():raise BundleBlocked('MISSING_'+label+':'+str(path))
    return path

def _real_log(path, calibration=None):
    log=_read(path);validate_log(log)
    if log['mode']!='REAL_LOG_TO_SIM' or log['provenance']['source']!='real_robot_log':
        raise BundleBlocked('REFERENCE_MUST_BE_ORIGINAL_REAL_ROBOT_LOG:'+str(path))
    if not log['provenance'].get('complete'):raise BundleBlocked('INCOMPLETE_REAL_LOG:'+str(path))
    for key in ('robot_state_replayed','object_state_replayed','direct_object_actuation','attachment'):
        if log['provenance'].get(key) is not False:raise BundleBlocked('REAL_LOG_ACTION_PROVENANCE_UNVERIFIED:'+key)
    if log['provenance'].get('simulator') not in (None,'none'):raise BundleBlocked('SIMULATOR_LOG_CANNOT_BE_RELABELLED_REAL')
    if calibration:
        for key in ('robot_model_id','controller_id','robot_calibration_id'):
            value=calibration['calibration_id'] if key=='robot_calibration_id' else calibration[key]
            if log['provenance'][key]!=value:raise BundleBlocked('FROZEN_CALIBRATION_MISMATCH:'+key)
    return log

def _tape_commands(tape,probe):
    frames=[f for f in tape if f['phase']==probe]
    if len(frames)<5:raise BundleBlocked('PROBE_COMMANDS_MISSING:'+probe)
    start=frames[0]['t']
    values=[]
    for frame in frames:
        u=frame['cartesian_input']
        values.append(u['reference_world_m']+u['direction_world']+[float(u['active']),frame['finger_effort']]+frame['finger_position'])
    return {'time_s':[round(f['t']-start,10) for f in frames], 'fields':FIELDS,
            'kind':'cartesian_constrained_drive','values':values}

def validate_tape(tape, protocol):
    if not isinstance(tape,list) or len(tape)<120:raise BundleBlocked('FULL_APPROACH_AND_PROBE_COMMAND_TAPE_REQUIRED')
    times=np.asarray([f.get('t',math.nan) for f in tape],float)
    if not np.isfinite(times).all() or abs(times[0])>1e-8 or not np.allclose(np.diff(times),1/240,rtol=0,atol=1e-7):
        raise BundleBlocked('ONLY_VERIFIED_240HZ_COMMAND_CONTRACT_SUPPORTED_NO_IMPLICIT_RESAMPLING')
    limits=ET.parse(ROOT/'config/piper.urdf').getroot()
    lo=np.array([float(limits.find(f"joint[@name='joint{i}']/limit").get('lower')) for i in range(1,7)])
    hi=np.array([float(limits.find(f"joint[@name='joint{i}']/limit").get('upper')) for i in range(1,7)])
    vel=np.array([float(limits.find(f"joint[@name='joint{i}']/limit").get('velocity')) for i in range(1,7)])
    cap=_read(ROOT/'config/semantic_interaction.json')['finger_effort_limit_n']
    for frame in tape:
        if set(frame)!=FRAME_KEYS:raise BundleBlocked('UNSUPPORTED_COMMAND_FRAME_FIELDS_NO_MEASURED_Q_OR_RAW_EFFORT_REPLAY')
        q=np.asarray(frame['arm_position'],float);v=np.asarray(frame['arm_velocity'],float)
        fingers=np.asarray(frame['finger_position'],float)
        if q.shape!=(6,) or v.shape!=(6,) or fingers.shape!=(2,) or not np.isfinite(np.r_[q,v,fingers,frame['finger_effort']]).all():raise BundleBlocked('INVALID_COMMAND_DIMENSION_OR_NUMBER')
        if np.any(q<lo) or np.any(q>hi) or np.any(np.abs(v)>vel+1e-10):raise BundleBlocked('COMMAND_EXCEEDS_OFFICIAL_JOINT_LIMIT')
        if not(0<=fingers[0]<=.05 and -.05<=fingers[1]<=0 and 0<=frame['finger_effort']<=cap):raise BundleBlocked('COMMAND_EXCEEDS_FROZEN_GRIPPER_LIMIT')
        if frame['finger_mode'] not in ('position','effort') or not isinstance(frame['compliant'],bool) or not isinstance(frame['retention_armed'],bool):raise BundleBlocked('UNSUPPORTED_CONTROLLER_MODE')
        u=frame['cartesian_input']
        if frame['compliant']:
            if not isinstance(u,dict) or set(u)!={'reference_world_m','direction_world','active'}:raise BundleBlocked('COMPLIANT_FRAME_REQUIRES_EXOGENOUS_CARTESIAN_INPUT')
            point=np.asarray(u['reference_world_m'],float);direction=np.asarray(u['direction_world'],float)
            if point.shape!=(3,) or direction.shape!=(3,) or not np.isfinite(np.r_[point,direction]).all() or not np.isclose(np.linalg.norm(direction),1,atol=1e-6) or not isinstance(u['active'],bool):raise BundleBlocked('INVALID_CARTESIAN_INPUT')
        elif u is not None:raise BundleBlocked('CARTESIAN_INPUT_ON_POSITION_CONTROLLER_FRAME')
    if any(f['phase']!='SETTLE' for f in tape[:120]):raise BundleBlocked('NATIVE_REPLAY_REQUIRES_INITIAL_HALF_SECOND_SETTLE_COMMANDS')
    if [p['probe_id'] for p in protocol]!=['P1','P2','P3','P4']:raise BundleBlocked('TRAIN_HELDOUT_PROTOCOL_ORDER')
    previous=-1
    for segment in protocol:
        probe=segment['probe_id']
        if not 0<float(segment.get('duration_s',0))<=12 or not 0<float(segment.get('speed_m_s',0))<=.0005:raise BundleBlocked('PROTOCOL_EXCEEDS_FROZEN_TIME_OR_SPEED_LIMIT')
        if segment.get('split')!=('heldout' if probe=='P4' else 'train'):raise BundleBlocked('PROTOCOL_SPLIT_INVALID')
        idx=[i for i,f in enumerate(tape) if f['phase']==probe]
        expected=round(float(segment['duration_s'])*240)
        if expected<5 or not idx or len(idx)!=expected or idx!=list(range(idx[0],idx[-1]+1)) or idx[0]<=previous:raise BundleBlocked('INCOMPLETE_OR_REORDERED_COMMAND_PROTOCOL:'+probe)
        if not all(tape[i]['compliant'] and tape[i]['retention_armed'] for i in idx):raise BundleBlocked('PROBE_REQUIRES_RETAINED_GRASP_AND_CONSTRAINED_DRIVE')
        previous=idx[-1]
    if not any(f['phase']=='APPROACH' for f in tape) or not any(f['retention_armed'] for f in tape[:next(i for i,f in enumerate(tape) if f['phase']=='P1')]):raise BundleBlocked('APPROACH_AND_ACTUAL_GRASP_COMMAND_HISTORY_REQUIRED')

def validate_bundle(bundle_path):
    """Validate evidence and return private orchestration context; never run a plant."""
    bundle_path=Path(bundle_path).resolve();base=bundle_path.parent;bundle=_read(bundle_path)
    if bundle.get('schema')!=SCHEMA or bundle.get('mode')!='REAL_LOG_TO_SIM':raise BundleBlocked('UNSUPPORTED_REAL_LOG_BUNDLE')
    files={'bundle':bundle_path}
    for key in ('command_tape','robot_calibration','servo_mapping','job_template','estimated_articulation','structured_memory','initial_scene'):
        files[key]=_file(base,bundle.get(key),key.upper())
    calibration=_read(files['robot_calibration'])
    if calibration.get('mode')!='REAL_LOG_TO_SIM' or calibration.get('source')!='real_robot_log' or calibration.get('hardware_calibration') is not True or calibration.get('repeats',0)<2 or calibration.get('no_contact_supervisor_certified') is not True:raise BundleBlocked('INDEPENDENT_REAL_ROBOT_CALIBRATION_REQUIRED')
    noise=NoiseScales(**calibration['noise_scales']);noise.validate()
    if noise.calibration_id!=calibration['calibration_id']:raise BundleBlocked('ROBOT_NOISE_CALIBRATION_ID_MISMATCH')
    mapping=_read(files['servo_mapping']);mapping_base=files['servo_mapping'].parent
    required={'independently_calibrated':True,'frozen_before_object_data':True,'object_contact_data_used':False,'q_measurements_used_as_commands':False,'command_kind':'cartesian_constrained_drive','sample_rate_hz':240,'current_or_sdk_effort_to_external_torque':False}
    if any(mapping.get(k)!=v for k,v in required.items()):raise BundleBlocked('INDEPENDENT_SERVO_MAPPING_EVIDENCE_REQUIRED')
    if mapping.get('native_servo_sha256')!=current_servo_hashes():raise BundleBlocked('CALIBRATED_SERVO_IMPLEMENTATION_CHANGED')
    for key in ('robot_model_id','controller_id'):
        if mapping.get(key)!=calibration[key]:raise BundleBlocked('SERVO_MAPPING_MODEL_MISMATCH:'+key)
    if not mapping.get('hardware_firmware_version') or mapping['hardware_firmware_version'].lower()=='unknown':raise BundleBlocked('HARDWARE_FIRMWARE_MAPPING_UNVERIFIED')
    maximum=float(mapping.get('predeclared_maximum_motion_loss',math.nan))
    if not math.isfinite(maximum) or maximum<=0:raise BundleBlocked('PREDECLARED_SERVO_VALIDATION_BOUND_REQUIRED')
    pairs=mapping.get('free_space_validation_pairs',[])
    if len(pairs)<2:raise BundleBlocked('TWO_INDEPENDENT_FREE_ROBOT_VALIDATIONS_REQUIRED')
    validation=[]
    for i,pair in enumerate(pairs):
        real_path=_file(mapping_base,pair.get('real_log'),'REAL_SERVO_VALIDATION');sim_path=_file(mapping_base,pair.get('sim_log'),'SIM_SERVO_VALIDATION')
        if _sha(real_path)!=pair.get('real_sha256') or _sha(sim_path)!=pair.get('sim_sha256'):raise BundleBlocked('SERVO_EVIDENCE_HASH_MISMATCH')
        real=_real_log(real_path,calibration);sim=_read(sim_path);validate_log(sim,independent_sim=True)
        if real['split']!='calibration' or sim['split']!='calibration' or real['provenance'].get('no_contact_supervisor_certified') is not True or sim['provenance'].get('no_contact_supervisor_certified') is not True:raise BundleBlocked('SERVO_VALIDATION_MUST_BE_NO_CONTACT')
        loss=noise_normalized_distance(real,sim,noise)
        if loss['motion_only_normalized_loss']>maximum:raise BundleBlocked('ROBOT_ONLY_RESPONSE_MAPPING_OUTSIDE_FROZEN_BOUND')
        validation.append(loss);files[f'servo_real_{i}']=real_path;files[f'servo_sim_{i}']=sim_path
    logs={};tape=_read(files['command_tape']);protocol=bundle.get('physics_protocol')
    if not isinstance(protocol,list):raise BundleBlocked('FROZEN_PHYSICS_PROTOCOL_REQUIRED')
    validate_tape(tape,protocol)
    for probe in ('P1','P2','P3','P4'):
        path=_file(base,bundle.get('reference_logs',{}).get(probe),'REAL_'+probe);files[probe]=path;log=_real_log(path,calibration)
        if log['probe_id']!=probe or log['split']!=('heldout' if probe=='P4' else 'train'):raise BundleBlocked('TRAIN_HELDOUT_SPLIT_MISMATCH')
        if command_hash(log['commands'])!=command_hash(_tape_commands(tape,probe)):raise BundleBlocked('REAL_OBSERVATIONS_AND_FULL_COMMAND_TAPE_DISAGREE:'+probe)
        logs[probe]=log
    if len({str(files[p]) for p in ('P1','P2','P3','P4')})!=4:raise BundleBlocked('SEPARATE_FROZEN_PROBE_LOGS_REQUIRED')
    fit=_read(files['estimated_articulation']);sanitized_estimate(fit)
    if fit.get('joint_type')!='revolute' or fit.get('confidence',0)<=.9:raise BundleBlocked('FROZEN_RELIABLE_REVOLUTE_ESTIMATE_REQUIRED')
    memory=_read(files['structured_memory'])
    if memory.get('GT_inputs') is not False:raise BundleBlocked('EE_ONLY_ESTIMATE_PROVENANCE_REQUIRED')
    accepted=[row for row in memory.get('fit_history',[]) if row.get('accepted')]
    if not accepted or accepted[-1]['fit']!=fit:raise BundleBlocked('SAVED_EE_ESTIMATE_AND_MEMORY_DISAGREE')
    support=memory['supporting_observations'][:accepted[-1]['observation_count']]
    if len(support)<5:raise BundleBlocked('INSUFFICIENT_EE_ESTIMATE_SUPPORT')
    if np.max(np.linalg.norm(np.asarray(support)[:,:3,3]-np.asarray(support)[0,:3,3],axis=1))<.005:raise BundleBlocked('EE_ESTIMATE_EXCITATION_BELOW_5MM')
    if bundle.get('estimate_support_scope')!='pre_physics_EE_only':raise BundleBlocked('KINEMATIC_ESTIMATE_MUST_EXCLUDE_P4')
    first_physics=next(f['t'] for f in tape if f['phase']=='P1')
    attempts=[a for a in memory.get('attempt_history',[]) if a.get('result')=='RELIABLE_REVOLUTE_ESTIMATE']
    if not attempts or not 0<=float(attempts[-1].get('end_s',math.inf))<=first_physics:raise BundleBlocked('SAVED_ESTIMATE_TIMING_MUST_PRECEDE_PHYSICS')
    template=_read(files['job_template'])
    forbidden={'replay_commands','plant','initial_estimate','initial_estimate_memory','reference_safety_memory','import_spec','budget_asset_id','robot_start_q','initial_articulation_rad','observation_mode'}
    if forbidden&set(template):raise BundleBlocked('JOB_TEMPLATE_CONTAINS_UNVERIFIED_CONTROL_OR_PLANT_FIELDS')
    for key in ('source','asset_root','plan'):
        if key not in template:raise BundleBlocked('NATIVE_SCENE_TEMPLATE_MISSING:'+key)
        template[key]=str(_resolve(files['job_template'].parent,template[key]))
    if not(Path(template['asset_root'])/'manifest.json').is_file() or not Path(template['plan']).is_file() or not(Path(template['source'])/'report.json').is_file():raise BundleBlocked('PREPARED_NATIVE_SCENE_REQUIRED')
    # Scope intentionally starts at the prepared q=0 prior. Nonzero real starts
    # must be re-zeroed at scene preparation, never inferred from object GT here.
    initial=_read(files['initial_scene'])
    if initial.get('source')!='measured_prepared_initial_scene' or initial.get('joint_coordinate_zero_is_initial') is not True or initial.get('robot_q_initial_verified') is not True:raise BundleBlocked('VERIFIED_INITIAL_SCENE_AND_ROBOT_START_REQUIRED')
    robot_q=np.asarray(initial['robot_q_initial_rad'],float)
    if robot_q.shape!=(6,) or not np.isfinite(robot_q).all():raise BundleBlocked('INVALID_INITIAL_ROBOT_Q')
    W=np.asarray(initial['T_world_asset'],float)
    if W.shape!=(4,4) or not np.allclose(W[3],[0,0,0,1]):raise BundleBlocked('INITIAL_ASSET_TRANSFORM_INVALID')
    budget=bundle.get('budget',{})
    if not 1<=int(budget.get('simulation_launches',0))<=48 or not 0<float(budget.get('wall_clock_s',0))<=25200:raise BundleBlocked('FINITE_FROZEN_REAL_LOG_BUDGET_REQUIRED')
    grid_config=bundle.get('grid',{})
    asset=Path(template['asset_root']);manifest=_read(asset/'manifest.json');urdf=asset/'urdf'/f"{manifest['asset_id']}.urdf"
    files.update(asset_manifest=asset/'manifest.json',asset_urdf=urdf,prepared_scene_source=Path(template['source'])/'report.json',prepared_grasp_plan=Path(template['plan']))
    inertia={'kind':'fixed_articulated_model','sha256':_sha(urdf),'scalar_J_eff':None,'frozen':True}
    grid=candidate_grid(grid_config.get('tau_c',[]),grid_config.get('b',[]),J_eff_prior=inertia,budget=int(budget.get('maximum_candidates',9)))
    if len(grid)+3>budget['simulation_launches']:raise BundleBlocked('BUDGET_MUST_RESERVE_THREE_INDEPENDENT_HELDOUT_ROLLOUTS')
    sensitivity=None
    if bundle.get('sensitivity_logs'):
        conditions={}
        for label,paths in bundle['sensitivity_logs'].items():
            if len(paths)<2:raise BundleBlocked('REAL_SENSITIVITY_REPEATS_REQUIRED')
            conditions[label]=[]
            for i,value in enumerate(paths):
                path=_file(base,value,'REAL_SENSITIVITY');files[f'sensitivity_{label}_{i}']=path;conditions[label].append(_real_log(path,calibration))
        if len(conditions)<2:raise BundleBlocked('REAL_SENSITIVITY_CONDITIONS_REQUIRED')
        sensitivity=sensitivity_report(conditions,noise)
    return dict(bundle=bundle,files=files,logs=logs,tape=tape,protocol=protocol,noise=noise,calibration=calibration,mapping=mapping,
        servo_validation=validation,fit=fit,memory=memory,support=support,template=template,initial=initial,W=W,grid=grid,
        inertia=inertia,sensitivity=sensitivity,identity={name:_sha(path) for name,path in files.items()})


def run_real_log_bundle(bundle_path,output,run_job):
    """Launch native independent rollouts via run_job(job)->report.

    The callback must execute Isaac or return an explicit failure; fake callbacks
    belong only in contract tests and cannot establish hardware results.
    """
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    try:context=validate_bundle(bundle_path)
    except (BundleBlocked,InvalidLog,KeyError,ValueError,TypeError,OSError) as error:
        result={'mode':'REAL_LOG_TO_SIM','status':'BLOCKED_BUNDLE_EVIDENCE','reason':str(error),'simulation_launches':0,'real_to_sim_success_proven':False}
        _write(output/'real_log_status.json',result);return result
    c=context;identity={'input_sha256':c['identity'],'adapter_sha256':_sha(__file__),'servo_sha256':current_servo_hashes()}
    identity_file=output/'bundle_identity.json'
    if identity_file.exists() and _read(identity_file)!=identity:raise BundleBlocked('BUNDLE_CHANGED_USE_NEW_OUTPUT_AND_RECOMPUTE_ALL_CANDIDATES')
    _write(identity_file,identity);_write(output/'servo_mapping_validation.json',c['servo_validation'])
    if c['sensitivity']:_write(output/'physics_sensitivity.json',c['sensitivity'])
    memory=copy.deepcopy(c['memory']);memory['supporting_observations']=c['support'];memory['fit_history']=[r for r in memory['fit_history'] if r.get('accepted')][-1:]
    _write(output/'pre_physics_structured_memory.json',memory)
    prior={**c['bundle'].get('initial_physics_prior',{'tau_c':0.,'b':0.}),'J_eff':c['inertia']}
    def compile_twins(path,result=None):
        if (path/'twin_versions.json').exists():return _read(path/'twin_versions.json')
        return write_twins(c['template']['asset_root'],path,c['fit'],c['W'],initial_physics_prior=prior,physics_estimate=result,ee_poses=c['support'],attempt_history=memory.get('attempt_history',[]),q_initial=0,operational_joint_limits_rad=[-.1745329252,.1745329252])
    twins=compile_twins(output/'twins_initial');first_heldout=next(i for i,f in enumerate(c['tape']) if f['phase']=='P4')
    training=c['tape'][:first_heldout]
    if training[-1]['phase']!='P3':raise BundleBlocked('TRAIN_TAPE_MUST_END_AT_P3')
    _write(output/'command_tape_training.json',training);_write(output/'command_tape_full.json',c['tape'])
    ledger_file=output/'rollout_ledger.json';ledger=_read(ledger_file) if ledger_file.exists() else {'launches':0,'active_wall_s':0.,'runs':{}}
    frozen=c['bundle']['budget'];cal=c['calibration'];safety=output/'pre_physics_structured_memory.json'
    def launch(name,version,params,heldout=False):
        folder=output/name;folder.mkdir(parents=True,exist_ok=True)
        asset=Path(version['asset_root']);job={**c['template'],'episode_id':c['logs']['P1']['episode_id'],'mode':'replay','role':'twin_from_real_log','observation_mode':'REAL_LOG_TO_SIM',
            'asset_root':str(asset),'frozen_proxy_sha256':_sha(asset/'manifest.json'),'plant':params,'output':str(folder),
            'replay_commands':str(output/('command_tape_full.json' if heldout else 'command_tape_training.json')),
            'reference_safety_memory':str(safety),'robot_start_q':c['initial']['robot_q_initial_rad'],'initial_articulation_rad':0.,
            'robot_calibration_id':cal['calibration_id'],'robot_model_id':cal['robot_model_id'],'controller_id':cal['controller_id'],
            'physics_protocol':c['protocol'],'fixed_fixture':c['initial']['fixture'],'continue_manipulation':False}
        file=folder/'report.json';jobfile=folder/'real_log_job.json'
        if file.exists():
            if not jobfile.exists() or {k:v for k,v in _read(jobfile).items() if k!='wall_clock_budget_s'}!={k:v for k,v in job.items() if k!='wall_clock_budget_s'}:raise BundleBlocked('REAL_LOG_ROLLOUT_CACHE_MISMATCH')
            return _read(file),folder
        if name in ledger['runs']:return {'status':'INTERRUPTED_ROLLOUT_REQUIRES_NEW_VERSION_NO_UNBUDGETED_RETRY'},folder
        if ledger['launches']>=frozen['simulation_launches'] or ledger['active_wall_s']>=frozen['wall_clock_s']:return {'status':'REAL_LOG_SIMULATION_BUDGET_EXHAUSTED'},folder
        remaining=frozen['wall_clock_s']-ledger['active_wall_s'];job['wall_clock_budget_s']=min(float(job.get('wall_clock_budget_s',1800)),remaining)
        ledger['launches']+=1;ledger['runs'][name]={'status':'RUNNING','job_sha256':command_hash(job)};_write(ledger_file,ledger);_write(jobfile,job);start=time.monotonic()
        try:report=run_job(job)
        except Exception as error:report={'status':'NATIVE_REPLAY_FAILED','reason':str(error)}
        finally:ledger['active_wall_s']+=time.monotonic()-start
        ledger['runs'][name].update(status=report['status']);_write(ledger_file,ledger);_write(file,report)
        return report,folder
    completed=[];failed=[]
    for candidate in c['grid']:
        report,folder=launch('fit/'+candidate['candidate_id'],twins['versions']['T1'],{key:candidate[key] for key in ('tau_c','b')})
        probes={}
        for probe in ('P1','P2','P3'):
            file=folder/'observable'/(probe+'.json')
            if file.exists():
                log=_read(file)
                try:validate_log(log,independent_sim=True);noise_normalized_distance(c['logs'][probe],log,c['noise'])
                except InvalidLog:continue
                probes[probe]=log
        if len(probes)==3:completed.append({**candidate,'probes':probes})
        else:failed.append({'candidate_id':candidate['candidate_id'],'status':report['status']})
    result=fit_resistance({p:c['logs'][p] for p in ('P1','P2','P3')},completed,c['noise'],J_eff_prior=c['inertia'],budget=c['bundle']['budget']['maximum_candidates'],sensitivity_evidence=c['sensitivity'],expected_grid=c['grid'],censored_candidates=failed) if completed else {'status':'NO_SAFE_TWIN_ROLLOUT','accepted_parameters':None,'failed_candidates':failed,'parameter_intervals':{k:[min(c['bundle']['grid'][k]),max(c['bundle']['grid'][k])] for k in ('tau_c','b')}}
    result.update(mode='REAL_LOG_TO_SIM',current_or_effort_used=False,parameter_semantics='effective simulator resistance; not independently calibrated real hinge torque')
    write_analysis(output,'physics_fit',result);fit_hash=_sha(output/'physics_fit.json');final=compile_twins(output/'twins_final',result)
    selections=[('B0','T0',prior),('B1','T1',prior)]
    if result.get('accepted_parameters'):selections.append(('B2','T2',result['accepted_parameters']))
    predictions={};failures={}
    for label,version,params in selections:
        report,folder=launch('heldout/'+label,final['versions'][version],{k:params[k] for k in ('tau_c','b')},True)
        file=folder/'observable/P4.json'
        if file.exists():
            try:
                log=_read(file);noise_normalized_distance(c['logs']['P4'],log,c['noise']);predictions[label]=log
            except InvalidLog as error:failures[label]=str(error)
        else:failures[label]=report['status']
    if _sha(output/'physics_fit.json')!=fit_hash:raise BundleBlocked('FIT_CHANGED_DURING_HELDOUT')
    heldout=heldout_comparison(c['logs']['P4'],predictions,c['noise']);heldout['fit_frozen_before_P4']=fit_hash
    heldout['native_rollout_failures']=failures;heldout['methods']['B3']={'status':'UNAVAILABLE','reason':'current/effort transfer calibration not implemented'}
    heldout['methods']['Oracle']={'status':'UNAVAILABLE','reason':'real reference articulation and physics truth unavailable'}
    write_analysis(output,'heldout_comparison',heldout)
    status={'mode':'REAL_LOG_TO_SIM','status':'NATIVE_REPLAY_AND_ANALYSIS_COMPLETED','physics_identification':result['status'],'physics_accepted':bool(result.get('accepted_parameters')),'heldout_models_completed':sorted(predictions),'simulator_safety_supervisor_only':True,'real_bilateral_contact_or_contact_manifold_required':False,'simulation_launches':ledger['launches'],'active_wall_s':ledger['active_wall_s'],'real_robot_commands_sent':False,'real_to_sim_success_proven':False,'heldout_improvement_measured':bool(result.get('accepted_parameters') and heldout.get('B2_improvement',{}).get('B1',{}).get('absolute_loss_reduction',0)>0),'success_claim_requires_review_of_original_real_logs_and_heldout_metric':True,'B3':'UNAVAILABLE'}
    _write(output/'real_log_status.json',status);return status
