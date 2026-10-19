"""Autonomous same-input oracle decomposition; no GT inputs enter the estimator."""
import concurrent.futures
import csv
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read(path):return json.loads(Path(path).read_text())
def write(path,value):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 path.write_text(json.dumps(value,indent=2,allow_nan=False))


def metric(reference,prediction,noise):
 from interactive_twin.sysid import noise_normalized_distance
 return noise_normalized_distance(reference,prediction,noise)


def training_metrics(reference,probes,noise):
 values={p:metric(reference[p],probes[p],noise) for p in ('P1','P2','P3')}
 return {'normalized_loss':float(np.mean([v['normalized_loss'] for v in values.values()])),
         'ee_position_rmse_m':float(np.mean([v['metrics']['ee_position_rmse_m'] for v in values.values()])),
         'probes':values}


def context(bench):
 old=bench.original/'episodes/dev_7320_00';out=bench.output/'physics_oracle';out.mkdir(parents=True,exist_ok=True)
 summary=read(old/'episode_summary.json');selected=summary['selected_job']
 c=bench.kinematic_context(selected,Path(selected['output']),out/'kinematics')
 twins=c['compile_twins'](out/'kinematics/twins')
 reference=read(old/'physics_reference/job_private.json')
 refdir=Path(reference['output'])
 # Explicit reuse of immutable physical reference and same-input command tape.
 import hashlib
 provenance={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [refdir/'command_tape.json',
              refdir/'job_private.json',refdir/'report.json',*[refdir/'observable'/f'P{i}.json' for i in range(1,5)]]}
 write(out/'reference_import_provenance.json',{'source':str(refdir),'sha256':provenance,
          'new_execution':False,'raw_data_modified':False,'reference_parameters_hidden_from_estimator':True,
          'GT_twin_is_oracle_not_Real2Sim_prior':True,'mode':'SIM_TO_SIM_BLIND_SYSID'})
 tape=read(refdir/'command_tape.json');end=next(i for i,f in enumerate(tape) if f['phase']=='P4')
 if tape[end-1]['phase']!='P3':raise RuntimeError('TRAIN_P4_BOUNDARY_INVALID')
 write(out/'command_train_only.json',tape[:end])
 # Multiple native rollouts share each immutable asset. Prepare the visual
 # import copy once before parallel app launches, avoiding directory races.
 from interactive_twin.visual_import import compatible_urdf
 for asset in (Path(reference['asset_root']),Path(twins['versions']['T1']['asset_root'])):
  manifest=read(asset/'manifest.json')
  compatible_urdf(asset/'urdf'/f"{manifest['asset_id']}.urdf",asset/'visual_compatibility')
 return old,out,c,twins,reference,refdir


def native_job(bench,reference,out,asset,params,tape,gpu,role='twin'):
 import hashlib
 job={**reference,'mode':'replay','role':role,'output':str(out),'asset_root':str(asset),
      'plant':dict(params),'replay_commands':str(tape),'initial_articulation_rad':0.,'gpu':gpu,
      'deadline_shanghai':bench.experiment['deadline_shanghai'],'wall_clock_budget_s':1800,
      'continue_manipulation':False,'frozen_proxy_sha256':hashlib.sha256((Path(asset)/'manifest.json').read_bytes()).hexdigest()}
 job.pop('import_spec',None)
 job['reference_safety_memory']=str(Path(reference['output'])/'structured_memory.json')
 return job


def run_decomposition(bench):
 from interactive_twin.sysid import NoiseScales,candidate_grid,fit_resistance,validate_log
 from interactive_twin.reporting import plot_heldout_predictions
 old,out,c,twins,reference,refdir=context(bench)
 noise=NoiseScales(**bench.calibration['noise_scales'])
 training={p:read(refdir/'observable'/f'{p}.json') for p in ('P1','P2','P3')}
 for log in training.values():validate_log(log,independent_sim=True)
 policy=bench.experiment['oracle'];grid=candidate_grid(policy['grid']['tau_c'],policy['grid']['b'],
       J_eff_prior=c['inertia_prior'],budget=policy['maximum_candidates'])
 source_asset=Path(reference['asset_root']);estimated_asset=Path(twins['versions']['T1']['asset_root'])
 fits={};train_results={}
 # P1 is tested first with GT kinematics in the diagnostic plant only. Commands
 # and safety still come from the authenticated EE-only interaction memory.
 for label,asset in [('P1',source_asset),('P3',estimated_asset)]:
  evaluated=[];failures=[]
  def rollout(item):
   index,candidate=item;cid=candidate['candidate_id']
   job=native_job(bench,reference,out/label/'train'/cid,asset,
       {'tau_c':candidate['tau_c'],'b':candidate['b']},out/'command_train_only.json',bench.gpus[index%len(bench.gpus)])
   # P3's previous autonomous grid is reused explicitly, never relabeled as a
   # new rollout. It uses the identical D fit, command, robot and physical plant.
   previous=old/'physics_candidates'/cid
   if label=='P3' and (previous/'report.json').exists():
    priorjob=read(previous/'job_private.json');status=read(previous/'report.json')['status']
    same=priorjob['plant']==job['plant'] and Path(priorjob['replay_commands']).read_bytes()==Path(job['replay_commands']).read_bytes()
    same=same and read(Path(priorjob['asset_root'])/'manifest.json')['prepared_geometry_sha256']==read(asset/'manifest.json')['prepared_geometry_sha256']
    same=same and Path(priorjob['initial_estimate']).read_bytes()==Path(reference['initial_estimate']).read_bytes()
    if not same:raise RuntimeError('P3_IMPORT_CONFIGURATION_MISMATCH')
    write(Path(job['output'])/'import_provenance.json',{'source':str(previous),'original_job':priorjob,
           'new_execution':False,'same_command_and_D_kinematics':True,'status':status})
    folder=previous
   else:
    report=bench.run(job);status=report['status'];folder=Path(job['output'])
   probes={p:read(folder/'observable'/f'{p}.json') for p in ('P1','P2','P3') if (folder/'observable'/f'{p}.json').exists()}
   if status!='REPLAY_COMPLETE' or len(probes)!=3 or not all(p['provenance'].get('complete') for p in probes.values()):
    return candidate,None,{'candidate_id':cid,'status':status}
   return candidate,probes,None
  lanes=[list(enumerate(grid))[i::len(bench.gpus)] for i in range(len(bench.gpus))]
  def lane(items):return [rollout(item) for item in items]
  with concurrent.futures.ThreadPoolExecutor(max_workers=len(bench.gpus)) as pool:
   results=[row for rows in pool.map(lane,lanes) for row in rows]
  for candidate,probes,error in results:
   if error:failures.append(error)
   else:evaluated.append({**candidate,'probes':probes})
  fit=fit_resistance(training,evaluated,noise,J_eff_prior=c['inertia_prior'],budget=9,
        sensitivity_evidence=bench.sensitivity,expected_grid=grid,censored_candidates=failures) if evaluated else {'status':'NO_SAFE_TWIN_ROLLOUT','accepted_parameters':None}
  fit.update(failed_candidates=failures,training_action_ids=['P1','P2','P3'],heldout_accessed_during_selection=False)
  write(out/label/'physics_fit.json',fit);fits[label]=fit
  train_results[label]={r['candidate_id']:training_metrics(training,r['probes'],noise) for r in evaluated}
  write(out/label/'train_metrics.json',train_results[label])
  if label=='P1':
   if fit.get('accepted_parameters') is None:
    write(out/'stop_gate.json',{'status':'PHYSICS_SYSID_FAILED','reason':fit['status'],
           'physics_expansion_allowed':False,'45621_optimizer_resumed':False})
    break  # Still report the P0/P2 diagnostic bounds, without more fitting.
   # Evaluate P1 wrong prior and its selected fit before allowing P3 fitting.
   params=fit['accepted_parameters']
   preliminary={}
   for name,phi in [('P1_wrong_prior',policy['wrong_prior']),('P1',params)]:
    job=native_job(bench,reference,out/'heldout'/name,source_asset,phi,refdir/'command_tape.json',bench.gpus[0])
    r=bench.run(job);p=Path(job['output'])/'observable/P4.json'
    if r['status']=='REPLAY_COMPLETE' and p.exists() and read(p)['provenance'].get('complete'):
     preliminary[name]=metric(read(refdir/'observable/P4.json'),read(p),noise)
   write(out/'P1_heldout_gate.json',preliminary)
   improved=len(preliminary)==2 and preliminary['P1']['metrics']['ee_position_rmse_m']<preliminary['P1_wrong_prior']['metrics']['ee_position_rmse_m']
   write(out/'stop_gate.json',{'status':'P1_HELDOUT_IMPROVED' if improved else 'PHYSICS_SYSID_FAILED_HELDOUT',
          'physics_expansion_allowed':improved,'45621_optimizer_resumed':False})
   if not improved:
    break
 # Estimates are fixed before heldout responses are used for evaluation.
 gt_params=dict(reference['plant'])  # oracle scene assembly only, never passed to fit_resistance
 specs={'P0':(source_asset,gt_params),'P1_wrong_prior':(source_asset,policy['wrong_prior']),
        'P2':(estimated_asset,gt_params)}
 if fits['P1'].get('accepted_parameters'):specs['P1']=(source_asset,fits['P1']['accepted_parameters'])
 if fits.get('P3',{}).get('accepted_parameters'):specs['P3']=(estimated_asset,fits['P3']['accepted_parameters'])
 write(out/'heldout_plan_frozen.json',{'methods':{k:{'asset':str(a),'plant':p} for k,(a,p) in specs.items()},
       'fits_saved_before_heldout':True,'P0_P2_parameters_scope':'oracle evaluation only, not controller guidance'})
 def evaluate(item):
  index,(label,(asset,params))=item
  job=native_job(bench,reference,out/'heldout'/label,asset,params,refdir/'command_tape.json',bench.gpus[index%len(bench.gpus)],
                 role='diagnostic_oracle' if label in ('P0','P2') else 'twin')
  existing=Path(job['output'])/'job_private.json'
  if existing.exists():
   original=read(existing)
   if original['plant']!=job['plant'] or original['asset_root']!=job['asset_root'] or original['replay_commands']!=job['replay_commands']:
    raise RuntimeError('HELDOUT_CACHE_INPUT_MISMATCH')
   job=original
  r=bench.run(job);folder=Path(job['output']);p=folder/'observable/P4.json'
  result={'group':label,'status':r['status'],'tau_c':params['tau_c'],'b':params['b'],
          'identifiability':fits.get(label,{}).get('status','ORACLE_NOT_FIT'),'report':str(folder/'report.json')}
  prediction=None
  if r['status']=='REPLAY_COMPLETE' and p.exists() and read(p)['provenance'].get('complete'):
   prediction=read(p);result['heldout']=metric(read(refdir/'observable/P4.json'),prediction,noise)
   probes={p:read(folder/'observable'/f'{p}.json') for p in ('P1','P2','P3')}
   result['train']=training_metrics(training,probes,noise)
  return label,result,prediction
 # Do not concurrently schedule two jobs to the same GPU.
 results=[]
 items=list(enumerate(specs.items()))
 for offset in range(0,len(items),len(bench.gpus)):
  with concurrent.futures.ThreadPoolExecutor(max_workers=len(bench.gpus)) as pool:results.extend(pool.map(evaluate,items[offset:offset+len(bench.gpus)]))
 table={label:result for label,result,_ in results};predictions={label:p for label,_,p in results if p is not None}
 for label in ('P1','P3'):
  if label not in table:
   table[label]={'group':label,'status':'NO_ACCEPTED_PARAMETERS' if label in fits else 'NOT_RUN_P1_STOP_GATE',
                 'tau_c':None,'b':None,'identifiability':fits.get(label,{}).get('status','NOT_EVALUATED')}
 write(out/'physics_decomposition.json',{'status':'EVALUATED','groups':table,
       'kinematic_error':read(Path(read(old/'episode_summary.json')['selected_report'])).get('evaluation'),
       'physics_units':'effective simulator parameters; no real torque scale claimed',
       'B3':'UNAVAILABLE_UNCALIBRATED_CURRENT','mode':'SIM_TO_SIM_BLIND_SYSID',
       'physics_stop_gate':read(out/'stop_gate.json')})
 csv_rows=[]
 for label,result in table.items():
  metrics=result.get('heldout',{}).get('metrics',{})
  csv_rows.append({'group':label,'status':result['status'],'train_RMSE_mm':result.get('train',{}).get('ee_position_rmse_m',float('nan'))*1000,
       'heldout_RMSE_mm':metrics.get('ee_position_rmse_m',float('nan'))*1000,
       'start_delay_error_s':metrics.get('start_time_error_s'),'velocity_error_m_s':metrics.get('ee_velocity_rmse_m_s'),
       'final_displacement_error_mm':metrics.get('final_displacement_error_m',float('nan'))*1000,
       'tau_c':result['tau_c'],'b':result['b'],'identifiability':result['identifiability']})
 with (out/'physics_decomposition.csv').open('w') as f:
  writer=csv.DictWriter(f,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
 if predictions:plot_heldout_predictions(read(refdir/'observable/P4.json'),predictions,out/'heldout_prediction.png')


def visual_prior(visual,manifest,W,policy):
 """Default vertical hinge at a visible panel boundary opposite its handle.

 Uses visual bounds, not dataset joint origin/axis. The perturbation is a frozen
 constructed benchmark uncertainty, not a claim of real reconstruction.
 """
 lo,hi=np.asarray(manifest['moving_source_bounds'],float)*float(manifest['scale_source_to_meters'])
 corners=np.asarray(list(itertools.product(*zip(lo,hi))))
 points=corners@W[:3,:3].T+W[:3,3]
 normal=np.asarray(visual['outward_normal_world'],float);normal/=np.linalg.norm(normal)
 up=np.array([0.,0.,1.]);lateral=np.cross(up,normal);lateral/=np.linalg.norm(lateral)
 anchor=np.asarray(visual['anchor_world_m'],float);center=points.mean(0)
 extents=[float(np.min(points@lateral)),float(np.max(points@lateral))]
 edge=max(extents,key=lambda v:abs(v-anchor@lateral))
 point=center+lateral*(edge-center@lateral)+normal*float(policy['axis_line_offset_m'])
 axis=Rotation.from_rotvec(normal*np.deg2rad(policy['axis_perturbation_deg'])).apply(up)
 radius=np.linalg.norm(np.cross(axis,anchor-point))
 return {'joint_type':'revolute','confidence':0.,'source':'constructed visual/default-articulation prior; no joint-axis/origin read',
         'revolute':{'axis':axis.tolist(),'point_on_axis':point.tolist(),'radius_m':float(radius)},
         'observed_range':None,'full_joint_limits_identified':False}


def standalone_visual_prior(compiled_asset,destination,default_limits,prior):
 """Keep the visual reframe, replacing inherited GT stops with a default window.

 The compiler's T0/T1 copies are geometry-audit intermediates, not the visual
 prior. Only this separately labeled asset is evaluated as T_prior.
 """
 import shutil,hashlib,xml.etree.ElementTree as ET
 from interactive_twin.twin import audit_initial_geometry
 destination=Path(destination)
 if destination.exists():return destination
 shutil.copytree(compiled_asset,destination)
 manifest=read(destination/'manifest.json')
 path=destination/'urdf'/f"{manifest['asset_id']}.urdf"
 tree=ET.parse(path);joint=next(j for j in tree.findall('joint') if j.get('name')==manifest['joint_name'])
 limits={'lower':float(default_limits[0]),'upper':float(default_limits[1])}
 for name,value in limits.items():joint.find('limit').set(name,format(value,'.17g'))
 ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
 # Zero-state geometry, scale, mass and inertia must remain identical.
 metadata=read(destination/'twin.json')
 audit=audit_initial_geometry(Path(compiled_asset)/'urdf'/path.name,path,np.asarray(metadata['T_world_asset_initial']))
 metadata.update(version='T_prior',kinematics_updated=False,physics_updated=False,
   estimated_articulation=None,visual_default_prior=prior,observed_range=None,
   physical_joint_limits={**limits,'units':'rad','source':'frozen DEV default operation window; unknown actual stops',
                          'full_joint_limits_identified':False},
   joint_coordinate_convention={'positive_direction':'visual/default estimated axis','GT_axis_sign_used':False},
   initial_geometry_audit=audit)
 manifest.update(prepared_urdf=str(path),prepared_geometry_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                 source_joint_limits_rad=limits)
 manifest['interactive_twin'].update(version='T_prior',twin_metadata=str(destination/'twin.json'),
      axis_source='visual/default prior',legacy_source_joint_limits_field_semantics='default window, not dataset GT stops',
      coordinate_sign_vs_prepared_prior=None)
 write(destination/'twin.json',metadata);write(destination/'manifest.json',manifest)
 return destination


def run_prior(bench):
 from interactive_twin.twin import write_twins
 from interactive_twin.sysid import NoiseScales
 old,out,c,twins,reference,refdir=context(bench)
 gate=out/'stop_gate.json'
 if not gate.exists() or not read(gate).get('physics_expansion_allowed'):
  write(bench.output/'prior_comparison.json',{'status':'NOT_RUN_PHYSICS_STOP_GATE','reason':'P1 has not established heldout improvement'});return
 full=out/'physics_decomposition.json'
 if not full.exists() or read(full).get('status')!='EVALUATED':return
 original_asset=Path(reference['asset_root']);manifest=read(original_asset/'manifest.json')
 visual_source=bench.original/'episodes/dev_7320_00/dev_kinematics/initial_visual_handle_world.json'
 if visual_source.exists():visual=read(visual_source)
 else:
  from interactive_twin.planning import make_deployment_template
  visual=make_deployment_template(original_asset,Path(reference['source'])/'report.json')['initial_visual_handle_world']
 W=np.eye(4);W[:3,:3]=c['scene']['asset_rotation'];W[:3,3]=c['scene']['asset_xyz']
 prior=visual_prior(visual,manifest,W,bench.experiment['prior'])
 prior_dir=bench.output/'visual_prior'
 write(prior_dir/'prior_definition.json',prior)
 artifact=prior_dir/'compiler_geometry_audit'
 if (artifact/'twin_versions.json').exists():prior_twins=read(artifact/'twin_versions.json')
 else:prior_twins=write_twins(original_asset,artifact,prior,W,initial_physics_prior=bench.experiment['prior']['physics'])
 asset=standalone_visual_prior(Path(prior_twins['versions']['T1']['asset_root']),prior_dir/'T_prior',
                              bench.config['physics']['operational_joint_limits_rad'],prior)
 job=native_job(bench,reference,prior_dir/'heldout',asset,bench.experiment['prior']['physics'],refdir/'command_tape.json',bench.gpus[0])
 report=bench.run(job);p=Path(job['output'])/'observable/P4.json'
 result={'status':report['status'],'prior_source':prior['source'],'GT_twin_excluded_from_Real2Sim_prior':True,
         'geometry_improvement_claimed':False,'fixed_DEV_perturbation':bench.experiment['prior'],
         'updated_model':'P3, selected before heldout evaluation','mode':'SIM_TO_SIM_BLIND_SYSID'}
 if p.exists() and read(p)['provenance'].get('complete'):
  result['T_prior']=metric(read(refdir/'observable/P4.json'),read(p),NoiseScales(**bench.calibration['noise_scales']))
  result['T_updated']=read(full)['groups'].get('P3',{}).get('heldout')
  if result['T_updated']:
   result['heldout_improved']=result['T_updated']['metrics']['ee_position_rmse_m']<result['T_prior']['metrics']['ee_position_rmse_m']
 write(bench.output/'prior_comparison.json',result)
