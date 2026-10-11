from pathlib import Path
import json,collections,statistics,hashlib,time
root=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/revolute10_fixed_20261011')
registered=json.loads((root/'REGISTERED_OBJECTS.json').read_text());rows=[]
for obj in registered['objects']:
 i=obj['asset_id'];p=root/i;physical=p/'official_01';f=physical/'report.json';reserved=(physical/'OFFICIAL_ATTEMPT_RESERVED.json').exists();target=obj['target_deg'];row={'asset_id':i,'target_deg':target,'official_attempts':int(reserved),'target_reached':False,'grasp_success':False,'max_actual_angle_deg':None,'normalized_completion':0.,'simulation_s':0.,'wall_s':0.,'pending':False}
 if f.exists():
  d=json.loads(f.read_text());events=json.loads((physical/'events.json').read_text()) if (physical/'events.json').exists() else []
  acquired=next((e['angle_deg'] for e in events if e.get('event')=='GRASP_VERIFIED'),None)
  row.update(grasp_success=d['initial_grasp'],max_actual_angle_deg=d['max_actual_angle_deg'],raw_target_reached=d['target_reached'],initial_actual_angle_deg=d['initial_actual_angle_deg'],grasp_acquisition_angle_deg=acquired,simulation_s=d['simulation_s'],wall_s=d['wall_s'],status=d['status'],peak_pad_load_n=d['peak_pad_load_n'],minimum_joint_margin_rad=d['minimum_joint_margin_rad'],shared_execution_logic=d['shared_execution_logic'])
  # Evidence attribution only; never consumed by manipulation.
  row['robot_contact_progress_deg']=max(0.,d['max_actual_angle_deg']-acquired) if acquired is not None else 0.
  row['target_reached']=bool(d['target_reached'] and d['initial_grasp'] and row['robot_contact_progress_deg']>0)
  row['normalized_completion']=min(1.,max(0.,d['max_actual_angle_deg']/target));row['normalized_completion_definition']='maximum actual angle / authored target; grasp success reported independently'
  status=d['status']
  row['failure_category']='none' if row['target_reached'] else 'speed protection' if 'SPEED' in status else 'collision' if any(x in status for x in ['COLLISION','LOADED_CONTACT','ROOT_HANDLE']) else 'contact loss' if 'CONTACT_LOST' in status else 'IK' if any(x in status for x in ['IK','MARGIN','REACHABLE_INTERVAL']) else 'perception/grasp' if 'HOLD_NOT_ESTABLISHED' in status else 'other'
  row['failure_reason']=None if row['target_reached'] else status
  if d['target_reached'] and not row['target_reached']:row['failure_reason']='No positive opening attributable to robot after grasp; passive initial motion is not credited'
  row['base_moves']=d['base_moves_after_grasp'];row['regrasps']=d['regrasps']
 elif (p/'planning_failure.json').exists():
  d=json.loads((p/'planning_failure.json').read_text());row.update(failure_category=d['failure_category'],failure_reason=d['failure_reason'],planning=d.get('planning'),grasp_success=None)
  audit=p/'planning/candidate_audit.json'
  if audit.exists():
   failures=collections.Counter(x.get('blocker','none') or 'none' for x in json.loads(audit.read_text()));row['planning_blocker_counts']=dict(failures)
   meaningful=[x for x in failures.most_common() if x[0]!='none']
   if meaningful:
    reachable=[x for x in json.loads(audit.read_text()) if 'reached_rad' in x]
    collision_candidates=[x for x in reachable if 'COLLISION' in (x.get('blocker') or '')]
    maximum_planned=max((x['reached_rad'] for x in reachable),default=0.)
    why=meaningful[0][0];row['dominant_planning_blocker']=why;row['maximum_kinematic_arc_before_approach_rad']=maximum_planned
    row['failure_category']='collision' if maximum_planned==0 and collision_candidates else 'IK'
    row['planning_failure_detail']='IK-solvable candidates collided before any feasible opening arc' if row['failure_category']=='collision' else 'Some arc samples feasible, but bounded initial grasp/approach/base planning produced no executable candidate'
  for n in ['setup/setup_cleanup.json','planning/planning_cleanup.json']:
   if (p/n).exists():row['wall_s']+=json.loads((p/n).read_text())['wall_s']
 elif (physical/'execution_failure.json').exists():
  row.update(failure_category='other',failure_reason=json.loads((physical/'execution_failure.json').read_text())['failure_reason'])
 elif (p/'batch_exception.json').exists():
  row.update(failure_category='other',failure_reason=json.loads((p/'batch_exception.json').read_text())['error'])
 else:row['pending']=True
 rows.append(row)
summary={'classification':'KNOWN_MODEL_DIAGNOSTIC','registered_assets_unchanged':[x['asset_id'] for x in rows]==registered['asset_ids'],'completed_cases':sum(not x['pending'] for x in rows),'full_task_successes':sum(x['target_reached'] for x in rows),'denominator':10,'physical_grasp_successes':sum(x['grasp_success'] is True for x in rows),'official_physical_attempts':sum(x['official_attempts'] for x in rows),'mean_normalized_completion':statistics.mean(x['normalized_completion'] for x in rows),'median_normalized_completion':statistics.median(x['normalized_completion'] for x in rows),'failure_categories':dict(collections.Counter(x.get('failure_category','pending') for x in rows)),'cases':rows,'updated_wall':time.time()}
(root/'BENCHMARK_RESULTS.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='cases'},indent=2));print('\n'.join(str({k:r[k] for k in ['asset_id','official_attempts','max_actual_angle_deg','target_reached','grasp_success','pending']}) for r in rows))
