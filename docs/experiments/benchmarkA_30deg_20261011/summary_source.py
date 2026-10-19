from pathlib import Path
import json,collections,math,sys,time
root=Path(sys.argv[1]);reg=json.loads((root/'REGISTERED_OBJECTS.json').read_text());cases=[]
for obj in reg['objects']:
 i=obj['asset_id'];p=root/i;out=p/'official_01';row={'asset_id':i,'handle_type':obj['handle_type'],'eligibility_handle_mesh':obj['eligibility_handle_mesh'],'target_deg':30.0,'planning':False,'grasp':None,'max_actual_angle_after_valid_grasp_deg':None,'raw_max_actual_angle_deg':None,'success_30_deg':False,'official_attempts':int((out/'OFFICIAL_ATTEMPT_RESERVED.json').exists()),'pending':False,'simulation_s':0.,'physical_wall_s':0.}
 f=p/'planning/planning_summary.json'
 if f.exists():
  d=json.loads(f.read_text());row.update(planning=d['found'],planned_max_angle_deg=math.degrees(d['planned_reached_rad']) if d['planned_reached_rad'] is not None else None,planning_wall_s=d['wall_s'])
 f=p/'planning/candidate.json'
 if f.exists():
  d=json.loads(f.read_text());row.update(actual_selected_grasp_mesh=d['grasp_geometry_selection']['mesh'],grasp_mesh_matches_eligibility_handle=d['grasp_geometry_selection']['mesh']==obj['eligibility_handle_mesh'],selected_base=d['base'])
 f=out/'BENCHMARK_A_METRICS.json'
 if f.exists():
  m=json.loads(f.read_text());row.update(grasp=m['grasp_confirmed'],max_actual_angle_after_valid_grasp_deg=m['max_actual_angle_during_opening_deg'],raw_max_actual_angle_deg=m['max_actual_angle_all_phases_deg'],success_30_deg=m['success_30_deg'],simulation_s=m['simulation_s'],physical_wall_s=m['wall_s'],first_physical_divergence=m['first_physical_divergence'],terminal_status=m['terminal_status'],peak_pad_loads_n=m['peak_pad_loads_n'],minimum_loaded_joint_margin_rad=m['minimum_loaded_joint_margin_rad'])
  status=m['terminal_status'];row['failure_reason']=None if row['success_30_deg'] else status
  row['failure_category']='none' if row['success_30_deg'] else 'speed protection' if 'SPEED_LIMIT' in status else 'collision/contact placement' if any(s in status for s in ['FINGER_BACK','COLLISION','LOADED_CONTACT']) else 'contact loss' if 'CONTACT_LOST' in status else 'grasp failure' if 'BILATERAL_HOLD' in status else 'IK/joint margin' if any(s in status for s in ['MARGIN','IK','REACHABLE']) else 'force overload' if 'FORCE_STOP' in status else 'other'
 elif (p/'planning_failure.json').exists():
  row.update(failure_category='planning failure',failure_reason=json.loads((p/'planning_failure.json').read_text())['failure_reason'])
  audit=p/'planning/candidate_audit.json'
  if audit.exists():row['planning_blocker_counts']=dict(collections.Counter(x.get('blocker') or 'none' for x in json.loads(audit.read_text())))
 elif (out/'execution_failure.json').exists():
  row.update(failure_category='other',failure_reason=json.loads((out/'execution_failure.json').read_text())['failure_reason'])
 elif (p/'plan_case_exception.json').exists() or (p/'execute_case_exception.json').exists():
  f=p/'plan_case_exception.json' if (p/'plan_case_exception.json').exists() else p/'execute_case_exception.json';row.update(failure_category='other',failure_reason=json.loads(f.read_text())['error'])
 else:row['pending']=True
 cases.append(row)
result={'classification':'KNOWN_MODEL_DIAGNOSTIC','benchmark':'Benchmark A standard hinged doors / 30 degrees','registered_asset_ids_unchanged':[r['asset_id'] for r in cases]==reg['asset_ids'],'completed_cases':sum(not c['pending'] for c in cases),'planning_coverage':sum(c['planning'] for c in cases),'physical_grasp_successes':sum(c['grasp'] is True for c in cases),'success_30_deg':sum(c['success_30_deg'] for c in cases),'denominator':10,'official_attempts':sum(c['official_attempts'] for c in cases),'failure_categories':dict(collections.Counter(c.get('failure_category','pending') for c in cases)),'cases':cases,'updated_wall':time.time()}
(root/'BENCHMARK_A_RESULTS.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2));print('\n'.join(str({k:c[k] for k in ['asset_id','planning','grasp','max_actual_angle_after_valid_grasp_deg','success_30_deg','pending']}) for c in cases))
