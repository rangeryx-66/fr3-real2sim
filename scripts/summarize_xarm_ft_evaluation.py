"""Compact post-episode publication manifest; never used by an estimator/action."""
import json,argparse,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();r=a.root
out={'reference':'136b6e0','truth_boundary':json.loads((r/'POST_EPISODE_TRUTH_ACCESS.json').read_text()),'cases':[]}
for i in range(3):
 c=r/('case_%02d'%i);ep=c/'episode';e=json.loads((c/'INDEPENDENT_EVALUATION.json').read_text());end=json.loads((ep/'RECOVERY_EPISODE_RESULT.json').read_text());report=json.loads((ep/'report.json').read_text());truth=json.loads((ep/'HIDDEN_AUTHORED_PHYSICS.json').read_text());last=e['updates'][-1];exp=json.loads((ep/'EXPORT_COMPLETE.json').read_text())
 item={'case':i,'seed':truth['configuration']['seed'],'authored_configuration':truth['configuration'],'authored_truth':last['authored_truth'],'parameters':last['estimated']['parameters'],'conditional_std':last['estimated']['conditional_std'],'observability':last['estimated']['observability'],'relative_error_provisional_parameters':last['relative_error'],'absolute_error_provisional_parameters':last['absolute_error'],'heldout_torque_rmse_nm':last['estimated'].get('heldout_torque_rmse_nm'),'heldout_motion':last.get('heldout_motion_predictions',{}),'simulation_s':report['simulation_s'],'wall_s_report_including_export':report['wall_s'],'wall_s_runner':end['wall_s'],'status':end['status'],'lifecycle_success':end['full_lifecycle_success'],'grasps':end['physical_grasp_attempts'],'recovery_grasps':end['recovery_grasps'],'observation_adjustments':end['Agent_adjustments'],'attempts':[],'updates':[],'telemetry_export':{n:v['buffer_records'] for n,v in exp['streams'].items()},'telemetry_issues':exp['telemetry_issues'],'video':json.loads((c/'VIDEO_INTEGRITY.json').read_text())}
 for v in end['attempts']:
  item['attempts'].append({k:v.get(k) for k in ['index','termination_reason','acquisition','opening_gain_deg','force_events','peak_pad_load_n','minimum_joint_margin_rad','physical_attempt_wall_s','short_success']})
 for v in e['updates']:
  item['updates'].append({k:v['estimated'].get(k) for k in ['parameters','conditional_std','observability','rank','scaled_condition','training_rmse_nm','heldout_torque_rmse_nm','supported_samples','heldout_samples']})
 if e['strict_posthoc_boundary_evaluation']:
  s=e['strict_posthoc_boundary_evaluation'][-1];item['strict_posthoc']={'rule':s['rule'],'model':s['model'],'heldout_motion_predictions':s.get('heldout_motion_predictions',{})}
 f=ep/'FT_STREAM_INTEGRITY.json'
 if f.exists():item['stream_integrity']=json.loads(f.read_text())
 out['cases'].append(item)
(r/'COMPACT_RESULTS.json').write_text(json.dumps(out,indent=2));print(r/'COMPACT_RESULTS.json')
