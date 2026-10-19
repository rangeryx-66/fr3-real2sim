"""Offline paired V2 summary. Does not import into any execution actor."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1]
text=(ROOT/'scripts/summarize_native_g1_visual_trials.py').read_text().replace("B=ROOT/'results/native_g1_visual_input_20261014'", "B=ROOT/'results/native_g1_visual_reliability_20261015'").replace("for asset in ['45671','45638','45600','45606']:", "for asset in ['45600','45606','45671']:")
exec(compile(text,__file__+'::saved_evidence_only','exec'),globals())
previous=json.loads((ROOT/'results/native_g1_visual_input_20261014/PAIRED_RESULTS.json').read_text())
prior={r['asset_id']:r for r in previous['cases']}
for row in summary['cases']:
 a=row['asset_id'];p=B/a/'physical_01';old=prior[a]
 row['previous_visual']={k:old.get(k) for k in ['visual_grasp','actual_gain_after_acquisition_deg','short_transmission_ge2deg','physical_motion_started','closure_attempt_started','simulation_s','wall_s','peak_raw_pad_load_n','max_relative_shift_mm']}
 row['detection_coverage']=bool(list((p/'rgbd').glob('initial*/sam3_seed_mask.npy')))
 row['plan_available']=(p/'visual_plan.json').exists()
 row['perception_recovery']=json.loads((p/'perception_recovery.json').read_text()) if (p/'perception_recovery.json').exists() else None
 row['region_extensions']=[json.loads(f.read_text()) for f in (p/'rgbd').glob('initial*/observed_region_extension.json')]
summary.update(reference='9ae41fb',freeze_commit='4e91cc2',registered_cases=['45600','45606','45671'],previous_matched_short_successes=1,previous_matched_scene_count=3,old_four_case_visual_result_unchanged='2/4',detection_coverage=sum(r['detection_coverage'] for r in summary['cases']),planning_coverage=sum(r['plan_available'] for r in summary['cases']),extra_physical_trials=False)
(B/'PAIRED_RESULTS.json').write_text(json.dumps(summary,indent=2))
