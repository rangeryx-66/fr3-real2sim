"""Aggregate the completed, byte-matched FR3/R1 tabletop benchmark."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/r1a7_fr3_exact_ab'
def read(path):return json.loads(path.read_text())

audit=read(OUT/'input_audit.json')
if not audit['all_matched'] or len(audit['rows'])!=10:raise SystemExit('FR3/R1 input audit incomplete')
fr3_audit=read(OUT/'fr3_candidate_audit.json')
r1_audit=read(OUT/'r1_candidate_audit.json')
fr3_run=read(OUT/'fr3_orchestration.json')
def arm(mode):
    rows=[read(OUT/mode/f'trial_{i:02d}.json') for i in range(1,11)]
    summary=read(OUT/mode/'summary.json')
    first=rows[0]
    check=r1_audit[mode]
    if check['total']!=len(first['candidates']) or check['path_valid']!=summary['candidate_counts']['path_valid']//10:
        raise SystemExit(f'{mode} candidate audit does not match physical trials')
    return dict(trials=len(rows),candidate_total=summary['candidate_counts']['raw'],
                path_valid_candidates=summary['candidate_counts']['path_valid'],
                executable_candidates=check['executable']*10,
                executable_per_unique_input=check['executable'],
                original_candidate_statuses=check['categories'],
                planning_success=summary['planning_successes'],
                contact_loss_events=summary['contact_loss_events'],
                recovery_events=summary['recovery_events'],
                final_lift_success=summary['successes'],
                final_categories=summary['categories'],
                selected_ranks=[r.get('selected_rank') for r in rows],
                lift_m=[r.get('lift_m') for r in rows if r.get('success')])

fr3_summary=fr3_run.get('summary') or {}
report=dict(input_audit=audit,unique_input_count=1,repeated_trials=10,
            fr3=dict(candidate_total=fr3_audit['total']*10,
                     executable_candidates=fr3_audit['executable']*10,
                     executable_per_unique_input=fr3_audit['executable'],
                     original_candidate_statuses=fr3_audit['categories'],
                     planning_success=sum(r['fr3_category']=='SUCCESS' for r in audit['rows']),
                     final_lift_success=fr3_summary.get('successes'),
                     final_categories=fr3_summary.get('categories')),
            r1_raw=arm('raw'),r1_adapted=arm('adapted'))
(OUT/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='input_audit'},indent=2))
