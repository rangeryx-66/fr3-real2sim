"""Separate original terminal codes from observed physical failure sequences."""
import csv,json,gzip
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';OUT=ROOT/'results/arena_complex_summary';rows=[];physical=[]
for path in sorted(RUN.glob('B_seed*.json')):
    r=json.loads(path.read_text())
    seed=r['seed'];phase=r['last_stage'];category=r['category'];mechanism=category;evidence=r.get('detail','')
    trace=RUN/f'trace_seed_{seed:04d}.json.gz';records=json.load(gzip.open(trace,'rt'))['records'] if trace.exists() else []
    close=r.get('close_samples',[]);hold=r.get('hold_samples',[]);lift=[v for v in records if v['phase']=='LIFT'];closed=bool(close) and all(min(v['forces'])>.1 for v in close)
    lost=bool(lift) and closed and max(lift[-1]['forces'])<=.1
    returned=bool(hold) and max(hold[-1]['forces'])<=.1 and abs(hold[-1]['z']-r['initial_target']['position'][2])<=.002
    physical.append(dict(seed=seed,target=r['target_class'],success=r['success'],original_category=category,close_bilateral_passed=closed,lost_both_contacts_by_lift_end=lost,observed_drop_back_to_support=lost and returned,max_lift_m=r.get('physics',{}).get('metrics',{}).get('max_target_lift_m'),hold_end_forces=json.dumps(hold[-1]['forces']) if hold else '',note='Drop diagnostic: passed original close check, both forces <=0.1 N at lift end, then <=2 mm from initial height at HOLD with no bilateral contact. Does not change success.'))
    if r['success']:continue
    first=next((x for x in records if max(x['forces'])>.1),None)
    endpoints=r.get('telemetry',{}).get('motion_endpoints',[])
    if r['selected_rank'] is None:mechanism='NO_EXECUTABLE_CANDIDATE'
    elif first and first['phase']=='PREGRASP' and category=='BAD_CONTACT':
        mechanism='PREGRASP_TARGET_CONTACT';evidence=f'first finger-target force {first["forces"]} N in PREGRASP; '+evidence
    elif phase=='APPROACH' and category=='APPROACH_FAIL' and first and first['phase']=='APPROACH':
        mechanism='APPROACH_TARGET_CONTACT_AND_TRACKING_ABORT';evidence=f'peak target contact {max(max(x["forces"]) for x in records):.2f} N; '+evidence
    elif phase=='HOLD' and category=='BAD_CONTACT' and r.get('hold_samples') and min(r['hold_samples'][-1]['forces'])<=.1:
        mechanism='GRASP_LOST_BY_HOLD';evidence=f'max target lift {r["physics"]["metrics"]["max_target_lift_m"]*100:.2f} cm; final forces {r["hold_samples"][-1]["forces"]}; '+evidence
    elif phase=='HOLD' and category=='BAD_CONTACT' and hold and min(hold[-1]['forces'])>.1:
        mechanism='RETAINED_CONTACT_INSUFFICIENT_LIFT';evidence=f'held with final forces {hold[-1]["forces"]}; target lift {(hold[-1]["z"]-r["initial_target"]["position"][2])*100:.2f} cm; '+evidence
    elif phase=='LIFT' and category=='APPROACH_FAIL':mechanism='LIFT_EXECUTION_ABORT'
    rows.append(dict(seed=seed,target=r['target_class'],original_category=category,last_stage=phase,observed_mechanism=mechanism,selected_rank=r['selected_rank'],coverage=r.get('telemetry',{}).get('commanded_geometry',{}).get('min_pad_coverage'),last_position_error_m=endpoints[-1]['position_error_m'] if endpoints else None,max_lift_m=r.get('physics',{}).get('metrics',{}).get('max_target_lift_m'),evidence=evidence))
OUT.mkdir(exist_ok=True,parents=True)
if rows:
    with (OUT/'failure_mechanisms.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
if physical:
    with (OUT/'physical_outcomes.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(physical[0]));w.writeheader();w.writerows(physical)
print(json.dumps(rows,indent=2))
