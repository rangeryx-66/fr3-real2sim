"""Audit bowl outcomes without treating expected rim settling as slip."""
import argparse,csv,gzip,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from settling_gate import compute_metrics

VERTICES=np.load(ROOT/'assets/arena_complex/bowl_mesh.npz')['vertices']
CONTACT_N=.1
TABLE_SUPPORT_M=.0015
SUSPENDED_M=.002
TVEL=.00025
WVEL=.5

def clearance(row):
    q=np.asarray(row['box_quat']);R=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix()
    return float(np.min(VERTICES@R.T+np.asarray(row['box']),axis=0)[2])

def segments(records,phase='MICRO_LIFT'):
    answer=[];current=[]
    for row in records:
        if row['phase']==phase:current.append(row)
        elif current:answer.append(current);current=[]
    if current:answer.append(current)
    return answer

def inspect(rows):
    if not rows:return dict(has_attempt=False)
    end=rows[-1]['t'];tail=[x for x in rows if x['t']>=end-.5]
    metric=compute_metrics(tail);window=metric['windows']['200ms']
    bilateral=all(min(x['forces'])>CONTACT_N for x in rows)
    final_clearance=clearance(rows[-1]);peak_clearance=max(map(clearance,rows))
    moving=(window['relative_translation_velocity_m_s']>TVEL or
            window['relative_angular_velocity_deg_s']>WVEL)
    stable=metric['terminal_bilateral'] and not moving
    returned=(peak_clearance>SUSPENDED_M and final_clearance<=TABLE_SUPPORT_M)
    return dict(has_attempt=True,bilateral_throughout=bilateral,
        final_suspended=bilateral and final_clearance>SUSPENDED_M,
        stable_last_500ms=stable,
        true_continuous_slip=bilateral and moving,
        contact_not_sustained=not bilateral,
        returned_to_table=returned,
        never_left_table_support=peak_clearance<=SUSPENDED_M,
        final_clearance_mm=final_clearance*1000,
        peak_clearance_mm=peak_clearance*1000,
        tail_translation_velocity_mm_s=window['relative_translation_velocity_m_s']*1000,
        tail_angular_velocity_deg_s=window['relative_angular_velocity_deg_s'])

def tally(items):
    fields=['returned_to_table','never_left_table_support','bilateral_throughout',
            'contact_not_sustained','final_suspended','stable_last_500ms',
            'true_continuous_slip']
    valid=[x for x in items if x.get('has_attempt')]
    return dict(episodes=len(items),with_attempt=len(valid),
        **{key:sum(bool(x.get(key)) for x in valid) for key in fields},
        final_clearance_mm=[x['final_clearance_mm'] for x in valid],
        tail_translation_velocity_mm_s=[x['tail_translation_velocity_mm_s'] for x in valid],
        tail_angular_velocity_deg_s=[x['tail_angular_velocity_deg_s'] for x in valid])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--formal',required=True);parser.add_argument('--diagnostic',required=True);parser.add_argument('--output',required=True);a=parser.parse_args()
    formal=Path(a.formal);diagnostic=Path(a.diagnostic)
    episode_csv=formal.parent/'settling_regrasp_summary/episodes.csv'
    attempt_count={}
    if episode_csv.exists():
        with episode_csv.open() as handle:
            attempt_count={row['id']:int(row['attempts']) for row in csv.DictReader(handle)}
    report={'definition':dict(
        contact_force_threshold_N=CONTACT_N,table_support_clearance_m=TABLE_SUPPORT_M,
        suspended_clearance_m=SUSPENDED_M,tail_window_s=.5,
        translation_velocity_threshold_m_s=TVEL,angular_velocity_threshold_deg_s=WVEL)}
    for mode in 'AB':
        episodes=[];attempts=[]
        for result_path in sorted(formal.glob(f'bowl_seed*_r*_{mode}.json')):
            trace=result_path.with_suffix('').with_suffix('.trace.json.gz')
            records=json.load(gzip.open(trace))['records'];runs=segments(records)
            count=attempt_count.get(result_path.stem,1 if mode=='A' else len(runs))
            inspected=[inspect(x) for x in (runs[-count:] if count else [])];attempts.extend(inspected)
            episodes.append(inspected[-1] if inspected else dict(has_attempt=False))
        report[f'formal_{mode}']=dict(episode_final_attempt=tally(episodes),all_attempts=tally(attempts),
            interpretation='full 10 cm lift was not executed after gate rejection; <8 cm only is not identifiable')
    diagnostic_items=[]
    for result_path in sorted(diagnostic.glob('bowl_settling_r*.json')):
        result=json.loads(result_path.read_text());records=json.load(gzip.open(result_path.with_suffix('.trace.json.gz')))['records']
        hold=[x for x in records if x['phase']=='HOLD'];item=inspect(hold)
        item.update(max_lift_m=result['max_lift_m'],min_hold_lift_m=result['min_hold_lift_m'],
                    hold_duration_s=result['hold_duration_s'])
        item['failed_only_height']=(item['bilateral_throughout'] and item['final_suspended'] and
            item['stable_last_500ms'] and result['hold_duration_s']>=2 and
            result['max_lift_m']<.08)
        diagnostic_items.append(item)
    diag=tally(diagnostic_items);diag['failed_only_height']=sum(x['failed_only_height'] for x in diagnostic_items)
    diag['max_lift_cm']=[x['max_lift_m']*100 for x in diagnostic_items]
    diag['min_hold_lift_cm']=[x['min_hold_lift_m']*100 for x in diagnostic_items]
    report['uninterrupted_diagnostic']=diag
    destination=Path(a.output);destination.parent.mkdir(parents=True,exist_ok=True);destination.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
