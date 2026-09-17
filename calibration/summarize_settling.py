import argparse,csv,gzip,json,sys
from pathlib import Path
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parent))
from settling_gate import compute_metrics,thresholds_from_controls,evaluate

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    source=Path(a.input);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    rows=[];control=[]
    for path in sorted(source.glob('*_settling_r*.json')):
        result=json.loads(path.read_text());trace=json.load(gzip.open(source/result['trace']['path'].split('/')[-1]))['records']
        micro=[x for x in trace if x['phase']=='MICRO_LIFT']
        duration=result.get('new_gate',{}).get('metrics',{}).get('duration_s')
        gate_rows=[x for x in micro if duration is None or x['t']<=micro[0]['t']+duration+1e-6]
        metric=compute_metrics(gate_rows)
        if result['target'] in ['soup','banana']:control.append(metric)
        rows.append((result,path,gate_rows,metric))
    thresholds=thresholds_from_controls(control)
    (out/'frozen_thresholds.json').write_text(json.dumps(thresholds.__dict__,indent=2))
    table=[];conf=dict(old=Counter(),new_without_probe=Counter(),new_with_probe=Counter())
    for result,path,micro,metric in rows:
        new=evaluate(micro,thresholds=thresholds,old_gate=result.get('old_gate'))
        truth=bool(result['success'])
        old=bool(result.get('old_gate',{}).get('passed'))
        conf['old'][('PASS' if old else 'REJECT','SUCCESS' if truth else 'FAIL')]+=1
        conf['new_without_probe'][('PASS' if new['passed'] else 'REJECT','SUCCESS' if truth else 'FAIL')]+=1
        probe=None;final_new=new
        w=metric['windows']['200ms']
        eligible=(not new['passed'] and metric['terminal_bilateral'] and metric['max_contact_gap_s']<=.05 and metric['max_cumulative_translation_m']<=.010 and metric['max_cumulative_rotation_deg']<=15 and w['relative_translation_velocity_m_s']<=thresholds.translation_velocity_m_s and w['relative_angular_velocity_deg_s']<=thresholds.angular_velocity_deg_s)
        if eligible:
            full=json.load(gzip.open(source/result['trace']['path'].split('/')[-1]))['records'];all_micro=[x for x in full if x['phase']=='MICRO_LIFT']
            gate_end=all_micro[0]['t']+result['new_gate']['metrics']['duration_s'];after=[x for x in all_micro if x['t']>=gate_end]
            if len(after)>=3:
                z=after[0]['tcp'][2];end=next((i for i,x in enumerate(after) if x['tcp'][2]>=z+.005),len(after)-1);until=after[end]['t']+.3;probe_rows=[x for x in after if x['t']<=until]
                if len(probe_rows)>=3:probe=evaluate(probe_rows,thresholds=thresholds,old_gate=result.get('old_gate'));final_new=probe
        conf['new_with_probe'][('PASS' if final_new['passed'] else 'REJECT','SUCCESS' if truth else 'FAIL')]+=1
        table.append(dict(id=result['id'],target=result['target'],success=truth,final=result['category'],old_pass=old,new_without_probe=new['passed'],new_with_probe=final_new['passed'],new_category=final_new['category'],probe_used=probe is not None,cum_mm=metric['max_cumulative_translation_m']*1000,cum_deg=metric['max_cumulative_rotation_deg'],velocity_200_mm_s=metric['windows']['200ms']['relative_translation_velocity_m_s']*1000,angular_200_deg_s=metric['windows']['200ms']['relative_angular_velocity_deg_s'],vertical_follow=metric['vertical_follow_ratio'],following_residual_mm=metric['following_residual_m']*1000))
    with open(out/'episodes.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=table[0].keys());w.writeheader();w.writerows(table)
    summary=dict(n=len(rows),thresholds=thresholds.__dict__,outcomes=Counter(x['final'] for x in table),confusion={name:{'|'.join(k):v for k,v in values.items()} for name,values in conf.items()})
    (out/'summary.json').write_text(json.dumps(summary,indent=2,default=dict));print(json.dumps(summary,indent=2,default=dict))

if __name__=='__main__':main()
