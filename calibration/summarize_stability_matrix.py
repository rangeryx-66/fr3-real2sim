import argparse,csv,json,statistics,gc
from pathlib import Path
from collections import Counter,defaultdict

def distribution(x):
    if not x:return dict(n=0)
    x=sorted(x);return dict(n=len(x),min=x[0],median=statistics.median(x),max=x[-1],mean=statistics.mean(x))

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);a=p.parse_args();source=Path(a.input);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    # A formal result can be tens of MB because it contains every MoveIt path.
    # Retain only the fields used below so 120 episodes are processed with the
    # memory footprint of one result rather than the whole multi-GB matrix.
    results=[]
    for path in sorted(source.glob('*_seed*_r*_?.json')):
        with path.open() as handle:r=json.load(handle)
        attempts=[]
        for attempt in r.get('attempts',[]):
            attempts.append({key:attempt[key] for key in (
                'attempt','refinement_id','offset_translation_TCP_m',
                'offset_rotation_TCP_deg','torque','stability','transport_probe'
            ) if key in attempt})
        results.append({key:r.get(key) for key in (
            'id','target','scene_seed','physics_repeat','mode','success',
            'category','successful_attempt','selected_refinement_id','wall_seconds'
        )}|{'attempts':attempts})
        del r;gc.collect()
    rows=[];stats={}
    for r in results:
        attempts=r.get('attempts',[]);last=attempts[-1] if attempts else {}
        rows.append(dict(id=r['id'],target=r['target'],seed=r['scene_seed'],repeat=r['physics_repeat'],mode=r['mode'],success=r['success'],category=r['category'],attempts=len(attempts),successful_attempt=r.get('successful_attempt'),first_stable=bool(attempts and attempts[0].get('stability',{}).get('passed')),selected_refinement=r.get('selected_refinement_id'),last_stability=last.get('stability',{}).get('category'),wall_seconds=r['wall_seconds']))
    for target in ['soup','banana','bowl','mug']:
        stats[target]={}
        for mode in 'AB':
            group=[r for r in results if r['target']==target and r['mode']==mode]
            stats[target][mode]=dict(n=len(group),success=sum(r['success'] for r in group),categories=dict(Counter(r['category'] for r in group)),first_attempt_success=sum(bool(r.get('attempts')) and r['attempts'][0].get('stability',{}).get('passed') and r['success'] for r in group),regrasp_conversions=sum(r['success'] and r.get('successful_attempt',1)>1 for r in group),attempts=distribution([len(r.get('attempts',[])) for r in group]),wall_seconds=distribution([r['wall_seconds'] for r in group]))
            attempts=[a for r in group for a in r.get('attempts',[])]
            stats[target][mode]['attempt_stability']=dict(Counter(a.get('stability',{}).get('category','MISSING') for a in attempts))
            stats[target][mode]['transport_probes']=sum('transport_probe' in a for a in attempts)
            stats[target][mode]['torque_cost']=distribution([a['torque']['torque_cost'] for a in attempts if a.get('torque',{}).get('valid')])
            stats[target][mode]['COM_lever_mm']=distribution([a['torque']['COM_contact_line_lever_m']*1000 for a in attempts if a.get('torque',{}).get('valid')])
            stats[target][mode]['gravity_roll_lever_mm']=distribution([a['torque']['gravity_roll_lever_m']*1000 for a in attempts if a.get('torque',{}).get('valid')])
    paired=Counter()
    lookup={(r['target'],r['scene_seed'],r['physics_repeat'],r['mode']):r for r in results}
    for target in ['soup','banana','bowl','mug']:
        for seed in sorted({r['scene_seed'] for r in results if r['target']==target}):
            for repeat in range(3):
                if (target,seed,repeat,'A') in lookup and (target,seed,repeat,'B') in lookup:
                    ares=lookup[target,seed,repeat,'A']['success'];bres=lookup[target,seed,repeat,'B']['success'];paired[f'{target}:{int(ares)}->{int(bres)}']+=1
    acceptance=dict(complete=len(results)==120,controls=all(stats[x]['B']['success']>=13 and stats[x]['B']['success']>=stats[x]['A']['success']-1 for x in ['soup','banana']),complex_combined=stats['bowl']['B']['success']+stats['mug']['B']['success']>=15,advance=all(stats[x]['B']['success']>=10 for x in ['bowl','mug']))
    with open(out/'episodes.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(sorted(rows,key=lambda x:(x['target'],x['seed'],x['repeat'],x['mode'])))
    selected_changes=[]
    for r in results:
        if r['mode']=='B' and (r.get('successful_attempt') or 0)>1:
            first=r['attempts'][0];last=r['attempts'][r['successful_attempt']-1]
            selected_changes.append(dict(id=r['id'],attempt=r['successful_attempt'],refinement_id=last['refinement_id'],offset_translation_TCP_m=last['offset_translation_TCP_m'],offset_rotation_TCP_deg=last['offset_rotation_TCP_deg'],torque_before=first['torque'],torque_after=last['torque']))
    summary=dict(n=len(results),stats=stats,paired=dict(paired),acceptance=acceptance,failures=dict(Counter(r['category'] for r in results)),successful_refinements=selected_changes)
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
