"""Summarize paired episodes without dependencies or silently dropping failures."""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

def exact_p(b,c):
    n=b+c
    return min(1.,2*sum(math.comb(n,k) for k in range(min(b,c)+1))/2**n) if n else 1.

def summarize(root):
    rows=[json.loads(p.read_text()) for p in sorted(root.glob('[AB]_seed_*.json'))]
    paired={}
    for r in rows:paired.setdefault(r['seed'],{})[r['mode']]=r
    assert all(set(v)=={'A','B'} for v in paired.values()), 'Unfinished pairs'
    audits=[]
    for seed,pair in sorted(paired.items()):
        a,b=pair['A'],pair['B']
        assert a['grasp_sha256']==b['grasp_sha256'],f'Unpaired grasps: {seed}'
        delta=max(abs(x-y) for oa,ob in zip(a['initial_layout'],b['initial_layout']) for x,y in zip(oa['position'],ob['position']))
        audits.append(dict(seed=seed,identical_grasp_hash=True,max_initial_position_difference_m=delta))
    metrics=['success','lift_ge_8cm','stable_hold_ge_2s','non_target_contact','non_target_disturbance']
    stats={}
    for metric in metrics:
        ac=sum(bool(p['A'][metric]) for p in paired.values());bc=sum(bool(p['B'][metric]) for p in paired.values())
        aonly=sum(bool(p['A'][metric]) and not p['B'][metric] for p in paired.values())
        bonly=sum(bool(p['B'][metric]) and not p['A'][metric] for p in paired.values())
        stats[metric]=dict(A=ac,B=bc,n=len(paired),B_minus_A_pp=100*(bc-ac)/len(paired),A_only=aonly,B_only=bonly,exact_p=exact_p(aonly,bonly))
    out=dict(pairs=len(paired),metrics=stats,pairing_audit=audits,failures={m:dict(Counter(r['category'] for r in rows if r['mode']==m)) for m in ['A','B']},scene_filtered=sum(r.get('scene_filtered_candidates',0) for r in rows if r['mode']=='B'),B_candidates=sum(len(r['candidates']) for r in rows if r['mode']=='B'),attached_target_filtered=[dict(seed=r['seed'],rank=c['rank'],stage=c['stage'],contacts=c.get('contacts')) for r in rows if r['mode']=='B' for c in r['candidates'] if c.get('attached_target_collision')])
    (root/'summary.json').write_text(json.dumps(out,indent=2))
    fields=['seed','mode','success','lift_ge_8cm','stable_hold_ge_2s','non_target_contact','non_target_disturbance','max_displacement_m','max_rotation_deg','selected_rank','planning_seconds','category','last_stage','scene_filtered_candidates']
    with (root/'episodes.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    lines=['| Metric | A | B | B − A (pp) | Paired exact p |','|---|---:|---:|---:|---:|']
    for name,m in stats.items():lines.append(f"| {name} | {m['A']}/{m['n']} | {m['B']}/{m['n']} | {m['B_minus_A_pp']:+.1f} | {m['exact_p']:.4g} |")
    (root/'summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(out,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);summarize(p.parse_args().directory)
