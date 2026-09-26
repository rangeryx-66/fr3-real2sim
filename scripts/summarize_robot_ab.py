#!/usr/bin/env python3
"""Summarize paired FR3/Piper Arena runs and verify identical grasp inputs."""
from __future__ import annotations
import argparse,json,statistics
from collections import Counter,defaultdict
from pathlib import Path

OBJECTS=('mustard','raisin','hidden_tuna','bowl','banana','sugar','soup','mug')

def load(root):
    out={}
    for obj in OBJECTS:
        for p in sorted((root/obj).glob('B_seed_*.json')):
            d=json.loads(p.read_text());out[(obj,int(d['seed']))]=d
    return out

def metrics(rows):
    cats=Counter(x['category'] for x in rows)
    checked=sum(len(x.get('candidates',[])) for x in rows)
    executable=sum(sum(c.get('status')=='VALID' for c in x.get('candidates',[])) for x in rows)
    status=Counter(c.get('status','UNKNOWN') for x in rows for c in x.get('candidates',[]))
    ranks=[x['selected_rank'] for x in rows if x.get('selected_rank') is not None]
    selected=sum(x.get('selected_rank') is not None for x in rows)
    exhausted=sum(
        x.get('selected_rank') is None
        and x.get('last_stage') in {'CANDIDATE_CHECK','PREGRASP','APPROACH'}
        for x in rows
    )
    return dict(n=len(rows),success=sum(bool(x['success']) for x in rows),categories=dict(cats),
                non_target_contact=sum(bool(x.get('non_target_contact')) for x in rows),
                non_target_disturbance=sum(bool(x.get('non_target_disturbance')) for x in rows),
                candidates_checked=checked,executable_candidates=executable,
                executable_ratio=executable/checked if checked else None,candidate_status=dict(status),
                episodes_with_selected_candidate=selected,
                no_executable_candidate=exhausted,
                mean_selected_rank=statistics.fmean(ranks) if ranks else None,
                mean_planning_seconds=statistics.fmean(x.get('planning_seconds',0.) for x in rows) if rows else None)

def main():
    p=argparse.ArgumentParser();p.add_argument('--fr3',type=Path,required=True);p.add_argument('--piper',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    all_rows={'FR3':load(a.fr3),'Piper':load(a.piper)}
    keys=sorted(set(all_rows['FR3'])|set(all_rows['Piper']))
    paired=[]
    for key in keys:
        f=all_rows['FR3'].get(key);q=all_rows['Piper'].get(key)
        if f and q:
            paired.append(dict(object=key[0],seed=key[1],same_grasp_sha256=f.get('grasp_sha256')==q.get('grasp_sha256'),FR3=f,Piper=q))
    report={'schema':'fr3_piper_paired_arena/v1','paired_count':len(paired),
            'all_paired_grasp_inputs_identical':bool(paired) and all(x['same_grasp_sha256'] for x in paired),
            'overall':{},'per_object':{},'pairs':paired}
    for robot,rows in all_rows.items():report['overall'][robot]=metrics(list(rows.values()))
    for obj in OBJECTS:
        report['per_object'][obj]={robot:metrics([x for (o,_),x in rows.items() if o==obj]) for robot,rows in all_rows.items()}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2))
    md=['# FR3 vs Piper paired Arena benchmark','',f"Paired episodes: {report['paired_count']}; identical AnyGrasp JSON: {report['all_paired_grasp_inputs_identical']}",'',
        '|Robot|Success|Contact|Disturbance|No executable candidate|Executable candidates|Mean rank|Mean planning (s)|','|---|---:|---:|---:|---:|---:|---:|---:|']
    for robot,m in report['overall'].items():
        md.append(f"|{robot}|{m['success']}/{m['n']}|{m['non_target_contact']}/{m['n']}|{m['non_target_disturbance']}/{m['n']}|{m['no_executable_candidate']}/{m['n']}|{m['executable_candidates']}/{m['candidates_checked']}|{m['mean_selected_rank']}|{m['mean_planning_seconds']}|")
    md+=['','|Object|FR3|Piper|','|---|---:|---:|']
    for obj,r in report['per_object'].items():md.append(f"|{obj}|{r['FR3']['success']}/{r['FR3']['n']}|{r['Piper']['success']}/{r['Piper']['n']}|")
    a.output.with_suffix('.md').write_text('\n'.join(md)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='pairs'},indent=2))
if __name__=='__main__':main()
