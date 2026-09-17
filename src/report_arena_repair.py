"""Read-only report of frozen repair runs and recorded physical drift."""
import json,gzip,csv,collections,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/arena_repair_summary';OUT.mkdir(parents=True,exist_ok=True)
def wilson(k,n):
    if not n:return None
    z=1.95996398454;p=k/n;d=1+z*z/n;c=(p+z*z/(2*n))/d
    h=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return [c-h,c+h]
def drift(path,r):
    if not path.exists():return {}
    records=json.load(gzip.open(path,'rt'))['records']
    before=[a for a in records if a['phase'] in ['PREGRASP','APPROACH']]
    physical=dict(preclose_peak_finger_target_force_N=max((max(a['forces']) for a in before),default=0.))
    close=r.get('close_samples',[]);hold=r.get('hold_samples',[])
    closed=bool(close) and all(min(a['forces'])>.1 for a in close)
    lift_records=[a for a in records if a['phase']=='LIFT']
    lost=bool(lift_records) and closed and max(lift_records[-1]['forces'])<=.1
    returned=bool(hold) and max(hold[-1]['forces'])<=.1 and abs(hold[-1]['z']-r['initial_target']['position'][2])<=.002
    physical.update(lost_both_contacts_by_lift_end=lost,observed_drop_back_to_support=lost and returned,
                    retained_contact_insufficient_lift=bool(hold) and not r['success'] and min(hold[-1]['forces'])>.1 and max(a['z'] for a in hold)<r['initial_target']['position'][2]+.08)
    initial=r.get('telemetry',{}).get('attachment_close_T_TCP_target')
    if initial is None:return physical
    X=np.array(initial);rows=[a for a in records if a['phase']=='LIFT']
    if not rows:return physical
    H=Rotation.from_quat(np.roll(np.array([a['tcp_quat'] for a in rows]),-1,axis=1)).as_matrix()
    O=Rotation.from_quat(np.roll(np.array([a['box_quat'] for a in rows]),-1,axis=1)).as_matrix()
    delta=np.array([a['box'] for a in rows])-np.array([a['tcp'] for a in rows])
    p=np.einsum('nji,nj->ni',H,delta);R=np.einsum('nji,njk->nik',H,O)
    return dict(**physical,max_relative_translation_mm=float(np.linalg.norm(p-X[:3,3],axis=1).max()*1000),max_relative_rotation_deg=float(np.rad2deg(Rotation.from_matrix(np.einsum('ij,njk->nik',X[:3,:3].T,R)).magnitude()).max()))
summary={};all_rows=[]
for cohort in ['original40','heldout32']:
    run=ROOT/f'results/arena_repair_{cohort}';rows=[];details=[]
    for path in sorted(run.glob('B_seed*.json')):
        r=json.loads(path.read_text());selected=next((c for c in r['candidates'] if c['rank']==r['selected_rank']),None)
        row=dict(cohort=cohort,seed=r['seed'],target=r['target_class'],success=r['success'],category=r['category'],stage=r['last_stage'],rank=r['selected_rank'],candidate_count=len(r['candidates']),scene_filtered=r.get('scene_filtered_candidates',0),old_obb_filtered=r.get('old_obb_filtered_candidates',0),mesh_filtered=r.get('mesh_filtered_candidates',0),no_candidate=r['selected_rank'] is None,contact=r['non_target_contact'],disturbance=r['non_target_disturbance'],lift_ge_8cm=r['lift_ge_8cm'],hold_ge_2s=r['stable_hold_ge_2s'],max_lift_m=r.get('physics',{}).get('metrics',{}).get('max_target_lift_m'),displacement_mm=r['max_displacement_m']*1000,rotation_deg=r['max_rotation_deg'],planning_seconds=r['planning_seconds'],mesh_seconds=r.get('telemetry',{}).get('mesh_geometry_seconds'),old_selected_coverage=selected['hand_geometry']['min_pad_coverage'] if selected else None)
        row.update(drift(run/f"trace_seed_{r['seed']:04d}.json.gz",r));rows.append(row)
        details.append(dict(seed=r['seed'],candidates=r['candidates'],attachment_segments=r.get('telemetry',{}).get('lift_attachment_segments',[])))
    n=len(rows);success=sum(r['success'] for r in rows)
    classes={}
    for name in sorted(set(r['target'] for r in rows)):
        a=[r for r in rows if r['target']==name];k=sum(r['success'] for r in a)
        classes[name]=dict(n=len(a),success=k,rate=k/len(a),failures=dict(collections.Counter(r['category'] for r in a if not r['success'])),contact=sum(r['contact'] for r in a),disturbance=sum(r['disturbance'] for r in a))
    summary[cohort]=dict(n=n,success=success,success_rate=success/n if n else None,ci95=wilson(success,n),contact=sum(r['contact'] for r in rows),disturbance=sum(r['disturbance'] for r in rows),classes=classes,categories=dict(collections.Counter(r['category'] for r in rows)),complete=(run/'COMPLETE.json').exists())
    all_rows+=rows;(OUT/f'{cohort}_details.json').write_text(json.dumps(details,indent=2))
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
if all_rows:
    keys=list(dict.fromkeys(k for r in all_rows for k in r))
    with open(OUT/'episodes.csv','w') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(all_rows)
lines=['# Arena execution repair results','', 'The old baseline was 19/40. Incomplete cohorts below are progress, not final estimates.','']
for cohort,s in summary.items():
    lines += [f"## {cohort}: {s['success']}/{s['n']} (complete: {s['complete']})",'', '| Object | Success | Contact | Disturbance |','|---|---:|---:|---:|']
    for name,c in s['classes'].items():lines.append(f"| {name} | {c['success']}/{c['n']} | {c['contact']} | {c['disturbance']} |")
    lines+=['',f"Categories: {s['categories']}",'']
lines+=['| Cohort | Seed | Target | Result | Rank | Contact | Disturbance |','|---|---:|---|---|---:|---:|---:|']
for r in all_rows:lines.append(f"| {r['cohort']} | {r['seed']} | {r['target']} | {r['category']} | {r['rank']} | {int(r['contact'])} | {int(r['disturbance'])} |")
(OUT/'RESULTS.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(summary,indent=2))
