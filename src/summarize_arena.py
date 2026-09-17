"""Read-only episode/candidate/contact audit for the frozen Arena cohort."""
import csv,gzip,json,math,hashlib,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';OUT=ROOT/'results/arena_complex_summary';OUT.mkdir(exist_ok=True,parents=True)
def tf(p,q):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();T[:3,3]=p;return T
def wilson(k,n):
    z=1.95996398454;p=k/n;d=1+z*z/n;c=(p+z*z/(2*n))/d;r=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return [c-r,c+r]
def distribution(v):
    return dict(n=len(v),percentiles=np.percentile(v,[0,25,50,75,100]).tolist()) if v else dict(n=0)
def write(name,rows):
    if not rows:return
    with (OUT/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
rows=[];candidates=[];diagnostics=[]
for file in sorted(RUN.glob('B_seed*.json')):
    r=json.loads(file.read_text());seed=r['seed'];cs=r['candidates'];selected=r.get('selected_rank');geom=r.get('telemetry',{}).get('commanded_geometry',{})
    assert r['margin_m']==.001 and r['min_pad_coverage']==.045 and r['lift_gain_scale']==2
    raw=RUN/f'inputs/seed_{seed:04d}_grasps.json'
    if raw.exists():
        assert hashlib.sha256(raw.read_bytes()).hexdigest()==r['grasp_sha256']
        data=json.loads(raw.read_text())
        if selected is not None:assert r['telemetry']['grasp']==next(g for g in data['grasps'] if g['rank']==selected)
    if r['success']:
        h=r['hold_samples'];assert h[-1]['t']-h[0]['t']>=2
        assert min(x['z'] for x in h)>=r['initial_target']['position'][2]+.08
        assert min(min(x['forces']) for x in h)>.1 and max(x['z'] for x in h)-min(x['z'] for x in h)<=.01
    for c in cs:
        candidates.append(dict(seed=seed,target=r['target_class'],rank=c['rank'],score=c['score'],status=c['status'],stage=c['stage'],coverage=c['hand_geometry']['min_pad_coverage'],selected=c['rank']==selected,attached_target_collision=c.get('attached_target_collision',False),detail=c.get('detail','')))
    row=dict(seed=seed,target=r['target_class'],success=r['success'],category=r['category'],stage=r['last_stage'],non_target_contact=r['non_target_contact'],disturbance=r['non_target_disturbance'],selected_rank=selected,selected_coverage=geom.get('min_pad_coverage'),candidate_count=len(cs),scene_filtered=sum(c['status']=='SCENE_COLLISION' for c in cs),pad_filtered=sum(c['status']=='INSUFFICIENT_PAD_OVERLAP' for c in cs),no_candidate=selected is None,lift_ge_8cm=r['lift_ge_8cm'],hold_ge_2s=r['stable_hold_ge_2s'],max_displacement_m=r['max_displacement_m'],max_rotation_deg=r['max_rotation_deg'],planning_seconds=r['planning_seconds'],detail=r.get('detail',''))
    rows.append(row)
    trace=RUN/f'trace_seed_{seed:04d}.json.gz'
    if not trace.exists():continue
    records=json.load(gzip.open(trace,'rt'))['records'];active=[x for x in records if x['phase'] in ['CLOSE','MICRO_LIFT','LIFT','HOLD']]
    diag=dict(seed=seed,target=r['target_class'],category=r['category'],selected_coverage=geom.get('min_pad_coverage'),endpoints=r.get('telemetry',{}).get('motion_endpoints',[]),phase_relative_motion={},contacts={})
    if active:
        start=active[0]['t']
        for finger in range(2):
            samples=[x for x in active if x['forces'][finger]>.1]
            diag['contacts'][str(finger)]=dict(first_time_after_close_s=samples[0]['t']-start if samples else None,peak_force_N=max(x['forces'][finger] for x in active),contact_record_count=len(samples),representative_contacts=[dict(t=x['t'],phase=x['phase'],tcp=x['tcp'],tcp_quat=x['tcp_quat'],target=x['box'],target_quat=x['box_quat'],data=x['finger_contacts'][finger]) for x in samples[::max(1,len(samples)//8)]])
        for phase in ['CLOSE','MICRO_LIFT','LIFT','HOLD']:
            seq=[x for x in active if x['phase']==phase]
            if not seq:continue
            Ts=[np.linalg.inv(tf(x['tcp'],x['tcp_quat']))@tf(x['box'],x['box_quat']) for x in seq]
            delta=[np.linalg.inv(Ts[0])@t for t in Ts]
            diag['phase_relative_motion'][phase]=dict(duration_s=seq[-1]['t']-seq[0]['t'],max_translation_m=max(np.linalg.norm(t[:3,3]) for t in delta),max_rotation_deg=max(np.rad2deg(Rotation.from_matrix(t[:3,:3]).magnitude()) for t in delta),bilateral_fraction=sum(min(x['forces'])>.1 for x in seq)/len(seq),T_TCP_target_start=Ts[0].tolist(),T_TCP_target_end=Ts[-1].tolist())
    diagnostics.append(diag)
write('episodes.csv',rows);write('candidates.csv',candidates)
stats={}
for name in sorted(set(r['target'] for r in rows))+['ALL']:
    rs=[r for r in rows if name=='ALL' or r['target']==name];n=len(rs);k=sum(r['success'] for r in rs)
    stats[name]=dict(n=n,success=k,rate=k/n,ci95=wilson(k,n),non_target_contact=sum(r['non_target_contact'] for r in rs),disturbance=sum(r['disturbance'] for r in rs),categories=dict(collections.Counter(r['category'] for r in rs)),no_candidate=sum(r['no_candidate'] for r in rs),candidate_count=sum(r['candidate_count'] for r in rs),scene_filtered=sum(r['scene_filtered'] for r in rs),pad_filtered=sum(r['pad_filtered'] for r in rs),selected_success_coverage=distribution([r['selected_coverage'] for r in rs if r['success']]),selected_failure_coverage=distribution([r['selected_coverage'] for r in rs if not r['success'] and r['selected_coverage'] is not None]))
summary=dict(stats=stats,candidate_failures=dict(collections.Counter(c['status'] for c in candidates)),bad_contact_above_gate=[r['seed'] for r in rows if r['category']=='BAD_CONTACT' and r['selected_coverage'] is not None and r['selected_coverage']>=.045],attached_target_collision_rejections=[dict(seed=c['seed'],rank=c['rank'],detail=c['detail']) for c in candidates if c['attached_target_collision']],episode_count=len(rows))
(OUT/'summary.json').write_text(json.dumps(summary,indent=2));(OUT/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2))
print(json.dumps(summary,indent=2))
