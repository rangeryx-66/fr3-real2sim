"""Audit the fixed-JSON experiments and export complete episode/replay tables."""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/contact_summary';OUT.mkdir(exist_ok=True)
BAD=[0,1,6,8,13,14,17];CONTROL=[3,4,5,7,9,10,11]
def read(group):return [json.loads(p.read_text()) for p in sorted((ROOT/'results'/group).glob('[AB]_seed*.json'))]
def csv_write(name,rows,fields):
    with (OUT/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
groups=['clutter_ab_20_v1','contact_margin1','contact_repaired','contact_repaired_v2']
rows=[];stats={};audits=[]
for group in groups:
    data=[r for r in read(group) if r['mode']=='B'];assert len(data)==20,(group,len(data))
    for r in data:
        rows.append(dict(run=group,**r))
        source=ROOT/f"results/clutter_ab_20_v1/inputs/seed_{r['seed']:04d}_grasps.json"
        assert r['grasp_sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
        if group!='clutter_ab_20_v1' and r['selected_rank'] is not None:
            raw=json.loads(source.read_text())['grasps'][r['selected_rank']]
            assert r['telemetry']['grasp']==raw
        if group.startswith('contact_repaired'):
            valid=[c['rank'] for c in r['candidates'] if c['status']=='VALID']
            assert r['selected_rank']==(min(valid) if valid else None)
            assert r['margin_m']==.001 and r['min_pad_coverage']==.045
        if group=='contact_repaired_v2':
            base=r['controller_gains']; assert base['scale']==1 and r['lift_gain_scale']==2
            lifted=r.get('lift_controller_gains')
            if lifted:
                assert lifted['kp'][-2:]==base['kp'][-2:] and lifted['kd'][-2:]==base['kd'][-2:]
                assert all(abs(a-2*b)<.01 for a,b in zip(lifted['kp'][:7],base['kp'][:7]))
                assert all(abs(a-2**.5*b)<1e-6 for a,b in zip(lifted['kd'][:7],base['kd'][:7]))
        if r['success']:
            assert r['lift_ge_8cm'] and r['stable_hold_ge_2s']
            hs=r['hold_samples'];assert hs[-1]['t']-hs[0]['t']>=2
            assert min(h['z'] for h in hs)>=r['initial_target']['position'][2]+.08
            assert min(min(h['forces']) for h in hs)>.1
            assert max(h['z'] for h in hs)-min(h['z'] for h in hs)<=.01
        audits.append(dict(run=group,seed=r['seed'],raw_hash_match=True))
    stats[group]=dict(n=20,success=sum(r['success'] for r in data),contact=sum(r['non_target_contact'] for r in data),disturbance=sum(r['non_target_disturbance'] for r in data),categories=dict(Counter(r['category'] for r in data)),scene_filtered=sum(sum(c['status']=='SCENE_COLLISION' for c in r['candidates']) for r in data),geometry_filtered=sum(sum(c['status']=='INSUFFICIENT_PAD_OVERLAP' for c in r['candidates']) for r in data))
fields=['run','seed','success','category','selected_rank','selected_score','non_target_contact','non_target_disturbance','max_displacement_m','max_rotation_deg','lift_ge_8cm','stable_hold_ge_2s','planning_seconds']
csv_write('formal_episodes.csv',rows,fields)
replay_groups=['contact_replay_clutter_r1','contact_replay_empty_r1','contact_replay_empty_r2','contact_new18_clutter','contact_new18_empty']
replays=[]
for group in replay_groups:
    for r in read(group):
        selected=ROOT/f"results/{'contact_margin1' if 'new18' in group else 'clutter_ab_20_v1'}/B_seed_{r['seed']:04d}.json"
        original=json.loads(selected.read_text());assert r['selected_rank']==original['selected_rank']
        raw=json.loads((ROOT/f"results/clutter_ab_20_v1/inputs/seed_{r['seed']:04d}_grasps.json").read_text())['grasps'][r['selected_rank']]
        assert r['telemetry']['grasp']==raw
        replays.append(dict(run=group,**r))
assert len(replays)==30,len(replays)
csv_write('pose_replays.csv',replays,fields+['physical_clutter'])
baseline={r['seed']:r for r in read('contact_margin1')}
classification=[]
for seed in BAD:
    observations=[baseline[seed]]+[r for r in replays if r['seed']==seed]
    assert len(observations)==4
    classification.append(dict(seed=seed,rank=baseline[seed]['selected_rank'],clutter_success=sum(r['success'] for r in observations if r['physical_clutter']),clutter_n=2,empty_success=sum(r['success'] for r in observations if not r['physical_clutter']),empty_n=2,primary='INSUFFICIENT_PAD_OVERLAP',family='GRIPPER_GEOMETRY'))
csv_write('original_bad_contact_classification.csv',classification,['seed','rank','clutter_success','clutter_n','empty_success','empty_n','primary','family'])
sweep=json.loads((ROOT/'results/margin_sweep/sweep.json').read_text());sweep_rows=[]
for seed in range(20):
    s={r['margin_m']:r for r in sweep if r['seed']==seed}
    sweep_rows.append(dict(seed=seed,remaining_0mm=s[0.]['remaining'],remaining_1mm=s[.001]['remaining'],remaining_2mm=s[.002]['remaining']))
csv_write('margin_candidates.csv',sweep_rows,['seed','remaining_0mm','remaining_1mm','remaining_2mm'])
out=dict(stats=stats,raw_input_audit=audits,original_bad_classification=classification,replay_n=len(replays),margin_sweep=sweep_rows,acceptance_16_of_20=stats['contact_repaired_v2']['success']>=16 and stats['contact_repaired_v2']['contact']==0 and stats['contact_repaired_v2']['disturbance']==0,calibration_warning='The 4.5% geometric threshold was calibrated on the diagnostic seeds; this is not held-out generalization evidence')
(OUT/'summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(stats,indent=2))
