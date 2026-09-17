"""Independently audit recorded physics evidence; never trusts success flags alone."""
import argparse
import json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args()
rows=[]
for file in sorted(Path(a.run).glob('trial_[0-9][0-9].json')):
    r=json.loads(file.read_text())
    if not r['success']:
        rows.append(dict(trial=r['trial'],category=r['category']));continue
    hold=r['hold_samples'];close=r['close_samples'];z0=r['initial_z']
    assert hold[-1]['t']-hold[0]['t']>=2
    assert max(h['z'] for h in hold)-z0>=.08
    assert min(h['z'] for h in hold)-z0>=.07
    assert max(h['z'] for h in hold)-min(h['z'] for h in hold)<=.01
    assert min(min(h['forces']) for h in hold)>.1
    assert min(min(h['forces']) for h in close)>.1
    assert [t['stage'] for t in r['executions']]==['PREGRASP','APPROACH','LIFT']
    rows.append(dict(trial=r['trial'],category='SUCCESS',lift_cm=round(100*(max(h['z'] for h in hold)-z0),3),hold_seconds=round(hold[-1]['t']-hold[0]['t'],3),min_finger_force_N=round(min(min(h['forces']) for h in hold),3),drift_mm=round(1000*(max(h['z'] for h in hold)-min(h['z'] for h in hold)),4),selected_rank=r['selected_rank']))
print(json.dumps(rows,indent=2))
