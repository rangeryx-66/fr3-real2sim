"""Audit target movement/contact BEFORE closure, without changing execution policy."""
import json,gzip
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';OUT=ROOT/'results/arena_complex_summary';rows=[]
for path in sorted(RUN.glob('B_seed*.json')):
    r=json.loads(path.read_text());trace=RUN/f'trace_seed_{r["seed"]:04d}.json.gz'
    if not trace.exists():continue
    records=json.load(gzip.open(trace,'rt'))['records'];z0=np.array(r['initial_target']['position']);R0=Rotation.from_quat(np.roll(r['initial_target']['quaternion_wxyz'],-1))
    first=next((v for v in records if max(v['forces'])>.1),None)
    row=dict(seed=r['seed'],target=r['target_class'],category=r['category'],selected_rank=r['selected_rank'],first_contact_phase=first['phase'] if first else None,first_contact_time=first['t'] if first else None,first_contact_forces=first['forces'] if first else None,phases={})
    for phase in ['PREGRASP','APPROACH','CLOSE','MICRO_LIFT','LIFT','HOLD']:
        vs=[v for v in records if v['phase']==phase]
        if not vs:continue
        contacts=[v for v in vs if max(v['forces'])>.1]
        row['phases'][phase]=dict(peak_finger_target_force_N=max(max(v['forces']) for v in vs),contact_steps=len(contacts),max_target_displacement_from_initial_m=max(float(np.linalg.norm(np.array(v['box'])-z0)) for v in vs),max_target_rotation_from_initial_deg=max(float(np.rad2deg((R0.inv()*Rotation.from_quat(np.roll(v['box_quat'],-1))).magnitude())) for v in vs),target_at_start=vs[0]['box'],target_at_end=vs[-1]['box'])
    rows.append(row)
OUT.mkdir(exist_ok=True,parents=True);(OUT/'early_contact_audit.json').write_text(json.dumps(rows,indent=2));print(json.dumps([(r['seed'],r['category'],r['first_contact_phase']) for r in rows]))
