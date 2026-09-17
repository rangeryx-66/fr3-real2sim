"""Offline original cohort surface sweep audit; no simulation or pose changes."""
import json,time,argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from frames import grasp_to_tcp
from mesh_hand_geometry import evaluate
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--output',default='results/arena_repair_diagnostics/mesh');parser.add_argument('--selected-only',action='store_true');args=parser.parse_args()
out=ROOT/args.output;out.mkdir(parents=True,exist_ok=True)
for path in sorted((ROOT/'results/arena_complex40').glob('B_seed_*.json')):
    result=json.loads(path.read_text());seed=result['seed'];dest=out/f'seed_{seed:04d}.json'
    if dest.exists():continue
    data=json.loads((ROOT/f'results/arena_complex40/inputs/seed_{seed:04d}_grasps.json').read_text())
    O=np.eye(4);O[:3,3]=result['initial_target']['position'];O[:3,:3]=Rotation.from_quat(np.roll(result['initial_target']['quaternion_wxyz'],-1)).as_matrix()
    rows=[];t=time.monotonic()
    for g in data['grasps']:
        if args.selected_only and g['rank']!=result['selected_rank']:continue
        _,H,_=grasp_to_tcp(data['T_B_C'],g['rotation'],g['translation'],g['depth'])
        metric=evaluate(result['target_class'],H,O)
        old=next(c for c in result['candidates'] if c['rank']==g['rank'])
        rows.append(dict(rank=g['rank'],score=g['score'],old_status=old['status'],old_coverage=old['hand_geometry']['min_pad_coverage'],selected=g['rank']==result['selected_rank'],mesh=metric))
    dest.write_text(json.dumps(dict(seed=seed,target=result['target_class'],category=result['category'],candidates=rows),indent=2))
    selected=next((r for r in rows if r['selected']),None)
    print(seed,result['target_class'],result['category'], 'selected', selected['mesh']['passed'] if selected else None,'remaining',sum(r['mesh']['passed'] for r in rows),'seconds',time.monotonic()-t,flush=True)
