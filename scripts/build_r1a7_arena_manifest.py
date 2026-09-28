"""Audit and index the 40 existing fresh Isaac Lab Arena FR3 inputs."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
FR3=Path('/data1/home/rangeryx/fr3_moveit_grasp/results/arena_complex40')
OUT=ROOT/'results/r1a7_arena_ab'
protocol=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

rows=[]
for target in protocol['classes']:
    entries=[];scenarios=[]
    class_dir=OUT/'inputs'/target;class_dir.mkdir(parents=True,exist_ok=True)
    for episode in (ep for ep in protocol['episodes'] if ep['target']==target):
        seed=episode['seed'];source=FR3/'inputs'
        cloud=source/f'seed_{seed:04d}_cloud.npz'
        grasps=source/f'seed_{seed:04d}_grasps.json'
        fr3=json.loads((FR3/f'B_seed_{seed:04d}.json').read_text())
        assert fr3['target_class']==target and fr3['grasp_sha256']==sha(grasps)
        with np.load(cloud) as data:
            assert all(key in data for key in ('points','mask','rgb','T_B_C','K'))
            target_points=int(np.count_nonzero(data['mask']))
            camera=data['T_B_C'].tolist()
        original=json.loads(grasps.read_text())
        assert original['frame']=='camera_optical' and np.allclose(original['T_B_C'],camera)
        entry=dict(seed=seed,target=target,fr3_trial=seed,cloud=str(cloud),
                   cloud_sha256=sha(cloud),grasps=str(grasps),grasps_sha256=sha(grasps),
                   grasp_count=len(original['grasps']),target_points=target_points,
                   fr3_category=fr3['category'],fr3_success=fr3['success'],
                   pose=episode['objects'][0],camera_T_B_C=camera)
        rows.append(entry);entries.append(entry);scenarios.append(dict(seed=seed))
        link=class_dir/f'trial_{len(entries):02d}_grasps.json'
        if link.is_symlink() or link.exists():link.unlink()
        link.symlink_to(grasps)
    (class_dir/'reference_manifest.json').write_text(json.dumps(entries,indent=2)+'\n')
    (class_dir/'scenarios.json').write_text(json.dumps(scenarios,indent=2)+'\n')
assert len(rows)==40 and len({r['seed'] for r in rows})==40
report=dict(source=str(FR3),protocol_sha256=sha(ROOT/'ARENA_COMPLEX_PROTOCOL.json'),
            inventory_sha256=sha(ROOT/'assets/arena_complex/inventory.json'),
            scenes=len(rows),objects=len(protocol['classes']),
            unique_cloud_hashes=len({r['cloud_sha256'] for r in rows}),
            unique_grasp_hashes=len({r['grasps_sha256'] for r in rows}),
            rows=rows)
OUT.mkdir(parents=True,exist_ok=True)
(OUT/'input_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
