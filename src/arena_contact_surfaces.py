"""Offline actual-contact pad-location diagnostics; never influences selection."""
import json,gzip
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';OUT=ROOT/'results/arena_complex_summary'
def tf(p,q):
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();T[:3,3]=p;return T
rows=[]
for path in sorted(RUN.glob('B_seed*.json')):
    r=json.loads(path.read_text());seed=r['seed']
    if r['selected_rank'] is None:continue
    records=json.load(gzip.open(RUN/f'trace_seed_{seed:04d}.json.gz','rt'))['records']
    row=dict(seed=seed,target=r['target_class'],category=r['category'],coverage=r['telemetry']['commanded_geometry']['min_pad_coverage'],stages={},normal_opposition={})
    for phase in ['CLOSE','MICRO_LIFT','LIFT','HOLD']:
        seq=[v for v in records if v['phase']==phase]
        if not seq:continue
        finger_stats=[]
        for finger in range(2):
            npoints=inside=normal_aligned=0;weight=inside_weight=0.;examples=[];normal_sum=np.zeros(3)
            for v in seq:
                H=tf(v['tcp'],v['tcp_quat']);O=tf(v['box'],v['box_quat']);c=v['finger_contacts'][finger]
                p=np.array(c['points_world_m']);normal=np.array(c['normals_world']);force=np.array(c['normal_force_N'])
                if not len(p):continue
                # Ignore zero-force proximity contacts for this descriptive surface
                # diagnostic only. Original success predicates and raw logs remain intact.
                active_points=force>1e-6;p=p[active_points];normal=normal[active_points];force=force[active_points]
                if not len(p):continue
                local=(p-H[:3,3])@H[:3,:3];norm=normal@H[:3,:3]
                in_pad=(abs(local[:,0])<=.00875)&(local[:,2]>=-.009)&(local[:,2]<=.0095)
                npoints+=len(p);inside+=int(in_pad.sum());normal_aligned+=int((abs(norm[:,1])>.7).sum());weight+=force.sum();inside_weight+=force[in_pad].sum();normal_sum+=(norm*force[:,None]).sum(0)
                if len(examples)<12:
                    examples.append(dict(t=v['t'],TCP_points=local.tolist(),target_points=((p-O[:3,3])@O[:3,:3]).tolist(),TCP_normals=norm.tolist(),force_N=force.tolist()))
            finger_stats.append(dict(finger=finger,points=npoints,pad_rectangle_point_fraction=inside/npoints if npoints else None,closing_normal_fraction=normal_aligned/npoints if npoints else None,pad_rectangle_force_fraction=float(inside_weight/weight) if weight else None,force_weighted_TCP_normal=(normal_sum/weight).tolist() if weight else None,examples=examples))
        row['stages'][phase]=finger_stats
        normals=[f['force_weighted_TCP_normal'] for f in finger_stats]
        if all(n is not None and np.linalg.norm(n)>1e-6 for n in normals):
            row['normal_opposition'][phase]=float(np.dot(normals[0],normals[1])/(np.linalg.norm(normals[0])*np.linalg.norm(normals[1])))
    rows.append(row)
OUT.mkdir(exist_ok=True,parents=True);(OUT/'actual_contact_surfaces.json').write_text(json.dumps(rows,indent=2))
print(json.dumps([{k:r[k] for k in ['seed','target','category','coverage']} for r in rows],indent=2))
