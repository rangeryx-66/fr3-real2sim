"""Read-only measured contact-to-render-mesh residuals in actual target frame."""
import json,gzip
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';OUT=ROOT/'results/arena_complex_summary';rows=[];meshes={}
for path in sorted(RUN.glob('B_seed*.json')):
    r=json.loads(path.read_text());name=r['target_class']
    if r['selected_rank'] is None:continue
    if name not in meshes:
        m=np.load(ROOT/f'assets/arena_complex/{name}_mesh.npz');meshes[name]=trimesh.Trimesh(vertices=m['vertices'],faces=m['triangles'],process=False)
    records=json.load(gzip.open(RUN/f'trace_seed_{r["seed"]:04d}.json.gz','rt'))['records'];points=[];phases=[]
    for v in records[::8]:
        if v['phase'] not in ['PREGRASP','APPROACH','CLOSE','MICRO_LIFT','LIFT','HOLD']:continue
        R=Rotation.from_quat(np.roll(v['box_quat'],-1)).as_matrix()
        for c in v['finger_contacts']:
            p=np.array(c['points_world_m']);f=np.array(c['normal_force_N'])
            if not len(p):continue
            p=p[f>.1]
            points.extend(((p-np.array(v['box']))@R).tolist());phases.extend([v['phase']]*len(p))
    if not points:continue
    ids=np.linspace(0,len(points)-1,min(500,len(points))).astype(int);p=np.array(points)[ids]
    nearest,distance,triangle=trimesh.proximity.closest_point(meshes[name],p)
    rows.append(dict(seed=r['seed'],target=name,category=r['category'],sampled_contact_count=len(ids),distance_to_GT_mesh_m_percentiles=np.percentile(distance,[0,50,90,95,100]).tolist(),fraction_over_2mm=float(np.mean(distance>.002)),fraction_over_5mm=float(np.mean(distance>.005)),note='PhysX contact point versus render mesh; finite separation/penetration and convex decomposition both contribute, not a new pass/fail criterion'))
OUT.mkdir(exist_ok=True,parents=True);(OUT/'contact_mesh_residuals.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
