"""Post-reconstruction scan metrics. GT is read only by this evaluation entry."""
from __future__ import annotations
import argparse,json,time
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def _align_mesh(mesh, gt):
    """Align only for the final GT evaluation, with a dependency-free fallback.

    Trimesh's ICP normally uses rtree for exact triangle proximity.  The
    reconstruction path itself never calls this function; if rtree is absent
    in the evaluation environment, Open3D point-to-point ICP provides the same
    post-hoc alignment without changing any fused geometry.
    """
    try:
        return trimesh.registration.mesh_other(mesh, gt, samples=5000, scale=False)
    except ModuleNotFoundError as exc:
        if "rtree" not in str(exc):
            raise
        import open3d as o3d
        state = np.random.get_state(); np.random.seed(20260913)
        try:
            src = mesh.sample(5000); dst = gt.sample(5000)
        finally:
            np.random.set_state(state)
        source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(src))
        target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(dst))
        result = o3d.pipelines.registration.registration_icp(
            source, target, 0.10, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return np.asarray(result.transformation), float(result.inlier_rmse)


def fscore(a,b,t):
    da=cKDTree(b).query(a)[0];db=cKDTree(a).query(b)[0];p=float(np.mean(da<t));r=float(np.mean(db<t))
    return {'precision':p,'recall':r,'fscore':2*p*r/max(p+r,1e-12)}


def local_noise(points,k=20):
    tree=cKDTree(points);_,nn=tree.query(points,k=min(k,len(points)))
    residual=[]
    for group in points[nn]:
        x=group-group.mean(0);_,s,_=np.linalg.svd(x,full_matrices=False);residual.append(s[-1]/np.sqrt(len(group)))
    return np.asarray(residual)


def boundary(mesh):
    edges=np.sort(mesh.edges,axis=1);unique,count=np.unique(edges,axis=0,return_counts=True);b=unique[count==1]
    return len(b),float(np.linalg.norm(mesh.vertices[b[:,0]]-mesh.vertices[b[:,1]],axis=1).sum()) if len(b) else 0.


def tracking_eval(dataset,reconstruction):
    rows=[]
    # Fixed object-frame ambiguity is removed with the first valid frame. GT poses
    # are used here only for post-hoc error, never for reconstruction/alignment.
    for pose_json in sorted((dataset/'poses').glob('*.json')):
        row=json.loads(pose_json.read_text());raw_stem=Path(row['object_mask']).stem;gt=Path(row['object_mask']).parents[1]/'eval_gt'/f'{raw_stem}_T_B_object.txt'
        pred=reconstruction/'ob_in_cam'/f'{pose_json.stem}.txt'
        if gt.exists() and pred.exists():rows.append((np.asarray(row['T_B_camera']),np.loadtxt(gt),np.loadtxt(pred)))
    if not rows:return {'available':False}
    relative=[]
    for C,G,P in rows:relative.append(np.linalg.inv(P)@np.linalg.inv(C)@G)
    X=relative[0];te=[];re=[]
    for C,G,P in rows:
        E=np.linalg.inv(G)@C@P@X;te.append(np.linalg.norm(E[:3,3]));re.append(np.degrees(Rotation.from_matrix(E[:3,:3]).magnitude()))
    return {'available':True,'frames':len(rows),'translation_mean_mm':float(np.mean(te)*1000),'translation_max_mm':float(np.max(te)*1000),'rotation_mean_deg':float(np.mean(re)),'rotation_max_deg':float(np.max(re))}


def evaluate(mesh_path:Path,gt_path:Path,dataset:Path,reconstruction:Path,texture_report=None,samples=20000):
    mesh=trimesh.load(mesh_path,force='mesh',process=False);gt=trimesh.load(gt_path,force='mesh',process=False)
    state=np.random.get_state();np.random.seed(20260913)
    try:
        T,cost=_align_mesh(mesh,gt);aligned=mesh.copy();aligned.apply_transform(T)
        a=aligned.sample(samples);b=gt.sample(samples)
        pts=mesh.sample(min(5000,max(1000,len(mesh.vertices))))
    finally:np.random.set_state(state)
    da=cKDTree(b).query(a)[0];db=cKDTree(a).query(b)[0]
    topology=mesh.copy();topology.merge_vertices(digits_vertex=6);topology.remove_unreferenced_vertices();bn,bl=boundary(topology)
    noise=local_noise(pts)
    report={'mesh':str(mesh_path),'geometry':{'chamfer_mm':float((da.mean()+db.mean())*500),'fscore_2mm':fscore(a,b,.002),'fscore_5mm':fscore(a,b,.005),
        'faces':int(len(mesh.faces)),'vertices':int(len(mesh.vertices)),'welded_vertices':int(len(topology.vertices)),'watertight':bool(topology.is_watertight),'euler_number':int(topology.euler_number),
        'components':int(len(topology.split(only_watertight=False))),'boundary_edges':int(bn),'boundary_length_m':bl,
        'local_plane_noise_median_mm':float(np.median(noise)*1000),'local_plane_noise_p95_mm':float(np.percentile(noise,95)*1000),'evaluation_alignment_cost':float(cost)},
        'tracking':tracking_eval(dataset,reconstruction)}
    if texture_report:report['texture']=json.loads(Path(texture_report).read_text())
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mesh',type=Path);p.add_argument('gt_mesh',type=Path);p.add_argument('dataset',type=Path);p.add_argument('reconstruction',type=Path);p.add_argument('--texture-report',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    r=evaluate(a.mesh,a.gt_mesh,a.dataset,a.reconstruction,a.texture_report)
    if a.output:a.output.write_text(json.dumps(r,indent=2))
    print(json.dumps(r,indent=2))
