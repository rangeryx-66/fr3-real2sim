"""Post-estimation GT error report and strict Isaac asset readback checks."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
import trimesh
from pxr import Usd,UsdGeom,UsdPhysics


def chamfer(a,b):
    from scipy.spatial import cKDTree
    return float((cKDTree(a).query(b)[0].mean()+cKDTree(b).query(a)[0].mean())/2)


def fscore(a,b,threshold):
    from scipy.spatial import cKDTree
    precision=float(np.mean(cKDTree(b).query(a)[0] < threshold))
    recall=float(np.mean(cKDTree(a).query(b)[0] < threshold))
    score=2*precision*recall/max(precision+recall,1e-12)
    return {'precision':precision,'recall':recall,'fscore':score}


def validate(asset_dir:Path,gt_mesh:Path,gt_physics:Path,reconstruction_mesh:Path,samples=20000):
    """This is the only module allowed to read GT mass/COM/inertia."""
    asset_dir=Path(asset_dir);meta=json.loads((asset_dir/"metadata.json").read_text());gt=json.loads(Path(gt_physics).read_text())
    stage=Usd.Stage.Open(meta["usd"]);root=stage.GetDefaultPrim();mass=UsdPhysics.MassAPI(root)
    read_mass=float(mass.GetMassAttr().Get());read_com=np.asarray(mass.GetCenterOfMassAttr().Get(),float)
    if not root or not root.HasAPI(UsdPhysics.RigidBodyAPI):raise RuntimeError("USD rigid body API missing")
    collision=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    if not collision:raise RuntimeError("USD collision geometry missing")
    recon=trimesh.load(reconstruction_mesh,force="mesh");truth=trimesh.load(gt_mesh,force="mesh")
    # BundleSDF chooses its own reconstructed object frame.  Frame alignment is
    # evaluation-only and is never fed back to the asset or estimator.
    T_G_R,alignment_cost=trimesh.registration.mesh_other(recon,truth,samples=min(samples,5000),scale=False)
    recon_eval=recon.copy();recon_eval.apply_transform(T_G_R)
    pa=recon_eval.sample(samples);pb=truth.sample(samples)
    R_G_R=np.asarray(T_G_R[:3,:3]);p_G_R=np.asarray(T_G_R[:3,3])
    est_com_G=R_G_R@read_com+p_G_R
    est_I=np.asarray(meta["inertia_kg_m2"]);est_I_G=R_G_R@est_I@R_G_R.T;gt_I=np.asarray(gt["inertia_matrix"])
    report={"usd_load":True,"rigid_body":True,"collision_prim_count":len(collision),
            "mass_readback_kg":read_mass,"com_readback_m":read_com.tolist(),
            "evaluation_alignment_T_GT_reconstruction":T_G_R.tolist(),
            "scan":{"chamfer_m":chamfer(pa,pb),"alignment_cost":float(alignment_cost),
                    "fscore_2mm":fscore(pa,pb,.002),"fscore_5mm":fscore(pa,pb,.005),
                    "reconstructed_faces":int(len(recon.faces)),"gt_faces":int(len(truth.faces))},
            "payload_id":{"mass_relative_error":abs(read_mass-gt["mass"])/gt["mass"],
              "com_error_m":float(np.linalg.norm(est_com_G-np.asarray(gt["center_of_mass"]))),
              "inertia_relative_frobenius_error":float(np.linalg.norm(est_I_G-gt_I)/np.linalg.norm(gt_I))}}
    (asset_dir/"validation.json").write_text(json.dumps(report,indent=2));return report


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("asset_dir",type=Path);p.add_argument("gt_mesh",type=Path);p.add_argument("gt_physics",type=Path);p.add_argument("reconstruction_mesh",type=Path);a=p.parse_args();print(json.dumps(validate(a.asset_dir,a.gt_mesh,a.gt_physics,a.reconstruction_mesh),indent=2))
