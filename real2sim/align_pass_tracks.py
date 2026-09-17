"""Align independently tracked passes from their repeated RGB-D reference view."""
from __future__ import annotations
import argparse,json,shutil
from pathlib import Path
import cv2,numpy as np,open3d as o3d


def cloud(dataset,stem='000000'):
    depth=cv2.imread(str(dataset/'depth'/f'{stem}.png'),cv2.IMREAD_UNCHANGED).astype(float)/1000
    mask=cv2.imread(str(dataset/'masks'/f'{stem}.png'),0)>0;K=np.loadtxt(dataset/'cam_K.txt');v,u=np.nonzero(mask&(depth>0));z=depth[v,u]
    xyz=np.c_[(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]
    p=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz));p=p.voxel_down_sample(.0015);p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.006,max_nn=30));return p


def _track_dir(path:Path):
    candidate=path/'ob_in_cam'
    return candidate if candidate.is_dir() else path


def align(combined_dataset:Path,combined_tracking:Path,pass_dataset:Path,pass_tracking:Path,output:Path):
    combined_tracking=_track_dir(combined_tracking);pass_tracking=_track_dir(pass_tracking)
    p0=cloud(combined_dataset,'000000');p1=cloud(pass_dataset,'000000')
    reg=o3d.pipelines.registration.registration_icp(p1,p0,.008,np.eye(4),o3d.pipelines.registration.TransformationEstimationPointToPlane(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    D=np.asarray(reg.transformation);P0=np.loadtxt(combined_tracking/'000000.txt');P1=np.loadtxt(pass_tracking/'000000.txt')
    X=np.linalg.inv(P1)@np.linalg.inv(D)@P0
    rows=[]
    for pose in sorted((pass_dataset/'poses').glob('*.json')):
        row=json.loads(pose.read_text());src=pass_tracking/f'{pose.stem}.txt';dst=combined_tracking/f"{row['combined_prepared_stem']}.txt"
        T=np.loadtxt(src)@X;np.savetxt(dst,T);rows.append({'pass_stem':pose.stem,'combined_stem':row['combined_prepared_stem'],'T_C_common_object':T.tolist()})
    report={'schema':'fr3_rgbd_pass_alignment/v1','gt_used':False,'method':'masked depth point-to-plane ICP at repeated reference view plus independent BundleSDF tracks',
        'icp_fitness':float(reg.fitness),'icp_inlier_rmse_m':float(reg.inlier_rmse),'T_camera_pass1_to_pass0':D.tolist(),'T_pass1_object_to_common_object':X.tolist(),'frames_written':len(rows)}
    output.write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('combined_dataset',type=Path);p.add_argument('combined_tracking',type=Path);p.add_argument('pass_dataset',type=Path);p.add_argument('pass_tracking',type=Path);p.add_argument('output',type=Path);a=p.parse_args();print(json.dumps(align(a.combined_dataset,a.combined_tracking,a.pass_dataset,a.pass_tracking,a.output),indent=2))
