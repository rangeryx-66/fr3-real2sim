"""Deterministic TSDF mesh extraction from BundleSDF-tracked RGB-D frames."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import cv2,numpy as np,open3d as o3d


def fuse(dataset:Path,reconstruction:Path,output:Path,voxel=.0015,trunc=.006):
    K=np.loadtxt(dataset/'cam_K.txt');intr=o3d.camera.PinholeCameraIntrinsic(640,480,K[0,0],K[1,1],K[0,2],K[1,2])
    volume=o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel,sdf_trunc=trunc,color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    used=[]
    for pose in sorted((dataset/'poses').glob('*.json')):
        stem=pose.stem;track=reconstruction/'ob_in_cam'/f'{stem}.txt'
        if not track.exists():continue
        rgb=cv2.imread(str(dataset/'rgb'/f'{stem}.png'),cv2.IMREAD_COLOR)[...,[2,1,0]]
        depth=cv2.imread(str(dataset/'depth'/f'{stem}.png'),cv2.IMREAD_UNCHANGED);mask=cv2.imread(str(dataset/'masks'/f'{stem}.png'),0)>0
        depth=depth.copy();depth[~mask]=0
        image=o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(rgb)),o3d.geometry.Image(depth),depth_scale=1000.,depth_trunc=1.,convert_rgb_to_intensity=False)
        volume.integrate(image,intr,np.loadtxt(track));used.append(stem)
    mesh=volume.extract_triangle_mesh();mesh.compute_vertex_normals()
    tri=np.asarray(mesh.triangles);clusters,ntri,areas=mesh.cluster_connected_triangles();clusters=np.asarray(clusters);ntri=np.asarray(ntri)
    if len(ntri):mesh.remove_triangles_by_mask(ntri[clusters]<max(30,int(ntri.max()*.002)))
    mesh.remove_unreferenced_vertices();mesh.remove_degenerate_triangles();mesh.remove_duplicated_triangles();mesh.remove_duplicated_vertices();mesh.compute_vertex_normals()
    output.mkdir(parents=True,exist_ok=True);ply=output/'mesh.ply';obj=output/'mesh.obj';o3d.io.write_triangle_mesh(str(ply),mesh,write_ascii=False);o3d.io.write_triangle_mesh(str(obj),mesh,write_ascii=False)
    report={'schema':'fr3_bundlesdf_tsdf/v1','pose_source':'BundleSDF ob_in_cam','gt_used':False,'voxel_m':voxel,'truncation_m':trunc,'frames_used':len(used),'faces':len(mesh.triangles),'vertices':len(mesh.vertices),'products':{'mesh':str(obj),'ply':str(ply)}}
    (output/'fusion_report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset',type=Path);p.add_argument('reconstruction',type=Path);p.add_argument('output',type=Path);a=p.parse_args();print(json.dumps(fuse(a.dataset,a.reconstruction,a.output),indent=2))
