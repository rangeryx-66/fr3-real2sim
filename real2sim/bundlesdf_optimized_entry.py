"""Project-owned entry point using the pinned BundleSDF implementation."""
from __future__ import annotations
import argparse, copy, os, sys, yaml
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle-root',type=Path,required=True);p.add_argument('--video-dir',type=Path,required=True);p.add_argument('--out-folder',type=Path,required=True)
    p.add_argument('--steps',type=int,default=3000);p.add_argument('--mesh-resolution',type=float,default=.002);p.add_argument('--truncation',type=float,default=.01)
    p.add_argument('--global-only',action='store_true',help='reuse a complete tracked output after a texture-stage interruption')
    p.add_argument('--tracking-only',action='store_true')
    a=p.parse_args();a.bundle_root=a.bundle_root.resolve();a.video_dir=a.video_dir.resolve();a.out_folder=a.out_folder.resolve()
    sys.path.insert(0,str(a.bundle_root));os.chdir(a.bundle_root)
    from bundlesdf import set_seed,BundleSdf,YcbineoatReader,cv2,np
    set_seed(0);a.out_folder.mkdir(parents=True,exist_ok=True)
    track=yaml.safe_load(open(a.bundle_root/'BundleTrack/config_ho3d.yml','r'))
    track['SPDLOG']=2;track['depth_processing']['zfar']=1.;track['depth_processing']['percentile']=98
    track['depth_processing']['denoise_cloud']=True;track['erode_mask']=1;track['debug_dir']=str(a.out_folder)+'/'
    track['bundle']['max_BA_frames']=16;track['bundle']['max_optimized_feature_loss']=.03
    track['feature_corres'].update(max_dist_neighbor=.015,max_normal_neighbor=30,max_dist_no_neighbor=.02,max_normal_no_neighbor=35,map_points=True,resize=480,rematch_after_nerf=True)
    track['keyframe']['min_rot']=4;track['ransac'].update(inlier_dist=.008,inlier_normal_angle=20,max_trans_neighbor=.03,max_rot_deg_neighbor=35,max_trans_no_neighbor=.03,max_rot_no_neighbor=100)
    track['p2p'].update(max_dist=.015,max_normal_angle=40)
    track_path=a.out_folder/'config_bundletrack.yml';yaml.safe_dump(track,open(track_path,'w'))
    nerf=yaml.safe_load(open(a.bundle_root/'config.yml','r'));online_dir=str(a.out_folder/'nerf_with_bundletrack_online')
    nerf.update(continual=True,trunc_start=a.truncation,trunc=a.truncation,mesh_resolution=.003,down_scale_ratio=1,fs_sdf=.1,far=1.,datadir=online_dir,save_dir=online_dir,notes='fr3 dual-pass scan v2')
    nerf_path=a.out_folder/'config_nerf.yml';yaml.safe_dump(nerf,open(nerf_path,'w'))
    if not a.global_only:
        tracker=BundleSdf(cfg_track_dir=str(track_path),cfg_nerf_dir=str(nerf_path),start_nerf_keyframes=5,use_gui=False)
        reader=YcbineoatReader(video_dir=str(a.video_dir))
        for i in range(len(reader.color_files)):
            color=cv2.imread(reader.color_files[i])[...,:3];depth=reader.get_depth(i);H,W=depth.shape
            color=cv2.resize(color,(W,H),interpolation=cv2.INTER_AREA);mask=reader.get_mask(i).astype(np.uint8)
            tracker.run(color,depth,reader.K.copy(),reader.id_strs[i],mask=mask,occ_mask=None,pose_in_model=np.eye(4))
        tracker.on_finish()
    if a.tracking_only:return
    set_seed(0);nerf=yaml.safe_load(open(nerf_path,'r'));nerf.update(n_step=a.steps,N_samples=128,N_samples_around_depth=256,first_frame_weight=1,down_scale_ratio=1,finest_res=512,num_levels=16,mesh_resolution=a.mesh_resolution,n_train_image=600,fs_sdf=.1,frame_features=2,rgb_weight=100,i_img=float('inf'))
    for k in ('i_mesh','i_nerf_normals','i_save_ray'):nerf[k]=nerf['i_img']
    nerf['save_dir']=copy.deepcopy(nerf['datadir']);global_dir=Path(nerf['datadir']);global_dir.mkdir(parents=True,exist_ok=True)
    global_cfg=global_dir/'config.yml';yaml.safe_dump(nerf,open(global_cfg,'w'))
    tracker=BundleSdf(cfg_track_dir=str(track_path),cfg_nerf_dir=str(global_cfg),start_nerf_keyframes=5);tracker.cfg_nerf=nerf
    tracker.run_global_nerf(reader=YcbineoatReader(video_dir=str(a.video_dir),downscale=1),get_texture=True,tex_res=2048,use_all_frames=True,interpolate_missing_vertices=False)
    tracker.on_finish()


if __name__=='__main__':main()
