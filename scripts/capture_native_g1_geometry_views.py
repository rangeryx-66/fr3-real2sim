"""Bounded scene-camera RGB-D study. No approach, closure or opening is run.

Scene/robot are initialized with the frozen native-G1 bootstrap. The only
post-initialization commands reposition a simulation camera; no robot motion,
object actuation, grasp generation, GT handle crop or controller is executed.
"""
from pathlib import Path
import argparse,hashlib,json,os,time,sys
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def main(a):
    root=Path(__file__).resolve().parents[1];job=json.loads(a.job.read_text());source=json.loads((Path(job['source'])/'report.json').read_text())
    a.output.mkdir(parents=True,exist_ok=True);os.environ['R1A7_RUN_DIR']=str(a.output)
    from native_g1_metric_rgbd_geometry import preserve,save_view
    observed=preserve(a.seed_folder);save_view(observed,a.output/'saved_single')
    P=observed['points'];anchor=np.median(P,axis=0);normal=np.asarray(observed['plane']['normal']);normal/=np.linalg.norm(normal)
    side=np.cross([0,0,1.],normal);side/=np.linalg.norm(side)
    capture_inputs=dict(source='observed SAM seeds + depth panel only',GT_handle_framing=False,
        target_world_m=anchor.tolist(),normal_world=normal.tolist(),side_world=side.tolist(),seed_folder=str(a.seed_folder),
        new_views_max=3,azimuths_deg=[-55,0,55],standoff_m=.7,height_offset_m=.1,
        sensing_mode='SIMULATION_ONLY_CAMERA_REPOSITION_NOT_ROBOT_WRIST_EXECUTION',manipulation_episodes=0,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),gpu=a.gpu,pid=os.getpid())
    (a.output/'CAPTURE_SPEC.json').write_text(json.dumps(capture_inputs,indent=2))
    from articulated_interaction_skill.scene import install
    from interactive_twin_refinement.assembly import install_setup_capture
    install();install_setup_capture()
    from xarm7_native_execution import bootstrap_job
    args=SimpleNamespace(source=Path(job['source']),asset_root=Path(job['asset_root']),output=a.output,gpu=a.gpu,ownership=None)
    app=None;began=time.perf_counter();records=[]
    try:
        scene=bootstrap_job(args,source['robot_base_pose'],job);app=scene['app'];world=scene['world'];world.pause()
        camera=scene['overview'][0];robot=scene['robot'];names=scene['names'];base=source['robot_base_pose']
        from scipy.spatial.transform import Rotation
        import cv2
        from xarm7_native_planning import NativeModel
        from wrist_reconstruction.self_observation import robot_projection_mask
        model=NativeModel(None,args.asset_root,source);q=np.asarray(robot.get_joint_positions());arm=scene['arm'];fingers=scene['fingers']
        poses=model.poses(q[arm],base,finger_q=q[fingers]);t0=float(world.current_time)
        inherited_clip=list(camera.get_clipping_range())
        # The overview camera inherits a 1m near plane, which clips a 0.7m
        # analysis view. Match the frozen loader's primary-camera clip range;
        # this changes this new observation camera only, never physics.
        camera.set_clipping_range(.05,3.)
        camera.add_distance_to_image_plane_to_frame()
        import omni.replicator.core as rep
        for i,angle in enumerate([-55,0,55]):
            out=a.output/('view_%02d'%i);out.mkdir(exist_ok=True)
            eye=anchor+.7*(np.cos(np.deg2rad(angle))*normal+np.sin(np.deg2rad(angle))*side)+[0,0,.1]
            forward=(anchor-eye)/np.linalg.norm(anchor-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right)
            R=np.column_stack((right,np.cross(forward,right),forward));camera.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros')
            # Explicit annotator capture without advancing the paused physics
            # timeline. world.render alone can leave a newly attached depth
            # annotator empty while paused.
            rep.orchestrator.step(delta_time=0.0,pause_timeline=True,rt_subframes=8)
            rgb=np.asarray(camera.get_rgba())[...,:3];depth=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data())
            if depth.ndim!=2 or depth.shape!=rgb.shape[:2]:
                raise ValueError('RGBD_CAPTURE_INVALID: depth/RGB shape mismatch')
            position,orientation=camera.get_world_pose(camera_axes='ros');C=np.eye(4);C[:3,:3]=Rotation.from_quat(np.roll(orientation,-1)).as_matrix();C[:3,3]=position
            K=np.asarray(camera.get_intrinsics_matrix());cv2.imwrite(str(out/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.save(out/'depth_m.npy',depth)
            meta=dict(K=K.tolist(),T_world_camera_optical=C.tolist(),depth_units='m optical Z',camera_source=capture_inputs['sensing_mode'],robot_q=q.tolist(),robot_joint_names=names,base=base,t_sim=float(world.current_time),azimuth_deg=angle,clipping_range_m=list(camera.get_clipping_range()),inherited_overview_clipping_range_m=inherited_clip)
            (out/'camera.json').write_text(json.dumps(meta,indent=2));np.save(out/'robot_self_exclusion.npy',robot_projection_mask(depth,K,C,model.robot_hulls,poses))
            records.append(dict(view=out.name,azimuth_deg=angle,depth_shape=list(depth.shape),valid_depth_pixels=int((np.isfinite(depth)&(depth>.02)&(depth<5)).sum()),physics_time_before=t0,physics_time_after=float(world.current_time)))
            print('VIEW_SAVED',out,flush=True)
        (a.output/'CAPTURE_COMPLETE.json').write_text(json.dumps(dict(views=records,wall_s=time.perf_counter()-began,world_paused=True,post_bootstrap_physics_steps=0,robot_motion_commands=0,gripper_commands=0,object_state_commands=0,manipulation_episodes=0),indent=2))
    finally:
        # SimulationApp.close may terminate the interpreter. Persist the
        # request before that call; actual exit is checked by the launcher.
        (a.output/'ACTOR_CLOSE_REQUESTED.json').write_text(json.dumps(dict(pid=os.getpid(),close_requested=True,wall_s=time.perf_counter()-began)))
        if app is not None:app.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--job',type=Path,required=True);p.add_argument('--seed-folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--gpu',type=int,default=0);main(p.parse_args())
