"""Oracle view acquisition at an ACTUALLY reached state, evaluation-only."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from wrist_reconstruction.geometry import proposed_views
from wrist_reconstruction.capture import sam_mask
from articulated_interaction_skill.capture import backproject


def scan(recorder,label,value):
    from isaacsim.sensors.camera import Camera
    import omni.replicator.core as rep
    root=recorder.output/'oracle_capture';folder=root/'states'/f'state_{len(recorder.states):03d}';folder.mkdir(parents=True,exist_ok=True)
    p=recorder.config['capture'];world=recorder.scene['world'];camera=Camera('/World/oracle_capture_rgbd',name='oracle_capture_rgbd',resolution=tuple(recorder.cal['resolution_wh']),frequency=30);camera.initialize();camera.set_horizontal_aperture(recorder.camera.get_horizontal_aperture());camera.set_focal_length(recorder.camera.get_focal_length());camera.set_clipping_range(.05,5.);camera.add_distance_to_image_plane_to_frame();camera.add_instance_id_segmentation_to_frame()
    views=[];world.pause()
    try:
        for proposal in proposed_views(recorder.center,np.asarray(recorder.runtime.initial_visual['outward_normal_world']),recorder.extent,p):
            T=proposal['T_camera'];camera.set_world_pose(T[:3,3],np.roll(Rotation.from_matrix(T[:3,:3]).as_quat(),1),camera_axes='ros')
            before=float(world.current_time)
            # A newly attached camera can return no data under plain paused
            # world.render(). Schedule annotators explicitly with zero time.
            rep.orchestrator.step(rt_subframes=8,delta_time=0.0,pause_timeline=True,wait_for_render=True)
            if abs(float(world.current_time)-before)>1e-9:raise RuntimeError('ORACLE_RENDER_ADVANCED_PHYSICS')
            dest=folder/f"view_{proposal['view_id']:02d}";rgb,depth,meta=recorder.save_frame(dest,camera)
            if meta['qa']['robot_pixel_ratio']>p['maximum_robot_pixel_ratio'] or meta['qa']['object_frame_clipped']:continue
            mask=sam_mask(dest,recorder.config['sam3'],recorder.job['object_prompt']);P,keep=backproject(depth,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),mask)
            np.savez_compressed(dest/'point_cloud.npz',points_world_m=P.astype(np.float32),rgb=rgb[keep],units='m');views.append({'view_id':proposal['view_id'],'directory':str(dest.relative_to(root)),'K':meta['K'],'T_world_camera_optical':meta['T_world_camera_optical'],'qa':meta['qa']})
    finally:world.play()
    state={'state_id':len(recorder.states),'label':label,'estimated_articulation_state':value,'views':views,'oracle_camera_trajectory':True,'robot_executable':False,'render_delta_time_s':0.0,'physics_time_advanced':False,'implementation_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    manifest=root/'multistate_capture.json';doc=json.loads(manifest.read_text()) if manifest.exists() else {'object_id':recorder.metadata['asset_id'],'units':'degrees' if recorder.job['skill']['joint_type']=='revolute' else 'meters','states':[],'status':'RUNNING','capture_mode':'ORACLE_CAPTURE_DIAGNOSTIC','counts_as_system_success':False,'object_joint_commands':False,'background':'same scene as wrist capture'}
    doc['states'].append(state);manifest.write_text(json.dumps(doc,indent=2));return state
