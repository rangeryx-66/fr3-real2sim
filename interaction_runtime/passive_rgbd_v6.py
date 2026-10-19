"""Write-only passive geometry dataset. Never installed in the Agent context.

Known state is used solely by the sparse capture scheduler. Privileged state
is written to a separate evaluation directory, never the geometry package.
"""
import json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from agent_command_protocol_v12 import atomic_json

CONFIG=dict(resolution_wh=[640,480],focal_px=420.,clipping_m=[.1,5.],spacing_deg=7.5,maximum_pairs=28,
            viewpoints=[[1.65,-1.10,.45],[1.35,1.25,.65]],render_updates=8)

class PassiveRGBDPair:
    def __init__(self,r):
        from isaacsim.sensors.camera import Camera
        from native_g1_wrist_views import look_at
        self.r=r;self.root=Path(r.job['passive_dataset_output']);self.eval=Path(r.job['passive_evaluation_output']);self.root.mkdir(parents=True,exist_ok=False);self.eval.mkdir(parents=True,exist_ok=True)
        self.cameras=[];self.transforms=[];self.index=[];self.evaluation=[];self.wall_s=0.;self.last_q=r.actual();self.previous_q=self.last_q;self.direction=0;self.last_phase=None;self.pending_direction=None
        m=r.model;b=np.asarray(m.manifest['static_source_bounds'])*m.manifest['scale_source_to_meters'];lo,hi=b;corners=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]]);P=corners@m.asset_T[:3,:3].T+m.asset_T[:3,3];center=(P.min(0)+P.max(0))/2
        normal=m.asset_T[:3,2].copy();normal[2]=0;normal/=np.linalg.norm(normal);side=np.cross([0,0,1],normal)
        calibration=[]
        for name,(front,lateral,height) in zip(['A','B'],CONFIG['viewpoints']):
            eye=center+front*normal+lateral*side+np.array([0,0,height]);C=look_at(eye,center)
            cam=Camera('/World/passive_fixed_rgbd_'+name,name='passive_fixed_rgbd_'+name,resolution=tuple(CONFIG['resolution_wh']),frequency=None)
            cam.set_world_pose(position=C[:3,3],orientation=np.roll(Rotation.from_matrix(C[:3,:3]).as_quat(),1),camera_axes='ros');cam.initialize();w,h=CONFIG['resolution_wh'];cam.set_horizontal_aperture(w*.01);cam.set_vertical_aperture(h*.01);cam.set_focal_length(CONFIG['focal_px']*.01);cam.set_clipping_range(*CONFIG['clipping_m']);cam.add_distance_to_image_plane_to_frame();self.cameras.append(cam);self.transforms.append(C)
            calibration.append(dict(camera_id=name,prim_path=cam.prim_path,K=np.asarray(cam.get_intrinsics_matrix()).tolist(),T_world_camera_optical=C.tolist(),resolution_wh=CONFIG['resolution_wh'],depth='float32 metres optical Z; zero/nonfinite means invalid',mount='simulated stationary calibrated camera; no hardware claim',pose_set_calls=1))
        atomic_json(self.root/'calibration.json',dict(cameras=calibration,clock='shared Isaac simulation clock; synchronized render product frame/time verified per pair',scene_camera_not_wrist=True))
        atomic_json(self.root/'README.json',dict(purpose='future independent offline rigid-motion/hinge reconstruction; no reconstruction performed',inputs=['RGB','optical-Z depth','calibration','timestamp','sequence','direction','phase','validity'],excluded=['GT joint angle','GT hinge axis','GT part transforms','hidden dynamics','segmentation','object meshes'],online_consumers=[],configuration=CONFIG))
        self.capture('initial')
    def capture(self,reason):
        import cv2
        if len(self.index)>=CONFIG['maximum_pairs']:return
        began=time.perf_counter();r=self.r
        for _ in range(CONFIG['render_updates']):r.world.render()
        seq=len(self.index);folder=self.root/('%03d'%seq);folder.mkdir();depths={};cams=[];valid=True
        for name,cam,C in zip(['A','B'],self.cameras,self.transforms):
            frame=cam.get_current_frame();reference=cam._fabric_time_annotator.get_data();rgb=np.asarray(cam.get_rgba())[...,:3].copy();depth=np.asarray(cam._custom_annotators['distance_to_image_plane'].get_data(),dtype=np.float32).copy();actual=r.matrix(*cam.get_world_pose(camera_axes='ros'));err=float(np.max(abs(actual-C)));finite=np.isfinite(depth)&(depth>0)&(depth<CONFIG['clipping_m'][1]);good=rgb.shape==(480,640,3) and depth.shape==(480,640) and finite.mean()>.01 and err<1e-5
            if not good:valid=False
            cv2.imwrite(str(folder/(name+'_rgb.png')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_PNG_COMPRESSION,6]);depths[name+'_depth_m']=depth
            cams.append(dict(camera_id=name,reference_time={k:int(v) for k,v in reference.items()} if isinstance(reference,dict) else {},callback_rendering_time=float(frame.get('rendering_time',-1)),valid_depth_fraction=float(finite.mean()),pose_max_abs_error=err,valid=bool(good)))
        ref=cams[0]['reference_time'];sync=bool(ref and ref==cams[1]['reference_time'] and ref.get('referenceTimeDenominator',0)>0)
        reference_s=ref.get('referenceTimeNumerator',0)/max(ref.get('referenceTimeDenominator',1),1)
        sync=bool(sync and abs(reference_s-float(r.world.current_time))<2e-6 and (not self.index or reference_s>self.index[-1]['render_reference_timestamp_s']))
        np.savez_compressed(folder/'depth_pair.npz',**depths)
        item=dict(sequence=seq,path=folder.name,render_reference_timestamp_s=reference_s,simulation_timestamp_s=float(r.world.current_time),physics_tick=r.tick,direction={-1:'closing',0:'stationary',1:'opening'}[self.direction],phase=r.phase,reason=reason,cameras=cams,synchronized=sync,valid=bool(valid and sync),missing_frames=[] if valid else [c['camera_id'] for c in cams if not c['valid']])
        self.index.append(item);self.evaluation.append(dict(sequence=seq,simulation_timestamp_s=float(r.world.current_time),joint_angle_rad=float(r.actual()),T_world_moving=r.moving().tolist()))
        self.last_q=r.actual();self.last_phase=r.phase;self.wall_s+=time.perf_counter()-began;self.flush()
    def after_step(self):
        q=self.r.actual();delta=q-self.previous_q;self.previous_q=q
        # Small dynamic jitter must not create continuous captures or false reversals.
        direction=int(np.sign(delta)) if abs(delta)>1e-6 else self.direction
        reversal=direction and self.direction and direction!=self.direction
        if reversal:self.pending_direction=direction
        if abs(q-self.last_q)>=np.deg2rad(CONFIG['spacing_deg']):self.direction=direction;self.capture('articulation_checkpoint');self.pending_direction=None
        elif self.pending_direction and abs(q-self.last_q)>=np.deg2rad(1.):self.direction=direction;self.capture('reverse_motion_checkpoint');self.pending_direction=None
        elif self.r.tick%120==0 and self.r.phase in ['PARK','SAFE_PARK_MOTION','RELEASE'] and abs(q-self.last_q)>=np.deg2rad(2.):self.direction=direction;self.capture('mode_boundary')
        if abs(delta)>1e-6:self.direction=direction
    def flush(self):
        atomic_json(self.root/'index.json',dict(pairs=self.index,total_pairs=len(self.index),valid_synchronized_pairs=sum(p['valid'] for p in self.index),capture_wall_s=self.wall_s,online_consumers=[]))
        atomic_json(self.eval/'privileged_states.json',dict(evaluation_only=True,not_geometry_package=True,states=self.evaluation))
    def close(self):
        if abs(self.r.actual()-self.last_q)>np.deg2rad(.5):self.capture('final_state')
        self.flush()
