"""Full-background RGB-D, robot-executed views; instance IDs are QA only."""
import copy,json,time,subprocess,os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from articulated_interaction_skill.capture import CaptureRecorder,backproject,matrix
from wrist_reconstruction.geometry import calibration,proposed_views,optical_to_tcp,visibility,coverage_views,distinct_view,observed_volume


def snapshot(camera):
    # Deep-copy all modalities from the same completed render. The old writer
    # mixed get_current_frame buffers with separately queried RGB/annotators.
    rgb=np.asarray(camera.get_rgba()).copy()[...,:3]
    depth=np.asarray(camera._custom_annotators['distance_to_image_plane'].get_data()).copy()
    seg=copy.deepcopy(camera._custom_annotators['instance_id_segmentation'].get_data())
    if rgb.shape[:2]!=depth.shape or seg['data'].shape!=depth.shape:raise RuntimeError('SYNCHRONIZED_FRAME_UNAVAILABLE')
    return rgb,depth,seg


def sam_mask(folder,policy,prompt):
    import cv2
    output=folder/'sensor_mask.npy'
    env=dict(os.environ,PYTHONPATH=policy['pythonpath'],PYTHONNOUSERSITE='1',CUDA_VISIBLE_DEVICES='2')
    cmd=[policy['python'],'scripts/infer_sam3_part.py','--rgb',str(folder/'rgb.png'),'--prompt',prompt,'--checkpoint',policy['checkpoint'],'--output',str(output)]
    with (folder/'sam3.log').open('w') as stream:subprocess.run(cmd,env=env,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=120)
    mask=np.load(output).astype(bool);cv2.imwrite(str(folder/'mask.png'),mask.astype(np.uint8)*255)
    return mask


class WristRecorder(CaptureRecorder):
    def __init__(self,scene,output,job):
        super().__init__(scene,output,job)
        from isaacsim.sensors.camera import Camera
        self.config=job['wrist_experiment'];self.cal=calibration(job['camera_calibration']);self.runtime=None;self.scan_logs=[];self.observed_cloud=None;self.last_view=None
        w,h=self.cal['resolution_wh'];K=self.cal['K']
        self.camera=Camera('/World/piper_wrist_rgbd',name='piper_wrist_rgbd',resolution=(w,h),frequency=30)
        self.camera.initialize();self.camera.set_horizontal_aperture(w*.01);self.camera.set_vertical_aperture(h*.01*K[0,0]/K[1,1]);self.camera.set_focal_length(K[0,0]*.01)
        self.camera.set_clipping_range(.05,5.);self.camera.add_distance_to_image_plane_to_frame();self.camera.add_instance_id_segmentation_to_frame()
        self.camera.set_world_pose(*scene['overview'][0].get_world_pose(camera_axes='ros'),camera_axes='ros')
        self.legacy_cameras=self.cameras;self.cameras=[self.camera]

    def sync(self,E):
        T=np.asarray(E)@self.cal['X'];self.camera.set_world_pose(T[:3,3],np.roll(Rotation.from_matrix(T[:3,:3]).as_quat(),1),camera_axes='ros')

    def save_frame(self,folder,camera,qa=True):
        import cv2
        folder.mkdir(parents=True,exist_ok=True);rgb,depth,seg=snapshot(camera)
        cv2.imwrite(str(folder/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));np.save(folder/'depth_m.npy',depth)
        K=np.asarray(camera.get_intrinsics_matrix());T=matrix(*camera.get_world_pose(camera_axes='ros'))
        meta={'T_world_ee':None if self.runtime is None else self.runtime.tcp().tolist(),'robot_q':None if self.runtime is None else self.runtime.arm_q().tolist(),'robot_base_pose':None if self.runtime is None else list(self.runtime.base),'K':K.tolist(),'T_world_camera_optical':T.tolist(),'camera_axes':'ROS optical','depth':'optical Z in meters','resolution_wh':list(rgb.shape[1::-1]),'mask_source':'SAM3 on full-background sensor RGB; instance segmentation QA only','camera_calibration_real':self.cal['real_hardware_calibrated'],'sim_nominal_extrinsics':not self.cal['real_hardware_calibrated']}
        if qa:
            ids=seg['data'];labels=seg['info']['idToLabels'];obj=[int(k) for k,v in labels.items() if self.asset_path in str(v)];robot=[int(k) for k,v in labels.items() if '/World/Piper/' in str(v)]
            om=np.isin(ids,obj);rm=np.isin(ids,robot);ys,xs=np.where(om)
            clipped=not len(xs) or xs.min()<4 or ys.min()<4 or xs.max()>=rgb.shape[1]-4 or ys.max()>=rgb.shape[0]-4
            meta['qa']={'robot_pixel_ratio':float(rm.mean()),'object_coverage':float(om.mean()),'object_frame_clipped':bool(clipped),'visible_moving_part_pixels':int(np.isin(ids,[int(k) for k,v in labels.items() if self.part_path in str(v)]).sum())}
            # No simulator mask file is supplied to reconstruction or registration.
            cv2.imwrite(str(folder/'qa_instance_overlay.png'),cv2.cvtColor(np.where(rm[...,None],np.array([255,0,0],np.uint8),rgb),cv2.COLOR_RGB2BGR))
        (folder/'camera.json').write_text(json.dumps(meta,indent=2));return rgb,depth,meta

    def initialize_observation(self):
        # Existing initial localization camera is used only to initialize a
        # sensor-derived object volume. Reconstruction views must all be wrist.
        world=self.scene['world']
        for _ in range(6):world.render()
        folder=self.output/'initial_sensor_observation';rgb,depth,meta=self.save_frame(folder,self.legacy_cameras[0])
        mask=sam_mask(folder,self.config['sam3'],self.job['object_prompt'])
        P,_=backproject(depth,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),mask,stride=4)
        if len(P)<200:raise RuntimeError('INITIAL_RGBD_OBJECT_UNAVAILABLE')
        self.initial_framing_cloud=P.copy();self.observed_cloud=P;self.center,self.extent,_=observed_volume(P)
        (folder/'observed_geometry.json').write_text(json.dumps({'center_m':self.center.tolist(),'extent_m':self.extent.tolist(),'center_method':'sensor bounds midpoint, not visible-surface median','source':'SAM3 + RGB-D, not GT mesh'},indent=2))

    def scan(self,label,value,articulation=None):
        r=self.runtime;p=self.config['capture'];start=time.monotonic()
        if self.observed_cloud is None:self.initialize_observation()
        # Prior geometry is a conservative CAMERA framing envelope only.
        # Current-state sensor samples update it; backend point clouds remain
        # separate actual observations and never contain the old-state prior.
        framing_clouds=[self.initial_framing_cloud]
        self.observed_cloud=self.initial_framing_cloud.copy()
        state_dir=self.output/'states'/f'state_{len(self.states):03d}';state_dir.mkdir(parents=True,exist_ok=True)
        from wrist_reconstruction.checkpoint import closed_views
        normal=np.asarray(r.initial_visual['outward_normal_world']);views=closed_views(self,state_dir,value);log={'label':label,'state_estimate':value,'proposals':[],'released':True,'camera_motion':'physical robot joints; no camera teleport view acquisition','reused_actual_views':len(views)}
        if views:(state_dir/'views_checkpoint.json').write_text(json.dumps({'label':label,'estimated_state':value,'views':views},indent=2))
        # Reobserve current shape from sensor data after release, not a door GT.
        proposals=coverage_views(self.observed_cloud,self.center,normal,self.cal,p)
        proposals=proposals[:p['maximum_pose_proposals']]
        for proposal_index,proposal in enumerate(proposals):
            if len(views)>=p['maximum_clean_views']:break
            if time.monotonic()-start>p['maximum_scan_wall_s']:break
            row={'view_id':proposal['view_id'],'requested_T_camera':proposal['T_camera'].tolist()};log['proposals'].append(row)
            if any(v['view_id']==proposal['view_id'] for v in views):
                row.update(status='REUSED_ACTUAL_VIEW');continue
            if not distinct_view(proposal['T_camera'],[np.asarray(v['T_world_camera_optical']) for v in views],p['minimum_camera_baseline_m'],p['minimum_camera_angle_deg']):
                row.update(status='SKIPPED_REDUNDANT_VIEW');continue
            try:
                if proposal.get('observed_volume_in_frame',1.)<1.:raise RuntimeError('OBJECT_WOULD_BE_CROPPED')
                if visibility(self.observed_cloud,proposal['T_camera'],self.cal['K'],self.cal['resolution_wh'])<p['minimum_initial_cloud_in_frame']:raise RuntimeError('OBJECT_WOULD_BE_CROPPED')
                r.scan_to(optical_to_tcp(proposal['T_camera'],self.cal['X']));r.hold(p['settle_s'])
                for _ in range(8):r.step()
                folder=state_dir/f"view_{proposal['view_id']:02d}";rgb,depth,meta=self.save_frame(folder,self.camera)
                qa=meta['qa']
                if qa['robot_pixel_ratio']>p['maximum_robot_pixel_ratio']:raise RuntimeError('QA_ROBOT_OCCLUSION')
                if qa['object_frame_clipped']:raise RuntimeError('QA_OBJECT_CROPPED')
                if qa['visible_moving_part_pixels']<p['minimum_moving_part_pixels']:raise RuntimeError('QA_MOVING_PART_NOT_VISIBLE')
                if not p['minimum_object_coverage']<=qa['object_coverage']<=p['maximum_object_coverage']:raise RuntimeError('QA_OBJECT_COVERAGE')
                mask=sam_mask(folder,self.config['sam3'],self.job['object_prompt']);P,keep=backproject(depth,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),mask)
                if len(P)<200:raise RuntimeError('SENSOR_SEGMENTATION_CLOUD_EMPTY')
                np.savez_compressed(folder/'point_cloud.npz',points_world_m=P.astype(np.float32),rgb=rgb[keep],units='m')
                views.append({'view_id':proposal['view_id'],'directory':str(folder.relative_to(self.output)),'K':meta['K'],'T_world_camera_optical':meta['T_world_camera_optical'],'qa':qa})
                row.update(status='ACQUIRED',actual_T_camera=meta['T_world_camera_optical'],qa=qa,base=list(r.base))
                (state_dir/'views_checkpoint.json').write_text(json.dumps({'label':label,'estimated_state':value,'views':views},indent=2))
                framing_clouds.append(P[::max(1,int(np.ceil(len(P)/30000)))])
                self.observed_cloud=np.concatenate(framing_clouds)
                self.center,self.extent,_=observed_volume(self.observed_cloud)
                updated={v['view_id']:v for v in coverage_views(self.observed_cloud,self.center,normal,self.cal,p)}
                for future in range(proposal_index+1,len(proposals)):
                    proposals[future]=updated[proposals[future]['view_id']]
                log['framing_source']='initial sensor volume plus current-state SAM3 RGB-D; camera planning only; not merged reconstruction input'
            except (subprocess.SubprocessError,TimeoutError) as e:
                row.update(status='REJECTED',reason='SAM3_SENSOR_INFERENCE_FAILED:'+str(e))
            except RuntimeError as e:
                row.update(status='REJECTED',reason=str(e))
                if not str(e).startswith(('SCAN_','MOBILE_VIEW_RECOVERY_','NO_SAFE_HOME_PATH_','HANDLE_REOBSERVE_','CAMERA_' ,'QA_','SENSOR_','OBJECT_WOULD_BE_')):raise
            (self.output/'camera_trajectory.json').write_text(json.dumps(self.scan_logs+[log],indent=2))
            if len(views)>=p['maximum_clean_views']:break
        if not self.config.get('maximum_range',{}).get('enabled'):r.scan_home()
        from wrist_reconstruction.oracle import scan as oracle_scan
        try:log['oracle_capture']=oracle_scan(self,label,value)
        except Exception as error:log['oracle_error']=str(error)
        self.scan_logs.append(log)
        state={'state_id':len(self.states),'label':label,'estimated_articulation_state':float(value),'articulation_estimate':articulation,'views':views,'timestamp_sim_s':r.time(),'robot_base_pose':list(r.base),'T_world_ee':r.tcp().tolist(),'last_physical_grasp_pose':r.grasp.tolist(),'grasp_still_stable':False,'released_for_capture':True,'clean_wrist_capture':len(views)>=p['minimum_clean_views'],'stop_reason':None if len(views)>=p['minimum_clean_views'] else 'INSUFFICIENT_CLEAN_REACHABLE_WRIST_VIEWS'}
        self.states.append(state);(state_dir/'state.json').write_text(json.dumps(state,indent=2));self.flush('RUNNING');return state

    def capture(self,label,value,*args,**kwargs):
        # A held-grasp external view cannot be called clean wrist capture.
        if self.runtime is None:return None
        self.runtime.recover(value,capture_label=label)
        return self.states[-1]

    def flush(self,status):
        super().flush(status)
        p=self.output/'multistate_capture.json';d=json.loads(p.read_text());d.update(schema='wrist-multistate-v1',capture_mode='wrist_camera_capture',background='unaltered normal scene RGB',mask_source='SAM3; simulator instance IDs QA only',real_hardware_calibrated=getattr(self,'cal',{}).get('real_hardware_calibrated',False));p.write_text(json.dumps(d,indent=2))
