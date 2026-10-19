"""Additional calibrated views, rendered without advancing object physics."""
import numpy as np
import json,ast
from pathlib import Path
from scipy.spatial.transform import Rotation
from articulated_interaction_skill.capture import CaptureRecorder,backproject,matrix


class MultiviewRecorder(CaptureRecorder):
    def capture(self,*args,**kwargs):
        world=self.scene['world']
        if len(self.cameras)==2:
            from isaacsim.sensors.camera import Camera
            camera=self.cameras[0];world.render();frame=camera.get_current_frame()
            seg=frame['instance_id_segmentation'];ids=np.asarray(seg['data']);labels=seg['info']['idToLabels']
            keys=[int(k) for k,v in labels.items() if self.asset_path in str(v)]
            depth=np.asarray(frame['distance_to_image_plane']);K=np.asarray(camera.get_intrinsics_matrix())
            T=matrix(*camera.get_world_pose(camera_axes='ros'));P,_=backproject(depth,K,T,np.isin(ids,keys))
            if len(P)<100:raise RuntimeError('ORBIT_INITIAL_VISUAL_CENTER_UNAVAILABLE')
            center=(np.quantile(P,.02,axis=0)+np.quantile(P,.98,axis=0))/2
            radius=max(.65,float(np.linalg.norm(np.ptp(P,axis=0)))*1.3)
            count=self.job.get('system_capture',{}).get('orbit_views',12)
            cameras=list(self.cameras)
            world.pause()
            try:
                for i in range(count):
                    angle=2*np.pi*i/count;eye=center+radius*np.array([np.cos(angle),np.sin(angle),.4 if i%2==0 else .7])
                    forward=(center-eye)/np.linalg.norm(center-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right)
                    R=np.column_stack((right,np.cross(forward,right),forward))
                    c=Camera(prim_path=f'/World/skill_orbit_{i:02d}',name=f'skill_orbit_{i:02d}',resolution=(512,512),frequency=30)
                    c.initialize();c.set_focal_length(c.get_horizontal_aperture()/(2*np.tan(np.deg2rad(self.job.get('system_capture',{}).get('orbit_horizontal_FOV_deg',64.)/2))));c.set_world_pose(eye,np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros');c.set_clipping_range(.02,5.)
                    c.add_distance_to_image_plane_to_frame();c.add_instance_id_segmentation_to_frame();cameras.append(c)
                self.cameras=cameras
            finally:world.play()
        # Camera/render updates cannot move the object to another state.
        world.pause()
        try:
            # Camera's event callback does not populate _current_frame while
            # timeline is paused. Pull the attached render annotators directly
            # after rendering; no physics step or object state command is used.
            for _ in range(12):world.render()
            for camera in self.cameras:
                frame=camera.get_current_frame()
                for key in ('distance_to_image_plane','instance_id_segmentation'):
                    frame[key]=camera._custom_annotators[key].get_data()
                    if frame[key] is None:raise RuntimeError('FROZEN_RENDER_ANNOTATOR_UNAVAILABLE:'+key)
            return self._visible_capture(*args,**kwargs)
        finally:world.play()

    def _visible_capture(self,*args,**kwargs):
        # Preserve the original calibrated writer, but do not discard an entire
        # state because a close-up camera becomes occluded. Missing views are
        # recorded explicitly; no pixel, depth or geometry is fabricated.
        tree=ast.parse((Path(__file__).resolve().parents[1]/'articulated_interaction_skill/capture.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CaptureRecorder')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='capture')
        text=ast.unparse(method)
        old="if mask.sum() < 100:\n            raise RuntimeError('CAPTURE_OBJECT_MASK_EMPTY')"
        replacement="if mask.sum() < 100:\n            missing.append({'camera_id':i,'reason':'OBJECT_NOT_VISIBLE_OR_OCCLUDED','instance_labels':labels})\n            continue"
        if text.count(old)!=1:raise RuntimeError('CAPTURE_WRITER_HOOK_CHANGED')
        text=text.replace(old,replacement).replace('views = []','views = []; missing=[]')
        text=text.replace('self.states.append(state)',"state['excluded_views']=missing\n    if len(views)<2:raise RuntimeError('INSUFFICIENT_VISIBLE_STATE_VIEWS')\n    self.states.append(state)")
        scope={'np':np,'json':json,'backproject':backproject,'matrix':matrix};exec(compile(text,str(Path(__file__)),'exec'),scope)
        return scope['capture'](self,*args,**kwargs)
