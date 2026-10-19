"""Calibrated state images from live simulation sensors; no object actuation."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def matrix(p,q):
 T=np.eye(4);T[:3,3]=p;T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();return T


def backproject(depth,K,T,mask,stride=2):
 y,x=np.indices(depth.shape);keep=(mask>0)&np.isfinite(depth)&(depth>.02)&(depth<3.)&((x%stride)==0)&((y%stride)==0)
 z=depth[keep];P=np.column_stack(((x[keep]-K[0,2])*z/K[0,0],(y[keep]-K[1,2])*z/K[1,1],z))
 return P@T[:3,:3].T+T[:3,3],keep


class CaptureRecorder:
 def __init__(self,scene,output,job):
  self.scene=scene;self.output=Path(output);self.job=job;self.states=[];self.metadata=json.loads((Path(job['asset_root'])/'manifest.json').read_text())
  self.cameras=scene['overview'];self.asset_path=scene['asset_path'];self.part_path=scene['contact_target_path']
  for camera in self.cameras:
   camera.add_distance_to_image_plane_to_frame();camera.add_instance_id_segmentation_to_frame()
  self.flush('RUNNING')

 def flush(self,status):
  if self.states and status not in ('RUNNING','MULTISTATE_TARGET_COMPLETE'):
   self.states[-1]['stop_reason']=status
   (self.output/'states'/f"state_{self.states[-1]['state_id']:03d}"/'state.json').write_text(json.dumps(self.states[-1],indent=2))
  meta=json.loads((Path(self.job['asset_root'])/'manifest.json').read_text())
  doc={'schema':'articulated-multistate-v1','object_id':meta['asset_id'],'moving_part_id':meta['door_link'],'joint_family_requested':self.job['skill']['joint_type'],'goal':self.job['skill']['goal'],'targets':self.job['skill']['targets'],'units':'degrees' if self.job['skill']['joint_type']=='revolute' else 'meters','states':self.states,'status':status,'geometry_source':meta['source_url'],'interaction_geometry':'frozen semantic approximation; not exact true geometry','articulation_labels':'EE/estimated model; simulator joint state not used online','mask_source':'simulator instance-ID render annotator; not real-world segmentation','physical_contact':True,'attachment':False,'object_commands':False}
  (self.output/'multistate_capture.json').write_text(json.dumps(doc,indent=2))

 def capture(self,label,estimated_state,t,base,E,grasp,stable,grip,stop_reason=None,articulation=None):
  import cv2
  state_dir=self.output/'states'/f'state_{len(self.states):03d}';state_dir.mkdir(parents=True,exist_ok=True)
  self.scene['world'].render();views=[]
  for i,camera in enumerate(self.cameras):
   frame=camera.get_current_frame();rgb=np.asarray(camera.get_rgba())[:,:,:3];depth=np.asarray(frame.get('distance_to_image_plane',[]),np.float32)
   seg=frame.get('instance_id_segmentation',{});ids=np.asarray(seg.get('data',[]));labels=seg.get('info',{}).get('idToLabels',{})
   if depth.shape!=rgb.shape[:2] or ids.shape!=depth.shape:raise RuntimeError('CAPTURE_SENSOR_FRAME_UNAVAILABLE')
   asset_ids=[int(k) for k,v in labels.items() if self.asset_path in str(v)];part_ids=[int(k) for k,v in labels.items() if self.part_path in str(v)]
   mask=np.isin(ids,asset_ids).astype(np.uint8);part=np.isin(ids,part_ids).astype(np.uint8)
   if mask.sum()<100:raise RuntimeError('CAPTURE_OBJECT_MASK_EMPTY')
   folder=state_dir/f'view_{i:02d}';folder.mkdir(exist_ok=True)
   K=np.asarray(camera.get_intrinsics_matrix(),float);T=matrix(*camera.get_world_pose(camera_axes='ros'))
   cv2.imwrite(str(folder/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));cv2.imwrite(str(folder/'mask.png'),mask*255);cv2.imwrite(str(folder/'moving_part_mask.png'),part*255);np.save(folder/'depth_m.npy',depth)
   P,keep=backproject(depth,K,T,mask);np.savez_compressed(folder/'point_cloud.npz',points_world_m=P.astype(np.float32),rgb=rgb[keep],units='m')
   metadata={'K':K.tolist(),'T_world_camera_optical':T.tolist(),'camera_axes':'ROS optical x right,y down,z forward','depth':'distance_to_image_plane / optical Z in meters','resolution_wh':list(rgb.shape[1::-1]),'instance_labels':labels,'visible_object_pixels':int(mask.sum()),'visible_moving_part_pixels':int(part.sum()),'timestamp_sim_s':t,'mask_source':'instance ID, geometry rendered from actual simulation'}
   (folder/'camera.json').write_text(json.dumps(metadata,indent=2));views.append({'view_id':i,'directory':str(folder.relative_to(self.output)),**{k:metadata[k] for k in ['K','T_world_camera_optical','visible_object_pixels','visible_moving_part_pixels']}})
  state={'object_id':self.metadata['asset_id'],'moving_part_id':self.metadata['door_link'],'robot_base_pose_units':['m','m','m','deg'],'grasp_pose_kind':'current measured official TCP proxy; contact centroid is not separately measured','T_world_initial_grasp':np.asarray(grasp).tolist(),'state_id':len(self.states),'label':label,'timestamp_sim_s':t,'estimated_articulation_state':float(estimated_state),'articulation_estimate':articulation,'robot_base_pose':list(base),'T_world_ee':np.asarray(E).tolist(),'T_world_grasp':np.asarray(E).tolist(),'grasp_still_stable':bool(stable),'stability_signal':grip,'full_tangential_slip_observed':False,'stop_reason':stop_reason,'views':views}
  (state_dir/'state.json').write_text(json.dumps(state,indent=2));self.states.append(state);self.flush('RUNNING');return state
