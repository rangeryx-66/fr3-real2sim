"""One-axis RGB-D feature tracking; no simulator state or asset handle inputs.
Known hinge geometry maps visually measured motion to the moving-link frame.
All visibility/fit quantities are diagnostics, never manipulation stop gates.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class RGBDHingeTracker:
 def __init__(self,rgb,depth,K,C,mask,origin,axis,T_closed,t=0.):
  self.K=np.asarray(K,float);self.C=np.asarray(C,float);self.origin=np.asarray(origin,float)
  self.axis=np.asarray(axis,float);self.axis/=np.linalg.norm(self.axis);self.T_closed=np.asarray(T_closed,float)
  self.gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);self.mask=np.asarray(mask,bool)
  valid=self.mask&np.isfinite(depth)&(depth>.02)&(depth<5.)
  pixels=cv2.goodFeaturesToTrack(self.gray,maxCorners=700,qualityLevel=.003,minDistance=3,mask=valid.astype(np.uint8)*255,blockSize=3)
  self.pixels=np.empty((0,2),np.float32) if pixels is None else pixels.reshape(-1,2)
  self.points=self.backproject(self.pixels,depth);self.angle=0.;self.velocity=0.;self.t=float(t)
  self.last_observed_t=float(t);self.last_observed_angle=0.;self.records=[]
  self.reference_points=self.points.copy();self.reference_pixels=self.pixels.copy();self.initial_gray=self.gray.copy()

 def backproject(self,pixels,depth):
  xy=np.rint(pixels).astype(int);z=depth[xy[:,1],xy[:,0]]
  p=np.column_stack(((pixels[:,0]-self.K[0,2])*z/self.K[0,0],(pixels[:,1]-self.K[1,2])*z/self.K[1,1],z))
  return p@self.C[:3,:3].T+self.C[:3,3]

 def delta(self,angle):
  R=Rotation.from_rotvec(self.axis*angle).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=self.origin-R@self.origin;return D

 def pose(self,angle=None):return self.delta(self.angle if angle is None else angle)@self.T_closed

 def project(self,points,angle):
  D=self.delta(angle);p=points@D[:3,:3].T+D[:3,3];q=(p-self.C[:3,3])@self.C[:3,:3]
  uv=q@self.K.T;return uv[:,:2]/uv[:,2,None],q[:,2]

 def state_at(self,t,phase=''):
  moving=phase.startswith('OPEN') or phase.startswith('PULL')
  return float(self.angle+(min(max(float(t)-self.t,0.),1.5)*self.velocity if moving else 0.))

 def update(self,rgb,depth,t,excluded=None,phase=''):
  gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);excluded=np.zeros(depth.shape,bool) if excluded is None else excluded
  predicted=self.state_at(t,phase);points=self.points;old=self.pixels
  quality={'source':'RGB-D temporal features + known hinge','GT_state_input':False,'feature_count_before':len(old),'phase':phase}
  if len(old):
   nxt,status,error=cv2.calcOpticalFlowPyrLK(self.gray,gray,old.reshape(-1,1,2),None,winSize=(25,25),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
   back,valid,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,nxt, None,winSize=(25,25),maxLevel=3)
   uv=nxt.reshape(-1,2);xy=np.rint(uv).astype(int);inside=(xy[:,0]>=0)&(xy[:,0]<depth.shape[1])&(xy[:,1]>=0)&(xy[:,1]<depth.shape[0]);keep=inside&status.ravel().astype(bool)&valid.ravel().astype(bool)&(np.linalg.norm(back.reshape(-1,2)-old,axis=1)<1.5)
   indexes=np.flatnonzero(inside);keep[indexes]&=~excluded[xy[indexes,1],xy[indexes,0]]
   points=points[keep];uv=uv[keep]
  else:uv=np.empty((0,2),np.float32)
  # Fit observed feature correspondences on one known hinge. Pixel outlier
  # rejection is perception estimation; none of these metrics stop execution.
  if len(uv)>=4:
   def residual(x):return (self.project(points,float(x[0]))[0]-uv).reshape(-1)
   fit=least_squares(residual,[predicted],bounds=([max(np.deg2rad(-15),predicted-np.deg2rad(15))],[min(np.deg2rad(105),predicted+np.deg2rad(15))]),loss='soft_l1',f_scale=1.,max_nfev=30)
   angle=float(fit.x[0]);errors=np.linalg.norm(self.project(points,angle)[0]-uv,axis=1);inliers=errors<3.
   if np.count_nonzero(inliers)>=4:
    points=points[inliers];uv=uv[inliers];fit=least_squares(lambda x:(self.project(points,float(x[0]))[0]-uv).reshape(-1),[angle],loss='soft_l1',f_scale=.7,max_nfev=20);angle=float(fit.x[0])
    rate=(angle-self.last_observed_angle)/max(float(t)-self.last_observed_t,1e-6)
    self.velocity=.5*self.velocity+.5*rate if phase.startswith('OPEN') else 0.
    self.last_observed_angle=angle;self.last_observed_t=float(t);source='OBSERVED_RGBD_FEATURE_MOTION'
   else:angle=predicted;source='VISUAL_HISTORY_PREDICTION_OCCLUDED'
   quality.update(pixel_rmse=float(np.sqrt(np.mean(residual([angle])**2))),feature_inliers=int(np.count_nonzero(inliers)))
  else:angle=predicted;source='VISUAL_HISTORY_PREDICTION_OCCLUDED'
  self.angle=angle;self.t=float(t);self.gray=gray;self.pixels=uv.astype(np.float32);self.points=points
  # Reacquire initial observed points by projected RGB correspondence; no GT
  # moving pose, instance segmentation or saved GT grasp is used.
  if len(self.pixels)<25 and len(self.reference_points):
   projected,_=self.project(self.reference_points,self.angle)
   reacquired,status,_=cv2.calcOpticalFlowPyrLK(self.initial_gray,gray,self.reference_pixels.reshape(-1,1,2),projected.astype(np.float32).reshape(-1,1,2),winSize=(25,25),maxLevel=3,flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
   uv=reacquired.reshape(-1,2);xy=np.rint(uv).astype(int);inside=(xy[:,0]>=0)&(xy[:,0]<depth.shape[1])&(xy[:,1]>=0)&(xy[:,1]<depth.shape[0]);keep=inside&status.ravel().astype(bool)&(np.linalg.norm(uv-projected,axis=1)<6.)
   ids=np.flatnonzero(inside);keep[ids]&=~excluded[xy[ids,1],xy[ids,0]]
   self.pixels=uv[keep].astype(np.float32);self.points=self.reference_points[keep].copy();quality['reacquired_features']=len(self.pixels)
  quality.update(t=float(t),angle_rad=self.angle,angle_deg=float(np.rad2deg(self.angle)),angular_velocity_rad_s=self.velocity,measurement_source=source,visible_features=len(self.pixels),T_moving_estimated=self.pose().tolist())
  self.records.append(quality);return quality
