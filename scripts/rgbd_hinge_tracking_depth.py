"""Known-hinge projective RGB-D registration, using an observed door template.
Dense depth remains observable when a few RGB corners are occluded. No GT
state, simulator segmentation, model mesh, or GT handle template is an input.
"""
import cv2
import numpy as np
from scipy.optimize import minimize_scalar
from rgbd_hinge_tracking import RGBDHingeTracker


class RGBDDepthHingeTracker(RGBDHingeTracker):
 def __init__(self,rgb,depth,K,C,mask,origin,axis,T_closed,t=0.):
  super().__init__(rgb,depth,K,C,mask,origin,axis,T_closed,t)
  yy,xx=np.nonzero(np.asarray(mask,bool)&np.isfinite(depth)&(depth>.02)&(depth<5.))
  ids=np.linspace(0,len(xx)-1,min(1600,len(xx))).astype(int)
  self.depth_template=self.backproject(np.column_stack((xx[ids],yy[ids])).astype(np.float32),depth)

 def depth_cost(self,angle,depth,excluded):
  pixels,z=self.project(self.depth_template,angle);xy=np.rint(pixels).astype(int)
  inside=(pixels[:,0]>=1)&(pixels[:,0]<depth.shape[1]-2)&(pixels[:,1]>=1)&(pixels[:,1]<depth.shape[0]-2)&(z>.02)
  ids=np.flatnonzero(inside)
  if len(ids):ids=ids[~excluded[xy[ids,1],xy[ids,0]]]
  if not len(ids):return .05,0
  obs=cv2.remap(depth.astype(np.float32,copy=False),pixels[ids,0].astype(np.float32).reshape(1,-1),pixels[ids,1].astype(np.float32).reshape(1,-1),cv2.INTER_LINEAR).ravel()
  valid=np.isfinite(obs)&(obs>.02)&(obs<5.);ids=ids[valid];obs=obs[valid]
  if not len(ids):return .05,0
  difference=np.abs(z[ids]-obs)
  # Robust depth residual plus missing-visibility cost is a fit objective,
  # never a sensor-quality/manipulation gate. All scores are archived.
  cost=float(np.mean(np.minimum(difference,.03))+.005*(1-len(ids)/len(self.depth_template)))
  return cost,len(ids)

 def update(self,rgb,depth,t,excluded=None,phase=''):
  excluded=np.zeros(depth.shape,bool) if excluded is None else excluded
  predicted=self.state_at(t,phase);old_time=self.last_observed_t;old_angle=self.last_observed_angle;old_velocity=self.velocity
  record=super().update(rgb,depth,t,excluded,phase);rgb_angle=record['angle_deg']
  objective=lambda a:self.depth_cost(a,depth,excluded)[0]+.02*(a-predicted)**2
  grid=np.deg2rad(np.arange(-5.,96.,1.));scores=[objective(a) for a in grid];best=int(np.argmin(scores));center=grid[best]
  fit=minimize_scalar(objective,bounds=(center-np.deg2rad(1.),center+np.deg2rad(1.)),method='bounded',options={'xatol':1e-5,'maxiter':25})
  cost,visible=self.depth_cost(float(fit.x),depth,excluded)
  if visible:
   self.angle=float(fit.x);interval=float(t)-old_time
   if interval>0.:self.velocity=.5*old_velocity+.5*(self.angle-old_angle)/interval if phase.startswith('OPEN') else 0.
   else:self.velocity=old_velocity if phase.startswith('OPEN') else 0.
   self.last_observed_angle=self.angle;self.last_observed_t=float(t)
   record.update(angle_rad=self.angle,angle_deg=float(np.rad2deg(self.angle)),angular_velocity_rad_s=self.velocity,T_moving_estimated=self.pose().tolist(),measurement_source='OBSERVED_PROJECTIVE_RGBD_DEPTH')
  else:
   self.angle=predicted;record.update(angle_rad=predicted,angle_deg=float(np.rad2deg(predicted)),T_moving_estimated=self.pose().tolist(),measurement_source='VISUAL_HISTORY_PREDICTION_OCCLUDED')
  record.update(depth_template_points=len(self.depth_template),depth_visible_points=visible,depth_fit_cost_m=cost,depth_grid_best_deg=float(np.rad2deg(center)),temporal_prior_m_per_rad2=.02,temporal_prediction_deg=float(np.rad2deg(predicted)),RGB_feature_angle_diagnostic_deg=rgb_angle,source='observed RGB-D door template + known hinge projective depth fit')
  return record
