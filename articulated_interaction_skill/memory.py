"""Prismatic wrapper uses existing classifier; no new reconstruction fitter."""
import json
import numpy as np
from interaction_identification.act2see_loop import InteractionMemory,unit
from interaction_identification.fitting import fit_articulation

class PrismaticMemory(InteractionMemory):
 def save(self):
  (self.output/'structured_memory.json').write_text(json.dumps({'source':'measured EE only','estimated_articulation':self.estimate,'attempt_history':self.attempts,'fit_history':self.estimates,'supporting_observations':self.poses,'GT_inputs':False}))
  if self.estimate:
   r=self.estimate['prismatic'];fmt=lambda v:' '.join(f'{x:.12g}' for x in v)
   # Minimal structured memory; no visual/collision/inertial model is replaced.
   values=[0.]+[float((np.asarray(T)[:3,3]-self.initial[:3,3])@np.asarray(r['axis'])) for T in self.poses]
   (self.output/'estimated_articulation.urdf').write_text('<robot name="EE_prismatic_estimate"><!-- Structured memory only; limits are observed range, NOT physical joint limits; no execution model. --><link name="world_estimate"/><link name="moving_estimate"/><joint name="estimated_joint" type="prismatic"><parent link="world_estimate"/><child link="moving_estimate"/><origin xyz="'+fmt(self.initial[:3,3])+'" rpy="0 0 0"/><axis xyz="'+fmt(r['axis'])+'"/><limit lower="'+str(min(values))+'" upper="'+str(max(values))+'" effort="0" velocity="0"/></joint></robot>')
 def try_fit(self):
  if len(self.poses)<12 or self.travel()<.005:return None
  f=fit_articulation(self.poses);accepted=f['joint_type']=='prismatic'
  self.estimates.append({'fit':f,'accepted':accepted,'observation_count':len(self.poses)})
  if accepted:
   axis=np.asarray(f['prismatic']['axis']);delta=np.asarray(self.poses[-1])[:3,3]-self.initial[:3,3];self.follow_sign=1. if axis@delta>=0 else -1.;self.estimate=f
  self.save();return f
 def tangent(self,T):return self.follow_sign*unit(self.estimate['prismatic']['axis'])
 def state(self,T):return float((np.asarray(T)[:3,3]-self.initial[:3,3])@self.tangent(T)) if self.estimate else 0.
 def angle_deg(self,T):return 0. # legacy display is explicitly not a prismatic state
 def consistency_error(self,T):
  if self.estimate is None:return None
  d=np.asarray(T)[:3,3]-self.initial[:3,3];a=self.tangent(T);return float(np.linalg.norm(d-a*(d@a)))
