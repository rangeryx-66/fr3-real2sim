"""RGB-D/SAM3 handle frame. Input files contain no asset or GT grasp metadata.
Uses a visible panel plane and the central visible handle cross section. This is
an observation-based adapter for the existing bar-side-pinch grasp family.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares

def estimate(folder):
 folder=Path(folder);meta=json.loads((folder/'camera.json').read_text());K=np.asarray(meta['K']);C=np.asarray(meta['T_world_camera_optical']);d=np.load(folder/'depth_m.npy');mask=np.load(folder/'handle_mask.npy').astype(bool)
 y,x=np.indices(d.shape);valid=np.isfinite(d)&(d>.02)&(d<5.)
 def points(m):
  z=d[m];P=np.column_stack(((x[m]-K[0,2])*z/K[0,0],(y[m]-K[1,2])*z/K[1,1],z));return P@C[:3,:3].T+C[:3,3]
 P=points(mask&valid);ring=ndimage.binary_dilation(mask,iterations=25)&~ndimage.binary_dilation(mask,iterations=4)&valid
 Q=points(ring);rng=np.random.default_rng(61)
 if len(Q)>6000:Q=Q[rng.choice(len(Q),6000,replace=False)]
 best=[]
 for _ in range(150):
  a,b,c=Q[rng.choice(len(Q),3,replace=False)];n=np.cross(b-a,c-a);n/=max(np.linalg.norm(n),1e-12);keep=np.abs((Q-a)@n)<.003
  if np.count_nonzero(keep)>len(best):best=Q[keep]
 center=np.median(best,axis=0);_,_,V=np.linalg.svd(best-center,full_matrices=False);normal=V[-1]
 if normal@(C[:3,3]-center)<0:normal=-normal
 h=(P-center)@normal;P=P[h>np.quantile(h,.10)]
 mid=np.median(P,axis=0);_,_,V=np.linalg.svd(P-mid,full_matrices=False);axis=V[0]-normal*(V[0]@normal);axis/=np.linalg.norm(axis)
 signaxis=np.argmax(abs(axis))
 if axis[signaxis]<0:axis=-axis
 s=(P-mid)@axis;lo,hi=np.quantile(s,[.05,.95]);center_s=(lo+hi)/2;central=P[np.abs(s-center_s)<(hi-lo)*.2]
 side=np.cross(normal,axis);side/=np.linalg.norm(side)
 uv=np.column_stack(((central-mid)@side,(central-mid)@normal));umin,umax=np.quantile(uv[:,0],[.02,.98]);width=umax-umin;u0=(umin+umax)/2;front=np.quantile(uv[:,1],.90)
 fit=least_squares(lambda v:np.linalg.norm(uv-v[:2],axis=1)-v[2],[u0,front-width/2,width/2],bounds=([umin,front-width,width*.2],[umax,front,width*1.5]),loss='soft_l1',f_scale=.001)
 residual=float(np.sqrt(np.mean(fit.fun**2)));curvature=float(np.ptp(np.quantile(uv[:,1],[.05,.95])))
 # A flat visible face does not identify thickness; use its visible-width prior,
 # report it explicitly, and retain the uncorrected surface as another candidate.
 cross_center=fit.x[:2] if curvature>width*.15 else np.array([u0,front-width/2])
 anchor=mid+axis*center_s+side*cross_center[0]+normal*cross_center[1]
 R=np.column_stack((axis,np.cross(-normal,axis),-normal));H=np.eye(4);H[:3,:3]=R;H[:3,3]=anchor
 result={'source':'actual RGB-D + RGB-only SAM3; visible panel plane and handle cross-section','GT_inputs':False,'simulator_segmentation_used':False,'anchor_world_m':anchor.tolist(),'axis_world':axis.tolist(),'outward_normal_world':normal.tolist(),'T_world_handle':H.tolist(),'dimensions_m':[float(hi-lo),float(width),float(width)],'observed_points':len(P),'panel_plane_inliers':len(best),'cross_section_fit_residual_m':residual,'visible_cross_section_depth_span_m':curvature,'cross_section_method':'circle fit' if curvature>width*.15 else 'visible-width thickness prior; unobserved backside not measured','front_surface_anchor_world_m':(mid+axis*center_s+side*u0+normal*front).tolist(),'input_sha256':{n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in ['rgb.png','depth_m.npy','camera.json','handle_mask.npy']}}
 # Alternative for a faceted bar: estimate a closing normal from an observed
 # side face, rather than treating its visible cross section as a circle.
 local=P[np.abs((P-anchor)@axis)<min(.025,(hi-lo)*.2)]
 cross=np.column_stack(((local-anchor)@side,(local-anchor)@normal));best_face=None
 for _ in range(500):
  pa,pb=cross[rng.choice(len(cross),2,replace=False)];edge=pb-pa
  if np.linalg.norm(edge)<.002:continue
  closing=np.array([edge[1],-edge[0]]);closing/=np.linalg.norm(closing)
  if abs(closing[0])<.8:continue
  support=np.abs((cross-pa)@closing)<.00025;score=float(support.sum()*abs(closing[0])**4)
  if best_face is None or score>best_face[0]:best_face=(score,support)
 if best_face is not None:
  observed=cross[best_face[1]];_,_,V=np.linalg.svd(observed-observed.mean(0),full_matrices=False);closing=V[-1]
  if closing[0]<0:closing=-closing
  face_normal=np.array([-closing[1],closing[0]])
  projected=np.column_stack((cross@closing,cross@face_normal));center2=(np.quantile(projected,.02,axis=0)+np.quantile(projected,.98,axis=0))/2
  uvcenter=closing*center2[0]+face_normal*center2[1]
  result['observed_side_face']={'anchor_world_m':(anchor+side*uvcenter[0]+normal*uvcenter[1]).tolist(),'outward_normal_world':(side*face_normal[0]+normal*face_normal[1]).tolist(),'observed_support_points':int(best_face[1].sum()),'local_cross_section_points':len(cross),'source':'RGB-D visible side-face line and observed cross-section bounds; unobserved opposing face assumed parallel','roll_from_panel_deg':float(np.rad2deg(np.arctan2(face_normal[0],face_normal[1])))}
 np.save(folder/'handle_points_world.npy',P);(folder/'visual_handle.json').write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);a=p.parse_args();print(json.dumps(estimate(a.input)))
