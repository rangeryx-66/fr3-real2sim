"""RGB-D-only panel/foreground regions. No scene assets or object identity inputs."""
import json
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull
from matplotlib.path import Path as Polygon

def cloud(folder):
 folder=Path(folder);m=json.loads((folder/'camera.json').read_text());K=np.array(m['K']);C=np.array(m['T_world_camera_optical']);d=np.load(folder/'depth_m.npy');y,x=np.indices(d.shape);z=d.reshape(-1);P=np.c_[(x.reshape(-1)-K[0,2])*z/K[0,0],(y.reshape(-1)-K[1,2])*z/K[1,1],z]@C[:3,:3].T+C[:3,3];valid=np.isfinite(d)&(d>.02)&(d<5)
 if (folder/'robot_self_exclusion.npy').exists():valid&=~np.load(folder/'robot_self_exclusion.npy')
 return m,d,x,y,P.reshape(d.shape+(3,)),valid

def panel(folder,seed_mask=None):
 m,d,x,y,P,valid=cloud(folder);region=valid.copy()
 if seed_mask is not None:
  yy,xx=np.where(seed_mask);pad=max(np.ptp(xx),np.ptp(yy))*.25+25
  region&=(x>=xx.min()-pad)&(x<=xx.max()+pad)&(y>=yy.min()-pad)&(y<=yy.max()+pad)&~ndimage.binary_dilation(seed_mask,iterations=4)
 Q=P[region];rng=np.random.default_rng(61)
 if len(Q)>8000:Q=Q[rng.choice(len(Q),8000,replace=False)]
 best=None
 for _ in range(180):
  a,b,c=Q[rng.choice(len(Q),3,replace=False)];n=np.cross(b-a,c-a);n/=max(np.linalg.norm(n),1e-12)
  if abs(n[2])>.5:continue
  keep=abs((Q-a)@n)<.003
  if best is None or keep.sum()>best.sum():best=keep
 A=Q[best];center=np.median(A,axis=0);_,_,V=np.linalg.svd(A-center,full_matrices=False);n=V[-1]
 if n@(np.array(m['T_world_camera_optical'])[:3,3]-center)<0:n=-n
 height=(P-center)@n;inliers=valid&(abs(height)<.003)
 return dict(center=center,normal=n,height=height,inliers=inliers,valid=valid,P=P,x=x,y=y,method='observed depth RANSAC vertical panel; 3mm inherited plane tolerance; measured robot exclusion')

def depth_proposal(folder):
 r=panel(folder);yy,xx=np.where(r['inliers']);bounds=np.c_[xx,yy];hull=ConvexHull(bounds);polygon=Polygon(bounds[hull.vertices]);inside=polygon.contains_points(np.c_[r['x'].reshape(-1),r['y'].reshape(-1)]).reshape(r['valid'].shape)
 foreground=r['valid']&inside&(r['height']>.003);labels,n=ndimage.label(foreground);candidates=[]
 for label in range(1,n+1):
  y,x=np.where(labels==label)
  if len(x)<30:continue
  extent=np.array([np.ptp(x)+1,np.ptp(y)+1]);elongation=float(extent.max()/max(extent.min(),1));score=float(np.sqrt(len(x))*elongation)
  candidates.append(dict(component=label,pixels=len(x),bbox_xyxy=[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)],score=score))
 if not candidates:raise RuntimeError('NO_OBSERVED_DEPTH_PROPOSAL')
 chosen=max(candidates,key=lambda c:c['score']);x0,y0,x1,y1=chosen['bbox_xyxy'];margin=.5*max(x1-x0,y1-y0);H,W=foreground.shape;box=[int(max(0,x0-margin)),int(max(0,y0-margin)),int(min(W,x1+margin)),int(min(H,y1+margin))]
 return dict(crop_xyxy=box,source='observed panel foreground components, measured robot exclusion, image elongation/area rank',GT_crop=False,proposal=chosen,candidates=candidates,panel_center_world_m=r['center'].tolist(),panel_normal_world=r['normal'].tolist(),padding_fraction_long_axis=.5)

def extend_observed_mask(folder):
 folder=Path(folder);seed=np.load(folder/'handle_mask.npy').astype(bool);r=panel(folder,seed);y,x=np.where(seed);padding=.1*max(np.ptp(x)+1,np.ptp(y)+1)+4
 roi=(r['x']>=x.min()-padding)&(r['x']<=x.max()+padding)&(r['y']>=y.min()-padding)&(r['y']<=y.max()+padding)
 heights=r['height'][seed&r['valid']];heights=heights[np.isfinite(heights)&(heights>0)];hist,edges=np.histogram(heights,bins=128);centers=(edges[:-1]+edges[1:])/2;weight=np.cumsum(hist);mean=np.cumsum(hist*centers);total=weight[-1];between=(mean[-1]*weight-mean*total)**2/np.maximum(weight*(total-weight),1);threshold=max(.003,float(centers[int(np.argmax(between))]));fg=r['valid']&roi&(r['height']>threshold);labels,n=ndimage.label(fg);ids=np.unique(labels[seed&fg]);ids=ids[ids>0];grown=np.isin(labels,ids)&fg
 np.save(folder/'sam3_seed_mask.npy',seed);np.save(folder/'handle_mask.npy',grown)
 audit=dict(source='SAM3 seeds + connected observed protruding depth surface; no GT',seed_pixels=int(seed.sum()),extended_pixels=int(grown.sum()),panel_center_world_m=r['center'].tolist(),panel_normal_world=r['normal'].tolist(),foreground_panel_tolerance_m=.003,observed_depth_layer_threshold_m=threshold,depth_layer_method='Otsu on positive SAM seed protrusion heights; no GT tuning',ROI_padding_fraction_long_axis=.1,GT_handle_surface_used=False)
 (folder/'observed_region_extension.json').write_text(json.dumps(audit,indent=2));return audit
