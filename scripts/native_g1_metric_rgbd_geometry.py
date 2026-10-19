"""Isolated, observation-only geometry study; never imported by frozen actors.

Inputs: metric depth, K/extrinsics, SAM-derived seeds and robot self mask.
No object asset, GT handle, collider or saved grasp input is accepted here.
Open measured patches and uncertain completions are deliberately separate.
"""
from pathlib import Path
import json
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import trimesh
from native_g1_observed_regions import cloud, panel


def load_seed(folder):
    folder=Path(folder)
    for name in ['sam3_seed_mask.npy','sam3_mask_raw.npy','handle_mask.npy']:
        if (folder/name).exists():return np.load(folder/name).astype(bool)
    raise ValueError('No archived observation-derived seed mask')


def projected_seed(folder, points):
    meta,depth,_,_,_,valid=cloud(folder)
    K=np.asarray(meta['K']);C=np.asarray(meta['T_world_camera_optical'])
    q=(points-C[:3,3])@C[:3,:3];uv=q@K.T;uv=uv[:,:2]/np.maximum(q[:,2,None],1e-8)
    xy=np.rint(uv).astype(int);H,W=depth.shape
    keep=(q[:,2]>.02)&(xy[:,0]>=0)&(xy[:,0]<W)&(xy[:,1]>=0)&(xy[:,1]<H)
    xy=xy[keep];z=q[keep,2];consistent=abs(depth[xy[:,1],xy[:,0]]-z)<.005
    seed=np.zeros_like(valid);seed[xy[consistent,1],xy[consistent,0]]=True
    return ndimage.binary_dilation(seed,iterations=1)&valid


def preserve(folder, seed=None, plane=None):
    folder=Path(folder);meta,d,x,y,P,valid=cloud(folder)
    archived_mask=(seed is None and (folder/'handle_mask.npy').exists())
    seed=load_seed(folder) if seed is None else seed
    r=panel(folder,seed) if plane is None else plane
    center=np.asarray(r['center']);normal=np.asarray(r['normal']);height=(P-center)@normal
    sy,sx=np.where(seed&valid)
    if not len(sx):raise ValueError('No depth-consistent observation seed')
    pad=.1*max(np.ptp(sx)+1,np.ptp(sy)+1)+4
    roi=(x>=sx.min()-pad)&(x<=sx.max()+pad)&(y>=sy.min()-pad)&(y<=sy.max()+pad)
    box_roi=roi.copy()
    # A seed bounding box can connect unrelated protruding door trim. Retain
    # the actual SAM image contour with a two-pixel neighborhood instead.
    roi &= ndimage.binary_dilation(seed,iterations=2)
    fg=valid&roi&(height>.003)
    # Keep the inherited depth-layer split only for confident seed selection,
    # not as a global deletion of lower, curved handle ends. Grow through
    # metric-neighbor continuity; touching image pixels across a depth jump
    # must not join raised trim/background to the handle.
    heights=height[seed&valid];heights=heights[np.isfinite(heights)&(heights>0)]
    hist,edges=np.histogram(heights,bins=128);c=(edges[:-1]+edges[1:])/2
    w=np.cumsum(hist);mu=np.cumsum(hist*c);between=(mu[-1]*w-mu*w[-1])**2/np.maximum(w*(w[-1]-w),1)
    threshold=max(.003,float(c[np.argmax(between)]))
    core=seed&fg&(height>threshold)
    # Preserve the inherited trusted body first. The defect is longitudinal
    # truncation, not a reason to replace already sound cross-sectional masks.
    if archived_mask:
        body=np.load(folder/'handle_mask.npy').astype(bool)&valid
    else:
        high=valid&box_roi&(height>threshold);lab,_=ndimage.label(high,np.ones((3,3)))
        positive=np.unique(lab[core]);body=high&np.isin(lab,positive[positive>0])
    f=(np.asarray(meta['K'])[0,0]+np.asarray(meta['K'])[1,1])/2
    limit=.003+2*np.median(d[seed&fg])/f
    # A handle mount can physically connect to door trim. Metric connectivity
    # alone cannot separate those semantics. Use local, observed depth layers
    # along the seed-derived axis, rather than one global height threshold.
    core_points=P[core];mid_core=np.median(core_points,axis=0)
    _,_,directions=np.linalg.svd(core_points-mid_core,full_matrices=False)
    long=directions[0]-normal*(directions[0]@normal);long/=np.linalg.norm(long)
    s=(P-mid_core)@long;lo,hi=np.quantile(s[core],[0,1]);seed_s=s[seed&fg]
    edges=np.linspace(seed_s.min(),seed_s.max(),33);local_mask=np.zeros_like(fg)
    for left,right in zip(edges[:-1],edges[1:]):
        strip=fg&(s>=left)&(s<=right);values=np.sort(height[strip])
        if len(values)<3:continue
        groups=np.split(values,np.where(np.diff(values)>limit)[0]+1)
        supported=[g for g in groups if len(g)>=3]
        if not supported:continue
        front=supported[-1]
        local_mask|=strip&(height>=front[0]-.000001)&(height<=front[-1]+.000001)
    fg &= local_mask;core &= fg
    H,W=fg.shape;ids=np.full(fg.shape,-1,int);ids[fg]=np.arange(int(fg.sum()));aa=[];bb=[]
    for dy,dx in [(0,1),(1,0),(1,1),(1,-1)]:
        y0=slice(0,H-dy);y1=slice(dy,H)
        x0=slice(max(0,-dx),min(W,W-dx));x1=slice(max(0,dx),min(W,W+dx))
        keep=fg[y0,x0]&fg[y1,x1]&(np.linalg.norm(P[y0,x0]-P[y1,x1],axis=2)<=limit)
        aa.append(ids[y0,x0][keep]);bb.append(ids[y1,x1][keep])
    aa=np.concatenate(aa);bb=np.concatenate(bb);_,labels=connected_components(coo_matrix((np.ones(len(aa)),(aa,bb)),shape=(int(fg.sum()),int(fg.sum()))),directed=False)
    chosen,count=np.unique(labels[ids[core]],return_counts=True);chosen=chosen[count>=3]
    mask=np.zeros_like(fg);mask[fg]=np.isin(labels,chosen)
    # Only extend missing end intervals. Do not repopulate low/background
    # layers inside the longitudinal span already supported by the body.
    body_s=s[body]
    if len(body_s):mask=body|(mask&((s<body_s.min())|(s>body_s.max())))
    points=P[mask]
    if len(points)<3:raise ValueError('Insufficient observed foreground geometry')
    mid=np.median(points,axis=0);_,_,V=np.linalg.svd(points-mid,full_matrices=False)
    axis=V[0]-normal*(V[0]@normal);axis/=np.linalg.norm(axis)
    if axis[np.argmax(abs(axis))]<0:axis=-axis
    side=np.cross(normal,axis);side/=np.linalg.norm(side)
    # Connect only neighboring measured pixels. Reject depth discontinuity
    # triangles using sampling resolution; never close a hidden surface.
    H,W=mask.shape;idx=np.arange(H*W).reshape(H,W);flat=P.reshape(-1,3)
    triangles=np.vstack([np.c_[idx[:-1,:-1].ravel(),idx[1:,:-1].ravel(),idx[:-1,1:].ravel()],
                         np.c_[idx[1:,:-1].ravel(),idx[1:,1:].ravel(),idx[:-1,1:].ravel()]])
    triangles=triangles[mask.ravel()[triangles].all(1)]
    pix=np.median(d[mask])/f;limit=.003+2*pix
    edge=np.linalg.norm(flat[triangles]-flat[np.roll(triangles,1,axis=1)],axis=2).max(1)
    triangles=triangles[edge<=limit]
    mesh=trimesh.Trimesh(flat,triangles,process=False);mesh.remove_unreferenced_vertices()
    audit=dict(GT_inputs=False,seed_pixels=int(seed.sum()),preserved_pixels=int(mask.sum()),
        panel_center_world_m=center.tolist(),panel_normal_world=normal.tolist(),axis_world=axis.tolist(),
        frame_world=np.column_stack((axis,side,normal)).tolist(),origin_world_m=mid.tolist(),
        filter='preserve inherited trusted body; extend only missing longitudinal ends with 32 local depth layers and metric-neighbor continuity in two-pixel seed neighborhood; no 10% height/5-95% length deletion',
        observed_triangle_edge_limit_m=float(limit),mesh_role='open observed-neighbor interpolation, not a completed solid')
    return dict(folder=str(folder),points=points,mesh=mesh,mask=mask,seed=seed,meta=meta,audit=audit,
                plane=dict(center=center,normal=normal))


def save_view(result, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    np.save(output/'preserved_points_world.npy',result['points']);np.save(output/'preserved_mask.npy',result['mask'])
    result['mesh'].export(output/'observed_patch.ply')
    (output/'GEOMETRY.json').write_text(json.dumps(result['audit'],indent=2))


def fuse(views, voxel=.0005):
    points=np.vstack([v['points'] for v in views]);keys=np.floor(points/voxel).astype(int)
    _,inverse=np.unique(keys,axis=0,return_inverse=True);count=np.bincount(inverse)
    averaged=np.column_stack([np.bincount(inverse,weights=points[:,i])/count for i in range(3)])
    # Keep per-view triangles: fusion does not introduce cross-occlusion faces.
    patches=trimesh.util.concatenate([v['mesh'] for v in views])
    return averaged,patches


def uncertain_profiles(points, frame, origin):
    """Three declared rear priors, with curved observed support retained.

    These convex longitudinal pieces are *hypothetical*. The open measured
    mesh remains the only observed-surface prediction. No GT fitting/selection.
    """
    Q=(points-origin)@frame;lo,hi=Q[:,0].min(),Q[:,0].max();bins=np.linspace(lo,hi,9)
    outputs=[]
    for scale in [.75,1.,1.25]:
        pieces=[]
        for a,b in zip(bins,bins[1:]):
            q=Q[(Q[:,0]>=a-.002)&(Q[:,0]<=b+.002)]
            if len(q)<8:continue
            umin,umax=q[:,1].min(),q[:,1].max();width=umax-umin;front=q[:,2].max();back=front-scale*width
            # Actual points are never overwritten by the hypothetical rear.
            V=np.vstack([q,[[s,u,back] for s in [a,b] for u in [umin,umax]]])
            pieces.append(trimesh.convex.convex_hull(V@frame.T+origin))
        outputs.append(dict(scale=scale,parts=pieces,role='uncertain rear closure; convex envelope is not measured geometry'))
    return outputs
