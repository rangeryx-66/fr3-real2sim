"""Keep measured boundary geometry, separate uncertain handle association.

RGB-D and SAM evidence only. Existing trusted body is preserved. End/contour
extensions need semantic support and must not contradict a depth-consistent
other view. Occlusion or absence of overlap is not negative evidence. All
measured points/patches survive in contextual outputs; uncertainty is not free
space. Frozen candidate families, weights and controllers are unchanged.
"""
import json,types
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh
from scipy import ndimage
from scipy.spatial import cKDTree
import native_g1_incremental_geometry as previous
import native_g1_multiview_input as frozen
import native_g1_metric_rgbd_geometry as metric
from native_g1_observed_regions import cloud


def other_evidence(points,folder):
    meta,depth,_,_,_,valid=cloud(folder)
    C=np.asarray(meta['T_world_camera_optical']);K=np.asarray(meta['K'])
    q=(points-C[:3,3])@C[:3,:3];uv=q@K.T
    xy=np.rint(uv[:,:2]/np.maximum(q[:,2,None],1e-8)).astype(int)
    H,W=depth.shape
    inside=(q[:,2]>.02)&(xy[:,0]>=0)&(xy[:,0]<W)&(xy[:,1]>=0)&(xy[:,1]<H)
    ids=np.flatnonzero(inside);x,y=xy[ids].T
    same=valid[y,x]&(abs(depth[y,x]-q[ids,2])<.005)  # inherited projection tolerance
    ids=ids[same];x,y=xy[ids].T
    seed=metric.load_seed(folder)
    neighborhood=ndimage.binary_dilation(seed,iterations=2)  # inherited contour neighborhood
    positive=np.zeros(len(points),bool);negative=positive.copy()
    positive[ids]=seed[y,x];negative[ids]=~neighborhood[y,x]
    return positive,negative


def semantic_views(folders):
    views,association=previous.observations(folders)
    supported=[];audit=[];evidence=[]
    for i,view in enumerate(views):
        folder=Path(view['folder']);yy,xx=np.where(view['mask'])
        body=np.load(folder/'handle_mask.npy').astype(bool)[yy,xx]
        own=metric.load_seed(folder)[yy,xx]
        positive=np.zeros(len(xx),bool);negative=positive.copy()
        for j,other in enumerate(views):
            if j==i:continue
            yes,no=other_evidence(view['points'],other['folder'])
            positive|=yes;negative|=no
        usable=body|((own|positive)&~negative)
        vertices=cKDTree(view['points']).query(view['mesh'].vertices)[1]
        mesh=view['mesh'].copy();mesh.update_faces(usable[vertices][mesh.faces].all(1));mesh.remove_unreferenced_vertices()
        supported.append(dict(view,mesh=mesh))
        evidence.append(dict(view=i,y=yy,x=xx,body=body,own_SAM=own,
                             other_SAM=positive,other_negative=negative,contact_supported=usable))
        audit.append(dict(index=i,folder=str(folder),all_measured_points=len(xx),
            trusted_body_points=int(body.sum()),extension_points=int((~body).sum()),
            contact_supported_points=int(usable.sum()),uncertain_association_points=int((~usable).sum()),
            extension_with_other_positive=int((~body&positive).sum()),
            extension_with_other_negative=int((~body&negative).sum()),
            measured_triangles=len(view['mesh'].faces),contact_supported_triangles=len(mesh.faces)))
    return views,supported,association,evidence,audit


def build_geometry(seed_folder,view_folders,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    folders=[Path(seed_folder),*map(Path,view_folders)]
    views,supported,association,evidence,audit=semantic_views(folders)
    by_folder={str(Path(v['folder'])):v for v in supported}
    contact_points=np.vstack([v['points'][e['contact_supported']] for v,e in zip(views,evidence)])
    profile_points,_=metric.fuse([dict(v,points=v['points'][e['contact_supported']])
                                  for v,e in zip(supported,evidence)],frozen.SPEC['voxel_m'])
    proxy=SimpleNamespace(**metric.__dict__)
    proxy.preserve=lambda folder,seed=None,plane=None:by_folder[str(Path(folder))]
    proxy.projected_seed=lambda folder,points:None
    proxy.uncertain_profiles=lambda points,frame,origin:metric.uncertain_profiles(profile_points,frame,origin)
    package=types.FunctionType(frozen.build_geometry.__code__,dict(frozen.__dict__,metric=proxy))
    data=package(folders[0],folders[1:],output)
    trimesh.util.concatenate([v['mesh'] for v in views]).export(output/'all_measured_patches.ply')
    np.savez_compressed(output/'all_measured_triangle_provenance.npz',
        triangle_view=np.concatenate([np.full(len(v['mesh'].faces),i) for i,v in enumerate(views)]))
    uncertain=[]
    for v,e in zip(views,evidence):
        idx=cKDTree(v['points']).query(v['mesh'].vertices)[1]
        m=v['mesh'].copy();m.update_faces((~e['contact_supported'][idx])[m.faces].any(1));m.remove_unreferenced_vertices();uncertain.append(m)
    uncertain=[m for m in uncertain if len(m.faces)]
    uncertain_mesh=(trimesh.util.concatenate(uncertain) if uncertain else
        trimesh.Trimesh(np.empty((0,3)),np.empty((0,3),dtype=int),process=False))
    uncertain_mesh.export(output/'uncertain_association_patches.ply')
    np.save(output/'contact_supported_points_world.npy',contact_points)
    arrays={key:np.concatenate([np.full(len(e['x']),e['view']) if key=='view' else e[key] for e in evidence])
            for key in ['view','y','x','body','own_SAM','other_SAM','other_negative','contact_supported']}
    np.savez_compressed(output/'semantic_pixel_provenance.npz',**arrays)
    data.update(source='independent wrist observations; measured context retained; uncertain semantic extensions separated from contact support',
                association=association,semantic_association=audit,
                all_measured_surfaces='all_measured_patches.ply',
                uncertain_association_surfaces='uncertain_association_patches.ply',
                contact_scoring_surfaces='observed_patches.ply; supported patches only; no GT surface',
                first_view_footprint_is_not_a_crop=True)
    data['spec']=dict(data['spec'],sensing='actuator-executed wrist RGB-D',reference='db7ccaa')
    data['incremental_feedback']=previous.provenance_arrays(supported,output,data)
    data['incremental_feedback']['semantic_association']=audit
    data['incremental_feedback']['unknown_regions']='unseen rear AND measured semantic boundary ambiguity; neither confirmed handle contact nor free space'
    (output/'INCREMENTAL_FEEDBACK.json').write_text(json.dumps(data['incremental_feedback'],indent=2))
    (output/'observed_geometry.json').write_text(json.dumps(data,indent=2))
    render_semantics(views,evidence,output,data)
    return data


def render_semantics(views,evidence,output,data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    F=np.column_stack((data['axis_world'],np.cross(data['normal_world'],data['axis_world']),data['normal_world']))
    origin=np.asarray(data['legacy_estimate']['anchor_world_m'])
    fig,axes=plt.subplots(1,2,figsize=(10,5))
    for view,e in zip(views,evidence):
        Q=(view['points']-origin)@F*1000;ok=e['contact_supported']
        axes[0].scatter(Q[ok,1],Q[ok,0],s=.5,color='#308f70')
        axes[0].scatter(Q[~ok,1],Q[~ok,0],s=2,color='#d38429')
        axes[1].scatter(Q[ok,2],Q[ok,0],s=.5,color='#308f70')
        axes[1].scatter(Q[~ok,2],Q[~ok,0],s=2,color='#d38429')
    for ax,xlabel in zip(axes,['Transverse (mm)','Panel-normal (mm)']):
        ax.set(xlabel=xlabel,ylabel='Longitudinal (mm)');ax.grid(alpha=.2)
    fig.suptitle('Green: supported handle contact | Orange: measured semantic ambiguity, preserved')
    fig.tight_layout();fig.savefig(output/'semantic_surfaces.png',dpi=150);plt.close(fig)
