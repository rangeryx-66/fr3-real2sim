"""Independent RGB-D observations, associated to accumulated measured surfaces.

No assets, handle transforms, GT masks or simulator colliders are inputs. The
frozen multi-view output packaging and scoring remain unchanged. Projection
associates observations; it never crops the new view's independently measured
surface. Per-view triangles never bridge views or hidden depth discontinuities.
"""
import json
import types
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
import native_g1_metric_rgbd_geometry as metric
import native_g1_multiview_input as frozen


def associated_patches(view, projected):
    """Grow whole new measured patches from accumulated support, not a crop.

    The inherited trusted body bypasses preserve()'s metric connectivity pass.
    Apply that same pixel-neighbor/depth-edge semantics to the complete body so
    isolated trim cannot propagate just because the semantic mask includes it.
    """
    from native_g1_observed_regions import cloud
    _,_,_,_,P,_=cloud(view['folder'])
    mask=view['mask'];H,W=mask.shape;ids=np.full(mask.shape,-1,int)
    ids[mask]=np.arange(int(mask.sum()));aa=[];bb=[]
    limit=view['audit']['observed_triangle_edge_limit_m']
    for dy,dx in [(0,1),(1,0),(1,1),(1,-1)]:
        y0=slice(0,H-dy);y1=slice(dy,H)
        x0=slice(max(0,-dx),min(W,W-dx));x1=slice(max(0,dx),min(W,W+dx))
        keep=mask[y0,x0]&mask[y1,x1]&(np.linalg.norm(P[y0,x0]-P[y1,x1],axis=2)<=limit)
        aa.append(ids[y0,x0][keep]);bb.append(ids[y1,x1][keep])
    aa=np.concatenate(aa);bb=np.concatenate(bb)
    graph=coo_matrix((np.ones(len(aa)),(aa,bb)),shape=(int(mask.sum()),int(mask.sum())))
    _,labels=connected_components(graph,directed=False)
    supported=np.unique(labels[ids[projected&mask]])
    retain=np.isin(labels,supported);chosen=mask.copy();chosen[mask]=retain
    mesh=view['mesh'].copy()
    # Vertices come directly from the same depth pixels, not an approximate
    # reconstruction. Keep only existing triangles in retained components.
    vertex_point=cKDTree(view['points']).query(mesh.vertices)[1]
    mesh.update_faces(retain[vertex_point][mesh.faces].all(1));mesh.remove_unreferenced_vertices()
    return dict(view,points=P[chosen],mask=chosen,mesh=mesh),dict(
        independent_points=len(view['points']),retained_points=int(chosen.sum()),
        unassociated_points=int(np.sum(mask&~chosen)),
        metric_edge_limit_m=limit,association='whole metric-connected patches supported by accumulated projection; no first-footprint cropping')


def observations(folders):
    views, audit = [], []
    accumulated = None
    for index, folder in enumerate(map(Path, folders)):
        view = metric.preserve(folder)  # own SAM3, depth, panel, self-exclusion
        association_pixels = None
        if accumulated is not None:
            projected = metric.projected_seed(folder, accumulated)
            association_pixels = int(np.sum(projected & view['mask']))
            if not association_pixels:
                # Keep the independent evidence for inspection, but do not
                # propagate an unassociated semantic detection into this handle.
                audit.append(dict(folder=str(folder), associated=False,
                                  independent_points=len(view['points']),
                                  reason='no depth-consistent support from accumulated observations'))
                continue
            view, patch_audit = associated_patches(view, projected)
        else:
            patch_audit = dict(independent_points=len(view['points']),retained_points=len(view['points']),unassociated_points=0)
        views.append(view)
        accumulated, _ = metric.fuse(views, frozen.SPEC['voxel_m'])
        audit.append(dict(folder=str(folder), associated=True,
                          association_pixels=association_pixels,
                          independent_points=len(view['points']),
                          patch_association=patch_audit,
                          association_source='accumulated measured geometry; inherited 5mm projection depth tolerance',
                          independent_mask_not_clipped_to_projection=True))
    return views, audit


def build_geometry(seed_folder, view_folders, output):
    folders = [Path(seed_folder), *map(Path, view_folders)]
    output = Path(output)
    views, association = observations(folders)
    by_folder = {str(Path(v['folder'])): v for v in views}

    def independent(folder, seed=None, plane=None):
        # The frozen packager asks for first-view projected seeds. Substitute
        # already associated independent observations, not a new reconstruction
        # algorithm; all output sections/rear priors/ranking stay frozen.
        return by_folder[str(Path(folder))]

    proxy = SimpleNamespace(**metric.__dict__)
    proxy.preserve = independent
    proxy.projected_seed = lambda folder, points: None
    package = types.FunctionType(frozen.build_geometry.__code__,
                                 dict(frozen.__dict__, metric=proxy))
    data = package(folders[0], folders[1:], output)
    data.update(source='independent SAM3 + metric RGB-D per view; accumulated association; open measured-patch union',
                association=association, first_view_footprint_is_not_a_crop=True)
    data['spec'] = dict(data['spec'], reference='a47fe09',
                        sensing='actuator-executed rigid wrist RGB-D',
                        fusion='independent observations with accumulated depth association')
    provenance = provenance_arrays(views, output, data)
    data['incremental_feedback'] = provenance
    (output / 'observed_geometry.json').write_text(json.dumps(data, indent=2))
    return data


def provenance_arrays(views, output, data):
    voxel = frozen.SPEC['voxel_m']
    points = np.vstack([v['points'] for v in views])
    keys, inverse = np.unique(np.floor(points / voxel).astype(np.int64), axis=0, return_inverse=True)
    # Sparse (fused voxel, observation) provenance; do not confuse voxel count
    # or raw point density with measured surface area.
    view_ids = np.concatenate([np.full(len(v['points']), i) for i, v in enumerate(views)])
    pairs = np.unique(np.column_stack((inverse, view_ids)), axis=0)
    triangle_view = np.concatenate([np.full(len(v['mesh'].faces), i) for i, v in enumerate(views)])
    np.savez_compressed(output/'measured_provenance.npz', voxel_keys=keys,
                        voxel_view_pairs=pairs, triangle_view=triangle_view)
    F = np.column_stack((data['axis_world'], np.cross(data['normal_world'], data['axis_world']), data['normal_world']))
    prefix_count = np.zeros(len(keys), dtype=int)
    prefix_sum = np.zeros((len(keys), 3))
    seen = set(); summaries = []; offset = 0
    for i, view in enumerate(views):
        own = inverse[offset:offset+len(view['points'])]; offset += len(view['points'])
        prefix_count += np.bincount(own, minlength=len(keys))
        for j in range(3):
            prefix_sum[:,j] += np.bincount(own, weights=view['points'][:,j], minlength=len(keys))
        unique = np.unique(own)
        novel = np.array([k for k in unique if int(k) not in seen], dtype=int)
        seen.update(map(int, unique))
        accumulated = points[np.isin(inverse, list(seen)) & (view_ids <= i)]
        np.save(output/('new_voxel_points_%02d.npy'%i), prefix_sum[novel]/prefix_count[novel,None])
        # Unique voxel-associated triangles are an approximate surface coverage
        # summary. Exact per-view mesh area also retained; overlap is not summed
        # into a claimed unique physical area.
        area = float(view['mesh'].area)
        summaries.append(dict(index=i, folder=view['folder'], observed_points=len(view['points']),
                              view_extent_mm=float(np.ptp(view['points']@F[:,0])*1000),
                              accumulated_extent_mm=float(np.ptp(accumulated@F[:,0])*1000),
                              newly_observed_voxels=len(novel), contributing_voxels=len(unique),
                              accumulated_voxels=len(seen), per_view_triangle_area_m2=area,
                              camera_world_pose=view['meta']['T_world_camera_optical'],
                              measured_surface_file=('single' if i==0 else 'accepted_%02d'%(i-1))+'/observed_patch.ply'))
    feedback = dict(views=summaries, voxel_m=voxel, provenance='measured_provenance.npz',
                    unknown_regions='unobserved rear and gaps remain uncertain; neither free nor solid',
                    observation_decision='Codex inspects images and incremental surfaces; no coverage threshold or automatic view policy',
                    contact_support='unchanged native-G1 contact scorer consumes all measured patches; compare candidate audit after build_geometry',
                    reachable_alternatives='evaluate_view validates Agent-requested alternatives using existing IK/collision checks')
    longitudinal=points@F[:,0];edges=np.linspace(longitudinal.min(),longitudinal.max(),9)
    bins=[]
    for lo,hi in zip(edges[:-1],edges[1:]):
        member=(longitudinal>=lo)&(longitudinal<=hi)
        contributors=np.unique(view_ids[member]).tolist()
        bins.append(dict(longitudinal_world_interval_m=[float(lo),float(hi)],
                         observation_indices=contributors,measured_points=int(member.sum()),
                         observation_diversity=len(contributors),
                         interpretation='surface samples only; multiple views do not prove unseen rear coverage'))
    feedback['longitudinal_observation_bins']=bins
    feedback['observed_normal_area_by_view']=[dict(index=i,
        area_weighted_normal_world=(v['mesh'].face_normals*v['mesh'].area_faces[:,None]).sum(0).tolist(),
        open_patch_area_m2=float(v['mesh'].area)) for i,v in enumerate(views)]
    (output/'INCREMENTAL_FEEDBACK.json').write_text(json.dumps(feedback, indent=2))
    render_feedback(views, output, data)
    return feedback


def render_feedback(views, output, data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    F = np.column_stack((data['axis_world'], np.cross(data['normal_world'], data['axis_world']), data['normal_world']))
    origin = np.asarray(data['legacy_estimate']['anchor_world_m'])
    fig, axes = plt.subplots(1, 3, figsize=(12, 5))
    for i, view in enumerate(views):
        q = (view['points']-origin)@F*1000
        axes[0].scatter(q[:,1],q[:,0],s=.5,label=Path(view['folder']).name)
        axes[1].scatter(q[:,2],q[:,0],s=.5)
    latest = np.load(output/('new_voxel_points_%02d.npy'%(len(views)-1)))
    q = (latest-origin)@F*1000
    axes[2].scatter(q[:,1],q[:,0],s=.6,color='#d76a22')
    for ax, title, xlabel in zip(axes, ['Measured surfaces by view','Measured depth profile: rear remains unknown','New voxels in latest observation'],['Transverse (mm)','Panel-normal (mm)','Transverse (mm)']):
        ax.set(title=title,xlabel=xlabel,ylabel='Longitudinal (mm)');ax.grid(alpha=.2)
    axes[0].legend(markerscale=5)
    fig.tight_layout();fig.savefig(output/'incremental_surfaces.png',dpi=150);plt.close(fig)
