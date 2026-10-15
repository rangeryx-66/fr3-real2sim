"""Independent GT-only pixel provenance diagnostic, never imported by actors."""
import argparse,json
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from xarm7_native_planning import NativeModel
from rank_prepared_handle_assets import load
from visualize_native_g1_gt_observed import distance,stats
from diagnose_incremental_wrist import accepted
from native_g1_observed_regions import cloud,panel
import native_g1_metric_rgbd_geometry as metric
import native_g1_incremental_geometry as incremental


def trace(parent,baseline,output,assembly_names):
    output.mkdir(parents=True,exist_ok=True)
    job=json.loads((parent/'job.json').read_text())
    model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
    _,parts=load(model.asset_urdf)
    selected=[p for p in parts if p[1]=='visual' and p[2] in assembly_names]
    M=[]
    for p in selected:
        m=p[3].copy();m.apply_transform(model.asset_T);M.append(m)
    assembly=trimesh.util.concatenate(M)
    P=np.load(baseline/'points_world.npy');errors=distance(assembly,P)
    bad=np.flatnonzero(errors>.003);keys=np.floor(P/.0005).astype(int)
    badkeys=set(map(tuple,keys[bad]));folders=accepted(parent/'episode')
    # Use original per-pixel voxel keys, not rounded averaged point keys.
    provenance=np.load(baseline/'measured_provenance.npz')
    badkeys=set(map(tuple,provenance['voxel_keys'][bad]))
    rows=[];viewrows=[];accumulated=[]
    fig,axes=plt.subplots(len(folders),3,figsize=(13,4*len(folders)),squeeze=False)
    for i,folder in enumerate(folders):
        meta,depth,_,_,Q,valid=cloud(folder)
        own=metric.preserve(folder);sam=metric.load_seed(folder)
        body=np.load(folder/'handle_mask.npy').astype(bool)
        view=own
        projection=np.zeros_like(sam)
        if accumulated:
            acc,_=metric.fuse(accumulated,.0005);projection=metric.projected_seed(folder,acc)
            view,_=incremental.associated_patches(own,projection)
        accumulated.append(view)
        yy,xx=np.where(view['mask']);pixelkeys=np.floor(Q[yy,xx]/.0005).astype(int)
        indices=[j for j,k in enumerate(pixelkeys) if tuple(k) in badkeys]
        local=[]
        for j in indices:
            y,x=int(yy[j]),int(xx[j]);q=Q[y,x]
            # Record neighborhood depth discontinuity using observed pixels only.
            neighborhood=Q[max(0,y-1):y+2,max(0,x-1):x+2]
            neighbor_valid=valid[max(0,y-1):y+2,max(0,x-1):x+2]
            deltas=np.linalg.norm(neighborhood[neighbor_valid]-q,axis=1)
            row=dict(view=folder.name,x=x,y=y,world_point_m=q.tolist(),depth_m=float(depth[y,x]),
                SAM_seed=bool(sam[y,x]),trusted_body=bool(body[y,x]),
                independent_preserved=bool(own['mask'][y,x]),accumulated_projection=bool(projection[y,x]),
                per_pixel_GT_error_mm=float(distance(assembly,q[None])[0]*1000),
                max_8neighbor_world_gap_mm=float(deltas.max()*1000),
                voxel_key=pixelkeys[j].tolist())
            rows.append(row);local.append(row)
        rgb=np.asarray(Image.open(folder/'rgb.png'))
        for col in range(3):axes[i,col].imshow(rgb)
        axes[i,0].contour(sam,levels=[.5],colors=['lime'],linewidths=.5)
        axes[i,1].contour(body,levels=[.5],colors=['orange'],linewidths=.5)
        axes[i,2].contour(view['mask'],levels=[.5],colors=['cyan'],linewidths=.5)
        if local:
            xs=[r['x'] for r in local];ys=[r['y'] for r in local]
            for ax in axes[i]:
                ax.scatter(xs,ys,s=22,facecolors='none',edgecolors='red')
                ax.set_xlim(min(xs)-25,max(xs)+25);ax.set_ylim(max(ys)+25,min(ys)-25)
        else:
            ys,xs=np.where(sam)
            for ax in axes[i]:ax.set_xlim(xs.min()-10,xs.max()+10);ax.set_ylim(ys.max()+10,ys.min()-10)
        for ax,title in zip(axes[i],['Own SAM seed','Trusted body','Associated independent surface']):
            ax.set_title(folder.name+' / '+title);ax.set_xlabel('image x');ax.set_ylabel('image y')
        viewrows.append(dict(view=folder.name,bad_voxel_pixel_contributors=len(local),
            seed_contributors=sum(r['SAM_seed'] for r in local),body_contributors=sum(r['trusted_body'] for r in local),
            association_only_additions=0,independent_preserved_points=len(own['points'])))
    fig.tight_layout();fig.savefig(output/'PRIVATE_pixel_trace.png',dpi=170);plt.close(fig)
    # Are these voxels represented by measured triangles, or only cloud samples?
    mesh=trimesh.load(baseline/'observed_patches.ply',process=False)
    vertexkeys=np.floor(mesh.vertices/.0005).astype(int)
    affected=np.array([tuple(k) in badkeys for k in vertexkeys])
    result=dict(bad_fused_voxel_count=len(bad),bad_fused_world_points=P[bad].tolist(),
                bad_fused_errors_mm=(errors[bad]*1000).tolist(),pixel_contributors=rows,
                views=viewrows,measured_triangles_touching_bad_voxels=int(np.sum(affected[mesh.faces].any(1))),
                evaluation_only_assembly=assembly_names,GT_use='independent pixel diagnosis only; never executable filtering')
    (output/'PIXEL_TRACE.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--parent',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--assembly',nargs='+',required=True);a=p.parse_args()
    trace(a.parent,a.baseline,a.output,a.assembly)
