"""Read-only GT/observed/contact diagnostic. No planner or Isaac execution.

GT is used only here for evaluation and plotting, never as an executable pose.
Original RGB-D, masks, geometry, plans and telemetry are opened read-only.
"""
from pathlib import Path
import argparse
import hashlib
import json
import time
import gzip
import pickle
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import fcl
from xarm7_native_planning import NativeModel, PAD_FRAMES
from revolute10_fixed import authored_handle_colliders
from handle_contact_quality import handle_surfaces
from piper_mobile_demo.owned_scene import Shape, meshes

COL = dict(gt='#888888', native='#278449', observed='#167bb5', inferred='#ed8b25',
           left='#2878bd', right='#bd3689', actual='#151515')
LABELS = ['gripper_link1', 'gripper_link2']
plt.rcParams.update({'font.size':8,'axes.titlesize':9,'axes.labelsize':8,
                     'xtick.labelsize':7,'ytick.labelsize':7,'legend.fontsize':6})


def stats(values):
    v = np.asarray(values)
    v = v[np.isfinite(v)]
    if not len(v):
        return dict(n=0)
    return dict(n=len(v), mean_mm=float(v.mean()*1000), median_mm=float(np.median(v)*1000),
                p90_mm=float(np.quantile(v,.90)*1000), p95_mm=float(np.quantile(v,.95)*1000))


def sample(mesh, count, seed=61):
    return trimesh.sample.sample_surface(mesh, count, seed=seed)[0]


def distance(mesh, points):
    # Bound temporary nearest-triangle arrays; no change to full source mesh.
    return np.concatenate([trimesh.proximity.closest_point(mesh, part)[1]
                           for part in np.array_split(points, max(1,int(np.ceil(len(points)/2000))))])


def pca(points):
    return np.linalg.svd(points-points.mean(0),full_matrices=False)[2][0]


def angle(a,b):
    return float(np.rad2deg(np.arccos(np.clip(abs(a@b)/np.linalg.norm(a)/np.linalg.norm(b),0,1))))


def unproject(depth, mask, K, C):
    yy,xx = np.where(mask & np.isfinite(depth) & (depth>.02) & (depth<5))
    z = depth[yy,xx]
    return np.c_[(xx-K[0,2])*z/K[0,0],(yy-K[1,2])*z/K[1,1],z]@C[:3,:3].T+C[:3,3]


def project(points, K, C, depth):
    q = (points-C[:3,3])@C[:3,:3]
    uv = q@K.T
    uv = uv[:,:2]/np.maximum(q[:,2,None],1e-12)
    xy = np.rint(uv).astype(int)
    H,W = depth.shape
    inside = (q[:,2]>.02)&(xy[:,0]>=0)&(xy[:,0]<W)&(xy[:,1]>=0)&(xy[:,1]<H)
    z = np.full(len(q),np.nan)
    z[inside] = depth[xy[inside,1],xy[inside,0]]
    valid = inside & np.isfinite(z)&(z>.02)&(z<5)
    return xy, valid, q[:,2]-z


def external_loft(data, parts):
    # Adjacent rectangular loft pieces share end caps. Exclude those internal
    # caps for surface-error statistics, without changing any execution mesh.
    axis = np.array(data['axis_world']); result=[]
    for i,p in enumerate(parts):
        m=trimesh.Trimesh(p['vertices'],p['faces'],process=False)
        s=m.vertices@axis;tri=s[m.faces];flat=np.ptp(tri,axis=1)<1e-8
        remove=flat&(((abs(tri.mean(1)-s.min())<1e-8)&(i>0))|
                     ((abs(tri.mean(1)-s.max())<1e-8)&(i<len(parts)-1)))
        result.append(trimesh.Trimesh(m.vertices,m.faces[~remove],process=False))
    return trimesh.util.concatenate(result)


def finish(fig, path):
    fig.subplots_adjust(wspace=.32,top=.82,bottom=.15)
    fig.savefig(path,dpi=180,bbox_inches='tight',facecolor='white')
    plt.close(fig)


def mesh3(ax, mesh, convert, color, alpha=.18):
    faces=mesh.triangles
    # Display only: subsampling does not enter numerical diagnostics.
    if len(faces)>12000:faces=faces[np.linspace(0,len(faces)-1,12000,dtype=int)]
    artist=Poly3DCollection(convert(faces),facecolor=color,edgecolor=color,
                            linewidths=.15,alpha=alpha)
    ax.add_collection3d(artist)


def view3(ax, xyz, title):
    lo=xyz.min(0);hi=xyz.max(0);pad=np.maximum((hi-lo)*.05,2)
    ax.set(xlim=(lo[0]-pad[0],hi[0]+pad[0]),ylim=(lo[1]-pad[1],hi[1]+pad[1]),
           zlim=(lo[2]-pad[2],hi[2]+pad[2]),xlabel='L (mm)',ylabel='W (mm)',zlabel='N (mm)',title=title)
    ax.set_box_aspect(np.maximum(hi-lo,12));ax.view_init(elev=24,azim=-62)
    for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:axis.set_major_locator(MaxNLocator(3))
    # Metric cross-sections carry the short-axis scales. Avoid illegible W/N
    # ticks on the proportionally rendered, very long handle overview.
    ax.set_yticks([]);ax.set_zticks([])
    ax.tick_params(pad=0,labelsize=6)
    ax.xaxis.labelpad=0;ax.yaxis.labelpad=0;ax.zaxis.labelpad=0


def section(ax, mesh, at, axis, convert, color, label, **kwargs):
    path=mesh.section(plane_origin=at,plane_normal=axis)
    if path is None:return []
    curves=path.discrete
    for i,curve in enumerate(curves):
        q=convert(curve);ax.plot(q[:,1],q[:,2],color=color,label=label if i==0 else None,**kwargs)
    return curves


def frames(model, T, base, width):
    p=model.poses(model.home,base,width=float(width));D=T@np.linalg.inv(p['tcp_link'])
    return {n:D@p[n] for n in LABELS}


def scan_closing(model,T,base,groups,finger_meshes=None):
    widths=np.linspace(.084,.001641,41);answer={}
    robot={n:Shape((finger_meshes or model.robot_hulls)[n],np.eye(4),True,n,n) for n in LABELS}
    for name,group in groups.items():
        solids=[Shape(m,np.eye(4),name!='GT_visual',str(i),'target') for i,m in enumerate(group)]
        first={};hits=[]
        for w in widths:
            F=frames(model,T,base,w);row=[]
            for n in LABELS:
                robot[n].place(F[n]);collisions=[]
                for s in solids:
                    if np.any(robot[n].bounds[0]>s.bounds[1]) or np.any(s.bounds[0]>robot[n].bounds[1]):continue
                    r=fcl.CollisionResult();fcl.collide(robot[n].object,s.object,fcl.CollisionRequest(num_max_contacts=16,enable_contact=True),r)
                    collisions.extend(r.contacts)
                row.append(bool(collisions))
                if collisions and n not in first:
                    first[n]=dict(aperture_mm=float(w*1000),world_contact_m=np.asarray(collisions[0].pos).tolist())
            hits.append(row)
        answer[name]=dict(first=first,intersections=hits)
    return widths,answer


def actual_contact_local(model, folder, actual):
    """Locate archived contacts in measured pre-integration finger frames.

    A point inside the nominal pad bounds does not measure a pressure area.
    Only trusted, privately archived telemetry pickle files are loaded.
    """
    wanted={n:r for n,r in actual['first_actual_positive_normal_contact'].items() if r}
    found={};chunks=folder/'physics_steps_chunks'
    for chunk in json.loads((chunks/'index.json').read_text())['chunks']:
        path=chunks/chunk['file']
        with gzip.open(path,'rb') as f:rows=pickle.load(f)
        for n,c in wanted.items():
            if n in found:continue
            row=next((r for r in rows if abs(r['t']-c['t_s'])<1e-6),None)
            if row is None:continue
            X=np.linalg.inv(np.asarray(row['finger_world_poses'][n]));b=model.pads[n]
            local=np.asarray([(X@np.r_[r['contact_point_world_m'],1])[:3] for r in c['contacts']])
            found[n]=dict(time_s=c['t_s'],geometry_pose_time=row.get('geometry_pose_time'),
                pad_bounds_mm=(b*1000).tolist(),contact_local_mm=(local*1000).tolist(),
                lateral_edge_margin_mm=(np.minimum(local[:,0]-b[0,0],b[1,0]-local[:,0])*1000).tolist(),
                longitudinal_edge_margin_mm=(np.minimum(local[:,1]-b[0,1],b[1,1]-local[:,1])*1000).tolist(),
                distance_to_pad_plane_mm=(abs(local[:,2]-b[0,2])*1000).tolist(),
                telemetry_chunk=chunk['file'],telemetry_chunk_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        if len(found)==len(wanted):break
    return found


def main(args):
    B=args.root;O=args.output;O.mkdir(exist_ok=True,parents=True)
    contacts=json.loads((B/'CONTACT_MISMATCH.json').read_text());rows=[]
    for asset in ['45671','45600','45606']:
        began=time.time();case=B/asset;p=case/'physical_01';plan=json.loads((p/'visual_plan.json').read_text())
        folders=list((p/'rgbd').glob('initial*/observed_geometry.json'))
        # Each executed case has one successful observed model. No new model is generated.
        if len(folders)!=1:raise ValueError('Ambiguous saved executed observation: '+asset)
        folder=folders[0].parent;data=json.loads(folders[0].read_text());P=np.load(folder/'handle_points_world.npy')
        meta=json.loads((folder/'camera.json').read_text());K=np.array(meta['K']);C=np.array(meta['T_world_camera_optical']);depth=np.load(folder/'depth_m.npy')
        rawmask=np.load(folder/'sam3_mask_raw.npy').astype(bool);seed=np.load(folder/'sam3_seed_mask.npy').astype(bool);final=np.load(folder/'handle_mask.npy').astype(bool)
        job=json.loads((case/'job.json').read_text());model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
        export=json.loads((p/'cooked_initial.json').read_text());selection=job['grasp_geometry_candidates'][0];allowed=authored_handle_colliders(export,selection)
        gt,native=handle_surfaces(model,export,allowed,selection);G=sample(gt,30000);L=pca(G)
        if L@np.array(data['axis_world'])<0:L=-L
        N=model.asset_T[:3,:3]@np.array(selection['outward_normal_root']);N-=L*(N@L);N/=np.linalg.norm(N);W=np.cross(N,L);F=np.column_stack((L,W,N));center=G.mean(0)
        convert=lambda x:(np.asarray(x)-center)@F*1000
        g=convert(G);q=convert(P);clouds={'SAM3_raw':unproject(depth,rawmask,K,C),'self_excluded_seed':unproject(depth,seed,K,C),'completed_mask_before_estimator_trim':unproject(depth,final,K,C),'retained_points':P}
        cloud_metrics={name:dict(distance_to_GT_visual=stats(distance(gt,x)),full_extent_LW_mm=np.ptp(convert(x),axis=0)[:2].tolist(),p05_p95_extent_LW_mm=np.ptp(np.quantile(convert(x),[.05,.95],axis=0),axis=0)[:2].tolist(),axis_error_deg=angle(pca(x),L)) for name,x in clouds.items()}
        xy,valid,dz=project(G,K,C,depth);visibility={};visible=None
        for tol in [.001,.002,.004]:
            vis=valid&(abs(dz)<=tol);uv=xy[vis];cover=lambda mask:float(mask[uv[:,1],uv[:,0]].mean()) if len(uv) else None
            visibility[str(tol)]=dict(depth_consistent_fraction=float(vis.mean()),behind_measured_depth_fraction=float((valid&(dz>tol)).mean()),front_of_measured_depth_fraction=float((valid&(dz<-tol)).mean()),SAM3_raw_coverage=cover(rawmask),seed_coverage=cover(seed),final_mask_coverage=cover(final))
            if tol==.002:visible=vis
        detection_context=[];captures=sorted((p/'rgbd').glob('initial*/visual_detection_attempts.json'))
        fig,axes=plt.subplots(1,len(captures),figsize=(7*len(captures),5),squeeze=False)
        for ax,record in zip(axes[0],captures):
            cmeta=json.loads((record.parent/'camera.json').read_text());cd=np.load(record.parent/'depth_m.npy');px,ok,dd=project(G,np.array(cmeta['K']),np.array(cmeta['T_world_camera_optical']),cd);vis=ok&(abs(dd)<=.002);uv=px[vis];audit=json.loads(record.read_text());attempts=[]
            ax.imshow(np.zeros(cd.shape),cmap='gray',vmin=0,vmax=1);ax.scatter(uv[:,0],uv[:,1],s=1,c='white',alpha=.5)
            for trial in audit['attempts']:
                box=trial.get('crop_xyxy');inside=None
                if box is not None:
                    x0,y0,x1,y1=box;inside=float(((uv[:,0]>=x0)&(uv[:,0]<x1)&(uv[:,1]>=y0)&(uv[:,1]<y1)).mean()) if len(uv) else None
                    if trial.get('kind')=='observed_depth_crop':ax.add_patch(plt.Rectangle((x0,y0),x1-x0,y1-y0,fill=False,color=COL['inferred'],linewidth=2))
                attempts.append(dict(kind=trial.get('kind'),raw_max=trial.get('raw_max'),accepted=trial.get('accepted'),GT_visible_sample_fraction_in_ROI=inside))
            for name,color in [('sam3_seed_mask.npy','#ab69cc'),('handle_mask.npy','#2ab080')]:
                if (record.parent/name).exists():ax.contour(np.load(record.parent/name),levels=[.5],colors=[color],linewidths=1)
            ax.set(title=record.parent.name+' | GT visible white; seed purple; final green',xlabel='original pixel x',ylabel='original pixel y');detection_context.append(dict(capture=record.parent.name,attempts=attempts))
        fig.suptitle(asset+' | observation/crop context, no raw RGB published');finish(fig,O/(asset+'_segmentation.png'))
        observed_extent_visible=np.ptp(g[visible],axis=0)[:2].tolist() if visible.any() else None
        nominal=external_loft(data,data['parts']);nearest=cKDTree(P);hyp_metrics=[];hmeshes=[]
        for hyp in data['hidden_geometry_hypotheses']:
            mesh=external_loft(data,hyp['parts']);hmeshes.append(mesh);S=sample(mesh,8000);_,v,z=project(S,K,C,depth);near=nearest.query(S)[0]<=.002
            labels={'observed_supported':v&(abs(z)<=.002)&near,'depth_visible_not_in_retained_cloud':v&(abs(z)<=.002)&~near,'inferred_occluded':v&(z>.002),'inferred_front_of_measured_surface':v&(z<-.002),'unassessable':~v};dist=distance(gt,S)
            hyp_metrics.append(dict(scale=hyp['hidden_depth_scale'],all_external_loft_surface=stats(dist),regions={k:stats(dist[m])|dict(fraction=float(m.mean())) for k,m in labels.items()}))
        T=np.array(json.loads((p/'visual_correction.json').read_text())['T_after']);width=plan['contact_geometry_audit']['robust_contact']['chosen_expected_aperture_m'];poses=frames(model,T,plan['base'],width)
        padcenters={n:(poses[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in LABELS};grasp_center=np.mean(list(padcenters.values()),axis=0)
        curves=gt.section(plane_origin=grasp_center,plane_normal=L);section_points=np.vstack(curves.discrete) if curves is not None else None
        local_center=None
        if section_points is not None:
            s=convert(section_points);c=(s.min(0)+s.max(0))/2;c[0]=convert(grasp_center)[0];local_center=center+F@(c/1000)
        refline=center+L*((grasp_center-center)@L);grasp_offsets=dict(functional_center_world_m=grasp_center.tolist(),global_PCA_line_offset_mm=float(np.linalg.norm(grasp_center-refline)*1000),local_section_envelope_offset_mm=float(np.linalg.norm(grasp_center-local_center)*1000) if local_center is not None else None,local_section_offset_WN_mm=((grasp_center-local_center)@F*1000)[1:].tolist() if local_center is not None else None,center_definition='midpoint of both inner pad patch centroids at common aperture; local GT centerline is section-envelope midpoint, not a saved Oracle grasp')
        widths,scan=scan_closing(model,T,plan['base'],{'observed_loft':[trimesh.Trimesh(x['vertices'],x['faces'],process=False) for x in data['parts']],'GT_visual':[gt],'GT_native':native})
        actual=contacts[asset]['updated_visual'];pred=plan['contact_geometry_audit']['first_contacts'];predpoints={}
        for n,c in pred.items():
            f=frames(model,T,plan['base'],c['width_m'])[n];predpoints[n]=(f@np.r_[c['local_contact_m'],1])[:3]
        finger_check={};cooked_fingers={}
        for physical,logical in [('left_finger',LABELS[0]),('right_finger',LABELS[1])]:
            chunks=[]
            for shape in export['shapes']:
                if shape.get('rigid_body_path','').split('/')[-1]!=physical:continue
                X=np.linalg.inv(np.asarray(shape['rigid_body_world_transform'])@PAD_FRAMES[physical])@np.asarray(shape['world_transform'])
                for m in meshes(shape):m=m.copy();m.apply_transform(X);chunks.append(m)
            if chunks:
                cooked=trimesh.util.concatenate(chunks);finger_check[logical]=dict(cooked_parts=len(chunks),planner_vertices_to_cooked=stats(distance(cooked,model.robot_hulls[logical].vertices)),cooked_vertices_to_planner=stats(distance(model.robot_hulls[logical],cooked.vertices)))
                if len(chunks)==1:cooked_fingers[logical]=cooked
        cooked_scan=scan_closing(model,T,plan['base'],{'GT_native':native},cooked_fingers)[1] if len(cooked_fingers)==2 else None
        pc_dist=distance(gt,P);native_pc=np.min([distance(m,P) for m in native],axis=0)
        # Projected masks: show only derived masks/GT samples, never publish raw RGB.
        fig=plt.figure(figsize=(17,5));ax=fig.add_subplot(131,projection='3d');mesh3(ax,gt,convert,COL['gt'],.10);ax.scatter(*q.T,c=pc_dist*1000,cmap='viridis',norm=Normalize(0,4),s=3);view3(ax,g,'GT surface + retained RGB-D points');fig.colorbar(ScalarMappable(norm=Normalize(0,4),cmap='viridis'),ax=ax,orientation='horizontal',fraction=.05,pad=.12,shrink=.85,label='point-to-GT distance (mm; clipped at 4)')
        ax=fig.add_subplot(132);ax.scatter(g[visible,0],g[visible,1],s=1,c='#bbbbbb',label='depth-consistent GT');ax.scatter(q[:,0],q[:,1],s=3,c=pc_dist*1000,cmap='viridis',vmin=0,vmax=4);ax.set(xlabel='L (mm)',ylabel='W (mm)',title='Visible extent (same GT frame)');ax.set_aspect('equal',adjustable='box')
        ax=fig.add_subplot(133);uy,ux=np.where(rawmask|final);vxy=xy[visible];lo=np.minimum([ux.min(),uy.min()],vxy.min(0))-10;hi=np.maximum([ux.max(),uy.max()],vxy.max(0))+10
        ax.imshow(np.zeros(depth.shape),cmap='gray',vmin=0,vmax=1);ax.contour(rawmask,levels=[.5],colors=['#ab69cc'],linewidths=1);ax.contour(final,levels=[.5],colors=['#2ab080'],linewidths=1);ax.scatter(vxy[:,0],vxy[:,1],s=.5,c='#dddddd',alpha=.35);ax.set(xlim=(lo[0],hi[0]),ylim=(hi[1],lo[1]),title='GT visible samples / SAM3 purple / final green',xlabel='original pixel x',ylabel='original pixel y');fig.suptitle(asset+' | OFFLINE GT EVALUATION ONLY');finish(fig,O/(asset+'_observed.png'))
        fig=plt.figure(figsize=(16,5));ax=fig.add_subplot(131,projection='3d');mesh3(ax,gt,convert,COL['gt'],.10);mesh3(ax,nominal,convert,COL['inferred'],.18);ax.scatter(*q.T,c=COL['observed'],s=2);view3(ax,np.vstack([g,q]),'GT gray / inferred loft orange / observations blue')
        ax=fig.add_subplot(132);ax.scatter(g[:,0],g[:,2],s=.3,c='#bbbbbb',label='GT full surface');ax.scatter(convert(sample(nominal,5000))[:,0],convert(sample(nominal,5000))[:,2],s=.5,c=COL['inferred'],label='nominal inferred surface');ax.scatter(q[:,0],q[:,2],s=2,c=COL['observed'],label='measured points');ax.set(xlabel='L (mm)',ylabel='N (mm)',title='Longitudinal / depth extent');ax.legend(fontsize=7)
        ax=fig.add_subplot(133);section(ax,gt,grasp_center,L,convert,COL['gt'],'GT visual',linewidth=2)
        for i,m in enumerate(native):section(ax,m,grasp_center,L,convert,COL['native'],'GT native' if i==0 else None,linestyle='--')
        for scale,m in zip([.75,1,1.25],hmeshes):section(ax,m,grasp_center,L,convert,COL['inferred'],f'inferred depth {scale}x',alpha=.5+scale/4)
        local=abs((P-grasp_center)@L)<.008;ax.scatter(q[local,1],q[local,2],s=8,c=COL['observed'],label='points in +/-8 mm L slab');gc=convert(grasp_center);ax.scatter(gc[1],gc[2],marker='+',s=100,c='black',label='functional grasp center');ax.set(xlabel='W (mm)',ylabel='N (mm)',title='Local grasp cross-section');ax.axis('equal');ax.legend(fontsize=6);fig.suptitle(asset+' | points are observed; every loft face is reconstructed/assumed');finish(fig,O/(asset+'_completion.png'))
        fig=plt.figure(figsize=(17,5));a1=fig.add_subplot(131,projection='3d');a2=fig.add_subplot(132);shown=[]
        for ax in [a1]:
            mesh3(ax,gt,convert,COL['gt'],.12)
            for m in native:mesh3(ax,m,convert,COL['native'],.06)
            for n,color in zip(LABELS,[COL['left'],COL['right']]):
                m=model.robot_hulls[n].copy();m.apply_transform(poses[n]);mesh3(ax,m,convert,color,.28);shown.append(convert(m.vertices))
                if n in predpoints:z=convert(predpoints[n]);ax.scatter(*z,marker='*',s=140,c=color,edgecolors='black')
                c=actual.get('first_actual_positive_normal_contact',{}).get(n)
                if c:
                    for contact in c['contacts']:
                        z=convert(contact['contact_point_world_m']);ax.scatter(*z,c='black',marker='s',s=25)
            z=convert(grasp_center);ax.scatter(*z,marker='+',s=80,c='black')
        view3(a1,np.vstack([g]+shown),'Both native fingers at predicted common aperture')
        section(a2,gt,grasp_center,L,convert,COL['gt'],'GT visual section',linewidth=2)
        for i,m in enumerate(native):section(a2,m,grasp_center,L,convert,COL['native'],'GT native section' if i==0 else None,linestyle='--')
        for n,color in zip(LABELS,[COL['left'],COL['right']]):
            m=model.robot_hulls[n].copy();m.apply_transform(poses[n]);v=convert(m.vertices)[:,1:];h=__import__('scipy').spatial.ConvexHull(v);v=v[h.vertices];a2.fill(v[:,0],v[:,1],color=color,alpha=.2,label=('left' if n==LABELS[0] else 'right')+' finger projection')
            z=convert(predpoints[n]);a2.scatter(z[1],z[2],c=color,marker='*',s=100,edgecolors='black',label='predicted first '+('L' if n==LABELS[0] else 'R'))
            c=actual['first_actual_positive_normal_contact'].get(n)
            if c:
                for i,item in enumerate(c['contacts']):
                    z=convert(item['contact_point_world_m']);a2.scatter(z[1],z[2],c='black',marker='s',s=20,label='actual first '+('L' if n==LABELS[0] else 'R') if i==0 else None)
        a2.set(xlabel='W (mm)',ylabel='N (mm)',title='W-N projection: stars predicted / squares actual\nFinger silhouettes at common aperture; overlap is not collision');a2.set_aspect('equal',adjustable='box');a2.legend(loc='upper right',fontsize=5)
        ax=fig.add_subplot(133)
        plot_scan=dict(scan)
        if cooked_scan:plot_scan['GT_native_cooked_fingers']=cooked_scan['GT_native']
        for j,(name,c) in enumerate(zip(plot_scan,['#e18c24','#888888','#278449','#8b4c29'])):
            for k,n in enumerate(LABELS):
                hit=np.array(plot_scan[name]['intersections'])[:,k];ax.plot(widths*1000,j*2+k+hit*.7,color=c,linestyle='-' if k==0 else '--',label=name+(' L' if k==0 else ' R'))
        ax.axvline(width*1000,c='purple',linewidth=1,label='chosen common aperture')
        for n in LABELS:
            c=actual.get('first_actual_positive_normal_contact',{}).get(n)
            if c:ax.axvline(c['measured_aperture_mm'],c='black',linestyle=':' if n==LABELS[0] else '-.',label='actual first '+('L' if n==LABELS[0] else 'R'))
        ax.invert_xaxis();ax.set(xlabel='closing aperture (mm), open to closed',ylabel='group / finger (+0.7 = intersection)',title='Same frozen pose, offline geometry substitution');ax.legend(fontsize=6);fig.suptitle(asset+f' | common aperture {width*1000:.3f} mm | no executable pose changed');finish(fig,O/(asset+'_grasp.png'))
        sensitivity=visibility;row=dict(asset_id=asset,GT_OFFLINE_ONLY=True,selected_observation=folder.name,clouds=cloud_metrics,GT_full_extent_LWN_mm=np.ptp(convert(gt.vertices),axis=0).tolist(),GT_depth_consistent_extent_LW_mm=observed_extent_visible,retained_axis_vs_GT_PCA_deg=angle(pca(P),L),executed_estimated_axis_vs_GT_PCA_deg=angle(np.array(data['axis_world']),L),visibility_sensitivity=sensitivity,detection_context=detection_context,completion_hypotheses=hyp_metrics,grasp_center=grasp_offsets,chosen_common_aperture_mm=width*1000,stored_first_contact_predictions=pred,offline_fixed_pose_closing=scan,closing_grid_step_mm=float((widths[0]-widths[1])*1000),actual_first_contact={n:None if c is None else {k:c[k] for k in ['t_s','phase','measured_aperture_mm','commanded_aperture_mm','normal_load_n']} for n,c in actual['first_actual_positive_normal_contact'].items()},native_finger_geometry_consistency=finger_check,retained_points_to_native_proxy=stats(native_pc),GT_visual_to_native_proxy=stats(np.min([distance(m,G) for m in native],axis=0)),wall_s=time.time()-began,input_sha256={str(f.relative_to(B)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [folder/'rgb.png',folder/'depth_m.npy',folder/'camera.json',folder/'sam3_mask_raw.npy',folder/'sam3_seed_mask.npy',folder/'handle_mask.npy',folder/'handle_points_world.npy',folder/'observed_geometry.json',p/'visual_plan.json',p/'visual_correction.json',p/'cooked_initial.json']})
        row['GT_native_to_visual_surface']=stats(distance(gt,sample(trimesh.util.concatenate(native),30000)))
        row['fixed_pose_actual_cooked_fingers']=cooked_scan
        row['archived_first_contact_local']=actual_contact_local(model,p,actual)
        rows.append(row);(O/'METRICS.json').write_text(json.dumps(rows,indent=2));print(asset,'DONE',round(row['wall_s'],1),'s',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
