"""Private observation board: measured RGB/depth/clouds, no scene annotations."""
import json,time
from pathlib import Path
import numpy as np
from scipy import ndimage
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from native_g1_observed_regions import cloud,panel

def inspect(folder,history):
    folder=Path(folder);began=time.perf_counter()
    meta,depth,x,y,P,valid=cloud(folder);rgb=np.asarray(Image.open(folder/'rgb.png').convert('RGB'))
    cues=json.loads((folder/'candidate_cues.json').read_text()) if (folder/'candidate_cues.json').exists() else []
    cue=max(cues,key=lambda c:c['score']) if cues else None
    mask=np.load(folder/(cue['name']+'.npy')) if cue else np.zeros_like(valid)
    visible=mask&valid;points=P[visible];np.save(folder/'cue_points_world.npy',points)
    report=dict(camera=meta,candidates=cues,highest_cue=cue,partial_points_are_not_complete_geometry=True,history=history)
    try:
        r=panel(folder,mask if mask.any() else None)
        report['observed_panel']=dict(center_world_m=r['center'].tolist(),normal_world=r['normal'].tolist(),source=r['method'])
    except Exception as error:report['panel_issue']=repr(error)
    if len(points):
        center=np.median(points,axis=0);_,_,axes=np.linalg.svd(points-center,full_matrices=False)
        report['cue_geometry']=dict(points=len(points),center_world_m=center.tolist(),PCA_axes_world=axes.tolist(),observed_bounds_world_m=[points.min(0).tolist(),points.max(0).tolist()],observed_axis_extent_m=np.ptp((points-center)@axes.T,axis=0).tolist(),incomplete=bool(cue['partial']))
    overlay=rgb.copy();overlay[mask]=(.45*overlay[mask]+.55*np.array([255,150,0] if cue and cue['partial'] else [0,255,70])).astype(np.uint8)
    Image.fromarray(overlay).save(folder/'agent_mask_overlay.png')
    fig=plt.figure(figsize=(14,10));ax=fig.add_subplot(221);ax.imshow(rgb);ax.set_title('Actual wrist RGB');ax.axis('off')
    ax=fig.add_subplot(222);ax.imshow(overlay);ax.set_title('SAM3 cue: orange=partial, green=valid; no GT');ax.axis('off')
    ax=fig.add_subplot(223);d=depth.copy();d[~np.isfinite(d)|(d<=.025)|(d>=3)]=np.nan
    values=d[np.isfinite(d)];lo,hi=np.quantile(values,[.01,.99]) if len(values) else [0.,1.]
    im=ax.imshow(d,cmap='viridis',vmin=lo,vmax=hi);fig.colorbar(im,ax=ax,label='Optical-Z depth (m)');ax.set_title('Measured metric depth');ax.axis('off')
    ax=fig.add_subplot(224,projection='3d')
    # Display only: context near observed cue, or whole frame when no cue exists.
    context=valid.copy()
    if cue and cue['bbox_xyxy']:
        x0,y0,x1,y1=cue['bbox_xyxy'];pad=.25*max(x1-x0,y1-y0)+25
        context&=(x>x0-pad)&(x<x1+pad)&(y>y0-pad)&(y<y1+pad)
    Q=P[context];colors=rgb[context]/255.;step=max(1,int(np.ceil(len(Q)/12000)))
    ax.scatter(Q[::step,0],Q[::step,1],Q[::step,2],s=.5,c=colors[::step],alpha=.35)
    if len(points):
        step=max(1,int(np.ceil(len(points)/5000)));a=points[::step];ax.scatter(a[:,0],a[:,1],a[:,2],s=1,c='#ee8811')
    ax.set(xlabel='World x (m)',ylabel='World y (m)',zlabel='World z (m)',title='Measured local cloud; orange=cue; unknown rear omitted')
    ax.view_init(elev=20,azim=-70)
    fig.suptitle(folder.name+' | simulation wrist sensor; observed surfaces only',fontsize=14);fig.tight_layout();fig.savefig(folder/'agent_inspection.png',dpi=140);plt.close(fig)
    report.update(inspection_wall_s=time.perf_counter()-began,files=['rgb.png','agent_mask_overlay.png','agent_inspection.png','depth_m.npy','camera.json','cue_points_world.npy'])
    (folder/'INSPECTION.json').write_text(json.dumps(report,indent=2));return report
