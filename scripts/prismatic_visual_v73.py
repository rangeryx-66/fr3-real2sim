"""Prismatic-only observed handle hypotheses, independent of native assets.

SAM weights/0.25 confidence stay fixed. Common semantic aliases and depth
foreground proposals are alternatives, not GT crops or automatic handle truth.
Depth-only proposals require an explicit Agent choice after visual inspection.
"""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull
from matplotlib.path import Path as Polygon
from PIL import Image,ImageDraw


def proposals(folder,maximum=6):
    from native_g1_observed_regions import panel
    folder=Path(folder);r=panel(folder)
    yy,xx=np.where(r['inliers']);h=ConvexHull(np.c_[xx,yy]);poly=Polygon(np.c_[xx,yy][h.vertices])
    inside=poly.contains_points(np.c_[r['x'].ravel(),r['y'].ravel()]).reshape(r['valid'].shape)
    fg=r['valid']&inside&(r['height']>.003)
    # Only join neighboring image pixels when their measured metric surfaces
    # are continuous, reusing the 4mm neighborhood of the geometry pipeline.
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    pixels=np.flatnonzero(fg);ids=np.full(fg.shape,-1);ids.ravel()[pixels]=np.arange(len(pixels));a=[];b=[]
    for dy,dx in [(0,1),(1,0),(1,1),(1,-1)]:
        y,x=np.where(fg);Y=y+dy;X=x+dx;ok=(Y>=0)&(Y<fg.shape[0])&(X>=0)&(X<fg.shape[1]);y=y[ok];x=x[ok];Y=Y[ok];X=X[ok]
        ok=fg[Y,X]&(np.linalg.norm(r['P'][y,x]-r['P'][Y,X],axis=1)<=.004)
        a.extend(ids[y[ok],x[ok]]);b.extend(ids[Y[ok],X[ok]])
    if not len(pixels):return []
    graph=coo_matrix((np.ones(len(a)),(a,b)),shape=(len(pixels),len(pixels)));_,labels=connected_components(graph,directed=False)
    rows=[]
    for label in np.unique(labels):
        pp=pixels[labels==label]
        if len(pp)<30:continue # unchanged existing mask minimum
        y,x=np.unravel_index(pp,fg.shape);Q=r['P'][y,x];center=np.median(Q,axis=0);_,_,V=np.linalg.svd(Q-center,full_matrices=False);extent=np.ptp((Q-center)@V.T,axis=0)
        border=bool((x==0).any() or (x==fg.shape[1]-1).any() or (y==0).any() or (y==fg.shape[0]-1).any())
        score=float(np.sqrt(len(pp))*extent[0]/max(extent[1],.003))
        rows.append(dict(pixels=pp,point_count=len(pp),center_world_m=center.tolist(),axis_world=V[0].tolist(),extent_m=extent.tolist(),bbox_xyxy=[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)],partial=border,score=score))
    rows=sorted(rows,key=lambda r:r['score'],reverse=True)[:maximum];masks={};board=Image.open(folder/'rgb.png').convert('RGB');draw=ImageDraw.Draw(board)
    for i,row in enumerate(rows):
        mask=np.zeros(fg.shape,bool);mask.ravel()[row.pop('pixels')]=True;masks['proposal_'+str(i)]=mask;row['id']=str(i);row['source']='metric observed panel protrusion, robot excluded; semantic handle hypothesis only'
        x0,y0,x1,y1=row['bbox_xyxy'];draw.rectangle((x0,y0,x1,y1),outline=(255,80,20),width=3);draw.text((x0,max(0,y0-18)),str(i),fill=(255,30,0))
    np.savez_compressed(folder/'depth_handle_proposals.npz',**masks);board.save(folder/'depth_handle_proposals.png')
    data=dict(rgb_sha256=hashlib.sha256((folder/'rgb.png').read_bytes()).hexdigest(),proposals=rows,GT_used=False,Agent_acceptance_required=True,unknown_backside_remains_unknown=True)
    data['sha256']=hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest();(folder/'depth_handle_proposals.json').write_text(json.dumps(data,indent=2));return data


def semantic_recovery(folder,checkpoint):
    import torch
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    folder=Path(folder);image=Image.open(folder/'rgb.png').convert('RGB');model=build_sam3_image_model(checkpoint_path=str(checkpoint),load_from_HF=False)
    processor=Sam3Processor(model,confidence_threshold=.25);audit=[];accepted=[]
    # Joint-type semantic alternatives, identical for every drawer, not a
    # sequence chosen from an object ID or successful outcome.
    for prompt in ['drawer handle','handle']:
        with torch.inference_mode(),torch.autocast('cuda',dtype=torch.bfloat16):
            state=processor.set_image(image);state=processor.set_text_prompt(state=state,prompt=prompt)
        scores=state['scores'].detach().float().cpu().numpy().ravel();masks=state['masks'].detach().cpu().numpy()
        for i,(score,mask) in enumerate(zip(scores,masks)):
            mask=mask.squeeze().astype(bool);border=bool(mask[0].any() or mask[-1].any() or mask[:,0].any() or mask[:,-1].any());name='v73_semantic_%d_%d'%(len(audit),i)
            np.save(folder/(name+'.npy'),mask);audit.append(dict(prompt=prompt,score=float(score),pixels=int(mask.sum()),partial=border,mask=name+'.npy'))
            if mask.sum()>=30 and not border:accepted.append((float(score),name,mask))
        if accepted:break
    if accepted:
        score,name,mask=max(accepted,key=lambda v:v[0]);np.save(folder/'handle_mask.npy',mask);np.save(folder/'sam3_mask_raw.npy',mask)
        rgb=np.array(image);rgb[mask]=(.45*rgb[mask]+.55*np.array([0,255,50])).astype(np.uint8);Image.fromarray(rgb).save(folder/'handle_mask.overlay.png')
        path=folder/'candidate_cues.json';cues=json.loads(path.read_text()) if path.exists() else [];y,x=np.where(mask)
        cues.append(dict(name=name,score=score,pixels=int(mask.sum()),bbox_xyxy=[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)],partial=False,kind='shared_prismatic_semantic_alias',eligible_for_grasp=True))
        path.write_text(json.dumps(cues,indent=2))
    out=dict(shared_prompts=['drawer handle','handle'],threshold=.25,weights_changed=False,candidates=audit,accepted=bool(accepted),GT_used=False)
    (folder/'SEMANTIC_RECOVERY_V73.json').write_text(json.dumps(out,indent=2));return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--checkpoint',type=Path);a=p.parse_args()
    semantic=semantic_recovery(a.folder,a.checkpoint) if a.checkpoint else None
    try:depth=proposals(a.folder)
    except Exception as e:depth=dict(available=False,reason=repr(e));(a.folder/'depth_handle_proposals.json').write_text(json.dumps(depth))
    print(json.dumps(dict(semantic=semantic,depth=depth)))
