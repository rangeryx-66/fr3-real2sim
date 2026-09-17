"""Run the pinned official VGGT model on masked scan RGB frames.

This stage predicts dense depth/confidence only.  It never writes geometry in
the metric fusion stage and never reads GT.  Metric scale is recovered later
from real RGB-D overlap.
"""
from __future__ import annotations
import argparse,hashlib,json,time
from pathlib import Path
import cv2,numpy as np,torch


def _sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()


def _prepare(path,hand_path):
    bgr=cv2.imread(str(path),cv2.IMREAD_COLOR);rgb=bgr[...,::-1].copy();hand=cv2.imread(str(hand_path),cv2.IMREAD_GRAYSCALE)>0
    rgb[hand]=0
    h,w=rgb.shape[:2];scale=518./max(h,w);nh=max(14,int(round(h*scale/14))*14);nw=max(14,int(round(w*scale/14))*14)
    resized=cv2.resize(rgb,(nw,nh),interpolation=cv2.INTER_AREA);canvas=np.zeros((518,518,3),np.uint8)
    y0=(518-nh)//2;x0=(518-nw)//2;canvas[y0:y0+nh,x0:x0+nw]=resized[:518-x0 if False else nh,:518-x0 if False else nw]
    return torch.from_numpy(canvas).permute(2,0,1).float()/255.,(x0,y0,nw,nh,(h,w))


def run(dataset:Path,output:Path,checkpoint:Path,chunk_size=12,device='cuda',max_frames=None):
    import sys
    sys.path.insert(0,'/data1/home/rangeryx/vggt')
    from vggt.models.vggt import VGGT
    output=Path(output);(output/'depth').mkdir(parents=True,exist_ok=True);(output/'confidence').mkdir(parents=True,exist_ok=True)
    model=VGGT();state=torch.load(checkpoint,map_location='cpu');
    if isinstance(state,dict) and 'model' in state and isinstance(state['model'],dict):state=state['model']
    if isinstance(state,dict) and 'state_dict' in state and isinstance(state['state_dict'],dict):state=state['state_dict']
    model.load_state_dict(state,strict=True);model.eval();model.to(device)
    poses=sorted((Path(dataset)/'poses').glob('*.json'));poses=poses[:max_frames] if max_frames else poses;meta=[];t0=time.perf_counter()
    for start in range(0,len(poses),chunk_size):
        batch=poses[start:start+chunk_size];imgs=[];maps=[]
        for p in batch:
            x,m=_prepare(Path(dataset)/'rgb'/f'{p.stem}.png',Path(dataset)/'masks_hand'/f'{p.stem}.png');imgs.append(x);maps.append(m)
        x=torch.stack(imgs).to(device)
        with torch.inference_mode():
            with torch.autocast(device_type='cuda' if str(device).startswith('cuda') else 'cpu',dtype=torch.float16,enabled=str(device).startswith('cuda')):
                pred=model(x)
        depth=pred['depth'].detach().float().cpu().numpy();conf=pred['depth_conf'].detach().float().cpu().numpy()
        for j,p in enumerate(batch):
            np.save(output/'depth'/f'{p.stem}.npy',depth[0,j,...,0].astype(np.float32));np.save(output/'confidence'/f'{p.stem}.npy',conf[0,j].astype(np.float32))
            c=conf[0,j];meta.append({'frame':p.stem,'input_mapping':maps[j],'depth_min':float(np.nanpercentile(depth[0,j,...,0],1)),'depth_median':float(np.nanmedian(depth[0,j,...,0])),'depth_max':float(np.nanpercentile(depth[0,j,...,0],99)),'confidence_median':float(np.nanmedian(c)),'confidence_p90':float(np.nanpercentile(c,90))})
    elapsed=time.perf_counter()-t0
    report={'schema':'fr3_vggt_inference/v1','gt_used':False,'model':'official VGGT','checkpoint':str(checkpoint),'checkpoint_sha256':_sha256(checkpoint),'frames':len(meta),'chunk_size':chunk_size,'elapsed_seconds':elapsed,'frames_per_second':len(meta)/max(elapsed,1e-9),'frames_meta':meta}
    (output/'inference_report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset',type=Path);p.add_argument('output',type=Path);p.add_argument('--checkpoint',type=Path,default=Path('/data1/home/rangeryx/vggt/checkpoint/model.pt'));p.add_argument('--chunk-size',type=int,default=12);p.add_argument('--device',default='cuda');p.add_argument('--max-frames',type=int);a=p.parse_args();print(json.dumps(run(a.dataset,a.output,a.checkpoint,a.chunk_size,a.device,a.max_frames),indent=2))
