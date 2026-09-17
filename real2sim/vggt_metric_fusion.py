"""Fuse VGGT depth only into missing/invalid real RGB-D pixels."""
from __future__ import annotations
import argparse,json,shutil,time
from pathlib import Path
import cv2,numpy as np


def fuse(dataset:Path,prediction:Path,output:Path,conf_quantile=.75,conflict_mm=5.,conflict_ratio=.10):
    dataset,prediction,output=map(Path,(dataset,prediction,output));
    if output.exists():shutil.rmtree(output)
    for d in ('rgb','depth','masks','gripper_masks','masks_hand','poses'):(output/d).mkdir(parents=True,exist_ok=True)
    shutil.copy2(dataset/'cam_K.txt',output/'cam_K.txt');rows=[];t0=time.perf_counter()
    for p in sorted((dataset/'poses').glob('*.json')):
        stem=p.stem;row=json.loads(p.read_text());depth=cv2.imread(str(dataset/'depth'/f'{stem}.png'),cv2.IMREAD_UNCHANGED).astype(np.float32)
        raw_mask=cv2.imread(str(dataset/'masks'/f'{stem}.png'),0)>0
        hand=cv2.imread(str(dataset/'masks_hand'/f'{stem}.png'),0)>0
        # Keep the source masks for provenance, but make the downstream object
        # mask explicitly disjoint from the hand.  This guarantees that neither
        # VGGT nor TSDF can integrate a gripper pixel even when the instance
        # mask and hand mask overlap at a depth discontinuity.
        mask=raw_mask&~hand
        vd=np.load(prediction/'depth'/f'{stem}.npy');conf=np.load(prediction/'confidence'/f'{stem}.npy');h,w=depth.shape
        # Undo the official 518-square pad for the original 640x480 image.
        scale=518./max(h,w);nh0=max(14,int(round(h*scale/14))*14);nw0=max(14,int(round(w*scale/14))*14);x0=(518-nw0)//2;y0=(518-nh0)//2;vd=vd[y0:y0+nh0,x0:x0+nw0];conf=conf[y0:y0+nh0,x0:x0+nw0];vd=cv2.resize(vd,(w,h),interpolation=cv2.INTER_LINEAR);conf=cv2.resize(conf,(w,h),interpolation=cv2.INTER_LINEAR)
        real=depth>0;valid=mask&real&np.isfinite(vd)&(vd>1e-6);overlap=mask&real&np.isfinite(vd)&(vd>1e-6);scale_ok=False;metric_scale=None;conflict=0.
        if int(overlap.sum())>=50:
            ratios=depth[overlap]/vd[overlap];metric_scale=float(np.median(ratios[np.isfinite(ratios)]));res=np.abs(depth[overlap]-vd[overlap]*metric_scale);conflict=float(np.mean(res>np.maximum(conflict_mm,conflict_ratio*depth[overlap])));scale_ok=bool(np.isfinite(metric_scale) and metric_scale>1e-6 and conflict<=.20)
        fill=np.zeros_like(mask);supplement=0
        if scale_ok:
            vals=conf[mask&np.isfinite(conf)];threshold=float(np.quantile(vals,conf_quantile)) if vals.size else np.inf
            fill=mask&~real&(conf>=threshold)&np.isfinite(vd)&(vd>1e-6);pred_metric=vd*metric_scale
            local=depth[real&mask];lo=float(np.percentile(local,1)*.5) if local.size else .01;hi=float(np.percentile(local,99)*1.5) if local.size else 3.;fill&=(pred_metric>lo)&(pred_metric<hi);depth[fill]=pred_metric[fill];supplement=int(fill.sum())
        shutil.copy2(dataset/'rgb'/f'{stem}.png',output/'rgb'/f'{stem}.png')
        cv2.imwrite(str(output/'masks'/f'{stem}.png'),(mask.astype(np.uint8)*255))
        shutil.copy2(dataset/'gripper_masks'/f'{stem}.png',output/'gripper_masks'/f'{stem}.png')
        shutil.copy2(dataset/'masks_hand'/f'{stem}.png',output/'masks_hand'/f'{stem}.png')
        cv2.imwrite(str(output/'depth'/f'{stem}.png'),np.clip(depth,0,65535).astype(np.uint16));shutil.copy2(p,output/'poses'/p.name)
        rows.append({'frame':stem,'raw_object_pixels':int(raw_mask.sum()),'gripper_pixels_excluded':int((raw_mask&hand).sum()),'real_depth_pixels':int((mask&real).sum()),'vggt_supplement_pixels':supplement,'object_pixels':int(mask.sum()),'metric_scale':metric_scale,'scale_valid':scale_ok,'conflict_fraction':conflict,'supplement_fraction':float(supplement/max(int(mask.sum()),1)),'prediction_rejected':not scale_ok})
    report={'schema':'fr3_vggt_metric_fusion/v1','gt_used':False,'real_depth_priority':True,'gripper_excluded':True,'object_mask_policy':'raw object mask intersected with inverse gripper mask before any geometry fusion','conflict_policy':'reject VGGT supplement when overlap conflict fraction > 0.20','confidence_quantile':conf_quantile,'conflict_mm':conflict_mm,'conflict_ratio':conflict_ratio,'frames':rows,'elapsed_seconds':time.perf_counter()-t0,'gripper_pixels_excluded':int(sum(x['gripper_pixels_excluded'] for x in rows)),'real_geometry_fraction':float(sum(x['real_depth_pixels'] for x in rows)/max(sum(x['real_depth_pixels']+x['vggt_supplement_pixels'] for x in rows),1)),'vggt_supplement_fraction':float(sum(x['vggt_supplement_pixels'] for x in rows)/max(sum(x['real_depth_pixels']+x['vggt_supplement_pixels'] for x in rows),1)),'conflict_rejected_frames':int(sum(x['prediction_rejected'] for x in rows))}
    (output/'fusion_report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset',type=Path);p.add_argument('prediction',type=Path);p.add_argument('output',type=Path);a=p.parse_args();print(json.dumps(fuse(a.dataset,a.prediction,a.output),indent=2))
