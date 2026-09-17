"""Quality audit and deterministic RGB-D preprocessing for BundleSDF.

This module never reads eval_gt.  It makes a new reader-compatible dataset and
keeps the raw acquisition immutable.
"""
from __future__ import annotations
import argparse, json, shutil
from pathlib import Path
import cv2
import numpy as np


DEFAULTS = dict(
    min_object_pixels=3500,
    min_depth_valid_ratio=.985,
    min_luma=18., max_luma=238.,
    # Dark printed pixels on packaged objects are legitimate.  Reject only broad
    # sensor clipping, while the mean-luma gate catches globally bad exposure.
    max_clipped_fraction=.25,
    min_laplacian_variance=8.,
    max_gripper_occlusion=.42,
    mask_close_px=3,
    mask_erode_px=1,
    gripper_guard_px=5,
    depth_median_px=3,
    max_local_depth_delta_mm=4,
)


def _odd_kernel(n):
    n=max(1,int(n));return np.ones((n|1,n|1),np.uint8)


def frame_metrics(rgb, depth_mm, mask, hand):
    m=mask>0;h=hand>0
    count=int(m.sum());valid=m & np.isfinite(depth_mm) & (depth_mm>0)
    gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
    luma=gray[m] if count else np.empty(0)
    hull=np.zeros_like(mask)
    contours,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        points=np.concatenate(contours,axis=0);cv2.fillConvexPoly(hull,cv2.convexHull(points),255)
    hand_near=(h & (cv2.dilate(hull,_odd_kernel(15))>0))
    occ=float(hand_near.sum()/max(count+hand_near.sum(),1))
    return dict(
        object_pixels=count,
        depth_valid_ratio=float(valid.sum()/max(count,1)),
        median_depth_m=float(np.median(depth_mm[valid])/1000) if valid.any() else None,
        luma_mean=float(luma.mean()) if luma.size else 0.,
        clipped_fraction=float(np.mean((luma<4)|(luma>251))) if luma.size else 1.,
        laplacian_variance=float(cv2.Laplacian(gray,cv2.CV_64F)[m].var()) if count else 0.,
        gripper_occlusion_ratio=occ,
    )


def reasons(metric,cfg):
    out=[]
    if metric['object_pixels']<cfg['min_object_pixels']:out.append('LOW_OBJECT_COVERAGE')
    if metric['depth_valid_ratio']<cfg['min_depth_valid_ratio']:out.append('LOW_VALID_DEPTH')
    if not cfg['min_luma']<=metric['luma_mean']<=cfg['max_luma']:out.append('BAD_EXPOSURE')
    if metric['clipped_fraction']>cfg['max_clipped_fraction']:out.append('CLIPPED_EXPOSURE')
    if metric['laplacian_variance']<cfg['min_laplacian_variance']:out.append('BLUR')
    if metric['gripper_occlusion_ratio']>cfg['max_gripper_occlusion']:out.append('GRIPPER_OCCLUSION')
    return out


def prepare(raw:Path, output:Path, config=None):
    raw,output=Path(raw),Path(output);cfg={**DEFAULTS,**(config or {})}
    manifest=json.loads((raw/'scan_manifest.json').read_text())
    if output.exists():shutil.rmtree(output)
    for d in ('rgb','depth','masks','gripper_masks','masks_hand','poses'): (output/d).mkdir(parents=True,exist_ok=True)
    shutil.copy2(raw/'cam_K.txt',output/'cam_K.txt')
    audit=[];accepted=[]
    for frame in manifest['frames']:
        rgb=cv2.imread(frame['rgb'],cv2.IMREAD_COLOR);depth=cv2.imread(frame['depth'],cv2.IMREAD_UNCHANGED)
        mask=cv2.imread(frame['object_mask'],cv2.IMREAD_GRAYSCALE);hand=cv2.imread(frame['gripper_mask'],cv2.IMREAD_GRAYSCALE)
        metric=frame_metrics(rgb,depth,mask,hand);why=reasons(metric,cfg)
        record={**frame,**metric,'accepted':not why,'rejection_reasons':why};audit.append(record)
        if why:continue
        # Close isolated pinholes, never close across the guarded gripper silhouette,
        # and erode one pixel to remove mixed-depth silhouette samples.
        clean=cv2.morphologyEx((mask>0).astype(np.uint8),cv2.MORPH_CLOSE,_odd_kernel(cfg['mask_close_px']))
        guard=cv2.dilate((hand>0).astype(np.uint8),_odd_kernel(cfg['gripper_guard_px']))
        clean[guard>0]=0;clean=cv2.erode(clean,_odd_kernel(cfg['mask_erode_px']))
        median=cv2.medianBlur(depth,int(cfg['depth_median_px'])|1)
        delta=np.abs(median.astype(np.int32)-depth.astype(np.int32))
        filtered=depth.copy();replace=(clean>0)&(delta<=cfg['max_local_depth_delta_mm']);filtered[replace]=median[replace]
        filtered[(clean>0)&((filtered==0)|~np.isfinite(filtered))]=0
        stem=f'{len(accepted):06d}'
        cv2.imwrite(str(output/'rgb'/f'{stem}.png'),rgb);cv2.imwrite(str(output/'depth'/f'{stem}.png'),filtered)
        cv2.imwrite(str(output/'masks'/f'{stem}.png'),clean*255);cv2.imwrite(str(output/'gripper_masks'/f'{stem}.png'),hand)
        cv2.imwrite(str(output/'masks_hand'/f'{stem}.png'),guard*255)
        pose={**record,'source_frame_id':frame['frame_id'],'prepared_stem':stem}
        (output/'poses'/f'{stem}.json').write_text(json.dumps(pose,indent=2));accepted.append(pose)
    if len(accepted)<8:raise RuntimeError(f'only {len(accepted)} quality frames survive')
    passes=sorted({int(x['pass_id']) for x in accepted})
    report={'schema':'fr3_scan_quality/v1','raw_manifest':str(raw/'scan_manifest.json'),'config':cfg,
            'input_frames':len(audit),'accepted_frames':len(accepted),'rejected_frames':len(audit)-len(accepted),
            'accepted_per_pass':{str(p):sum(x['pass_id']==p for x in accepted) for p in passes},'frames':audit}
    (output/'quality_report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('raw',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    print(json.dumps(prepare(a.raw,a.output),indent=2))
