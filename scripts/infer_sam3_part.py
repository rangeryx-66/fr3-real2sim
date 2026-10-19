"""Run official SAM3 text segmentation on one captured RGB frame.

Use a Python environment containing facebookresearch/sam3. A checkpoint is
mandatory; this script never substitutes simulator segmentation for SAM3.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rgb', type=Path, required=True)
    parser.add_argument('--prompt', default='cabinet door handle')
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--min-score', type=float, default=0.25)
    parser.add_argument('--center-crop-size',type=int,default=0,
                        help='fixed center ROI on actual RGB; 0 uses the full frame')
    args = parser.parse_args()
    if not args.checkpoint.is_file():
        raise FileNotFoundError(f'SAM3 checkpoint unavailable: {args.checkpoint}')
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    import torch

    full_image = Image.open(args.rgb).convert('RGB')
    crop_size=args.center_crop_size
    if crop_size:
        if crop_size<=0 or crop_size>min(full_image.size):
            raise ValueError('center crop must fit inside the RGB image')
        left=(full_image.width-crop_size)//2
        top=(full_image.height-crop_size)//2
        crop_box=(left,top,left+crop_size,top+crop_size)
        image=full_image.crop(crop_box)
    else:
        crop_box=(0,0,full_image.width,full_image.height)
        image=full_image
    # Official SAM3 image examples enable CUDA bfloat16 autocast. The 3.1
    # checkpoint mixes bfloat16 activations and float32 linear weights.
    with torch.inference_mode(), torch.autocast('cuda',dtype=torch.bfloat16,
                                                 enabled=torch.cuda.is_available()):
        model = build_sam3_image_model(checkpoint_path=str(args.checkpoint), load_from_HF=False)
        processor = Sam3Processor(model,confidence_threshold=args.min_score)
        state=processor.set_image(image)
        requested=args.prompt;prompts=[requested]
        if ' with ' in requested:prompts.append('entire '+requested.split(' with ',1)[0])
        attempts=[];used=requested
        for description in prompts:
            processor.reset_all_prompts(state)
            output=processor.set_text_prompt(state=state,prompt=description)
            trial=np.asarray(output['scores'].detach().float().cpu(),dtype=float).reshape(-1)
            attempts.append({'prompt':description,'instances':len(trial),'maximum_score':float(max(trial)) if len(trial) else None})
            if len(trial) and max(trial)>=args.min_score:used=description;break
    scores = np.asarray(output['scores'].detach().float().cpu(), dtype=float).reshape(-1)
    masks = np.asarray(output['masks'].detach().cpu())
    if len(scores) == 0 or max(scores) < args.min_score:
        raise RuntimeError(f'SAM3 returned no confident handle mask: instances={len(scores)}, '
                           f'max_score={max(scores) if len(scores) else None}, '
                           f'threshold={args.min_score}')
    complete=[]
    for index in range(len(scores)):
        candidate=np.squeeze(masks[index]).astype(bool)
        if candidate.shape!=(image.height,image.width) or candidate.sum()<30:
            continue
        border=(candidate[0].any() or candidate[-1].any() or
                candidate[:,0].any() or candidate[:,-1].any())
        if not border:
            complete.append(index)
    if not complete:
        raise RuntimeError('SAM3 found no complete mask inside the camera-center ROI')
    best=int(max(complete,key=lambda index:scores[index]))
    local_mask = np.squeeze(masks[best]).astype(bool)
    if local_mask.shape != (image.height, image.width) or np.count_nonzero(local_mask) < 30:
        raise RuntimeError(f'SAM3 mask invalid: shape={local_mask.shape}, pixels={local_mask.sum()}')
    mask=np.zeros((full_image.height,full_image.width),dtype=bool)
    left,top,right,bottom=crop_box
    mask[top:bottom,left:right]=local_mask
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output, mask)
    rgb = np.asarray(full_image).copy()
    rgb[mask] = (0.45 * rgb[mask] + 0.55 * np.array([0, 255, 50])).astype(np.uint8)
    Image.fromarray(rgb).save(args.output.with_suffix('.overlay.png'))
    metadata = {'source': 'facebookresearch/sam3', 'prompt': used,'requested_prompt':requested,'prompt_attempts':attempts,'threshold':args.min_score,
                'checkpoint': str(args.checkpoint.resolve()), 'score': float(scores[best]),
                'instance_index': best, 'num_instances': len(scores),
                'complete_instances':complete,
                'selection':'highest SAM3 score among masks not clipped by ROI border',
                'mask_pixels': int(mask.sum()), 'shape': list(mask.shape),
                'center_crop_box_xyxy':list(crop_box)}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata), flush=True)


if __name__ == '__main__':
    main()
