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
    args = parser.parse_args()
    if not args.checkpoint.is_file():
        raise FileNotFoundError(f'SAM3 checkpoint unavailable: {args.checkpoint}')
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    import torch

    image = Image.open(args.rgb).convert('RGB')
    with torch.inference_mode():
        model = build_sam3_image_model(checkpoint_path=str(args.checkpoint), load_from_HF=False)
        processor = Sam3Processor(model)
        output = processor.set_text_prompt(state=processor.set_image(image), prompt=args.prompt)
    scores = np.asarray(output['scores'].detach().cpu(), dtype=float).reshape(-1)
    masks = np.asarray(output['masks'].detach().cpu())
    if len(scores) == 0 or max(scores) < args.min_score:
        raise RuntimeError('SAM3 returned no confident handle mask')
    best = int(np.argmax(scores))
    mask = np.squeeze(masks[best]).astype(bool)
    if mask.shape != (image.height, image.width) or np.count_nonzero(mask) < 30:
        raise RuntimeError(f'SAM3 mask invalid: shape={mask.shape}, pixels={mask.sum()}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output, mask)
    rgb = np.asarray(image).copy()
    rgb[mask] = (0.45 * rgb[mask] + 0.55 * np.array([0, 255, 50])).astype(np.uint8)
    Image.fromarray(rgb).save(args.output.with_suffix('.overlay.png'))
    metadata = {'source': 'facebookresearch/sam3', 'prompt': args.prompt,
                'checkpoint': str(args.checkpoint.resolve()), 'score': float(scores[best]),
                'instance_index': best, 'num_instances': len(scores),
                'mask_pixels': int(mask.sum()), 'shape': list(mask.shape)}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata), flush=True)


if __name__ == '__main__':
    main()
