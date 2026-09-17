#!/usr/bin/env python3
"""Make a compact Mix3R vs MV-SAM3D preview montage on the remote host."""
from __future__ import annotations
import argparse
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--objects", nargs="+", required=True)
    args = ap.parse_args()
    root = Path(args.root)
    objs = args.objects
    labels = []
    for o in objs:
        labels.extend([(o, "Mix3R"), (o, "MV-SAM3D")])
    tiles = []
    tile_w, tile_h = 640, 360
    for o, method in labels:
        if method == "Mix3R":
            p = root / o / "views_08" / "mix3r_preview.png"
        else:
            p = root.parent / "mv_sam3d_unseen_static_v1" / "rendered" / f"{o}_8v.png"
        im = Image.open(p).convert("RGB")
        im.thumbnail((tile_w, tile_h))
        tile = Image.new("RGB", (tile_w, tile_h + 36), (16, 18, 24))
        tile.paste(im, ((tile_w - im.width) // 2, 36 + (tile_h - im.height) // 2))
        d = ImageDraw.Draw(tile)
        d.text((12, 10), f"{o} — {method}", fill=(255, 255, 255))
        tiles.append(tile)
    cols = 2
    rows = (len(tiles) + cols - 1) // cols
    canvas = Image.new("RGB", (cols * tile_w, rows * (tile_h + 36)), (8, 10, 14))
    for i, tile in enumerate(tiles):
        canvas.paste(tile, ((i % cols) * tile_w, (i // cols) * (tile_h + 36)))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.out, quality=95)
    print(args.out)
    return 0
if __name__ == "__main__":
    raise SystemExit(main())
