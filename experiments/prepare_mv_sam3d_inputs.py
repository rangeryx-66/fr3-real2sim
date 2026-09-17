"""Prepare official MV-SAM3D input directories from a scan-station capture.

This adapter only reorganizes measured RGB/masks.  It never reads ``eval_gt``
and does not alter the RGB-D/robot-pose data used by the reconstruction
pipelines.  The generated directory follows the upstream single-object format:
``images/<id>.png`` plus ``<object>/<id>.png`` RGBA masks (alpha=foreground).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw


def _json_safe(value):
    """Convert numpy scalars/arrays in scan metadata to JSON-native values."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def _natural(paths):
    return sorted(paths, key=lambda p: (0, int(p.stem)) if p.stem.isdigit() else (1, p.stem))


def _matrix_from_pose(path: Path):
    try:
        d = json.loads(path.read_text())
        for key in ("T_B_camera", "T_B_C", "T_base_camera"):
            if key in d:
                return np.asarray(d[key], dtype=float).reshape(4, 4)
    except Exception:
        pass
    return None


def _records(scan_dir: Path):
    rgb_dir = scan_dir / "rgb"
    mask_dir = scan_dir / "masks"
    pose_dir = scan_dir / "poses"
    frames = []
    manifest = scan_dir / "scan_manifest.json"
    by_id: dict[int, dict[str, Any]] = {}
    if manifest.exists():
        try:
            data = json.loads(manifest.read_text())
            for row in data.get("frames", []):
                by_id[int(row["frame_id"])] = row
        except Exception:
            by_id = {}
    for rgb in _natural(rgb_dir.glob("*.png")):
        if not (mask_dir / rgb.name).exists():
            continue
        try:
            frame_id = int(rgb.stem)
        except ValueError:
            frame_id = len(frames)
        row = dict(by_id.get(frame_id, {}))
        pose = None
        pose_json = pose_dir / f"{rgb.stem}.json"
        if pose_json.exists():
            pose = _matrix_from_pose(pose_json)
        if pose is not None:
            row["T_B_camera_from_file"] = pose.tolist()
        row["frame_id"] = frame_id
        row["rgb_path"] = str(rgb)
        row["mask_path"] = str(mask_dir / rgb.name)
        frames.append(row)
    return frames


def _selection(n: int, count: int):
    if count <= 0 or count > n:
        raise ValueError(f"view count {count} is invalid for {n} usable frames")
    # The official stationary capture has two interleaved rings/passes.  For
    # 2/4/8 views use the same azimuths on each ring so the subsets retain both
    # overlap and elevation diversity.  Other counts use uniform temporal bins.
    if n >= 2 * 4 and count in (2, 4, 8):
        half = n // 2
        if count == 2:
            ids = [0, half]
        else:
            per = count // 2
            ids = list(np.linspace(0, half - 1, per, dtype=int))
            ids += list(half + np.linspace(0, n - half - 1, per, dtype=int))
        return sorted(set(ids))
    return sorted(set(np.linspace(0, n - 1, count, dtype=int)))


def _mask_stats(mask: np.ndarray):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return dict(area_fraction=0.0, bbox=None)
    return dict(
        area_fraction=float(mask.mean()),
        bbox=[int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
    )


def _mask_iou(a: np.ndarray, b: np.ndarray):
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 0.0


def _montage(items, out_path: Path):
    if not items:
        return
    thumbs = []
    for name, rgb_path, mask_path in items:
        img = Image.open(rgb_path).convert("RGB")
        mask = np.asarray(Image.open(mask_path).convert("L")) > 0
        overlay = np.asarray(img).copy()
        overlay[mask] = (0.65 * overlay[mask] + 0.35 * np.array([40, 220, 80])).astype(np.uint8)
        canvas = Image.fromarray(overlay)
        canvas.thumbnail((360, 270))
        draw = ImageDraw.Draw(canvas)
        draw.rectangle((0, 0, canvas.width - 1, canvas.height - 1), outline=(255, 255, 0), width=2)
        draw.text((6, 6), name, fill=(255, 255, 0))
        thumbs.append(canvas)
    cols = min(4, len(thumbs)); rows = (len(thumbs) + cols - 1) // cols
    w = max(x.width for x in thumbs); h = max(x.height for x in thumbs)
    out = Image.new("RGB", (cols * w, rows * h), (20, 20, 20))
    for i, thumb in enumerate(thumbs):
        out.paste(thumb, ((i % cols) * w, (i // cols) * h))
    out.save(out_path)


def prepare(scan_dir: Path, output_dir: Path, object_name: str, counts):
    rows = _records(scan_dir)
    if not rows:
        raise RuntimeError(f"no RGB/mask pairs found in {scan_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    all_manifest = dict(schema="mv_sam3d_input_selection/v1", source_scan=str(scan_dir), object=object_name, selections={})
    for count in counts:
        ids = _selection(len(rows), int(count))
        exp = output_dir / f"views_{count:02d}"
        images = exp / "images"
        masks = exp / object_name
        images.mkdir(parents=True, exist_ok=True); masks.mkdir(parents=True, exist_ok=True)
        selected = []
        for new_id, source_idx in enumerate(ids):
            row = rows[source_idx]
            rgb = Image.open(row["rgb_path"]).convert("RGB")
            mask = np.asarray(Image.open(row["mask_path"]).convert("L")) > 0
            rgba = np.zeros((mask.shape[0], mask.shape[1], 4), dtype=np.uint8)
            rgba[..., 3] = mask.astype(np.uint8) * 255
            rgb.save(images / f"{new_id}.png")
            Image.fromarray(rgba, mode="RGBA").save(masks / f"{new_id}.png")
            stat = _mask_stats(mask)
            selected.append({
                "input_id": int(new_id), "source_index": int(source_idx),
                "source_frame_id": int(row["frame_id"]), "rgb": row["rgb_path"],
                "mask": row["mask_path"], **stat,
                "T_B_camera": _json_safe(row.get("T_B_camera") or row.get("T_B_camera_from_file")),
                "T_B_TCP": _json_safe(row.get("T_B_TCP")),
                "intrinsics": _json_safe(row.get("intrinsics")),
            })
        masks_np = [np.asarray(Image.open(masks / f"{i}.png"))[..., 3] > 0 for i in range(len(selected))]
        pairwise = [_mask_iou(masks_np[i], masks_np[i + 1]) for i in range(len(masks_np) - 1)]
        selection = dict(view_count=count, selected=selected,
                         adjacent_mask_iou=pairwise,
                         adjacent_mask_iou_median=float(np.median(pairwise)) if pairwise else None,
                         adjacent_mask_iou_min=float(np.min(pairwise)) if pairwise else None)
        (exp / "selection.json").write_text(json.dumps(selection, indent=2))
        _montage([(f"view {i} / src {r['source_frame_id']}", r["rgb"], r["mask"]) for i, r in enumerate(selected)], exp / "input_mask_overlay.png")
        all_manifest["selections"][str(count)] = selection
    (output_dir / "selection_manifest.json").write_text(json.dumps(all_manifest, indent=2))
    return all_manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan-dir", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--object", default="stuffed_toy")
    ap.add_argument("--counts", default="2,4,8")
    args = ap.parse_args()
    out = prepare(args.scan_dir, args.output, args.object, [int(x) for x in args.counts.split(",") if x.strip()])
    print(json.dumps({"output": str(args.output), "views": {k: len(v["selected"]) for k, v in out["selections"].items()}}, indent=2))


if __name__ == "__main__":
    main()
