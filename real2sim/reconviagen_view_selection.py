"""GT-free view selection and RGBA preparation for ReconViaGen v0.5.

The selector consumes only the accepted RGB-D frames, robot poses and
BundleSDF object tracks.  It does not read eval_gt or any simulator mesh.  It
keeps adjacent views connected by silhouette overlap while greedily covering
different directions and scan passes.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np


def _norm(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.zeros_like(v)


def _load_mask(path: Path) -> np.ndarray:
    value = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if value is None:
        raise FileNotFoundError(path)
    return value > 0


def _silhouette(mask: np.ndarray, size: int = 128) -> np.ndarray:
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return np.zeros((size, size), dtype=np.uint8)
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    crop = mask[y0:y1, x0:x1].astype(np.uint8) * 255
    side = max(crop.shape)
    pad_y = (side - crop.shape[0]) // 2
    pad_x = (side - crop.shape[1]) // 2
    square = np.zeros((side, side), dtype=np.uint8)
    square[pad_y : pad_y + crop.shape[0], pad_x : pad_x + crop.shape[1]] = crop
    return cv2.resize(square, (size, size), interpolation=cv2.INTER_NEAREST) > 0


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 0.0


def _direction_label(direction: np.ndarray) -> str:
    """Label object-to-camera direction in the tracked object frame.

    The labels are a reporting convention only.  +X/-X, +Y/-Y and +Z/-Z are
    explicitly retained in the JSON so that no semantic orientation is
    assumed by the reconstruction backend.
    """
    axis = int(np.argmax(np.abs(direction)))
    sign = "+" if direction[axis] >= 0 else "-"
    names = ("x", "y", "z")
    return f"{sign}{names[axis]}"


def _frame_paths(dataset: Path, stem: str) -> dict[str, Path]:
    hand_name = "masks_hand" if (dataset / "masks_hand" / f"{stem}.png").exists() else "gripper_masks"
    paths = {
        "rgb": dataset / "rgb" / f"{stem}.png",
        "depth": dataset / "depth" / f"{stem}.png",
        "mask": dataset / "masks" / f"{stem}.png",
        "hand": dataset / hand_name / f"{stem}.png",
    }
    # Scan-station acquisition may provide a tight, padded RGB crop for the
    # generator while retaining full-resolution RGB-D for metric alignment.
    # Keep both paths explicit; quality and metric code continue to use the
    # uncropped frame, while ReconViaGen receives the crop.
    crop = dataset / "rgb_crops" / f"{stem}.png"
    if crop.exists():
        paths["rgb_crop"] = crop
    return paths


def _quality(paths: dict[str, Path]) -> tuple[dict[str, Any], np.ndarray]:
    rgb = cv2.imread(str(paths["rgb"]), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(paths["depth"]), cv2.IMREAD_UNCHANGED)
    mask = _load_mask(paths["mask"])
    hand = _load_mask(paths["hand"])
    if rgb is None or depth is None:
        raise FileNotFoundError(f"missing RGB-D frame: {paths}")
    clean = mask & ~hand
    obj = int(clean.sum())
    valid = clean & np.isfinite(depth) & (depth > 0)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    luma = gray[clean]
    quality = {
        "object_pixels": obj,
        "object_fraction": float(obj / max(mask.size, 1)),
        "depth_valid_ratio": float(valid.sum() / max(obj, 1)),
        "luma_mean": float(luma.mean()) if luma.size else 0.0,
        "clipped_fraction": float(np.mean((luma < 4) | (luma > 251))) if luma.size else 1.0,
        "gripper_pixels_excluded": int((mask & hand).sum()),
        "gripper_occlusion_ratio": float((mask & hand).sum() / max(mask.sum() + hand.sum(), 1)),
    }
    return quality, _silhouette(clean)


def _read_tracking(dataset: Path, tracking: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for pose_path in sorted((dataset / "poses").glob("*.json")):
        stem = pose_path.stem
        track_path = tracking / "ob_in_cam" / f"{stem}.txt"
        if not track_path.exists():
            continue
        row = json.loads(pose_path.read_text())
        paths = _frame_paths(dataset, stem)
        if not all(p.exists() for p in paths.values()):
            continue
        try:
            T_C_O = np.loadtxt(track_path).reshape(4, 4)
            T_B_C = np.asarray(row["T_B_camera"], dtype=float).reshape(4, 4)
        except (KeyError, ValueError):
            continue
        quality, silhouette = _quality(paths)
        C_O = np.linalg.inv(T_C_O)[:3, 3]
        # ``C_O`` is the camera centre expressed in the object frame.  Keep
        # both directions explicit: the ray used to view the object points
        # camera -> object (``-C_O``), while the reciprocal object -> camera
        # direction is ``C_O``.  Older reports called the former
        # ``object_to_camera_direction``; retaining both fields avoids a
        # silent sign error when auditing top/bottom/front/back coverage.
        camera_to_object = _norm(-C_O)
        object_to_camera = _norm(C_O)
        # The robot pose is retained for audit and downstream alignment.  No
        # ground-truth object transform is used.
        rows.append(
            {
                "stem": stem,
                "pass_id": int(row.get("pass_id", 0)),
                "source_frame_id": row.get("source_frame_id", row.get("frame_id")),
                "timestamp_s": row.get("timestamp_s"),
                "paths": {k: str(v) for k, v in paths.items()},
                "T_B_camera": T_B_C.tolist(),
                "T_base_camera": T_B_C.tolist(),
                "T_C_object": T_C_O.tolist(),
                "camera_center_object_m": C_O.tolist(),
                "camera_to_object_direction": camera_to_object.tolist(),
                "object_to_camera_direction": object_to_camera.tolist(),
                "direction_label": _direction_label(camera_to_object),
                "quality": quality,
                "silhouette": silhouette,
            }
        )
    return rows


def _accept(row: dict[str, Any], min_pixels: int, min_depth: float, max_occ: float) -> bool:
    q = row["quality"]
    return bool(
        q["object_pixels"] >= min_pixels
        and q["depth_valid_ratio"] >= min_depth
        and q["gripper_occlusion_ratio"] <= max_occ
        and 12.0 <= q["luma_mean"] <= 248.0
        and q["clipped_fraction"] <= 0.35
    )


def _angle(a: dict[str, Any], b: dict[str, Any]) -> float:
    da = np.asarray(a["camera_to_object_direction"] if "camera_to_object_direction" in a else a["object_to_camera_direction"], dtype=float)
    db = np.asarray(b["camera_to_object_direction"] if "camera_to_object_direction" in b else b["object_to_camera_direction"], dtype=float)
    return float(math.degrees(math.acos(np.clip(float(np.dot(da, db)), -1.0, 1.0))))


def _quality_score(row: dict[str, Any]) -> float:
    q = row["quality"]
    depth = np.clip((q["depth_valid_ratio"] - 0.95) / 0.05, 0.0, 1.0)
    exposure = np.exp(-abs(q["luma_mean"] - 128.0) / 110.0)
    occ = 1.0 - np.clip(q["gripper_occlusion_ratio"] / 0.42, 0.0, 1.0)
    size = np.clip(q["object_fraction"] / 0.08, 0.0, 1.0)
    return float(0.35 * depth + 0.25 * exposure + 0.25 * occ + 0.15 * size)


def _select(rows: list[dict[str, Any]], count: int, min_adj_overlap: float) -> list[dict[str, Any]]:
    if not rows:
        raise RuntimeError("no quality RGB-D frames available for ReconViaGen")
    # Seed with the cleanest frame.  Keep the pass in the objective so a second
    # physical grasp is represented whenever it supplies a new direction.
    seed = max(rows, key=_quality_score)
    selected = [seed]
    remaining = [r for r in rows if r is not seed]
    while remaining and len(selected) < count:
        scored = []
        for cand in remaining:
            angles = [_angle(cand, old) for old in selected]
            sil = cand["silhouette"]
            overlaps = [_iou(sil, old["silhouette"]) for old in selected]
            # Keep a connected sequence: a candidate must overlap at least one
            # selected view, while its reward is high when it adds a direction.
            max_overlap = max(overlaps)
            min_angle = min(angles)
            if max_overlap < min_adj_overlap:
                continue
            direction_gain = min(min_angle, 120.0) / 120.0
            overlap = np.clip(max_overlap, 0.0, 1.0)
            pass_gain = 1.0 if any(cand["pass_id"] != old["pass_id"] for old in selected) else 0.0
            score = 0.48 * direction_gain + 0.27 * overlap + 0.15 * _quality_score(cand) + 0.10 * pass_gain
            scored.append((score, cand))
        if not scored:
            # If a scan has a genuine direction gap, retain the best quality
            # frame and report the low overlap rather than silently hallucinating.
            cand = max(remaining, key=_quality_score)
        else:
            cand = max(scored, key=lambda x: x[0])[1]
        selected.append(cand)
        remaining.remove(cand)
    # Order by a nearest-neighbour path to make adjacent model inputs overlap.
    ordered = [selected.pop(0)]
    while selected:
        nxt = min(selected, key=lambda r: (1.0 - _iou(ordered[-1]["silhouette"], r["silhouette"]) + _angle(ordered[-1], r) / 180.0))
        ordered.append(nxt)
        selected.remove(nxt)
    return ordered


def _copy_rgba(rows: Iterable[dict[str, Any]], output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for idx, row in enumerate(rows):
        rgb_path = Path(row["paths"].get("rgb_crop", row["paths"]["rgb"]))
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        mask = _load_mask(Path(row["paths"]["mask"]))
        hand = _load_mask(Path(row["paths"]["hand"]))
        if rgb_path != Path(row["paths"]["rgb"]):
            # The crop is generated from the full mask using a deterministic
            # 15% bbox padding.  This keeps the RGB-D frame and its intrinsics
            # untouched for metric alignment while supplying a compact model
            # input with more object pixels.
            ys, xs = np.where(mask)
            if len(xs):
                pad = 0.15 * max(float(xs.max() - xs.min() + 1), float(ys.max() - ys.min() + 1))
                x0 = max(int(np.floor(xs.min() - pad)), 0)
                y0 = max(int(np.floor(ys.min() - pad)), 0)
                x1 = min(int(np.ceil(xs.max() + 1 + pad)), mask.shape[1])
                y1 = min(int(np.ceil(ys.max() + 1 + pad)), mask.shape[0])
                mask = mask[y0:y1, x0:x1]
                hand = hand[y0:y1, x0:x1]
        clean = (mask & ~hand).astype(np.uint8) * 255
        if rgb is None:
            raise FileNotFoundError(rgb_path)
        if rgb.shape[:2] != clean.shape:
            raise ValueError(f"RGB crop/mask shape mismatch: {rgb.shape[:2]} vs {clean.shape}")
        rgba = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGBA)
        rgba[:, :, 3] = clean
        out_path = output / f"view_{idx:03d}_{row['stem']}.png"
        cv2.imwrite(str(out_path), cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
        row["input_path"] = str(out_path)
        row["sanitized_object_pixels"] = int(clean.astype(bool).sum())
        row["mask_policy"] = "object_mask AND NOT gripper_mask"
        row["input_is_crop"] = bool(rgb_path != Path(row["paths"]["rgb"]))
        row["input_shape_hw"] = [int(rgb.shape[0]), int(rgb.shape[1])]


def select_views(dataset: Path, tracking: Path, output: Path, count: int = 12,
                 min_pixels: int = 3500, min_depth: float = 0.985,
                 max_occ: float = 0.42, min_adj_overlap: float = 0.18,
                 min_direction_span_deg: float = 90.0) -> dict[str, Any]:
    dataset, tracking, output = map(Path, (dataset, tracking, output))
    rows = _read_tracking(dataset, tracking)
    for row in rows:
        row["quality_score"] = _quality_score(row)
    rejected = [r for r in rows if not _accept(r, min_pixels, min_depth, max_occ)]
    accepted = [r for r in rows if _accept(r, min_pixels, min_depth, max_occ)]
    chosen = _select(accepted, min(count, len(accepted)), min_adj_overlap)
    input_dir = output / "inputs"
    _copy_rgba(chosen, input_dir)
    for row in rows + chosen:
        row.pop("silhouette", None)
    pairwise = []
    for i in range(len(chosen) - 1):
        a, b = chosen[i], chosen[i + 1]
        # Re-read normalized silhouettes only for the selected pair report.
        sa = _silhouette(_load_mask(Path(a["paths"]["mask"])) & ~_load_mask(Path(a["paths"]["hand"])))
        sb = _silhouette(_load_mask(Path(b["paths"]["mask"])) & ~_load_mask(Path(b["paths"]["hand"])))
        pairwise.append({"a": a["stem"], "b": b["stem"], "silhouette_iou": _iou(sa, sb), "direction_angle_deg": _angle(a, b)})
    dirs = np.asarray([r["object_to_camera_direction"] for r in chosen], dtype=float)
    direction_coverage = {
        "unique_labels": sorted({r["direction_label"] for r in chosen}),
        "min_pair_angle_deg": float(min((x["direction_angle_deg"] for x in pairwise), default=0.0)),
        "max_pair_angle_deg": float(max((x["direction_angle_deg"] for x in pairwise), default=0.0)),
        "mean_pair_angle_deg": float(np.mean([x["direction_angle_deg"] for x in pairwise])) if pairwise else 0.0,
        "angular_span_deg": float(max((math.degrees(math.acos(np.clip(np.dot(a, b), -1, 1))) for a in dirs for b in dirs), default=0.0)),
    }
    # A generator can produce a closed prior from a narrow scan, but that is
    # not the same as having measured front/back/top/bottom evidence.  Keep
    # the run auditable: downstream code may continue in degraded mode, while
    # the manifest explicitly says whether the selected input met the view
    # diversity contract.  This check is GT-free and only uses tracked camera
    # rays.
    direction_coverage["required_min_angular_span_deg"] = float(min_direction_span_deg)
    direction_coverage["direction_coverage_sufficient"] = bool(
        direction_coverage["angular_span_deg"] >= min_direction_span_deg
        and len(direction_coverage["unique_labels"]) >= 3
    )
    if not direction_coverage["direction_coverage_sufficient"]:
        direction_coverage["coverage_warning"] = (
            "selected views do not span the requested multi-direction contract; "
            "completion may be generated-only and a new pass/regrasp is required"
        )
    manifest = {
        "schema": "fr3_reconviagen_v05_view_selection/v1",
        "backend": "ReconViaGen v0.5",
        "gt_used": False,
        "selection_policy": "quality-filtered greedy direction coverage with connected silhouette overlap; nearest-neighbour input order",
        "direction_convention": "camera_to_object_direction is the viewing ray (-camera_center_in_object); object_to_camera_direction is its reciprocal",
        "robot_pose_convention": "T_base_camera (alias T_B_camera) is retained per frame; T_C_object is the tracking pose used to form T_base_object",
        "dataset": str(dataset),
        "tracking": str(tracking),
        "input_dir": str(input_dir),
        "input_views": [r["input_path"] for r in chosen],
        "selected": chosen,
        "candidates": rows,
        "rejected_count": len(rejected),
        "accepted_count": len(accepted),
        "pairwise_adjacent": pairwise,
        "pairwise_overlap_mean": float(np.mean([x["silhouette_iou"] for x in pairwise])) if pairwise else 0.0,
        "pairwise_overlap_min": float(min((x["silhouette_iou"] for x in pairwise), default=0.0)),
        "direction_coverage": direction_coverage,
        "passes": {str(p): sum(r["pass_id"] == p for r in chosen) for p in sorted({r["pass_id"] for r in chosen})},
        "quality_thresholds": {"min_object_pixels": min_pixels, "min_depth_valid_ratio": min_depth, "max_gripper_occlusion": max_occ, "min_adjacent_overlap": min_adj_overlap, "min_direction_span_deg": min_direction_span_deg},
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "view_selection.json").write_text(json.dumps(manifest, indent=2))
    return manifest


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("dataset", type=Path)
    p.add_argument("tracking", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--num-views", type=int, default=12)
    p.add_argument("--min-object-pixels", type=int, default=3500)
    p.add_argument("--min-depth-valid", type=float, default=0.985)
    p.add_argument("--max-gripper-occlusion", type=float, default=0.42)
    p.add_argument("--min-adjacent-overlap", type=float, default=0.18)
    p.add_argument("--min-direction-span-deg", type=float, default=90.0)
    a = p.parse_args()
    print(json.dumps(select_views(a.dataset, a.tracking, a.output, a.num_views, a.min_object_pixels, a.min_depth_valid, a.max_gripper_occlusion, a.min_adjacent_overlap, a.min_direction_span_deg), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
