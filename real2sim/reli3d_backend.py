"""Independent ReLi3D scan adapter and RGB-D metric constraint.

The scan-station acquisition is deliberately treated as an immutable source.  This
module only consumes its RGB, depth, masks and measured ``T_base_camera`` poses and
writes a ReLi3D-compatible object directory.  It does not import or modify the
grasp executor.  ``eval_gt`` is never opened by the preparation or alignment
functions; callers may use it only in a separate final evaluation step.

The station camera uses a right/down/+Z-forward ROS camera basis.  ReLi3D's
``coordinate_system=ogl`` uses right/up/-Z-forward.  The proper 180 degree X
rotation ``diag(1,-1,-1)`` maps the former to the latter while preserving the
image pixel correspondence.  Robot poses therefore remain the source of every
camera extrinsic; no pose is estimated from images.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
import trimesh
from scipy.spatial import cKDTree


CAMERA_STATION_TO_OGL = np.diag([1.0, -1.0, -1.0, 1.0])


@dataclass(frozen=True)
class ReLi3DInputConfig:
    image_size: int = 512
    crop_padding: float = 0.12
    # The released checkpoint is trained/configured for at most four
    # conditioning views (``max_condition_count=4``).  All scan frames remain
    # in QA metadata, while four representative views are fed to the model by
    # default.
    max_views: int = 4
    # The training data uses an object-centric camera radius of roughly 2--3
    # units.  Scan-station poses are metric metres (~0.30 m radius), so only
    # the model-input translations are normalized.  The original metric pose
    # is retained per frame for RGB-D alignment.
    model_camera_radius: float = 2.4
    min_object_pixels: int = 1800
    min_depth_valid_ratio: float = 0.96
    min_laplacian_variance: float = 5.0
    min_luma: float = 5.0
    max_luma: float = 250.0
    max_clipped_fraction: float = 0.40
    gripper_guard_px: int = 2
    min_component_fraction: float = 0.008
    min_hole_fraction: float = 0.0005
    max_hole_fraction: float = 0.12
    min_view_separation_deg: float = 7.0


def _path(dataset: Path, value: str | Path) -> Path:
    p = Path(value)
    return p if p.is_absolute() else dataset / p


def _records(dataset: Path) -> list[dict[str, Any]]:
    """Load view records from either scan-station manifest schema."""
    manifest_path = dataset / "scan_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        views = manifest.get("views") or manifest.get("frames") or []
        if views:
            return list(views)
    rows = []
    for pose_path in sorted((dataset / "poses").glob("*.json")):
        rows.append(json.loads(pose_path.read_text()))
    if not rows:
        raise FileNotFoundError(f"No scan-station views found under {dataset}")
    return rows


def _read_depth(path: Path) -> tuple[np.ndarray, float]:
    raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise FileNotFoundError(path)
    depth = raw.astype(np.float32)
    valid = depth[np.isfinite(depth) & (depth > 0)]
    # The station writes uint16 millimetres.  Keep support for metre-valued
    # float captures without relying on an object-specific constant.
    scale = 0.001 if valid.size and float(np.median(valid)) > 20.0 else 1.0
    return depth * scale, scale


def _largest_clean_component(mask: np.ndarray, hand: np.ndarray, cfg: ReLi3DInputConfig) -> tuple[np.ndarray, dict[str, Any]]:
    """Remove hand overlap/fragments while retaining legitimate handles/holes.

    Only a two-pixel hand guard and a 3x3 close are used.  The silhouette is not
    eroded.  Small enclosed holes are filled and large holes are retained (they
    may be true hollow geometry such as a handle).
    """
    original = mask.astype(bool)
    guard = cv2.dilate(hand.astype(np.uint8), np.ones((2 * cfg.gripper_guard_px + 1,) * 2, np.uint8)) > 0
    work = original & ~guard
    work = cv2.morphologyEx(work.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) > 0
    n, labels, stats, cents = cv2.connectedComponentsWithStats(work.astype(np.uint8), 8)
    kept = np.zeros_like(work, dtype=bool)
    components = []
    if n > 1:
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest = int(np.argmax(areas)) + 1
        largest_area = max(int(stats[largest, cv2.CC_STAT_AREA]), 1)
        x0 = int(stats[largest, cv2.CC_STAT_LEFT]); y0 = int(stats[largest, cv2.CC_STAT_TOP])
        x1 = x0 + int(stats[largest, cv2.CC_STAT_WIDTH]); y1 = y0 + int(stats[largest, cv2.CC_STAT_HEIGHT])
        diag = math.hypot(x1 - x0, y1 - y0)
        for label in range(1, n):
            area = int(stats[label, cv2.CC_STAT_AREA])
            cx, cy = cents[label]
            near = (x0 - .06 * diag <= cx <= x1 + .06 * diag and
                    y0 - .06 * diag <= cy <= y1 + .06 * diag)
            if label == largest or (area >= max(16, int(cfg.min_component_fraction * largest_area)) and near):
                kept |= labels == label
                components.append(area)
    if not components and work.any():
        # ``connectedComponentsWithStats`` can return only a background label
        # for a one-pixel-wide numerical edge case; retain that valid mask
        # rather than turning a usable frame into an empty one.
        kept = work.copy()
    # Fill only tiny pinholes, preserving large true holes.
    inv = (~kept).astype(np.uint8)
    flood = inv.copy()
    ff_mask = np.zeros((flood.shape[0] + 2, flood.shape[1] + 2), np.uint8)
    cv2.floodFill(flood, ff_mask, (0, 0), 2)
    holes = (inv == 1) & (flood != 2)
    hole_count = 0
    hole_area = 0
    hn, hl, hs, _ = cv2.connectedComponentsWithStats(holes.astype(np.uint8), 8)
    object_area = max(int(kept.sum()), 1)
    for label in range(1, hn):
        area = int(hs[label, cv2.CC_STAT_AREA])
        if area <= max(16, int(cfg.min_hole_fraction * object_area)):
            kept[hl == label] = True
            hole_count += 1
            hole_area += area
    # The hand guard is applied again after hole filling.  This makes the
    # exclusion monotonic and guarantees gripper pixels never enter alpha.
    kept[guard] = False
    stats_out = {
        "components_kept": int(len(components)),
        "component_areas": components,
        "hole_count_filled": hole_count,
        "hole_area_filled_px": hole_area,
        "mask_changed_fraction": float(np.mean(kept != original)),
        "gripper_guard_fraction": float(guard.mean()),
    }
    return kept, stats_out


def _bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)


def _crop_bounds(mask: np.ndarray, padding: float) -> tuple[int, int, int, int] | None:
    box = _bbox(mask)
    if box is None:
        return None
    x0, y0, x1, y1 = box
    # ReLi3D's conditioning image is square.  Cropping a rectangular bbox and
    # then stretching it to square changes the observed object geometry (the
    # station views are usually taller than they are wide).  Use one isotropic
    # square crop instead; intrinsics then receive the same scale in x/y and
    # the image encoder sees the real silhouette aspect ratio.
    height, width = mask.shape[:2]
    side = int(math.ceil(max(x1 - x0, y1 - y0) * (1.0 + 2.0 * float(padding))))
    side = max(1, min(side, width, height))
    center_x = 0.5 * (x0 + x1)
    center_y = 0.5 * (y0 + y1)
    left = int(round(center_x - 0.5 * side))
    top = int(round(center_y - 0.5 * side))
    # Shift the square back into the sensor image without changing its side.
    left = min(max(left, 0), width - side)
    top = min(max(top, 0), height - side)
    return left, top, left + side, top + side


def _frame_qa(rgb: np.ndarray, depth_m: np.ndarray, mask: np.ndarray, hand: np.ndarray,
              clean: np.ndarray, clean_stats: dict[str, Any], cfg: ReLi3DInputConfig) -> dict[str, Any]:
    box = _bbox(clean)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    count = int(clean.sum())
    if box is None or count == 0:
        return {"accepted": False, "reject_reasons": ["EMPTY_MASK"], "object_pixels": 0,
                "depth_valid_ratio": 0.0, **clean_stats}
    x0, y0, x1, y1 = box
    values = gray[clean]
    valid = clean & np.isfinite(depth_m) & (depth_m > 0.03)
    clipped = float(np.mean((values < 3) | (values > 252)))
    hand_near = cv2.dilate(hand.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    reasons = []
    if count < cfg.min_object_pixels:
        reasons.append("LOW_OBJECT_PIXELS")
    if float(valid.sum() / max(count, 1)) < cfg.min_depth_valid_ratio:
        reasons.append("LOW_DEPTH_VALIDITY")
    luma = float(values.mean()) if values.size else 0.0
    if not cfg.min_luma <= luma <= cfg.max_luma:
        reasons.append("BAD_EXPOSURE")
    if clipped > cfg.max_clipped_fraction:
        reasons.append("CLIPPED_EXPOSURE")
    sharp = float(cv2.Laplacian(gray, cv2.CV_64F)[clean].var())
    if sharp < cfg.min_laplacian_variance:
        reasons.append("BLUR")
    # Segmentation labels may be disjoint; this conservative overlap proxy is
    # based on the hand silhouette entering the object bbox/dilated edge.
    hand_in_bbox = int(hand[y0:y1, x0:x1].sum())
    occ = float((hand_near & clean).sum() / max(count, 1))
    if occ > 0.02:
        reasons.append("GRIPPER_OCCLUSION")
    return {
        "accepted": not reasons,
        "reject_reasons": reasons,
        "object_pixels": count,
        "object_fraction": float(clean.mean()),
        "bbox_xyxy": [x0, y0, x1, y1],
        "bbox_width_fraction": float((x1 - x0) / rgb.shape[1]),
        "bbox_height_fraction": float((y1 - y0) / rgb.shape[0]),
        "depth_valid_ratio": float(valid.sum() / max(count, 1)),
        "depth_median_m": float(np.median(depth_m[valid])) if valid.any() else None,
        "luma_mean": luma,
        "clipped_fraction": clipped,
        "laplacian_variance": sharp,
        "gripper_pixels": int((hand > 0).sum()),
        "gripper_pixels_in_bbox": hand_in_bbox,
        "gripper_occlusion_ratio": occ,
        **clean_stats,
    }


def _project_depth(depth_m: np.ndarray, mask: np.ndarray, K: np.ndarray,
                   T_base_camera: np.ndarray, max_points: int = 12000) -> np.ndarray:
    valid = mask & np.isfinite(depth_m) & (depth_m > 0.03)
    ys, xs = np.where(valid)
    if len(xs) > max_points:
        # Deterministic spatial subsampling keeps frame balance.
        ids = np.linspace(0, len(xs) - 1, max_points).astype(np.int64)
        xs, ys = xs[ids], ys[ids]
    if len(xs) == 0:
        return np.empty((0, 3), dtype=np.float64)
    z = depth_m[ys, xs].astype(np.float64)
    x = (xs - K[0, 2]) * z / K[0, 0]
    y = (ys - K[1, 2]) * z / K[1, 1]
    pc = np.c_[x, y, z]
    return pc @ T_base_camera[:3, :3].T + T_base_camera[:3, 3]


def estimate_object_frame(records: list[dict[str, Any]], dataset: Path, K: np.ndarray,
                          cleaned: dict[int, np.ndarray]) -> tuple[np.ndarray, dict[str, Any], np.ndarray]:
    points = []
    for i, row in enumerate(records):
        if i not in cleaned:
            continue
        depth, _ = _read_depth(_path(dataset, row["depth"]))
        T = np.asarray(row.get("T_base_camera", row.get("T_B_camera")), dtype=float)
        points.append(_project_depth(depth, cleaned[i], K, T))
    all_points = np.concatenate([p for p in points if len(p)], axis=0) if any(len(p) for p in points) else np.empty((0, 3))
    if len(all_points) < 100:
        raise RuntimeError("insufficient RGB-D points to estimate object frame")
    lo = np.percentile(all_points, 1.0, axis=0)
    hi = np.percentile(all_points, 99.0, axis=0)
    origin = (lo + hi) / 2.0
    T_base_object = np.eye(4, dtype=float)
    T_base_object[:3, 3] = origin
    info = {
        "gt_used": False,
        "source": "RGB-D mask projection with T_base_camera; robust 1/99 percent bounds",
        "axis_convention": "base axes retained; origin is robust bounds midpoint",
        "T_base_object_estimate": T_base_object.tolist(),
        "robust_bounds_base_m": [lo.tolist(), hi.tolist()],
        "extent_m": (hi - lo).tolist(),
        "point_count": int(len(all_points)),
    }
    return T_base_object, info, all_points - origin


def _view_direction(T_object_camera_station: np.ndarray, object_center: np.ndarray | None = None) -> np.ndarray:
    c = T_object_camera_station[:3, 3]
    if object_center is not None:
        c = c - object_center
    n = np.linalg.norm(c)
    return c / max(n, 1e-12)


def _angular_distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))))


def _select_views(rows: list[dict[str, Any]], qas: list[dict[str, Any]], dirs: list[np.ndarray], cfg: ReLi3DInputConfig) -> list[int]:
    candidates = [i for i, q in enumerate(qas) if q.get("accepted")]
    if not candidates:
        raise RuntimeError("no quality scan frames survive")
    # Greedy farthest-point selection provides a deterministic mix of rings and
    # azimuths while retaining sharp, large, well-exposed views.
    def score(i: int) -> float:
        q = qas[i]
        return (math.log1p(q.get("object_pixels", 0)) + 0.4 * math.log1p(q.get("laplacian_variance", 0.0))
                + 20.0 * q.get("depth_valid_ratio", 0.0) - 30.0 * q.get("gripper_occlusion_ratio", 0.0))
    selected: list[int] = []
    # Seed with the best horizontal and high view when available.
    for ring in ("horizontal", "high"):
        pool = [i for i in candidates if rows[i].get("ring") == ring or rows[i].get("scan_view", {}).get("ring") == ring]
        if pool:
            best = max(pool, key=score)
            if best not in selected:
                selected.append(best)
    while len(selected) < min(cfg.max_views, len(candidates)):
        remaining = [i for i in candidates if i not in selected]
        if not remaining:
            break
        if not selected:
            selected.append(max(remaining, key=score)); continue
        def utility(i: int) -> float:
            novelty = min(_angular_distance(dirs[i], dirs[j]) for j in selected)
            return score(i) + 0.08 * novelty
        nxt = max(remaining, key=utility)
        if min(_angular_distance(dirs[nxt], dirs[j]) for j in selected) < cfg.min_view_separation_deg and len(selected) < len(candidates):
            # Once directional coverage is saturated, allow the best remaining
            # frame rather than silently dropping an otherwise valid observation.
            if all(min(_angular_distance(dirs[i], dirs[j]) for j in selected) < cfg.min_view_separation_deg for i in remaining):
                selected.append(max(remaining, key=score)); continue
        selected.append(nxt)
    return selected


def _fov_from_intrinsics(K: np.ndarray, width: int, height: int) -> list[float]:
    return [float(2.0 * math.atan(width / (2.0 * max(K[0, 0], 1e-9)))),
            float(2.0 * math.atan(height / (2.0 * max(K[1, 1], 1e-9))))]


def _contact_sheet(images: list[tuple[np.ndarray, str]], output: Path, cols: int = 6, tile=(240, 220)) -> None:
    if not images:
        return
    tw, th = tile
    rows = int(math.ceil(len(images) / cols))
    sheet = np.zeros((rows * th, cols * tw, 3), np.uint8)
    for idx, (image, label) in enumerate(images):
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        canvas = np.zeros((th, tw, 3), np.uint8)
        scale = min((tw - 8) / image.shape[1], (th - 30) / image.shape[0])
        resized = cv2.resize(image, (max(1, int(image.shape[1] * scale)), max(1, int(image.shape[0] * scale))))
        y = 24 + (th - 24 - resized.shape[0]) // 2; x = (tw - resized.shape[1]) // 2
        canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        cv2.putText(canvas, label, (5, 17), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1, cv2.LINE_AA)
        r, c = divmod(idx, cols); sheet[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = canvas
    cv2.imwrite(str(output), sheet)


def prepare_input(dataset: Path, output: Path, cfg: ReLi3DInputConfig | None = None) -> dict[str, Any]:
    """Create ReLi3D's ``object/transforms.json`` and QA mosaics."""
    cfg = cfg or ReLi3DInputConfig()
    dataset, output = Path(dataset).resolve(), Path(output).resolve()
    if output.exists():
        shutil.rmtree(output)
    object_name = "scan_object"
    object_dir = output / object_name
    for name in ("rgba", "rgb", "masks", "depth", "poses"):
        (object_dir / name).mkdir(parents=True, exist_ok=True)
    K0 = np.loadtxt(dataset / "cam_K.txt").reshape(3, 3)
    rows = _records(dataset)
    qas: list[dict[str, Any]] = []
    cleaned: dict[int, np.ndarray] = {}
    directions_station: list[np.ndarray] = []
    loaded: dict[int, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]] = {}
    for i, row in enumerate(rows):
        rgb = cv2.imread(str(_path(dataset, row["rgb"])), cv2.IMREAD_COLOR)
        depth, depth_scale = _read_depth(_path(dataset, row["depth"]))
        mask = cv2.imread(str(_path(dataset, row["object_mask"])), cv2.IMREAD_GRAYSCALE)
        hand_key = row.get("gripper_mask") or row.get("hand_mask")
        hand = cv2.imread(str(_path(dataset, hand_key)), cv2.IMREAD_GRAYSCALE) if hand_key else np.zeros_like(mask)
        if rgb is None or mask is None or hand is None:
            qas.append({"accepted": False, "reject_reasons": ["MISSING_FRAME"]}); directions_station.append(np.zeros(3)); continue
        clean, clean_stats = _largest_clean_component(mask > 0, hand > 0, cfg)
        qa = _frame_qa(rgb, depth, clean, hand > 0, clean, clean_stats, cfg)
        qa.update({"source_index": i, "source_frame_id": int(row.get("frame_id", i)), "pass_id": int(row.get("pass_id", row.get("scan_view", {}).get("pass_id", 0))), "ring": row.get("ring", row.get("scan_view", {}).get("ring")), "azimuth_deg": row.get("azimuth_deg", row.get("scan_view", {}).get("azimuth_deg")), "elevation_deg": row.get("elevation_deg", row.get("scan_view", {}).get("elevation_deg")), "depth_scale_m_per_unit": depth_scale})
        qas.append(qa); loaded[i] = (rgb, depth, clean, hand > 0, qa)
        cleaned[i] = clean
        T = np.asarray(row.get("T_base_camera", row.get("T_B_camera")), dtype=float)
        directions_station.append((T[:3, 3] - np.array([.0, .0, .0])) / max(np.linalg.norm(T[:3, 3]), 1e-12))
    # Reject quality-failed frames before estimating the common object frame;
    # an exposed table, blurred mask, or invalid depth must not move the metric
    # origin used by every accepted view.
    frame_cleaned = {i: m for i, m in cleaned.items() if qas[i].get("accepted", False)}
    if not frame_cleaned:
        raise RuntimeError("no quality scan frames survive for object-frame estimation")
    T_base_object, frame_info, points_object = estimate_object_frame(rows, dataset, K0, frame_cleaned)
    T_object_base = np.linalg.inv(T_base_object)
    # Recompute camera directions in the estimated object frame.
    for i, row in enumerate(rows):
        Tbc = np.asarray(row.get("T_base_camera", row.get("T_B_camera")), dtype=float)
        Toc_station = T_object_base @ Tbc
        directions_station[i] = _view_direction(Toc_station)
    selected = _select_views(rows, qas, directions_station, cfg)
    # Keep robot/RGB-D metric poses immutable, while putting the c2w magnitude
    # in the range used by the released ReLi3D checkpoint.  This is a model
    # coordinate normalization, not a GT-derived scale estimate; metric scale
    # is recovered later from the measured depth cloud.
    metric_c2ws: dict[int, np.ndarray] = {}
    camera_radii: list[float] = []
    for i in selected:
        row = rows[i]
        Tbc = np.asarray(row.get("T_base_camera", row.get("T_B_camera")), dtype=float)
        T_metric = T_object_base @ Tbc @ CAMERA_STATION_TO_OGL
        metric_c2ws[i] = T_metric
        camera_radii.append(float(np.linalg.norm(T_metric[:3, 3])))
    median_camera_radius = float(np.median(camera_radii)) if camera_radii else 0.0
    if cfg.model_camera_radius > 0.0 and median_camera_radius > 1e-9:
        model_pose_scale = float(cfg.model_camera_radius / median_camera_radius)
    else:
        model_pose_scale = 1.0
    frames = []
    mosaics_rgb: list[tuple[np.ndarray, str]] = []
    mosaics_mask: list[tuple[np.ndarray, str]] = []
    for out_index, i in enumerate(selected):
        row = rows[i]; rgb, depth, clean, hand, qa = loaded[i]
        bounds = _crop_bounds(clean, cfg.crop_padding)
        if bounds is None:
            continue
        x0, y0, x1, y1 = bounds
        crop_rgb = rgb[y0:y1, x0:x1]; crop_mask = clean[y0:y1, x0:x1]
        sx, sy = cfg.image_size / crop_rgb.shape[1], cfg.image_size / crop_rgb.shape[0]
        crop_rgb = cv2.resize(crop_rgb, (cfg.image_size, cfg.image_size), interpolation=cv2.INTER_AREA)
        crop_mask = cv2.resize(crop_mask.astype(np.uint8), (cfg.image_size, cfg.image_size), interpolation=cv2.INTER_NEAREST) > 0
        K = np.array([[K0[0, 0] * sx, 0., (K0[0, 2] - x0) * sx],
                      [0., K0[1, 1] * sy, (K0[1, 2] - y0) * sy], [0., 0., 1.]], dtype=float)
        stem = f"{out_index:04d}"
        rgba = cv2.cvtColor(crop_rgb, cv2.COLOR_BGR2RGBA); rgba[..., 3] = crop_mask.astype(np.uint8) * 255
        cv2.imwrite(str(object_dir / "rgba" / f"{stem}.png"), rgba)
        cv2.imwrite(str(object_dir / "rgb" / f"{stem}.png"), crop_rgb)
        cv2.imwrite(str(object_dir / "masks" / f"{stem}.png"), crop_mask.astype(np.uint8) * 255)
        # Do not let table/background depth bleed through a resized silhouette.
        # A tiny morphology close can add edge pixels whose sensor depth belongs
        # to the table.  A robust per-frame depth envelope removes those pixels
        # without eroding the RGB/mask silhouette; the raw acquisition remains
        # immutable.
        depth_values = depth[clean & np.isfinite(depth) & (depth > 0.03)]
        if len(depth_values) >= 32:
            dlo, dhi = np.percentile(depth_values, [0.5, 99.5])
            depth_valid = clean & (depth >= max(0.03, dlo - 0.01)) & (depth <= dhi + 0.01)
        else:
            depth_valid = clean & np.isfinite(depth) & (depth > 0.03)
        depth_crop = np.where(depth_valid[y0:y1, x0:x1], depth[y0:y1, x0:x1], 0.0)
        depth_out = cv2.resize(depth_crop, (cfg.image_size, cfg.image_size), interpolation=cv2.INTER_NEAREST)
        # Store millimetres for unambiguous audit, with an explicit scale field.
        cv2.imwrite(str(object_dir / "depth" / f"{stem}.png"), np.clip(depth_out * 1000., 0, 65535).astype(np.uint16))
        cv2.imwrite(str(object_dir / "poses" / f"{stem}.json"), np.zeros((1, 1), np.uint8)) if False else None
        Tbc = np.asarray(row.get("T_base_camera", row.get("T_B_camera")), dtype=float)
        Toc_metric = metric_c2ws[i]
        Toc_ogl = Toc_metric.copy()
        Toc_ogl[:3, 3] *= model_pose_scale
        fov = _fov_from_intrinsics(K, cfg.image_size, cfg.image_size)
        frame = {
            "view_index": out_index, "source_index": i, "source_frame_id": int(row.get("frame_id", i)),
            "pass_id": int(row.get("pass_id", row.get("scan_view", {}).get("pass_id", 0))),
            "ring": row.get("ring", row.get("scan_view", {}).get("ring")),
            "azimuth_deg": row.get("azimuth_deg", row.get("scan_view", {}).get("azimuth_deg")),
            "elevation_deg": row.get("elevation_deg", row.get("scan_view", {}).get("elevation_deg")),
            "file_path": f"{object_name}/rgba/{stem}.png" if False else f"rgba/{stem}.png",
            "width": cfg.image_size, "height": cfg.image_size,
            "transform_matrix": Toc_ogl.tolist(),
            "metric_transform_matrix": Toc_metric.tolist(),
            "model_pose_scale": model_pose_scale,
            "camera_fov": fov,
            "camera_principal_point": [float(K[0, 2]), float(K[1, 2])],
            "intrinsics": K.tolist(), "T_base_camera": Tbc.tolist(),
            "camera_convention": "station ROS right/down/+Z -> ReLi3D OGL right/up/-Z via diag(1,-1,-1)",
            "crop_bbox_xyxy": [x0, y0, x1, y1], "crop_padding": cfg.crop_padding,
            "source_rgb": str(_path(dataset, row["rgb"])), "source_depth": str(_path(dataset, row["depth"])),
            "source_mask": str(_path(dataset, row["object_mask"])), "source_gripper_mask": str(_path(dataset, row.get("gripper_mask") or row.get("hand_mask"))),
            "qa": qa,
        }
        frames.append(frame)
        overlay = rgb.copy(); overlay[clean] = (0.55 * overlay[clean] + 0.45 * np.array([0, 220, 0])).astype(np.uint8); overlay[hand] = (0.55 * overlay[hand] + 0.45 * np.array([0, 0, 255])).astype(np.uint8)
        cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 0), 2)
        mosaics_rgb.append((rgb, f"{out_index:02d} src{i:02d}"))
        mosaics_mask.append((overlay, f"{out_index:02d} src{i:02d}"))
        (object_dir / "poses" / f"{stem}.json").write_text(json.dumps(frame, indent=2))
    transforms = {
        "object_uid": f"reli3d_scan_{dataset.name}", "coordinate_system": "ogl", "dataset_is_repaired": True,
        "gt_used": False, "camera_pose_source": "robot T_base_camera", "object_frame": frame_info,
        "frames": frames,
    }
    (object_dir / "transforms.json").write_text(json.dumps(transforms, indent=2))
    # The official CLI expects object directories directly under input-root.
    # Keep a root-level copy of transforms metadata for scripts and humans.
    _contact_sheet(mosaics_rgb, output / "rgb_contact_sheet.png")
    _contact_sheet(mosaics_mask, output / "mask_overlay_contact_sheet.png")
    selected_dirs = [directions_station[i].tolist() for i in selected if i < len(directions_station)]
    adjacent = []
    for a, b in zip(frames[:-1], frames[1:]):
        ma = cv2.imread(str(object_dir / "masks" / f"{int(a['view_index']):04d}.png"), 0) > 0
        mb = cv2.imread(str(object_dir / "masks" / f"{int(b['view_index']):04d}.png"), 0) > 0
        inter = int((ma & mb).sum()); union = int((ma | mb).sum()); adjacent.append(float(inter / max(union, 1)))
    report = {
        "schema": "fr3_reli3d_input/v1", "gt_used_for_preparation": False,
        "dataset": str(dataset), "input_root": str(output), "object_dir": str(object_dir),
        "config": cfg.__dict__, "input_frames": len(rows), "quality_accepted": int(sum(q.get("accepted", False) for q in qas)),
        "selected_views": len(frames), "selected_source_indices": [int(x["source_index"]) for x in frames],
        "selected_view_records": frames, "all_frame_qa": qas,
        "view_direction_coverage": {
            "directions_object_frame": selected_dirs,
            "unique_azimuth_bins": int(len({int(round((math.degrees(math.atan2(d[1], d[0])) % 360) / 30)) for d in selected_dirs})) if selected_dirs else 0,
        },
        "pairwise_adjacent_mask_iou_mean": float(np.mean(adjacent)) if adjacent else 0.0,
        "pairwise_adjacent_mask_iou_min": float(np.min(adjacent)) if adjacent else 0.0,
        "model_pose_normalization": {
            "enabled": bool(abs(model_pose_scale - 1.0) > 1e-9),
            "target_camera_radius": float(cfg.model_camera_radius),
            "median_metric_camera_radius": median_camera_radius,
            "translation_scale_for_reli3d": model_pose_scale,
            "metric_transforms_preserved": True,
        },
        "object_frame": frame_info, "measured_points_object_count": int(len(points_object)),
        "products": {"transforms": str(object_dir / "transforms.json"), "rgb_contact_sheet": str(output / "rgb_contact_sheet.png"), "mask_overlay_contact_sheet": str(output / "mask_overlay_contact_sheet.png")},
    }
    (output / "input_report.json").write_text(json.dumps(report, indent=2))
    return report


def _sample_mesh(mesh: trimesh.Trimesh, n: int = 30000) -> np.ndarray:
    state = np.random.get_state(); np.random.seed(20260914)
    try:
        return mesh.sample(min(n, max(1000, len(mesh.faces) * 3)))
    finally:
        np.random.set_state(state)


def _load_rgbd_points(input_report: dict[str, Any], input_root: Path) -> tuple[np.ndarray, np.ndarray]:
    object_dir = Path(input_report["object_dir"])
    frame_points = []
    all_points = []
    for frame in input_report["selected_view_records"]:
        stem = f"{int(frame['view_index']):04d}"
        depth = cv2.imread(str(object_dir / "depth" / f"{stem}.png"), cv2.IMREAD_UNCHANGED).astype(np.float64) / 1000.0
        mask = cv2.imread(str(object_dir / "masks" / f"{stem}.png"), 0) > 0
        K = np.asarray(frame["intrinsics"], dtype=float)
        # ``transform_matrix`` is normalized for ReLi3D's learned coordinate
        # range.  RGB-D points must use the original robot metric c2w kept
        # alongside it.
        T = np.asarray(frame.get("metric_transform_matrix", frame["transform_matrix"]), dtype=float)
        ys, xs = np.where(mask & np.isfinite(depth) & (depth > .03))
        if len(xs) > 10000:
            ids = np.linspace(0, len(xs) - 1, 10000).astype(int); xs, ys = xs[ids], ys[ids]
        z = depth[ys, xs]; pc = np.c_[(xs - K[0, 2]) * z / K[0, 0], -(ys - K[1, 2]) * z / K[1, 1], -z]
        pts = pc @ T[:3, :3].T + T[:3, 3]
        frame_points.append(pts); all_points.append(pts)
    if not all_points:
        raise RuntimeError("no RGB-D points in ReLi3D input")
    # Return per-frame arrays for coverage/conflict audit and one aggregate cloud.
    return np.concatenate(all_points, axis=0), np.asarray([len(p) for p in frame_points], dtype=int)


def _pca_align(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, dict[str, Any]]:
    """Rigid orientation + uniform scale from measured RGB-D, no GT."""
    cs, ct = source.mean(0), target.mean(0)
    xs, xt = source - cs, target - ct
    _, _, Vts = np.linalg.svd(xs.T @ xs)
    _, _, Vtt = np.linalg.svd(xt.T @ xt)
    axes_s, axes_t = Vts.T, Vtt.T
    source_extent = np.percentile(source, 99.0, axis=0) - np.percentile(source, 1.0, axis=0)
    target_extent = np.percentile(target, 99.0, axis=0) - np.percentile(target, 1.0, axis=0)
    # Enumerate PCA axis permutations/signs; score with a cheap symmetric
    # nearest-neighbour Chamfer.  This handles model-canonical orientation.
    import itertools
    best = None
    tree_t = cKDTree(target)
    for perm in itertools.permutations(range(3)):
        P = axes_t[:, perm] @ axes_s.T
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            R = P @ np.diag(signs)
            if np.linalg.det(R) < 0:
                continue
            yp = xs @ R.T
            # The two clouds are deliberately sampled at different densities;
            # estimate metric scale from robust extents instead of assuming a
            # one-to-one point correspondence.
            src_rot_extent = np.percentile(yp, 99.0, axis=0) - np.percentile(yp, 1.0, axis=0)
            scale = float(np.median(target_extent / np.maximum(src_rot_extent, 1e-9)))
            scale = abs(scale)
            pred = yp * scale + ct
            # Subsample to keep alignment deterministic and inexpensive.
            ids = np.linspace(0, len(pred) - 1, min(len(pred), 12000)).astype(int)
            d1 = tree_t.query(pred[ids])[0]
            tree_p = cKDTree(pred)
            ids_t = np.linspace(0, len(target) - 1, min(len(target), 12000)).astype(int)
            d2 = tree_p.query(target[ids_t])[0]
            score = float(np.median(d1) + np.median(d2))
            if best is None or score < best[0]:
                best = (score, R, scale, pred, (cs, ct))
    if best is None:
        raise RuntimeError("PCA metric alignment failed")
    _, R, scale, pred, centers = best
    t = centers[1] - scale * (centers[0] @ R.T)
    return R, scale, t, {"pca_source_center": centers[0].tolist(), "measured_center": centers[1].tolist()}


def metric_align_and_constrain(raw_mesh: Path, input_report_path: Path, output: Path,
                               support_mm: float = 5.0, conflict_mm: float = 10.0,
                               blend: float = .80) -> dict[str, Any]:
    """Align ReLi3D mesh to RGB-D metric frame and constrain observed vertices."""
    raw_mesh, input_report_path, output = Path(raw_mesh), Path(input_report_path), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = json.loads(input_report_path.read_text())
    measured, _ = _load_rgbd_points(report, Path(report["input_root"]))
    loaded = trimesh.load(raw_mesh, force="mesh", process=False)
    if not isinstance(loaded, trimesh.Trimesh):
        raise RuntimeError(f"ReLi3D output is not a single mesh: {raw_mesh}")
    mesh = loaded.copy()
    source_vertices = np.asarray(mesh.vertices, dtype=float)
    # Use surface samples for orientation search, then apply the transform to
    # the complete UV mesh.  All correspondences come from measured RGB-D.
    source_samples = _sample_mesh(mesh, 18000)
    R, scale, t, align_extra = _pca_align(source_samples, measured)
    mesh.apply_transform(np.block([[scale * R, t.reshape(3, 1)], [np.zeros((1, 3)), np.ones((1, 1))]]))
    vertices = np.asarray(mesh.vertices, dtype=float)
    tree = cKDTree(measured)
    distances, ids = tree.query(vertices, k=1)
    observed = distances <= support_mm / 1000.0
    # Snap only generated vertices already supported by real observations.  A
    # weighted local average avoids copying single-pixel depth outliers.
    if np.any(observed):
        local_k = min(8, len(measured))
        neigh_d, neigh_i = tree.query(vertices[observed], k=local_k)
        if local_k == 1:
            neigh_d = neigh_d[:, None]; neigh_i = neigh_i[:, None]
        local = measured[neigh_i]
        weights = 1.0 / np.maximum(neigh_d, 1e-5) ** 2
        target_v = (local * weights[..., None]).sum(1) / np.maximum(weights.sum(1, keepdims=True), 1e-12)
        vertices[observed] = (1.0 - blend) * vertices[observed] + blend * target_v
        mesh.vertices = vertices
    # Conflict is reported on generated surface samples, not only vertices.
    aligned_samples = _sample_mesh(mesh, 30000)
    d_s = cKDTree(measured).query(aligned_samples)[0]
    conflict = d_s > conflict_mm / 1000.0
    metric_obj = output / "metric_constrained_mesh.obj"
    metric_glb = output / "metric_constrained_mesh.glb"
    mesh.export(metric_obj); mesh.export(metric_glb)
    measured_lo, measured_hi = measured.min(0), measured.max(0)
    mesh_lo, mesh_hi = vertices.min(0), vertices.max(0)
    result = {
        "schema": "fr3_reli3d_metric_constraint/v1", "gt_used": False,
        "source_mesh": str(raw_mesh), "metric_mesh": str(metric_obj), "metric_glb": str(metric_glb),
        "alignment": {"uniform_scale": float(scale), "rotation": R.tolist(), "translation_m": t.tolist(), **align_extra},
        "rgbd_measurement": {"point_count": int(len(measured)), "aabb_min_m": measured_lo.tolist(), "aabb_max_m": measured_hi.tolist(), "extent_m": (measured_hi - measured_lo).tolist()},
        "mesh_aabb_m": {"min": mesh_lo.tolist(), "max": mesh_hi.tolist(), "extent": (mesh_hi - mesh_lo).tolist()},
        "observed_vertex_fraction": float(observed.mean()), "generated_only_vertex_fraction": float((~observed).mean()),
        "generated_surface_conflict_fraction": float(conflict.mean()), "support_radius_mm": support_mm, "conflict_radius_mm": conflict_mm,
        "constraint_blend": blend,
        "products": {"obj": str(metric_obj), "glb": str(metric_glb)},
    }
    (output / "metric_constraint_report.json").write_text(json.dumps(result, indent=2))
    return result


def extract_texture_and_obj(metric_glb: Path, output: Path) -> dict[str, Any]:
    """Export the ReLi3D UV mesh plus its native base-color texture."""
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(metric_glb, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError("metric GLB did not load as a mesh")
    texture_path = output / "reli3d_basecolor.png"
    material = getattr(getattr(mesh, "visual", None), "material", None)
    tex = getattr(material, "baseColorTexture", None) if material is not None else None
    texture_source = "reli3d_native_basecolor"
    if tex is not None:
        try:
            tex.save(texture_path)
        except Exception:
            arr = np.asarray(tex)
            cv2.imwrite(str(texture_path), arr[..., ::-1] if arr.ndim == 3 else arr)
    else:
        # Keep the product explicit rather than silently claiming a native
        # texture.  A neutral fallback lets USD/CoACD validation proceed.
        texture_source = "neutral_fallback_missing_reli3d_texture"
        cv2.imwrite(str(texture_path), np.full((4, 4, 3), 128, np.uint8))
    obj = output / "textured_mesh.obj"
    mesh.export(obj)
    result = {"schema": "fr3_reli3d_texture/v1", "mesh": str(obj), "texture": str(texture_path), "source": texture_source,
              "has_uv": bool(getattr(mesh.visual, "uv", None) is not None), "vertex_count": int(len(mesh.vertices)), "face_count": int(len(mesh.faces))}
    (output / "texture_report.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Prepare ReLi3D input or metric-constrain its mesh")
    sub = p.add_subparsers(dest="command", required=True)
    a = sub.add_parser("prepare"); a.add_argument("dataset", type=Path); a.add_argument("output", type=Path)
    b = sub.add_parser("metric"); b.add_argument("mesh", type=Path); b.add_argument("input_report", type=Path); b.add_argument("output", type=Path)
    args = p.parse_args()
    result = prepare_input(args.dataset, args.output) if args.command == "prepare" else metric_align_and_constrain(args.mesh, args.input_report, args.output)
    print(json.dumps(result, indent=2))
