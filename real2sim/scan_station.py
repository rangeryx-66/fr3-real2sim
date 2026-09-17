"""Independent scan-station acquisition and RGB-D-only object-frame setup.

The grasp executor is deliberately outside this module.  The station skill
starts at the post-placement state: the object is released on a clean table
patch, the arm is parked, and only then are camera views acquired.  RGB-D and
the measured camera poses create the object frame used by metric alignment;
the simulator's GT pose is written under ``eval_gt`` for evaluation only.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
from urllib.request import Request, urlopen

import cv2
import numpy as np


def _look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return a ROS-camera rotation whose +Z axis points at ``target``."""
    z = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
    z /= max(float(np.linalg.norm(z)), 1e-12)
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z, up))) > 0.98:
        up = np.array([0.0, 1.0, 0.0])
    x = np.cross(z, up)
    x /= max(float(np.linalg.norm(x)), 1e-12)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


@dataclass(frozen=True)
class ScanView:
    index: int
    ring: str
    azimuth_deg: float
    elevation_deg: float
    position: tuple[float, float, float]
    look_at: tuple[float, float, float]

    @property
    def pass_id(self) -> int:
        return 0 if self.ring == "horizontal" else 1


@dataclass(frozen=True)
class ScanStationConfig:
    # A clean patch toward the far side of the Arena table keeps the fixed FR3
    # base outside the near-field camera frustum for every azimuth.
    center_m: tuple[float, float, float] = (0.70, 0.25, 0.052)
    radius_m: float = 0.30
    horizontal_elevation_deg: float = 14.0
    high_elevation_deg: float = 34.0
    views_per_ring: int = 12
    crop_padding: float = 0.15
    min_bbox_height_fraction: float = 0.40
    max_bbox_height_fraction: float = 0.70
    min_depth_valid_ratio: float = 0.985
    max_gripper_occlusion: float = 0.03
    # The bbox ratio catches a hand crossing the target.  This image-wide
    # limit catches a parked arm that is visible elsewhere in the frame, so
    # the ReconViaGen input remains a clean object-only crop.
    max_full_gripper_fraction: float = 0.002
    minimum_accepted_views: int = 18

    def views(self) -> list[ScanView]:
        if self.views_per_ring < 4:
            raise ValueError("views_per_ring must be at least four")
        center = np.asarray(self.center_m, dtype=float)
        result: list[ScanView] = []
        index = 0
        # Both rings use 30-degree spacing by default.  This keeps adjacent
        # views highly overlapping while the second ring changes elevation and
        # exposes the top and upper side wall.
        for ring, elevation in (
            ("horizontal", self.horizontal_elevation_deg),
            ("high", self.high_elevation_deg),
        ):
            er = math.radians(elevation)
            horizontal = self.radius_m * math.cos(er)
            z = center[2] + self.radius_m * math.sin(er)
            for j in range(self.views_per_ring):
                az = 360.0 * j / self.views_per_ring
                ar = math.radians(az)
                position = (center[0] + horizontal * math.cos(ar),
                            center[1] + horizontal * math.sin(ar), z)
                result.append(ScanView(
                    index=index, ring=ring, azimuth_deg=az,
                    elevation_deg=elevation, position=tuple(map(float, position)),
                    look_at=tuple(map(float, center)),
                ))
                index += 1
        return result


class StationClient:
    """Small HTTP client so the skill does not alter ``src/plant.py``."""

    def __init__(self, port: int = 18930, timeout_s: float = 30.0):
        self.base = f"http://127.0.0.1:{int(port)}"
        self.timeout_s = timeout_s

    def state(self, endpoint: str = "calibration") -> dict[str, Any]:
        with urlopen(self.base + "/" + endpoint, timeout=self.timeout_s) as f:
            return json.load(f)

    def submit(self, command: dict[str, Any]) -> str:
        req = Request(
            self.base,
            data=json.dumps(command).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urlopen(req, timeout=self.timeout_s) as f:
            return str(json.load(f)["id"])

    def wait(self, token: str, timeout_s: float = 120.0) -> dict[str, Any]:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            payload = self.state("control")
            result = payload.get("results", {}).get(token)
            if result is not None:
                return result
            time.sleep(0.05)
        try:
            self.submit({"op": "stop"})
        finally:
            raise TimeoutError(f"scan-station command timed out: {token}")

    def command(self, command: dict[str, Any], timeout_s: float = 120.0) -> dict[str, Any]:
        return self.wait(self.submit(command), timeout_s)


def _bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.where(mask)
    if not len(xs):
        return None
    return int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)


def _crop_bounds(mask: np.ndarray, padding: float) -> tuple[int, int, int, int] | None:
    box = _bbox(mask)
    if box is None:
        return None
    x0, y0, x1, y1 = box
    pad = float(padding) * max(x1 - x0, y1 - y0)
    return (
        max(int(math.floor(x0 - pad)), 0),
        max(int(math.floor(y0 - pad)), 0),
        min(int(math.ceil(x1 + pad)), mask.shape[1]),
        min(int(math.ceil(y1 + pad)), mask.shape[0]),
    )


def _frame_quality(rgb: np.ndarray, depth_m: np.ndarray,
                   mask: np.ndarray, hand: np.ndarray,
                   cfg: ScanStationConfig) -> dict[str, Any]:
    box = _bbox(mask)
    h, w = mask.shape
    if box is None:
        return {
            "object_pixels": 0, "object_fraction": 0.0,
            "bbox_height_fraction": 0.0, "bbox_width_fraction": 0.0,
            "depth_valid_ratio": 0.0, "gripper_occlusion_ratio": 1.0,
            "gripper_pixels": int(hand.sum()),
            "full_gripper_fraction": float(hand.mean()),
            "accepted": False, "reject_reason": "EMPTY_OBJECT_MASK",
        }
    x0, y0, x1, y1 = box
    area = max((x1 - x0) * (y1 - y0), 1)
    valid = mask & np.isfinite(depth_m) & (depth_m > 0.03)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    luma = gray[mask]
    # Segmentation labels are disjoint, so direct mask intersection is zero
    # even when a gripper hides the object.  Count hand pixels in the object
    # bbox as the conservative occlusion proxy.
    hand_in_bbox = int(hand[y0:y1, x0:x1].sum())
    full_gripper_fraction = float(hand.mean())
    depth_ratio = float(valid.sum() / max(int(mask.sum()), 1))
    hfrac = float((y1 - y0) / max(h, 1))
    wfrac = float((x1 - x0) / max(w, 1))
    occ = float(hand_in_bbox / area)
    clipped = float(np.mean((luma < 4) | (luma > 251))) if luma.size else 1.0
    accepted = bool(
        cfg.min_bbox_height_fraction <= hfrac <= cfg.max_bbox_height_fraction
        and depth_ratio >= cfg.min_depth_valid_ratio
        and occ <= cfg.max_gripper_occlusion
        and full_gripper_fraction <= cfg.max_full_gripper_fraction
        and 8.0 <= (float(luma.mean()) if luma.size else 0.0) <= 250.0
        and clipped <= 0.35
    )
    reason = None
    if not accepted:
        if not cfg.min_bbox_height_fraction <= hfrac <= cfg.max_bbox_height_fraction:
            reason = "BAD_FRAMING"
        elif depth_ratio < cfg.min_depth_valid_ratio:
            reason = "BAD_DEPTH"
        elif (occ > cfg.max_gripper_occlusion
              or full_gripper_fraction > cfg.max_full_gripper_fraction):
            reason = "GRIPPER_OCCLUSION"
        elif not luma.size or not 8.0 <= float(luma.mean()) <= 250.0:
            reason = "BAD_EXPOSURE"
        else:
            reason = "CLIPPED_EXPOSURE"
    return {
        "object_pixels": int(mask.sum()),
        "object_fraction": float(mask.mean()),
        "bbox_xyxy": [x0, y0, x1, y1],
        "bbox_height_fraction": hfrac,
        "bbox_width_fraction": wfrac,
        "depth_valid_ratio": depth_ratio,
        "gripper_pixels": int(hand.sum()),
        "gripper_pixels_in_bbox": hand_in_bbox,
        "gripper_occlusion_ratio": occ,
        "full_gripper_fraction": full_gripper_fraction,
        "luma_mean": float(luma.mean()) if luma.size else 0.0,
        "clipped_fraction": clipped,
        "accepted": accepted,
        "reject_reason": reason,
    }


def _copy_crop(record: dict[str, Any], output: Path, padding: float) -> dict[str, Any]:
    output = Path(output)
    rgb = cv2.imread(str(record["rgb"]), cv2.IMREAD_COLOR)
    mask = cv2.imread(str(record["object_mask"]), cv2.IMREAD_GRAYSCALE) > 0
    hand_path = record.get("gripper_mask") or record.get("hand_mask")
    hand = cv2.imread(str(hand_path), cv2.IMREAD_GRAYSCALE) > 0
    bounds = _crop_bounds(mask, padding)
    if rgb is None or bounds is None:
        raise RuntimeError(f"cannot crop scan frame {record}")
    x0, y0, x1, y1 = bounds
    for name in ("rgb_crops", "masks_crops", "masks_hand_crops"):
        (output / name).mkdir(parents=True, exist_ok=True)
    stem = Path(record["rgb"]).stem
    rgb_path = output / "rgb_crops" / f"{stem}.png"
    mask_path = output / "masks_crops" / f"{stem}.png"
    hand_crop_path = output / "masks_hand_crops" / f"{stem}.png"
    cv2.imwrite(str(rgb_path), rgb[y0:y1, x0:x1])
    cv2.imwrite(str(mask_path), mask[y0:y1, x0:x1].astype(np.uint8) * 255)
    cv2.imwrite(str(hand_crop_path), hand[y0:y1, x0:x1].astype(np.uint8) * 255)
    result = dict(record)
    result["rgb_crop"] = str(rgb_path)
    result["object_mask_crop"] = str(mask_path)
    result["gripper_mask_crop"] = str(hand_crop_path)
    result["crop_bbox_xyxy"] = [x0, y0, x1, y1]
    result["crop_padding_fraction"] = float(padding)
    result["crop_clipped"] = bool(x0 == 0 or y0 == 0 or x1 == mask.shape[1] or y1 == mask.shape[0])
    return result


def build_metric_tracking(scan_root: Path, max_points_per_frame: int = 5000) -> dict[str, Any]:
    """Estimate a stationary object frame from the captured RGB-D only.

    The frame origin is the midpoint of robust scan point bounds in the robot
    base frame, with base axes retained as the object axes.  No simulator pose
    or GT mesh is read.  This is sufficient for metric scale and common-frame
    fusion; the orientation is intentionally documented as a scan convention.
    """
    scan_root = Path(scan_root)
    poses = sorted((scan_root / "poses").glob("*.json"))
    K = np.loadtxt(scan_root / "cam_K.txt")
    points: list[np.ndarray] = []
    rng = np.random.default_rng(20260914)
    for pose_path in poses:
        row = json.loads(pose_path.read_text())
        depth = cv2.imread(str(scan_root / "depth" / f"{pose_path.stem}.png"), cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(str(scan_root / "masks" / f"{pose_path.stem}.png"), cv2.IMREAD_GRAYSCALE) > 0
        hand_path = scan_root / "masks_hand" / f"{pose_path.stem}.png"
        hand = cv2.imread(str(hand_path), cv2.IMREAD_GRAYSCALE) > 0 if hand_path.exists() else np.zeros_like(mask)
        if depth is None or depth.shape != mask.shape:
            continue
        z = depth.astype(np.float64) / 1000.0
        valid = mask & ~hand & np.isfinite(z) & (z > 0.03) & (z < 2.0)
        v, u = np.where(valid)
        if not len(u):
            continue
        p_cam = np.column_stack(((u - K[0, 2]) * z[v, u] / K[0, 0],
                                 (v - K[1, 2]) * z[v, u] / K[1, 1], z[v, u]))
        T_B_C = np.asarray(row["T_B_camera"], dtype=float).reshape(4, 4)
        p_base = p_cam @ T_B_C[:3, :3].T + T_B_C[:3, 3]
        # The official Scalable Real2Sim asset-generation entry point caps the
        # reconstruction input at roughly 1800 images.  Keeping a bounded point
        # sample per frame prevents the stationary high-frame-rate scan from
        # accumulating hundreds of millions of redundant RGB-D points while
        # preserving the same RGB-D-only metric-frame construction.
        sample_count = int(max_points_per_frame)
        if sample_count > 0 and len(p_base) > sample_count:
            p_base = p_base[rng.choice(len(p_base), sample_count, replace=False)]
        points.append(p_base)
    if not points:
        raise RuntimeError("RGB-D object points unavailable for scan-station frame")
    all_points = np.concatenate(points, axis=0)
    lo = np.percentile(all_points, 1.0, axis=0)
    hi = np.percentile(all_points, 99.0, axis=0)
    center = (lo + hi) / 2.0
    T_B_O = np.eye(4)
    T_B_O[:3, 3] = center
    tracking_dir = scan_root / "tracking" / "ob_in_cam"
    tracking_dir.mkdir(parents=True, exist_ok=True)
    for pose_path in poses:
        row = json.loads(pose_path.read_text())
        T_B_C = np.asarray(row["T_B_camera"], dtype=float).reshape(4, 4)
        T_C_O = np.linalg.inv(T_B_C) @ T_B_O
        np.savetxt(tracking_dir / f"{pose_path.stem}.txt", T_C_O, fmt="%.10f")
    result = {
        "gt_used": False,
        "source": "RGB-D object masks + T_base_camera; robust 1/99% point bounds",
        "object_frame": "base axes retained; origin at robust RGB-D bounds midpoint",
        "T_base_object_estimate": T_B_O.tolist(),
        "point_count": int(len(all_points)),
        "robust_bounds_base_m": [lo.tolist(), hi.tolist()],
        "tracking_dir": str(tracking_dir),
    }
    (scan_root / "tracking" / "object_frame_estimate.json").write_text(json.dumps(result, indent=2))
    return result


class ScanStationSkill:
    """Acquire a clean, stationary, close-range two-ring scan."""

    def __init__(self, client: StationClient, output: Path, target: str,
                 config: ScanStationConfig | None = None):
        self.client = client
        self.output = Path(output)
        self.target = target
        self.config = config or ScanStationConfig()
        self.output.mkdir(parents=True, exist_ok=True)

    def run(self) -> dict[str, Any]:
        prep = self.client.command({
            "op": "scan_station_prepare",
            "target": self.target,
            "center": list(self.config.center_m),
        })
        if not prep.get("ok"):
            raise RuntimeError(f"scan station prepare failed: {prep}")
        records: list[dict[str, Any]] = []
        views = self.config.views()
        start = time.monotonic()
        for view in views:
            record = self.client.command({
                "op": "scan_station_view",
                "output": str(self.output),
                "frame_id": view.index,
                "pass_id": view.pass_id,
                "ring": view.ring,
                "azimuth_deg": view.azimuth_deg,
                "elevation_deg": view.elevation_deg,
                "position": list(view.position),
                "look_at": list(view.look_at),
            })
            if not record.get("ok"):
                raise RuntimeError(f"scan station capture failed: {record}")
            # The simulator returns paths and the current masks.  Recompute
            # quality from the actual saved images, then make the high-res crop.
            rgb = cv2.imread(str(record["rgb"]), cv2.IMREAD_COLOR)
            depth_raw = cv2.imread(str(record["depth"]), cv2.IMREAD_UNCHANGED)
            mask = cv2.imread(str(record["object_mask"]), cv2.IMREAD_GRAYSCALE) > 0
            hand = cv2.imread(str(record["gripper_mask"]), cv2.IMREAD_GRAYSCALE) > 0
            quality = _frame_quality(rgb, depth_raw.astype(float) / 1000.0, mask, hand, self.config)
            record["scan_view"] = {
                "index": view.index, "ring": view.ring,
                "azimuth_deg": view.azimuth_deg, "elevation_deg": view.elevation_deg,
                "position_m": list(view.position), "look_at_m": list(view.look_at),
            }
            record["quality"] = quality
            record = _copy_crop(record, self.output, self.config.crop_padding)
            records.append(record)
        accepted = [r for r in records if r["quality"]["accepted"]]
        if len(accepted) < self.config.minimum_accepted_views:
            raise RuntimeError(f"only {len(accepted)} scan-station views passed QA")
        frame_estimate = build_metric_tracking(self.output)
        manifest = {
            "schema": "fr3_scan_station/v1",
            "gt_used_for_acquisition_or_alignment": False,
            "target": self.target,
            "state": "placed_released_arm_parked",
            "camera_policy": "near-field fixed intrinsics; two high-overlap rings",
            "mask_policy": "object instance mask; gripper instance mask excluded",
            "crop_policy": {"padding_fraction": self.config.crop_padding, "source": "object bbox"},
            "config": self.config.__dict__,
            "views": records,
            "accepted_count": len(accepted),
            "rejected_count": len(records) - len(accepted),
            "timing_seconds": {"capture": time.monotonic() - start},
            "object_frame_estimate": frame_estimate,
            "gt_eval_dir": str(self.output / "eval_gt"),
        }
        (self.output / "scan_manifest.json").write_text(json.dumps(manifest, indent=2))
        (self.output / "station_report.json").write_text(json.dumps(manifest, indent=2))
        return manifest

    def run_official_continuous(
        self,
        frame_count: int = 1800,
        horizontal_fraction: float = 0.5,
        phase_offset_deg: float = 0.4,
    ) -> dict[str, Any]:
        """Acquire the official high-rate image count in one simulator command.

        Scalable Real2Sim's published asset-generation script retains at most
        1800 images after its DINO/frame-gap selection.  Its ``ImageSaver``
        publishes at 33 Hz while the object is in the display/grasp state.  The
        station simulator uses the equivalent 30 Hz RGB-D cadence and a smooth
        two-ring camera orbit, so adjacent frames have high overlap while the
        second ring changes elevation.  Capturing in one simulator-thread batch
        avoids 1800 HTTP round trips and, crucially, keeps the exact RGB, depth,
        mask and pose timestamp synchronized.

        This method is intentionally separate from ``run``: existing sparse
        scan experiments remain byte-for-byte reproducible and the grasp
        executor is never imported or modified here.
        """
        frame_count = int(frame_count)
        if frame_count < 8:
            raise ValueError("frame_count must be at least eight")
        horizontal_fraction = float(horizontal_fraction)
        if not 0.2 <= horizontal_fraction <= 0.8:
            raise ValueError("horizontal_fraction must be in [0.2, 0.8]")
        horizontal_frames = max(4, int(round(frame_count * horizontal_fraction)))
        horizontal_frames = min(horizontal_frames, frame_count - 4)
        high_frames = frame_count - horizontal_frames

        prep = self.client.command({
            "op": "scan_station_prepare",
            "target": self.target,
            "center": list(self.config.center_m),
        })
        if not prep.get("ok"):
            raise RuntimeError(f"scan station prepare failed: {prep}")

        start = time.monotonic()
        batch = self.client.command({
            "op": "scan_station_batch",
            "output": str(self.output),
            "num_frames": frame_count,
            "horizontal_frames": horizontal_frames,
            "radius_m": float(self.config.radius_m),
            "center": list(self.config.center_m),
            "horizontal_elevation_deg": float(self.config.horizontal_elevation_deg),
            "high_elevation_deg": float(self.config.high_elevation_deg),
            "high_phase_offset_deg": float(phase_offset_deg),
        }, timeout_s=max(900.0, frame_count / 2.0))
        if not batch.get("ok"):
            raise RuntimeError(f"continuous scan batch failed: {batch}")

        records: list[dict[str, Any]] = []
        for pose_path in sorted((self.output / "poses").glob("*.json")):
            row = json.loads(pose_path.read_text())
            if not row.get("ok", True):
                continue
            rgb = cv2.imread(str(row["rgb"]), cv2.IMREAD_COLOR)
            depth_raw = cv2.imread(str(row["depth"]), cv2.IMREAD_UNCHANGED)
            mask = cv2.imread(str(row["object_mask"]), cv2.IMREAD_GRAYSCALE) > 0
            hand = cv2.imread(str(row["gripper_mask"]), cv2.IMREAD_GRAYSCALE) > 0
            if rgb is None or depth_raw is None:
                row["quality"] = {"accepted": False, "reject_reason": "RGBD_UNAVAILABLE"}
            else:
                quality = _frame_quality(
                    rgb, depth_raw.astype(float) / 1000.0, mask, hand, self.config
                )
                row["quality"] = quality
                row["scan_view"] = {
                    "index": int(row.get("frame_id", len(records))),
                    "ring": row.get("ring", "unknown"),
                    "azimuth_deg": float(row.get("azimuth_deg", 0.0)),
                    "elevation_deg": float(row.get("elevation_deg", 0.0)),
                    "position_m": np.asarray(row.get("T_B_camera", np.eye(4)), dtype=float)[:3, 3].tolist(),
                    "look_at_m": list(self.config.center_m),
                }
                if quality["accepted"]:
                    row = _copy_crop(row, self.output, self.config.crop_padding)
            records.append(row)

        accepted = [r for r in records if (r.get("quality") or {}).get("accepted")]
        # For the official stream the QA requirement scales with the requested
        # frame count; the sparse-run default of 18 must not make an 8-frame
        # protocol smoke test impossible.
        required = max(8, int(np.ceil(frame_count * 0.90)))
        if len(accepted) < required:
            raise RuntimeError(
                f"only {len(accepted)}/{frame_count} continuous views passed QA; "
                f"required {required}"
            )
        # One thousand points per frame is sufficient for the metric object-frame
        # estimate at this cadence and keeps this bookkeeping stage bounded.
        frame_estimate = build_metric_tracking(self.output, max_points_per_frame=1000)
        manifest = {
            "schema": "fr3_scan_station/v1",
            "gt_used_for_acquisition_or_alignment": False,
            "target": self.target,
            "state": "placed_released_arm_parked",
            "camera_policy": "official_like_continuous_30fps_two_high_overlap_rings",
            "acquisition_mode": "continuous_camera_orbit_station_equivalent",
            "official_reference": {
                "image_saver_period_s": 0.03,
                "reconstruction_input_cap": 1800,
                "reference": "Scalable Real2Sim run_asset_generation.py downsample_images(num_images=1800)",
                "relative_motion_equivalent": "camera orbit around stationary payload",
            },
            "mask_policy": "object instance mask; gripper instance mask excluded",
            "crop_policy": {"padding_fraction": self.config.crop_padding, "source": "object bbox"},
            "config": self.config.__dict__,
            "trajectory": {
                "requested_frames": frame_count,
                "captured_frames": len(records),
                "horizontal_frames": horizontal_frames,
                "high_frames": high_frames,
                "nominal_fps": 30.0,
                "horizontal_elevation_deg": self.config.horizontal_elevation_deg,
                "high_elevation_deg": self.config.high_elevation_deg,
                "high_phase_offset_deg": phase_offset_deg,
                "adjacent_overlap_intent": "continuous_8_physics_steps_per_frame",
            },
            "views": records,
            "accepted_count": len(accepted),
            "rejected_count": len(records) - len(accepted),
            "timing_seconds": {"capture_and_qa": time.monotonic() - start},
            "object_frame_estimate": frame_estimate,
            "gt_eval_dir": str(self.output / "eval_gt"),
        }
        (self.output / "scan_manifest.json").write_text(json.dumps(manifest, indent=2))
        (self.output / "station_report.json").write_text(json.dumps(manifest, indent=2))
        return manifest
