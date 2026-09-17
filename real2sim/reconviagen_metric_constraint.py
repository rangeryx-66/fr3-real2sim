"""Metric alignment and measured-surface constraint for ReconViaGen v0.5.

ReconViaGen supplies a complete shape prior in an arbitrary normalized frame.
This module estimates one uniform metric scale and a rigid transform from the
actual RGB-D observations and BundleSDF camera/object tracks.  It then projects
only rays that are visibly supported by real depth back toward the measured
surface.  Unobserved faces stay generated and are explicitly accounted for in
the report.  Ground truth is never opened in this path.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import trimesh
from scipy.spatial import cKDTree


def _norm(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-12)


def _apply(points: np.ndarray, R: np.ndarray, t: np.ndarray, scale: float) -> np.ndarray:
    return (points * scale) @ R.T + t


def _sample_mesh(mesh: trimesh.Trimesh, n: int, seed: int) -> np.ndarray:
    state = np.random.get_state()
    np.random.seed(seed)
    try:
        return np.asarray(mesh.sample(min(n, max(len(mesh.faces), 1))), dtype=np.float64)
    finally:
        np.random.set_state(state)


def _load_rows(dataset: Path, tracking: Path) -> list[tuple[str, dict[str, Any], np.ndarray]]:
    rows = []
    for pose_path in sorted((dataset / "poses").glob("*.json")):
        stem = pose_path.stem
        track_path = tracking / "ob_in_cam" / f"{stem}.txt"
        if not track_path.exists():
            continue
        row = json.loads(pose_path.read_text())
        T_C_O = np.loadtxt(track_path).reshape(4, 4)
        rows.append((stem, row, T_C_O))
    return rows


def _rgbd_points(dataset: Path, tracking: Path, max_points: int = 120000, seed: int = 20260914) -> tuple[np.ndarray, dict[str, Any]]:
    K = np.loadtxt(dataset / "cam_K.txt").astype(float)
    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    points = []
    frame_rows = []
    base_point_count = 0
    base_object_poses = []
    for stem, row, T_C_O in _load_rows(dataset, tracking):
        rgb_path = dataset / "rgb" / f"{stem}.png"
        depth_path = dataset / "depth" / f"{stem}.png"
        mask_path = dataset / "masks" / f"{stem}.png"
        hand_path = dataset / ("masks_hand" if (dataset / "masks_hand" / f"{stem}.png").exists() else "gripper_masks") / f"{stem}.png"
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        hand = cv2.imread(str(hand_path), cv2.IMREAD_GRAYSCALE)
        if depth is None or mask is None or hand is None:
            continue
        m = (mask > 0) & ~(hand > 0) & np.isfinite(depth) & (depth > 0)
        v, u = np.where(m)
        if len(u) == 0:
            continue
        z = depth[v, u].astype(np.float64) / 1000.0
        xyz_c = np.c_[(u - cx) * z / fx, (v - cy) * z / fy, z]
        # Use the recorded robot pose explicitly.  T_C_O is the tracked
        # object-in-camera transform (camera <- object); T_base_camera is the
        # camera pose in the robot base (base <- camera).  We therefore form
        # T_base_object = T_base_camera @ T_C_object and map base points back
        # with its inverse.  This is algebraically equivalent to inv(T_C_O),
        # but keeps the robot frame in the metric evidence and audit report.
        T_B_C_raw = row.get("T_base_camera", row.get("T_B_camera"))
        if T_B_C_raw is None:
            raise RuntimeError(f"missing T_base_camera/T_B_camera in pose row {stem}")
        T_B_C = np.asarray(T_B_C_raw, dtype=float).reshape(4, 4)
        T_B_O = T_B_C @ T_C_O
        T_O_B = np.linalg.inv(T_B_O)
        xyz_b = (xyz_c @ T_B_C[:3, :3].T) + T_B_C[:3, 3]
        xyz_o = (xyz_b @ T_O_B[:3, :3].T) + T_O_B[:3, 3]
        points.append(xyz_o)
        base_point_count += int(len(xyz_b))
        base_object_poses.append(T_B_O)
        frame_rows.append({
            "frame": stem,
            "raw_valid_pixels": int(len(u)),
            "gripper_excluded_pixels": int((mask.astype(bool) & hand.astype(bool)).sum()),
            "T_base_camera": T_B_C.tolist(),
            "T_base_object": T_B_O.tolist(),
        })
    if not points:
        raise RuntimeError("RGB-D contains no valid object points")
    cloud = np.concatenate(points, axis=0)
    rng = np.random.default_rng(seed)
    if len(cloud) > max_points:
        cloud = cloud[rng.choice(len(cloud), max_points, replace=False)]
    if base_object_poses:
        translations = np.asarray([x[:3, 3] for x in base_object_poses], dtype=float)
        base_pose_summary = {
            "frames": int(len(base_object_poses)),
            "translation_mean_m": translations.mean(axis=0).tolist(),
            "translation_span_m": (translations.max(axis=0) - translations.min(axis=0)).tolist(),
        }
    else:
        base_pose_summary = {"frames": 0}
    return cloud, {
        "frames": frame_rows,
        "points": int(len(cloud)),
        "base_points_before_sampling": int(base_point_count),
        "source": "RGB-D depth + masks + T_base_camera + T_C_object",
        "robot_metric_frame": "base",
        "T_base_camera_used": True,
        "T_base_object_summary": base_pose_summary,
        "gt_used": False,
    }


def _candidate_rotations() -> list[np.ndarray]:
    out = []
    for perm in itertools.permutations(range(3)):
        P = np.zeros((3, 3))
        for i, j in enumerate(perm):
            P[i, j] = 1.0
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            R = np.diag(signs) @ P
            if np.linalg.det(R) > 0:
                out.append(R)
    return out


def _initial_align(generated: trimesh.Trimesh, observed: np.ndarray, seed: int) -> tuple[float, np.ndarray, np.ndarray, list[dict[str, Any]]]:
    gp = np.asarray(generated.vertices, dtype=float)
    gc = gp.mean(axis=0)
    oc = observed.mean(axis=0)
    ge = np.ptp(gp, axis=0)
    oe = np.ptp(observed, axis=0)
    scale = float(np.median(oe / np.maximum(ge, 1e-8)))
    src = _sample_mesh(generated, 12000, seed)
    tree = cKDTree(observed)
    candidates = []
    for R in _candidate_rotations():
        t = oc - scale * (R @ gc)
        probe = _apply(src, R, t, scale)
        d = tree.query(probe, k=1, workers=-1)[0]
        candidates.append({"scale": scale, "rotation": R, "translation": t, "rmse_m": float(np.sqrt(np.mean(np.minimum(d, 0.05) ** 2))), "median_m": float(np.median(np.minimum(d, 0.05)))})
    best = min(candidates, key=lambda x: (x["rmse_m"], x["median_m"]))
    return float(best["scale"]), np.asarray(best["rotation"]), np.asarray(best["translation"]), candidates


def _icp(points: np.ndarray, observed: np.ndarray, scale: float, R: np.ndarray, t: np.ndarray, threshold: float = 0.05) -> tuple[float, np.ndarray, np.ndarray, dict[str, Any]]:
    try:
        import open3d as o3d
    except ImportError:
        return scale, R, t, {"used": False, "reason": "open3d unavailable"}
    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(_apply(points, R, t, scale)))
    dst = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(observed))
    result = o3d.pipelines.registration.registration_icp(
        src, dst, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80),
    )
    # ICP's transform acts after the already scaled/rotated source.
    X = np.asarray(result.transformation)
    R_new = X[:3, :3] @ R
    t_new = X[:3, :3] @ t + X[:3, 3]
    return scale, R_new, t_new, {"used": True, "fitness": float(result.fitness), "inlier_rmse_m": float(result.inlier_rmse), "iterations": 80}


def _projection_support(mesh: trimesh.Trimesh, dataset: Path, tracking: Path, conflict_m: float, support_m: float) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Classify vertices by actual RGB-D rays, excluding the gripper mask."""
    V = np.asarray(mesh.vertices, dtype=float)
    N = np.asarray(mesh.vertex_normals, dtype=float)
    support_hits = np.zeros(len(V), dtype=np.uint16)
    conflict_hits = np.zeros(len(V), dtype=np.uint16)
    visible_checks = 0
    for stem, _row, T_C_O in _load_rows(dataset, tracking):
        depth = cv2.imread(str(dataset / "depth" / f"{stem}.png"), cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(str(dataset / "masks" / f"{stem}.png"), cv2.IMREAD_GRAYSCALE)
        hpath = dataset / "masks_hand" / f"{stem}.png"
        if not hpath.exists():
            hpath = dataset / "gripper_masks" / f"{stem}.png"
        hand = cv2.imread(str(hpath), cv2.IMREAD_GRAYSCALE)
        if depth is None or mask is None or hand is None:
            continue
        K = np.loadtxt(dataset / "cam_K.txt").astype(float)
        C = V @ T_C_O[:3, :3].T + T_C_O[:3, 3]
        NC = N @ T_C_O[:3, :3].T
        z = C[:, 2]
        u = np.rint(K[0, 0] * C[:, 0] / np.maximum(z, 1e-8) + K[0, 2]).astype(int)
        v = np.rint(K[1, 1] * C[:, 1] / np.maximum(z, 1e-8) + K[1, 2]).astype(int)
        inside = (z > 0.05) & (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
        if not inside.any():
            continue
        idx = np.flatnonzero(inside)
        real_mask = (mask[v[idx], u[idx]] > 0) & (hand[v[idx], u[idx]] == 0)
        d = depth[v[idx], u[idx]].astype(float) / 1000.0
        finite = real_mask & np.isfinite(d) & (d > 0)
        # A front-facing surface is required before a ray can constrain the
        # generated mesh.  This prevents an unseen back vertex behind the
        # silhouette from being mislabeled as a depth conflict.
        view = _norm(-C[idx])
        front = np.einsum("ij,ij->i", _norm(NC[idx]), view) > 0.05
        resid = np.abs(z[idx] - d)
        good = finite & front
        support_hits[idx[good & (resid <= support_m)]] += 1
        conflict_hits[idx[good & (resid >= conflict_m)]] += 1
        visible_checks += int(good.sum())
    # A single projected disagreement can be a silhouette/rasterization
    # outlier.  Require two independent RGB-D views before rejecting a
    # generated face; support still needs only one valid measured view.
    supported = support_hits >= 1
    conflict = conflict_hits >= 2
    return supported, conflict, {"visible_ray_checks": visible_checks, "supported_vertices": int(supported.sum()), "conflict_vertices": int(conflict.sum()), "support_hits_ge_2": int((support_hits >= 2).sum()), "conflict_hits_ge_2": int((conflict_hits >= 2).sum()), "support_m": support_m, "conflict_m": conflict_m, "min_conflict_views": 2}


def constrain(generated_path: Path, dataset: Path, tracking: Path, output: Path, observed_mesh_path: Path | None = None, support_mm: float = 6.0, conflict_mm: float = 10.0, blend: float = 0.85) -> dict[str, Any]:
    t0 = time.perf_counter()
    generated_path, dataset, tracking, output = map(Path, (generated_path, dataset, tracking, output))
    output.mkdir(parents=True, exist_ok=True)
    generated = trimesh.load(generated_path, force="mesh", process=False)
    if not isinstance(generated, trimesh.Trimesh) or len(generated.faces) < 4:
        raise RuntimeError(f"invalid generated mesh: {generated_path}")
    observed, obs_report = _rgbd_points(dataset, tracking)
    if observed_mesh_path is not None and Path(observed_mesh_path).exists():
        tsdf = trimesh.load(observed_mesh_path, force="mesh", process=False)
        if isinstance(tsdf, trimesh.Trimesh) and len(tsdf.vertices):
            observed = np.concatenate([observed, np.asarray(tsdf.vertices, dtype=float)], axis=0)
            if len(observed) > 160000:
                rng = np.random.default_rng(20260914)
                observed = observed[rng.choice(len(observed), 160000, replace=False)]
            obs_report["observed_tsdf_mesh"] = str(observed_mesh_path)
    scale, R, t, candidates = _initial_align(generated, observed, 20260914)
    src_for_icp = _sample_mesh(generated, 18000, 20260915)
    scale, R, t, icp_report = _icp(src_for_icp, observed, scale, R, t)
    aligned = generated.copy()
    aligned.apply_scale(scale)
    X = np.eye(4); X[:3, :3] = R; X[:3, 3] = t
    aligned.apply_transform(X)
    # Trimesh 3.x (simrecon) and 4.x (robotics) expose slightly different
    # cleanup method names; keep the metric path portable across both.
    if hasattr(aligned, "remove_degenerate_faces"):
        aligned.remove_degenerate_faces()
    elif hasattr(aligned, "update_faces"):
        aligned.update_faces(np.ones(len(aligned.faces), dtype=bool))
    if hasattr(aligned, "remove_duplicate_faces"):
        aligned.remove_duplicate_faces()
    aligned.remove_unreferenced_vertices()
    aligned.process(validate=True)
    supported, conflict, support_report = _projection_support(aligned, dataset, tracking, conflict_mm / 1000.0, support_mm / 1000.0)
    obs_tree = cKDTree(observed)
    if supported.any():
        d, nn = obs_tree.query(np.asarray(aligned.vertices)[supported], k=1, workers=-1)
        indices = np.flatnonzero(supported)
        V = np.asarray(aligned.vertices)
        V[indices] = (1.0 - blend) * V[indices] + blend * observed[nn]
        aligned.vertices = V
    # Area-weighted surface fractions are less sensitive to mesh tessellation.
    areas = np.asarray(aligned.area_faces, dtype=float)
    face_supported = supported[np.asarray(aligned.faces)].mean(axis=1) >= 1.0 / 3.0
    face_conflict = conflict[np.asarray(aligned.faces)].any(axis=1)
    total_area = float(areas.sum())
    obs_area = float(areas[face_supported].sum())
    gen_area = float(areas[~face_supported].sum())
    conflict_area = float(areas[face_conflict].sum())
    kept = ~face_conflict
    kept_obs_area = float(areas[face_supported & kept].sum())
    kept_gen_area = float(areas[(~face_supported) & kept].sum())
    # A generated surface that projects into a valid measured ray but disagrees
    # by the conflict threshold is not allowed to survive merely to close the
    # mesh.  Reject its incident faces conservatively; this may leave a hole,
    # which is preferable to inserting hallucinated geometry.  Unobserved
    # faces (no valid ray) remain the ReconViaGen completion.
    rejected_faces = int(face_conflict.sum())
    if rejected_faces:
        keep = ~face_conflict
        if hasattr(aligned, "update_faces"):
            aligned.update_faces(keep)
        else:
            aligned.faces = np.asarray(aligned.faces)[keep]
        aligned.remove_unreferenced_vertices()
        if hasattr(aligned, "process"):
            aligned.process(validate=True)
    obj = output / "metric_constrained_mesh.obj"
    ply = output / "metric_constrained_mesh.ply"
    aligned.export(obj); aligned.export(ply)
    ext = (np.asarray(aligned.vertices).max(axis=0) - np.asarray(aligned.vertices).min(axis=0)).tolist()
    raw_edges = getattr(aligned, "edges_sorted", None)
    if raw_edges is None:
        raw_edges = np.asarray(aligned.edges_unique)
    edges = np.sort(np.asarray(raw_edges), axis=1) if len(raw_edges) else np.empty((0, 2), dtype=int)
    if len(edges):
        _, counts = np.unique(edges, axis=0, return_counts=True)
        boundary_edges = int(np.sum(counts == 1))
    else:
        boundary_edges = 0
    report = {
        "schema": "fr3_reconviagen_v05_metric_constraint/v1",
        "gt_used": False,
        "generated_mesh": str(generated_path),
        "observed_source": obs_report,
        "alignment": {"uniform_scale": scale, "rotation": R.tolist(), "translation_m": t.tolist(), "pca_candidates": [{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in c.items()} for c in candidates], "icp": icp_report},
        "constraint": {"policy": "real RGB-D ray support wins; conflicting generated faces are rejected; only unsupported faces remain completion", "support_mm": support_mm, "conflict_mm": conflict_mm, "blend_to_measured": blend, **support_report, "pre_rejection_surface_area_m2": total_area, "pre_rejection_rgbd_observed_surface_fraction": obs_area / max(total_area, 1e-12), "pre_rejection_generated_only_surface_fraction": gen_area / max(total_area, 1e-12), "generated_rgbd_conflict_surface_fraction": conflict_area / max(total_area, 1e-12), "rejected_conflict_faces": rejected_faces, "rejected_conflict_surface_fraction": conflict_area / max(total_area, 1e-12), "final_surface_area_m2": float(kept_obs_area + kept_gen_area), "final_rgbd_observed_surface_fraction": kept_obs_area / max(kept_obs_area + kept_gen_area, 1e-12), "final_generated_only_surface_fraction": kept_gen_area / max(kept_obs_area + kept_gen_area, 1e-12)},
        "mesh": {"vertices": int(len(aligned.vertices)), "faces": int(len(aligned.faces)), "watertight": bool(aligned.is_watertight), "boundary_edges": boundary_edges, "aabb_extent_m": ext},
        "products": {"obj": str(obj), "ply": str(ply)},
        "elapsed_seconds": time.perf_counter() - t0,
    }
    (output / "metric_constraint_report.json").write_text(json.dumps(report, indent=2))
    (output / "alignment_transform.txt").write_text("\n".join(" ".join(f"{x:.12g}" for x in row) for row in X) + "\n")
    return report


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("generated_mesh", type=Path)
    p.add_argument("dataset", type=Path)
    p.add_argument("tracking", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--observed-mesh", type=Path)
    p.add_argument("--support-mm", type=float, default=6.0)
    p.add_argument("--conflict-mm", type=float, default=10.0)
    p.add_argument("--blend", type=float, default=0.85)
    a = p.parse_args()
    print(json.dumps(constrain(a.generated_mesh, a.dataset, a.tracking, a.output, a.observed_mesh, a.support_mm, a.conflict_mm, a.blend), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
