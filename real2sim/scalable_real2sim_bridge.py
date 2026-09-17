"""Bridge the frozen FR3 grasp handoff to the official Scalable Real2Sim flow.

This module deliberately keeps the grasp executor out of the downstream path.  It
validates a completed stable-grasp record, copies the scan into the YCBInEOAT layout
expected by the pinned BundleSDF checkout, runs the vendor ``run_video`` entry point
(which includes BundleSDF tracking and the official global NeRF/texture pass), and
then adapts the official SDFormat/CoACD output to the project's Isaac USD package.

The official payload-ID code currently assumes an IIWA model and an identified IIWA
robot parameter directory.  FR3 records therefore use the existing measured FR3
adapter only when those IIWA prerequisites are absent.  The distinction is recorded
in the manifest; no GT inertial value is ever read by this module.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable

import numpy as np


DEFAULT_OFFICIAL_ROOT = Path("/data1/home/rangeryx/scalable-real2sim")
PINNED_SCALABLE_REAL2SIM = "a8e4d97cbb0c3ea887a69fa313bcd3a252c5a8a3"
PINNED_BUNDLESDF = "4029bb7504b5aa9af2e9bc7161704b9e82df3d32"
PINNED_PAYLOAD_ID = "c52e31cf26c83b33aee5e56f805e1d4d710fd549"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _git_rev(path: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
        ).strip()
    except Exception:
        return None


def verify_official_checkout(root: Path) -> dict[str, Any]:
    """Verify the vendor checkout without mutating its dirty working tree."""
    root = Path(root)
    bundle = root / "scalable_real2sim" / "BundleSDF"
    payload = root / "scalable_real2sim" / "robot_payload_id"
    if not (bundle / "run_custom.py").exists():
        raise FileNotFoundError(f"official BundleSDF checkout missing: {bundle}")
    commits = {
        "scalable_real2sim": _git_rev(root),
        "BundleSDF": _git_rev(bundle),
        "robot_payload_id": _git_rev(payload),
    }
    if commits["scalable_real2sim"] != PINNED_SCALABLE_REAL2SIM:
        raise RuntimeError(f"unexpected Scalable Real2Sim commit: {commits}")
    if commits["BundleSDF"] != PINNED_BUNDLESDF:
        raise RuntimeError(f"unexpected BundleSDF commit: {commits}")
    if commits["robot_payload_id"] != PINNED_PAYLOAD_ID:
        raise RuntimeError(f"unexpected robot_payload_id commit: {commits}")
    return commits


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _last_attempt(grasp: dict[str, Any]) -> dict[str, Any]:
    attempts = grasp.get("attempts") or []
    if not attempts or not isinstance(attempts[-1], dict):
        return {}
    return attempts[-1]


def validate_stable_handoff(grasp_path: Path) -> dict[str, Any]:
    """Fail closed unless the frozen executor really completed a stable grasp.

    The check accepts the project's top-level ``SUCCESS`` records and older records
    whose stable flags live in the final attempt.  It does not inspect or infer GT
    dynamics.  A ``mode=GT`` record is allowed for an explicitly isolated oracle
    replay, but its provenance is retained in the returned handoff record.
    """
    grasp_path = Path(grasp_path).resolve()
    grasp = _load_json(grasp_path)
    attempt = _last_attempt(grasp)
    stable = attempt.get("stability") or attempt.get("hold", {}).get("stability") or {}
    flags = dict(grasp.get("flags") or stable.get("flags") or {})
    success = bool(grasp.get("success")) and str(grasp.get("category", "")) in {
        "SUCCESS",
        "STABLE",
    }
    required = ("PICKED", "RETAINED", "CLEAR_TABLE", "lift_ge_8cm")
    if not success or any(not bool(flags.get(k)) for k in required) or bool(flags.get("DROP")):
        raise RuntimeError(
            "frozen grasp handoff rejected: requires SUCCESS with "
            f"{required} and no DROP; id={grasp.get('id')} category={grasp.get('category')} flags={flags}"
        )
    params = grasp.get("parameters") or {}
    return {
        "grasp_result": str(grasp_path),
        "grasp_result_sha256": sha256(grasp_path),
        "id": grasp.get("id", grasp_path.stem),
        "target": grasp.get("target", "unknown"),
        "seed": grasp.get("seed"),
        "mode": grasp.get("mode", "unknown"),
        "category": grasp.get("category"),
        "flags": flags,
        "stable": True,
        "force_total_N": params.get("force_total_N"),
        "pad_target_mu": params.get("mu"),
        "frozen_executor_unchanged": True,
        "handoff_stage": "after_FREE_SPACE_STABLE_before_scan",
    }


def _frame_stems(scan_dir: Path) -> list[str]:
    stems = sorted(p.stem for p in (scan_dir / "rgb").glob("*.png"))
    if len(stems) < 8:
        raise RuntimeError(f"scan needs at least 8 RGB frames, found {len(stems)}")
    return stems


def _read_mask(path: Path) -> np.ndarray:
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    return image > 0


def _normalise_depth(path: Path, destination: Path) -> dict[str, Any]:
    """Copy depth as official uint16 millimetres, recording any conversion."""
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(path)
    converted = False
    if np.issubdtype(image.dtype, np.floating):
        # Scan-station depth is metres when floating point.  Preserve the metric
        # values while satisfying YcbineoatReader's uint16-mm convention.
        image = np.rint(np.clip(image, 0.0, 65.535) * 1000.0).astype(np.uint16)
        converted = True
    elif image.dtype != np.uint16:
        image = image.astype(np.uint16)
        converted = True
    destination.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(destination), image)
    valid = image > 0
    return {
        "source": str(path),
        "destination": str(destination),
        "dtype": str(image.dtype),
        "valid_fraction": float(np.mean(valid)),
        "converted_to_uint16_mm": converted,
    }


def _copy_binary_mask(source: Path, destination: Path, exclude: np.ndarray | None = None) -> dict[str, Any]:
    import cv2

    mask = _read_mask(source)
    if exclude is not None:
        if exclude.shape != mask.shape:
            raise ValueError(f"mask shape mismatch: {source}")
        mask &= ~exclude
    destination.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(destination), (mask.astype(np.uint8) * 255))
    return {"source": str(source), "destination": str(destination), "pixels": int(mask.sum())}


def _manifest_frames(scan_dir: Path) -> list[dict[str, Any]]:
    manifest_path = scan_dir / "scan_manifest.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"scan manifest missing: {manifest_path}")
    manifest = _load_json(manifest_path)
    # The held-object ObjectScan schema calls this list ``frames`` while the
    # clean station acquisition schema calls it ``views``.  Both contain the
    # same synchronized RGB-D/pose records; accepting both keeps the official
    # bridge usable for the denser station stream without changing BundleSDF.
    frames = manifest.get("frames") or manifest.get("views")
    if not isinstance(frames, list) or not frames:
        raise ValueError(f"scan manifest has no frames: {manifest_path}")
    return [f for f in frames if isinstance(f, dict) and f.get("ok", True)]


def prepare_official_dataset(scan_dir: Path, destination: Path, handoff: dict[str, Any]) -> dict[str, Any]:
    """Create a clean, immutable copy in the official BundleSDF input layout."""
    scan_dir, destination = Path(scan_dir).resolve(), Path(destination).resolve()
    if not scan_dir.exists():
        raise FileNotFoundError(scan_dir)
    stems = _frame_stems(scan_dir)
    frames = _manifest_frames(scan_dir)
    by_stem: dict[str, dict[str, Any]] = {}
    for f in frames:
        # Dual-pass manifests may retain per-pass frame_id values (0..N) while
        # their file stems are 010000, 010001, ... .  Match the recorded RGB
        # basename first so poses cannot silently be assigned to another frame.
        rgb_name = Path(str(f.get("rgb", ""))).stem
        if rgb_name:
            by_stem[rgb_name] = f
        if f.get("frame_id") is not None:
            by_stem.setdefault(str(f.get("frame_id")), f)
    for folder in ("rgb", "depth", "masks", "gripper_masks", "masks_hand", "poses"):
        (destination / folder).mkdir(parents=True, exist_ok=True)
    qa: list[dict[str, Any]] = []
    pose_records: list[dict[str, Any]] = []
    source_manifest = _load_json(scan_dir / "scan_manifest.json")
    for index, stem in enumerate(stems):
        rgb_src = scan_dir / "rgb" / f"{stem}.png"
        depth_src = scan_dir / "depth" / f"{stem}.png"
        mask_src = scan_dir / "masks" / f"{stem}.png"
        gripper_src = scan_dir / "gripper_masks" / f"{stem}.png"
        if not all(p.exists() for p in (rgb_src, depth_src, mask_src, gripper_src)):
            raise FileNotFoundError(f"unsynchronised official frame {stem}")
        import cv2

        rgb = cv2.imread(str(rgb_src), cv2.IMREAD_COLOR)
        if rgb is None:
            raise FileNotFoundError(rgb_src)
        gripper = _read_mask(gripper_src)
        object_info = _copy_binary_mask(mask_src, destination / "masks" / f"{stem}.png", gripper)
        gripper_info = _copy_binary_mask(gripper_src, destination / "gripper_masks" / f"{stem}.png")
        # BundleSDF's official reader has no separate argument for the hand mask, but
        # its data conventions use masks_hand for explicit occlusion audits.
        _copy_binary_mask(gripper_src, destination / "masks_hand" / f"{stem}.png")
        depth_info = _normalise_depth(depth_src, destination / "depth" / f"{stem}.png")
        shutil.copy2(rgb_src, destination / "rgb" / f"{stem}.png")
        frame = by_stem.get(stem) or by_stem.get(str(index)) or {}
        T_B_camera = frame.get("T_B_camera", frame.get("T_base_camera"))
        T_B_TCP = frame.get("T_B_TCP")
        intrinsics = frame.get("intrinsics")
        if T_B_camera is None or T_B_TCP is None or intrinsics is None:
            raise ValueError(f"robot pose/intrinsics missing for frame {stem}")
        pose = {
            "frame_id": int(frame.get("frame_id", index)),
            "file_stem": stem,
            "pass_id": int(frame.get("pass_id", 0)),
            "T_base_camera": T_B_camera,
            "T_B_camera": T_B_camera,
            "T_B_TCP": T_B_TCP,
            "intrinsics": intrinsics,
            "source_pose": str(scan_dir / "scan_manifest.json"),
            "gt_excluded": True,
        }
        (destination / "poses" / f"{stem}.json").write_text(json.dumps(pose, indent=2))
        pose_records.append(pose)
        obj_mask = _read_mask(destination / "masks" / f"{stem}.png")
        qa.append({
            "stem": stem,
            "pass_id": pose["pass_id"],
            "rgb_shape": list(rgb.shape),
            "object_pixels": int(obj_mask.sum()),
            "gripper_pixels": int(gripper.sum()),
            "gripper_overlap_after_exclusion": 0,
            "depth_valid_fraction": depth_info["valid_fraction"],
            "object_fraction": float(np.mean(obj_mask)),
            "depth_conversion": depth_info["converted_to_uint16_mm"],
            "accepted": bool(obj_mask.sum() > 0 and depth_info["valid_fraction"] > 0),
            "gt_excluded": True,
            "object_mask": object_info,
            "gripper_mask": gripper_info,
        })
    cam_k = scan_dir / "cam_K.txt"
    if not cam_k.exists():
        raise FileNotFoundError(cam_k)
    shutil.copy2(cam_k, destination / "cam_K.txt")
    # Official YcbineoatReader discovers K from cam_K.txt.  The sidecar retains all
    # robot poses because the vendor tracker itself estimates ob_in_cam from RGB-D.
    (destination / "robot_camera_poses.json").write_text(json.dumps({
        "schema": "fr3_robot_camera_poses/v1",
        "camera_frame": source_manifest.get("camera_frame", "camera_optical"),
        "base_frame": source_manifest.get("base_frame", "fr3_link0"),
        "tcp_frame": source_manifest.get("tcp_frame", "fr3_hand_tcp"),
        "frames": pose_records,
        "source_scan_manifest": str(scan_dir / "scan_manifest.json"),
        "gt_excluded": True,
    }, indent=2))
    passes = source_manifest.get("passes") or []
    pass_ids = sorted({int(q["pass_id"]) for q in pose_records})
    regrasp_count = sum(1 for q in passes if q.get("requires_independent_stable_grasp"))
    result = {
        "schema": "scalable_real2sim_official_input/v1",
        "source_scan": str(scan_dir),
        "destination": str(destination),
        "target": handoff.get("target"),
        "frames": len(stems),
        "passes": pass_ids,
        "regrasp_count": int(regrasp_count),
        "qa": qa,
        "accepted_frames": int(sum(q["accepted"] for q in qa)),
        "source_manifest_sha256": sha256(scan_dir / "scan_manifest.json"),
        "cam_K_sha256": sha256(cam_k),
        "gt_excluded": True,
        "excluded_paths": ["eval_gt", "gt_mesh", "gt_physics"],
        "pose_usage": "sidecar_for_metric_frame_and_payload_alignment; BundleSDF tracker retains official RGB-D pose estimation",
        "handoff": handoff,
    }
    (destination / "official_input_manifest.json").write_text(json.dumps(result, indent=2))
    return result


def run_official_bundlesdf(
    scan_dir: Path,
    output: Path,
    official_root: Path,
    bundle_python: Path | None = None,
    force: bool = False,
) -> dict[str, Any]:
    """Invoke the pinned official ``run_custom.py --mode run_video`` entry point."""
    from .bundlesdf_adapter import run as adapter_run

    output = Path(output).resolve()
    recon_json = output / "reconstruction.json"
    if recon_json.exists() and not force:
        result = _load_json(recon_json)
        result["reused_existing"] = True
        return result
    if bundle_python is not None:
        # Preserve the venv launcher path.  The official BundleSDF environment
        # uses ``.venv/bin/python`` as a symlink into the shared robotics
        # interpreter; resolving it here drops the venv's site-packages (for
        # example dearpygui) from ``sys.path``.  ``absolute()`` normalizes the
        # path without following the symlink.
        os.environ["BUNDLESDF_PYTHON"] = str(Path(bundle_python).expanduser().absolute())
    start = time.perf_counter()
    result = adapter_run(Path(scan_dir), output, Path(official_root), optimized=False)
    result.update({
        "elapsed_seconds": time.perf_counter() - start,
        "official_entrypoint": "BundleSDF/run_custom.py --mode run_video",
        "official_global_refinement": "run_one_video_global_nerf invoked by run_video",
        "official_texture": "run_global_nerf(get_texture=True, use_all_frames=True, tex_res=2048)",
    })
    recon_json.write_text(json.dumps(result, indent=2))
    return result


def _canonicalization_transform(raw_mesh: Path, canonical_mesh: Path) -> np.ndarray:
    """Recover the raw→canonical rigid transform from the official output vertices."""
    import trimesh

    raw = trimesh.load(raw_mesh, force="mesh", process=False)
    canonical = trimesh.load(canonical_mesh, force="mesh", process=False)
    if len(raw.vertices) != len(canonical.vertices):
        raise ValueError("official canonicalization changed vertex count")
    A = np.asarray(raw.vertices, dtype=float)
    B = np.asarray(canonical.vertices, dtype=float)
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = cb - R @ ca
    err = np.linalg.norm((A @ R.T + T[:3, 3]) - B, axis=1).mean()
    if not np.isfinite(err) or err > 1e-5:
        raise RuntimeError(f"failed to recover official canonicalization transform: {err} m")
    return T


def canonicalize_official_mesh(reconstruction: dict[str, Any], output: Path, official_root: Path) -> dict[str, Any]:
    """Use the vendor canonicalizer and keep the generated material files together."""
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    raw_mesh = Path(reconstruction["products"]["textured_mesh"]).resolve()
    raw_dir = raw_mesh.parent
    mesh_dir = output / "bundle_sdf_mesh"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    # Keep the vendor output untouched and canonicalize an isolated copy.
    shutil.copy2(raw_mesh, mesh_dir / "textured_mesh.obj")
    for suffix in (".mtl",):
        candidate = raw_dir / (raw_mesh.stem + suffix)
        if candidate.exists():
            shutil.copy2(candidate, mesh_dir / candidate.name)
    tex = Path(reconstruction["products"]["texture"]).resolve()
    shutil.copy2(tex, mesh_dir / tex.name)
    canonical_mesh = mesh_dir / "textured_mesh.obj"
    code = (
        "from scalable_real2sim.output.canonicalize import canonicalize_mesh_from_file; "
        f"canonicalize_mesh_from_file({str(canonical_mesh)!r}, {str(canonical_mesh)!r})"
    )
    env = {**os.environ, "PYTHONPATH": str(Path(official_root).resolve())}
    subprocess.run([sys.executable, "-c", code], env=env, check=True)
    T_raw_to_canonical = _canonicalization_transform(raw_mesh, canonical_mesh)
    result = {
        "raw_mesh": str(raw_mesh),
        "canonical_mesh": str(canonical_mesh),
        "texture": str(mesh_dir / tex.name),
        "T_raw_to_canonical": T_raw_to_canonical.tolist(),
        "canonicalizer": "scalable_real2sim.output.canonicalize.canonicalize_mesh_from_file",
        "gt_excluded": True,
    }
    (output / "canonicalization.json").write_text(json.dumps(result, indent=2))
    return result


def _tracking_first_pose(tracking_dir: Path) -> np.ndarray:
    tracks = sorted(Path(tracking_dir).glob("*.txt"))
    if not tracks:
        raise FileNotFoundError(f"BundleSDF tracking output is empty: {tracking_dir}")
    return np.loadtxt(tracks[0]).reshape(4, 4)


def adapt_payload_data(
    payload_run: Path,
    official_input: Path,
    canonical_mesh: Path,
    scan_manifest: Path,
    tracking_dir: Path,
    output: Path,
    grasp_handoff: dict[str, Any],
) -> dict[str, Any]:
    """Write official object system-ID files from measured FR3 arrays.

    The names and point-cloud contract match the official script.  The estimator used
    later remains the project's FR3 adapter because the vendor solver's plant is IIWA.
    """
    payload_run = Path(payload_run).resolve()
    output = Path(output).resolve()
    baseline_npz, payload_npz = payload_run / "system_id_baseline.npz", payload_run / "system_id_payload.npz"
    if not baseline_npz.exists() or not payload_npz.exists():
        raise FileNotFoundError(f"FR3 system-ID records missing under {payload_run}")
    payload = np.load(payload_npz, allow_pickle=True)
    baseline = np.load(baseline_npz, allow_pickle=True)
    required = {"t", "q", "dq", "tau", "T_B_TCP", "jacobian_TCP", "guard_passed"}
    missing = required - set(payload.files)
    if missing:
        raise ValueError(f"payload record missing {sorted(missing)}")
    if not bool(np.asarray(payload["guard_passed"]).reshape(-1)[-1]):
        raise RuntimeError("payload record did not finish FREE_SPACE_STABLE")
    obj_dir = output / "system_id_data"
    obj_dir.mkdir(parents=True, exist_ok=True)
    t = np.asarray(payload["t"], dtype=float)
    q = np.asarray(payload["q"], dtype=float)
    dq = np.asarray(payload["dq"], dtype=float)
    tau = np.asarray(payload["tau"], dtype=float)
    ddq = np.gradient(dq, t, axis=0, edge_order=2)
    # The current FR3 bridge does not expose a separate WSG position channel in the
    # payload NPZ.  Preserve a measured-command placeholder and mark it explicitly;
    # the FR3 estimator does not use this field.
    wsg = np.zeros(len(t), dtype=float)
    for name, value in {
        "joint_positions.npy": q,
        "joint_velocities.npy": dq,
        "joint_accelerations.npy": ddq,
        "joint_torques.npy": tau,
        "sample_times_s.npy": t,
        "wsg_positions.npy": wsg,
    }.items():
        np.save(obj_dir / name, value)
    T_C_O = _tracking_first_pose(tracking_dir)
    manifest = _load_json(scan_manifest)
    first_records = manifest.get("frames") or manifest.get("views") or []
    if not first_records:
        raise ValueError(f"scan manifest has no synchronized records: {scan_manifest}")
    first = first_records[0]
    T_B_C = np.asarray(first.get("T_B_camera", first.get("T_base_camera")), dtype=float)
    T_B_TCP = np.asarray(first["T_B_TCP"], dtype=float)
    T_C_TCP = np.linalg.inv(T_B_C) @ T_B_TCP
    T_TCP_O = np.linalg.inv(T_C_TCP) @ T_C_O
    import trimesh

    mesh = trimesh.load(canonical_mesh, force="mesh", process=False)
    if len(mesh.vertices) > 5000:
        # trimesh's public sample API has no stable ``seed=`` keyword across the
        # versions used by Isaac/BundleSDF.  Seed NumPy around the call instead.
        state = np.random.get_state()
        np.random.seed(20260914)
        try:
            points = np.asarray(mesh.sample(5000), dtype=float)
        finally:
            np.random.set_state(state)
    else:
        points = np.asarray(mesh.vertices, dtype=float)
    homogeneous = np.c_[points, np.ones(len(points))]
    points_tcp = (T_TCP_O @ homogeneous.T).T[:, :3]
    np.save(obj_dir / "manipuland_cloud_link7_frame.npy", points_tcp)
    metadata = {
        "schema": "scalable_real2sim_official_object_system_id/v1",
        "official_expected_directory": str(obj_dir),
        "robot_frame_substitution": "fr3_hand_tcp_for_official_link7",
        "T_TCP_object": T_TCP_O.tolist(),
        "sample_count": int(len(t)),
        "baseline_samples": int(len(baseline["t"])),
        "wsg_positions_source": "unavailable_in_existing_FR3_payload_npz; zero command placeholder",
        "measured_fields": ["q", "dq", "ddq_derived", "joint_torque", "T_B_TCP", "jacobian_TCP"],
        "payload_guard": "FREE_SPACE_STABLE",
        "gt_excluded": True,
        "handoff": grasp_handoff,
    }
    (output / "official_object_system_id_manifest.json").write_text(json.dumps(metadata, indent=2))
    return {
        "object_system_id": str(obj_dir),
        "baseline_npz": str(baseline_npz),
        "payload_npz": str(payload_npz),
        "samples": int(len(t)),
        "baseline_samples": int(len(baseline["t"])),
        "T_TCP_object": T_TCP_O.tolist(),
        "wsg_positions_source": metadata["wsg_positions_source"],
        "gt_excluded": True,
    }


def run_fr3_payload_id(payload_run: Path, output: Path) -> dict[str, Any]:
    """Run the existing FR3 baseline-subtracted estimator, with provenance."""
    from .payload_id import identify

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    estimate = identify(
        Path(payload_run) / "system_id_baseline.npz",
        Path(payload_run) / "system_id_payload.npz",
        output,
    )
    result = json.loads(output.read_text())
    result.update({
        "estimator": "fr3_payload_id_adapter_using_official_object_data_contract_v1",
        "official_algorithm_reference": "scalable_real2sim.robot_payload_id (IIWA-only vendor plant; FR3 adapter)",
        "elapsed_seconds": time.perf_counter() - start,
        "gt_excluded": True,
    })
    output.write_text(json.dumps(result, indent=2))
    return result


def official_payload_id_status(official_root: Path, robot_id_dir: Path | None) -> dict[str, Any]:
    """Explain whether the vendor IIWA solver can be invoked safely for this FR3 run."""
    if robot_id_dir is None:
        return {
            "status": "not_run_fr3_adapter_selected",
            "reason": "no IIWA identified_robot_params directory supplied; vendor script hard-codes iiwa.dmd.yaml/iiwa_link_7",
            "vendor_commit": _git_rev(Path(official_root) / "scalable_real2sim/robot_payload_id"),
            "gt_excluded": True,
        }
    robot_id_dir = Path(robot_id_dir).resolve()
    params = list(robot_id_dir.glob("gripper_position_*/identified_robot_params.npy"))
    if not params:
        return {
            "status": "not_run_fr3_adapter_selected",
            "reason": f"no official identified_robot_params.npy under {robot_id_dir}; IIWA solver not applicable",
            "vendor_commit": _git_rev(Path(official_root) / "scalable_real2sim/robot_payload_id"),
            "gt_excluded": True,
        }
    return {
        "status": "available_but_not_invoked",
        "reason": "FR3 model compatibility must be audited before invoking IIWA vendor plant",
        "candidate_robot_id_files": [str(p) for p in params],
        "vendor_commit": _git_rev(Path(official_root) / "scalable_real2sim/robot_payload_id"),
        "gt_excluded": True,
    }


def run_official_sdformat(
    name: str,
    visual_mesh: Path,
    inertial: Path,
    output: Path,
    official_root: Path,
) -> dict[str, Any]:
    """Call the vendor SDFormat helper, including its CoACD decomposition."""
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    sdf_path = output / f"{name}_bundle_sdf.sdf"
    # The vendor helper writes mesh URIs relative to ``output_path.parent`` and
    # rejects an input mesh outside that directory.  Stage an exact copy next to
    # the SDF; the project asset still uses ``visual_mesh`` as its source and this
    # copy is retained for an auditable official-SDF package.
    staged_mesh = output / "visual_mesh_for_sdf.obj"
    shutil.copy2(Path(visual_mesh).resolve(), staged_mesh)
    script = f"""
import json, numpy as np
from pathlib import Path
from scalable_real2sim.output.sdformat import create_sdf
p=json.loads(Path({str(inertial)!r}).read_text())
create_sdf(model_name={name!r}, mesh_parts_dir_name={f'{name}_bundle_sdf_parts'!r},
  output_path=Path({str(sdf_path)!r}), visual_mesh_path=Path({str(staged_mesh)!r}),
  collision_mesh_path=Path({str(staged_mesh)!r}), mass=float(p['mass']),
  center_of_mass=np.asarray(p['center_of_mass'],float),
  moment_of_inertia=np.asarray(p['inertia_matrix'],float), use_hydroelastic=False,
  use_coacd=True, coacd_kwargs={{'threshold':0.05, 'max_convex_hull':24, 'seed':20260912}})
"""
    env = {**os.environ, "PYTHONPATH": str(Path(official_root).resolve())}
    start = time.perf_counter()
    subprocess.run([sys.executable, "-c", script], env=env, check=True)
    parts_dir = output / f"{name}_bundle_sdf_parts"
    parts = sorted(parts_dir.glob("*.obj"))
    if not sdf_path.exists() or not parts:
        raise RuntimeError("official create_sdf did not produce SDF and convex parts")
    collision_dir = output / "collision"
    collision_dir.mkdir(exist_ok=True)
    copied = []
    for part in parts:
        target = collision_dir / part.name
        shutil.copy2(part, target)
        copied.append(str(target))
    manifest = {
        "method": "official scalable_real2sim.output.sdformat.create_sdf + CoACD",
        "threshold": 0.05,
        "max_convex_hull": 24,
        "seed": 20260912,
        "parts": copied,
        "sdf": str(sdf_path),
        "source_mesh": str(visual_mesh),
        "staged_mesh": str(staged_mesh),
        "gt_excluded": True,
    }
    collision_manifest = collision_dir / "collision_manifest.json"
    collision_manifest.write_text(json.dumps(manifest, indent=2))
    return {
        **manifest,
        "collision_manifest": str(collision_manifest),
        "elapsed_seconds": time.perf_counter() - start,
    }


def metric_frame_from_tracking(scan_manifest: Path, tracking_dir: Path, canonicalization: dict[str, Any]) -> dict[str, Any]:
    """Compute the TCP→canonical-object transform using robot pose + tracking only."""
    manifest = _load_json(scan_manifest)
    first_records = manifest.get("frames") or manifest.get("views") or []
    if not first_records:
        raise ValueError(f"scan manifest has no synchronized records: {scan_manifest}")
    first = first_records[0]
    T_B_C = np.asarray(first.get("T_B_camera", first.get("T_base_camera")), dtype=float)
    T_B_TCP = np.asarray(first["T_B_TCP"], dtype=float)
    T_C_O_raw = _tracking_first_pose(tracking_dir)
    T_C_TCP = np.linalg.inv(T_B_C) @ T_B_TCP
    T_TCP_O_raw = np.linalg.inv(T_C_TCP) @ T_C_O_raw
    T_raw_to_canonical = np.asarray(canonicalization["T_raw_to_canonical"], dtype=float)
    T_TCP_O_canonical = T_TCP_O_raw @ np.linalg.inv(T_raw_to_canonical)
    return {
        "T_base_camera": T_B_C.tolist(),
        "T_base_TCP": T_B_TCP.tolist(),
        "T_camera_object_raw": T_C_O_raw.tolist(),
        "T_TCP_object_raw": T_TCP_O_raw.tolist(),
        "T_raw_to_canonical": T_raw_to_canonical.tolist(),
        "T_TCP_object_canonical": T_TCP_O_canonical.tolist(),
        "frame_chain": "T_TCP_object = inv(inv(T_B_camera) @ T_B_TCP) @ T_camera_object; canonical adjustment applied",
        "alignment_inputs": ["robot T_base_camera/T_B_TCP", "BundleSDF ob_in_cam", "official canonicalization transform"],
        "gt_excluded": True,
    }


def audit_reconstruction_coverage(
    official_input: Path,
    tracking_dir: Path,
    canonical_mesh: Path,
    canonicalization: dict[str, Any],
    output: Path,
    depth_tolerance_m: float = 0.006,
) -> dict[str, Any]:
    """Audit direct RGB-D support for the generated mesh without using GT.

    A face is counted as depth-supported when its centroid projects inside the
    real object mask and agrees with measured depth.  This is a conservative
    diagnostic only: it does not fill holes or modify reconstruction.  The
    reported coverage is therefore support of the generated surface, and the
    remainder is explicitly labelled generated/unobserved.
    """
    import cv2
    import trimesh

    official_input = Path(official_input).resolve()
    tracking_dir = Path(tracking_dir).resolve()
    canonical_mesh = Path(canonical_mesh).resolve()
    output = Path(output).resolve()
    pose_manifest = _load_json(official_input / "robot_camera_poses.json")
    frame_meta = {str(f.get("file_stem")): f for f in pose_manifest.get("frames", [])}
    K = np.loadtxt(official_input / "cam_K.txt").reshape(3, 3)
    mesh = trimesh.load(canonical_mesh, force="mesh", process=False)
    centers = np.asarray(mesh.triangles_center, dtype=float)
    areas = np.asarray(mesh.area_faces, dtype=float)
    if len(centers) == 0 or not np.isfinite(areas).all() or float(areas.sum()) <= 0:
        raise RuntimeError("canonical mesh has no finite surface area for coverage audit")
    inverse_canonical = np.linalg.inv(np.asarray(canonicalization["T_raw_to_canonical"], dtype=float))
    per_frame: list[dict[str, Any]] = []
    supported_by_stem: dict[str, np.ndarray] = {}
    for track_path in sorted(tracking_dir.glob("*.txt")):
        stem = track_path.stem
        T_camera_object_raw = np.loadtxt(track_path).reshape(4, 4)
        T_camera_object = T_camera_object_raw @ inverse_canonical
        X = (T_camera_object @ np.c_[centers, np.ones(len(centers))].T).T[:, :3]
        z = X[:, 2]
        u = K[0, 0] * X[:, 0] / np.maximum(z, 1e-9) + K[0, 2]
        v = K[1, 1] * X[:, 1] / np.maximum(z, 1e-9) + K[1, 2]
        depth = cv2.imread(str(official_input / "depth" / f"{stem}.png"), cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(str(official_input / "masks" / f"{stem}.png"), cv2.IMREAD_GRAYSCALE)
        if depth is None or mask is None:
            raise FileNotFoundError(f"coverage inputs missing for frame {stem}")
        depth_m = depth.astype(np.float64) * 0.001
        in_image = (
            (z > 0) & (u >= 0) & (u < depth_m.shape[1])
            & (v >= 0) & (v < depth_m.shape[0])
        )
        hit = np.zeros(len(centers), dtype=bool)
        indices = np.where(in_image)[0]
        ui = np.rint(u[indices]).astype(int)
        vi = np.rint(v[indices]).astype(int)
        # A projected coordinate can be strictly inside the image while its
        # nearest integer rounds onto the first pixel outside (e.g. v=479.8
        # -> 480).  Drop only those boundary samples before indexing; this is
        # an audit guard and does not alter the reconstructed mesh.
        rounded_valid = (
            (ui >= 0) & (ui < depth_m.shape[1])
            & (vi >= 0) & (vi < depth_m.shape[0])
        )
        indices = indices[rounded_valid]
        ui = ui[rounded_valid]
        vi = vi[rounded_valid]
        measured = depth_m[vi, ui]
        hit[indices] = (
            (mask[vi, ui] > 0) & (measured > 0)
            & (np.abs(measured - z[indices]) <= float(depth_tolerance_m))
        )
        supported_by_stem[stem] = hit
        frame = frame_meta.get(stem, {})
        camera_center_object = -T_camera_object[:3, :3].T @ T_camera_object[:3, 3]
        camera_center_object /= max(float(np.linalg.norm(camera_center_object)), 1e-12)
        per_frame.append({
            "stem": stem,
            "pass_id": int(frame.get("pass_id", 0)),
            "depth_supported_faces": int(hit.sum()),
            "depth_supported_area_fraction": float(areas[hit].sum() / areas.sum()),
            "azimuth_deg_object": float(np.degrees(np.arctan2(camera_center_object[1], camera_center_object[0]))),
            "elevation_deg_object": float(np.degrees(np.arcsin(np.clip(camera_center_object[2], -1.0, 1.0)))),
            "gripper_excluded": True,
            "gt_excluded": True,
        })
    pass_ids = sorted({int(f["pass_id"]) for f in per_frame})
    cumulative = np.zeros(len(centers), dtype=bool)
    pass_stats = []
    for pass_id in pass_ids:
        before = cumulative.copy()
        stems = [f["stem"] for f in per_frame if int(f["pass_id"]) == pass_id]
        for stem in stems:
            cumulative |= supported_by_stem[stem]
        pass_stats.append({
            "pass_id": pass_id,
            "frames": len(stems),
            "cumulative_depth_supported_area_fraction": float(areas[cumulative].sum() / areas.sum()),
            "new_depth_supported_area_fraction": float(areas[cumulative & ~before].sum() / areas.sum()),
            "new_faces": int((cumulative & ~before).sum()),
            "gt_excluded": True,
        })
    azimuth = np.asarray([f["azimuth_deg_object"] for f in per_frame], dtype=float)
    elevation = np.asarray([f["elevation_deg_object"] for f in per_frame], dtype=float)
    az_bins = ((azimuth + 22.5) // 45).astype(int) % 8
    edges = np.asarray(mesh.edges_sorted)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    texture_path = canonical_mesh.parent / "material_0.png"
    texture_stats: dict[str, Any] = {"path": str(texture_path), "exists": texture_path.exists()}
    if texture_path.exists():
        texture = cv2.imread(str(texture_path), cv2.IMREAD_UNCHANGED)
        texture_stats.update({
            "shape": list(texture.shape),
            "nonzero_pixel_fraction": float(np.mean(np.any(texture != 0, axis=2))),
        })
    uv = getattr(mesh.visual, "uv", None)
    texture_stats["uv_vertex_fraction"] = float(
        np.mean(np.isfinite(uv).all(axis=1)) if uv is not None and len(uv) else 0.0
    )
    result = {
        "schema": "scalable_real2sim_coverage_audit/v1",
        "frames": len(per_frame),
        "accepted_frames": len(per_frame),
        "pass_stats": pass_stats,
        "per_frame": per_frame,
        "surface_coverage_proxy": float(areas[cumulative].sum() / areas.sum()),
        "generated_or_unobserved_surface_proxy": float(areas[~cumulative].sum() / areas.sum()),
        "depth_tolerance_m": float(depth_tolerance_m),
        "viewing_direction": {
            "azimuth_bins_8_count": np.bincount(az_bins, minlength=8).tolist(),
            "azimuth_range_deg": [float(azimuth.min()), float(azimuth.max())],
            "elevation_range_deg": [float(elevation.min()), float(elevation.max())],
            "top_gt30deg_frames": int(np.sum(elevation > 30)),
            "bottom_lt_minus30deg_frames": int(np.sum(elevation < -30)),
            "side_frames": int(np.sum(np.abs(elevation) <= 30)),
            "source": "BundleSDF ob_in_cam tracking; diagnostic only",
        },
        "mesh_topology": {
            "vertices": int(len(mesh.vertices)),
            "faces": int(len(mesh.faces)),
            "extents_m": [float(x) for x in mesh.extents],
            "watertight": bool(mesh.is_watertight),
            "boundary_edges": int(np.sum(counts == 1)),
        },
        "texture": texture_stats,
        "gt_excluded": True,
        "note": "Coverage is measured support of generated faces, not a GT object coverage estimate.",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    return result


def run_asset_builder(
    name: str,
    visual_mesh: Path,
    texture: Path,
    collision_manifest: Path,
    inertial: Path,
    output: Path,
) -> dict[str, Any]:
    from .asset_builder import build

    start = time.perf_counter()
    result = build(name, Path(visual_mesh), Path(texture), Path(collision_manifest), Path(inertial), Path(output).resolve())
    result["elapsed_seconds"] = time.perf_counter() - start
    return result


def run_fresh_isaac(asset: Path, output_dir: Path, isaac_python: Path, gpu: int = 3) -> dict[str, Any]:
    """Run the existing validator as a genuinely new Isaac process."""
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    image = output_dir / "fresh_reload.png"
    command = [str(isaac_python), "-u", "calibration/validate_real2sim_usd.py", "--asset", str(Path(asset).resolve()), "--output", str(image), "--gpu", str(gpu)]
    env = {**os.environ, "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y", "PYTHONPATH": str(Path(__file__).resolve().parents[1])}
    log = output_dir / "fresh_isaac.log"
    start = time.perf_counter()
    with log.open("w") as stream:
        proc = subprocess.run(command, cwd=Path(__file__).resolve().parents[1], env=env, stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=1800)
    result_path = output_dir / "reload_validation.json"
    result = _load_json(result_path) if result_path.exists() else {"loaded": True, "log": str(log)}
    result.update({"command": command, "log": str(log), "elapsed_seconds": time.perf_counter() - start, "fresh_process": True})
    result_path.write_text(json.dumps(result, indent=2))
    return result


def run_gt_evaluation(asset_dir: Path, gt_mesh: Path, gt_physics: Path, reconstruction_mesh: Path) -> dict[str, Any]:
    from .validate_asset import validate

    start = time.perf_counter()
    result = validate(Path(asset_dir), Path(gt_mesh), Path(gt_physics), Path(reconstruction_mesh), samples=20000)
    result["elapsed_seconds"] = time.perf_counter() - start
    Path(asset_dir, "validation.json").write_text(json.dumps(result, indent=2))
    return result
