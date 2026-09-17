"""Run the frozen FR3 grasp handoff through the official Scalable Real2Sim branch.

The scan and payload records are supplied explicitly so this runner is restartable.
The grasp executor is only read for the stable-handoff check; no grasp code is
imported or modified here.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from real2sim.scalable_real2sim_bridge import (  # noqa: E402
    DEFAULT_OFFICIAL_ROOT,
    adapt_payload_data,
    audit_reconstruction_coverage,
    canonicalize_official_mesh,
    metric_frame_from_tracking,
    official_payload_id_status,
    prepare_official_dataset,
    run_asset_builder,
    run_fresh_isaac,
    run_fr3_payload_id,
    run_gt_evaluation,
    run_official_bundlesdf,
    run_official_sdformat,
    validate_stable_handoff,
    verify_official_checkout,
)


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--grasp-result", type=Path, required=True)
    p.add_argument("--scan-dir", type=Path, required=True)
    p.add_argument("--payload-run", type=Path, required=True, help="directory containing system_id_baseline.npz and system_id_payload.npz")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
    p.add_argument("--bundle-python", type=Path)
    p.add_argument("--isaac-python", type=Path, default=Path("/data1/home/rangeryx/isaaclab-arena/.venv/bin/python"))
    p.add_argument("--robot-id-dir", type=Path)
    p.add_argument("--gt-mesh", type=Path)
    p.add_argument("--gt-physics", type=Path)
    p.add_argument("--gpu", type=int, default=3)
    p.add_argument("--name", default="soup_scalable_real2sim_official")
    p.add_argument("--force-reconstruct", action="store_true")
    p.add_argument("--skip-isaac", action="store_true")
    p.add_argument("--skip-gt-eval", action="store_true")
    return p


def main() -> int:
    a = _parser().parse_args()
    root = Path(a.output).resolve()
    root.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    stages: dict[str, object] = {}

    handoff = validate_stable_handoff(a.grasp_result)
    stages["stable_handoff"] = handoff
    commits = verify_official_checkout(a.official_root)
    stages["official_commits"] = commits

    official_input = root / "official_input" / str(handoff["target"])
    stages["prepare_input"] = prepare_official_dataset(a.scan_dir, official_input, handoff)

    recon_dir = root / "bundle_sdf"
    stages["bundlesdf"] = run_official_bundlesdf(
        official_input, recon_dir, a.official_root, a.bundle_python, a.force_reconstruct
    )
    recon = stages["bundlesdf"]
    if not isinstance(recon, dict):
        raise RuntimeError("BundleSDF stage did not return a manifest")

    stages["canonicalization"] = canonicalize_official_mesh(recon, root, a.official_root)
    canonical = stages["canonicalization"]
    if not isinstance(canonical, dict):
        raise RuntimeError("canonicalization stage did not return a manifest")
    canonical_mesh = Path(canonical["canonical_mesh"])
    tracking_dir = Path(recon["products"]["tracking"])
    stages["coverage_audit"] = audit_reconstruction_coverage(
        official_input,
        tracking_dir,
        canonical_mesh,
        canonical,
        root / "coverage_audit.json",
    )
    # ``robot_camera_poses.json`` is the sidecar with the per-frame transforms;
    # official_input_manifest.json intentionally contains QA/provenance only.
    stages["metric_frame"] = metric_frame_from_tracking(
        official_input / "robot_camera_poses.json", tracking_dir, canonical
    )

    # The official object system-ID directory is created even though the solver
    # itself is IIWA-specific.  This keeps the exact file contract auditable.
    stages["official_object_data"] = adapt_payload_data(
        a.payload_run,
        official_input,
        canonical_mesh,
        official_input / "robot_camera_poses.json",
        tracking_dir,
        root / "official_payload_id",
        handoff,
    )
    stages["official_payload_id"] = official_payload_id_status(a.official_root, a.robot_id_dir)
    raw_payload_path = root / "inertial_tcp.json"
    stages["fr3_payload_id"] = run_fr3_payload_id(a.payload_run, raw_payload_path)

    # Existing frame transform is intentionally limited to the recorded robot pose
    # and BundleSDF track.  Re-express the estimate in the official canonical frame.
    import numpy as np

    raw = json.loads(raw_payload_path.read_text())
    # payload_id.py reports COM/inertia in the measured TCP frame.  The
    # canonicalized BundleSDF mesh has its own object frame.  metric_frame above
    # provides T_TCP_object_canonical (object -> TCP); invert it to express the
    # measured inertial estimate in the mesh frame.  Do not infer this from GT or
    # from a field that the FR3 estimator does not emit.
    metric = stages["metric_frame"]
    if not isinstance(metric, dict):
        raise RuntimeError("metric frame stage did not return a transform")
    T_tcp_canon = np.asarray(metric["T_TCP_object_canonical"], dtype=float)
    T_canon_tcp = np.linalg.inv(T_tcp_canon)
    R_canon_tcp = T_canon_tcp[:3, :3]
    raw["center_of_mass"] = (R_canon_tcp @ np.asarray(raw["center_of_mass"], dtype=float) + T_canon_tcp[:3, 3]).tolist()
    raw["inertia_matrix"] = (R_canon_tcp @ np.asarray(raw["inertia_matrix"], dtype=float) @ R_canon_tcp.T).tolist()
    raw["T_TCP_object"] = T_tcp_canon.tolist()
    raw["expressed_in"] = "official_BundleSDF_canonical_object"
    raw["frame_source"] = "robot T_base_camera/T_B_TCP + official BundleSDF ob_in_cam + vendor canonicalizer; no GT"
    canonical_inertial = root / "inertial_object.json"
    canonical_inertial.write_text(json.dumps(raw, indent=2))

    from real2sim.inertia_regularization import regularize

    simulation_inertial = root / "inertial_object_simulation.json"
    stages["observability_fallback"] = regularize(canonical_inertial, canonical_mesh, simulation_inertial)
    collision_dir = root / "official_collision"
    stages["collision"] = run_official_sdformat(a.name, canonical_mesh, simulation_inertial, collision_dir, a.official_root)
    collision = stages["collision"]
    if not isinstance(collision, dict):
        raise RuntimeError("collision stage did not return a manifest")
    asset_dir = root / "asset"
    stages["asset"] = run_asset_builder(
        a.name,
        canonical_mesh,
        Path(canonical["texture"]),
        Path(collision["collision_manifest"]),
        simulation_inertial,
        asset_dir,
    )
    asset = stages["asset"]
    if not isinstance(asset, dict):
        raise RuntimeError("asset stage did not return a manifest")

    if not a.skip_isaac:
        stages["fresh_isaac"] = run_fresh_isaac(Path(asset["usd"]), root / "fresh_isaac", a.isaac_python, a.gpu)
    if not a.skip_gt_eval:
        if not a.gt_mesh or not a.gt_physics:
            stages["gt_evaluation"] = {"status": "not_run", "reason": "--gt-mesh and --gt-physics not supplied"}
        else:
            stages["gt_evaluation"] = run_gt_evaluation(asset_dir, a.gt_mesh, a.gt_physics, canonical_mesh)

    stages["timing"] = {"total_seconds": time.perf_counter() - started}
    manifest = {
        "schema": "fr3_scalable_real2sim_official_handoff/v1",
        "pipeline": "frozen FR3 grasp -> stable handoff -> official BundleSDF run_video (tracking + global refinement + texture) -> FR3 payload adapter with official data contract -> official CoACD/SDFormat -> Isaac USD -> fresh Isaac reload",
        "grasp_executor_modified": False,
        "gt_used_for_estimator": False,
        "gt_used_only_for_final_evaluation": bool(a.gt_mesh and a.gt_physics),
        "official_commits": commits,
        "stages": stages,
        "products": {
            "raw_bundle_sdf_mesh": recon["products"].get("mesh"),
            "raw_bundle_sdf_textured_mesh": recon["products"].get("textured_mesh"),
            "canonical_textured_mesh": str(canonical_mesh),
            "collision_sdf": collision.get("sdf"),
            "collision_manifest": collision.get("collision_manifest"),
            "usd": asset.get("usd"),
        },
        "differences_from_official_cli": [
            "The official top-level script also launches Nerfstudio/Frosting/Neuralangelo alternatives; this run intentionally executes the requested BundleSDF branch only.",
            "The vendor robot_payload_id commit is IIWA-specific (iiwa.dmd.yaml/iiwa_link_7); measured FR3 records use the existing FR3 baseline-subtracted adapter with the official object data filenames and an explicit observability fallback.",
            "BundleSDF's run_custom.py estimates ob_in_cam from RGB-D; robot T_base_camera/T_B_TCP are preserved and used for metric TCP/object frame conversion, not silently substituted into the vendor tracker.",
        ],
    }
    (root / "scalable_real2sim_pipeline_manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
