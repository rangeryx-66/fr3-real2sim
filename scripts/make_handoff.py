#!/usr/bin/env python3
"""Read-only, fail-closed agent handoff snapshots for existing FR3 artifacts."""

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import sys
import tempfile
from pathlib import Path


SCHEMA = "fr3_agent_handoff/v1"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    path = Path(path).expanduser().resolve(strict=True)
    with path.open(encoding="utf-8") as stream:
        return json.load(stream), path


def artifact(path):
    path = Path(path).expanduser().resolve(strict=True)
    require(path.is_file() and path.stat().st_size > 0, f"missing/empty artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"path": str(path), "sha256": digest.hexdigest()}


def base(stage, target, sources, checks, next_allowed):
    return {
        "schema": SCHEMA,
        "stage": stage,
        "target_id": target,
        "source_artifacts": {name: artifact(path) for name, path in sources.items()},
        "checks": checks,
        "next_allowed": next_allowed,
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    }


def grasp(args):
    row, path = read_json(args.result)
    flags = row.get("flags") or {}
    support = row.get("final_support") or {}
    require(row.get("mode") == "ANYGRASP", "production handoff requires ANYGRASP mode")
    require(row.get("success") is True and row.get("category") == "SUCCESS", "grasp did not succeed")
    require(all(flags.get(key) is True for key in ("PICKED", "RETAINED", "CLEAR_TABLE", "lift_ge_8cm")), "physical success flags incomplete")
    require(flags.get("DROP") is not True, "target dropped")
    require(support.get("passed") is True and support.get("currently_clear") is True, "target is not free-space stable")
    target = row.get("target")
    require(isinstance(target, str) and target, "target missing")
    return base("GRASP_STABLE", target, {"grasp_result": path},
                {"seed": row.get("seed"), "selected_rank": row.get("selected_rank"),
                 "physical_flags": {key: flags.get(key) for key in ("PICKED", "RETAINED", "CLEAR_TABLE", "lift_ge_8cm", "DROP")},
                 "free_space_stable": True}, ["PayloadID", "PhysicalPlacement"])


def scan(args):
    row, path = read_json(args.manifest)
    require(row.get("schema") == "fr3_scan_station/v1", "unexpected scan manifest schema")
    require(row.get("state") == "placed_released_arm_parked", "station is not released/parked")
    target = row.get("target")
    require(isinstance(target, str) and target, "scan target missing")
    accepted = [view for view in row.get("views", []) if (view.get("quality") or {}).get("accepted") is True]
    minimum = max(1, int((row.get("config") or {}).get("minimum_accepted_views", 18)))
    require(len(accepted) >= minimum and len(accepted) == row.get("accepted_count"), "scan QA count inconsistent or too low")
    for view in accepted:
        for field in ("rgb", "depth", "object_mask", "gripper_mask"):
            file = Path(view.get(field, ""))
            if not file.is_absolute():
                file = path.parent / file
            require(file.is_file() and file.stat().st_size > 0, f"accepted frame missing {field}: {file}")
        require(view.get("intrinsics") is not None and view.get("T_B_camera") is not None,
                "accepted frame missing intrinsics or robot camera pose")
    sources = {"scan_manifest": path}
    continuity = "INDEPENDENT_SCENE"
    if args.placement_record:
        placement, placement_path = read_json(args.placement_record)
        require(placement.get("target_id") == target and all(placement.get(key) is True for key in
                ("released", "arm_parked", "object_stationary")), "placement record does not verify same released target")
        require(placement.get("pose") is not None and placement.get("frame") and placement.get("timestamp") and placement.get("run_id"),
                "placement record lacks pose/frame/timestamp/run_id")
        sources["placement_record"] = placement_path
        continuity = "VERIFIED_BY_PLACEMENT_RECORD"
    return base("SCAN_ACCEPTED", target, sources,
                {"accepted_views": len(accepted), "rejected_views": row.get("rejected_count"),
                 "accepted_frame_ids": [view.get("frame_id") for view in accepted],
                 "identity_continuity": continuity, "gt_used_for_acquisition_or_alignment": row.get("gt_used_for_acquisition_or_alignment")},
                ["MV-SAM3D"])


def mv(args):
    scan_row, scan_path = read_json(args.scan_handoff)
    require(scan_row.get("schema") == SCHEMA and scan_row.get("stage") == "SCAN_ACCEPTED", "SCAN_ACCEPTED handoff required")
    selection, selection_path = read_json(args.selection)
    require(selection.get("schema") == "mv_sam3d_input_selection/v1", "unexpected view selection schema")
    target = scan_row["target_id"]
    require(selection.get("object") == target, "selection target differs from scan target")
    choices = selection.get("selections") or {}
    key = str(args.views)
    require(key in choices and args.views in (2, 4, 8), "requested 2/4/8-view selection absent")
    chosen = choices[key]
    selected = chosen.get("selected") or []
    require(chosen.get("view_count") == args.views and len(selected) == args.views, "view count inconsistent")
    require(all(item.get("area_fraction", 0) > 0 and item.get("T_B_camera") is not None for item in selected),
            "selected mask/camera pose invalid")
    accepted_ids = (scan_row.get("checks") or {}).get("accepted_frame_ids") or []
    require(all(item.get("source_frame_id") in accepted_ids for item in selected),
            "selection includes a frame that did not pass scan QA")
    glb = Path(args.glb).expanduser().resolve(strict=True)
    return base("VISUAL_ONLY_MV_SAM3D", target,
                {"scan_handoff": scan_path, "selection_manifest": selection_path, "raw_glb": glb},
                {"selected_view_count": args.views,
                 "selected_frame_ids": [item.get("source_frame_id") for item in selected],
                 "adjacent_mask_iou": chosen.get("adjacent_mask_iou"),
                 "identity_continuity": (scan_row.get("checks") or {}).get("identity_continuity"),
                 "metric_scale_verified": False, "physics_ready": False}, ["MetricAlignmentAndTexture"])


def payload(args):
    grasp_row, grasp_path = read_json(args.grasp_handoff)
    require(grasp_row.get("schema") == SCHEMA and grasp_row.get("stage") == "GRASP_STABLE", "GRASP_STABLE handoff required")
    summary, summary_path = read_json(args.summary)
    target = grasp_row["target_id"]
    row = summary.get(target)
    require(isinstance(row, dict), f"payload summary has no entry for {target}")
    fit = (((row.get("methods") or {}).get("drake") or {}).get("fit") or {})
    poses = row.get("accepted_poses", 0)
    mass = fit.get("mass_kg")
    com = fit.get("center_of_mass_m")
    valid = (fit.get("accepted") in (True, 1) and isinstance(poses, int) and poses >= 4
             and isinstance(mass, (int, float)) and math.isfinite(mass) and mass > 0
             and isinstance(com, list) and len(com) == 3
             and all(isinstance(x, (int, float)) and math.isfinite(x) for x in com))
    checks = {"accepted_poses": poses, "fit_accepted": fit.get("accepted") in (True, 1),
              "fit_reason": fit.get("reason"), "fit_source": fit.get("source"),
              "residual_rms_Nm": fit.get("residual_rms_Nm"),
              "observability": fit.get("observability"),
              "mass_sigma_kg": fit.get("mass_sigma_kg"),
              "center_of_mass_sigma_m": fit.get("center_of_mass_sigma_m"),
              "inertia_source": "GEOMETRY_FALLBACK"}
    if valid:
        checks.update({"mass_kg": mass, "center_of_mass_m": com,
                       "com_frame": fit.get("frame"),
                       "mass_source": "IDENTIFIED", "com_source": "IDENTIFIED_Q_COMPENSATED"})
    else:
        checks.update({"mass_source": "UNAVAILABLE", "com_source": "GEOMETRY_FALLBACK"})
    return base("PAYLOAD_IDENTIFIED" if valid else "PAYLOAD_FALLBACK", target,
                {"grasp_handoff": grasp_path, "payload_summary": summary_path}, checks,
                ["AssetExport"] if valid else ["GeometryFallbackReview"])


def write_atomic(path, data):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".handoff-", suffix=".json", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="stage", required=True)
    commands = {}
    for name, fields, function in (
        ("grasp", ("result",), grasp),
        ("scan", ("manifest",), scan),
        ("mv", ("scan-handoff", "selection", "glb"), mv),
        ("payload", ("grasp-handoff", "summary"), payload),
    ):
        command = subs.add_parser(name)
        for field in fields:
            command.add_argument("--" + field, required=True)
        command.add_argument("--output", required=True)
        commands[name] = function
    subs.choices["scan"].add_argument("--placement-record")
    subs.choices["mv"].add_argument("--views", type=int, default=8)
    args = parser.parse_args()
    try:
        result = commands[args.stage](args)
        write_atomic(args.output, result)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        print(f"HANDOFF_REJECTED: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"stage": result["stage"], "target_id": result["target_id"], "output": str(Path(args.output).resolve())}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
