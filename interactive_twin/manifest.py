"""Outcome-independent, frozen articulated-object benchmark inventory.

Dataset preparation may inspect URDF joints. Online controllers receive only the
whitelist from ``controller_episode``; initialization/evaluation metadata is not
an online observation. Selection never reads execution result files.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import random
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

SCHEMA = "interactive-twin-manifest-v1"
DEFAULT_POLICY = {
    "dev_asset_id": "7320",
    "test_asset_count": 4,
    "episodes_per_asset": 3,
    "seed": 732061660,
    "required_joint_type": "revolute",
    "min_handle_length_m": 0.02543,
    "max_handle_width_m": 0.1,
    "min_handle_aspect_ratio": 1.6,
    "min_visual_rear_clearance_m": 0.005,
    "selection_order": "category_round_robin_then_visual_clearance_length_id",
    "initial_articulation_offsets_deg": [0.0, 1.0, 2.0],
    "along_handle_offset_bound_m": 0.01,
    "relative_translation_bound_m": [0.01, 0.01, 0.0],
    "relative_yaw_bound_deg": 3.0,
    "placement_rule": "visual_handle_anchor_align_DEV_nominal_plus_handle_frame_perturbation",
    "budgets": {
        "grasp_candidates": 12,
        "probe_directions": 4,
        "sysid_simulations": 24,
        "wall_clock_seconds": 1800,
    },
    "minimum_joint_margin_rad": 0.05,
    "grasp_family": "bar-side-pinch",
    "candidate_grid": {
        "along_handle_offsets_m": [-0.005, 0.0, 0.005],
        "depth_offsets_m": [0.0, 0.002],
        "finger_swap_degrees": [0.0, 180.0],
    },
    "geometry_acceptance": "screening_only_actual_contact_and_safety_required",
    "raw_triangle_policy": "diagnostic_only_not_an_execution_veto",
}


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _positive(value: Any, default: float = 0.0) -> float:
    try:
        n = float(value)
        return n if math.isfinite(n) else default
    except (TypeError, ValueError):
        return default


def _joints(urdf: Path) -> list[dict]:
    rows = []
    for joint in ET.parse(urdf).getroot().findall("joint"):
        if joint.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        rows.append({
            "name": joint.get("name"), "type": joint.get("type"),
            "parent_link": joint.find("parent").get("link"),
            "moving_link": joint.find("child").get("link"),
            "limits": {key: float(limit.get(key)) if limit is not None and limit.get(key) is not None else None
                       for key in ("lower", "upper")},
        })
    return rows


def _resolve_urdf(root: Path, meta: dict) -> Path | None:
    asset_id = str(meta.get("asset_id", root.name))
    for path in (root / "urdf" / f"{asset_id}.urdf", root / f"{asset_id}.urdf"):
        if path.is_file():
            return path.resolve()
    supplied = meta.get("prepared_urdf")
    if supplied:
        path = Path(supplied)
        path = path if path.is_absolute() else root / path
        if path.is_file():
            return path.resolve()
    paths = sorted(root.glob("urdf/*.urdf"))
    return paths[0].resolve() if len(paths) == 1 else None


def scan_prepared(prepared_roots: Iterable[str | Path]) -> list[dict]:
    """Recursively inventory actual prepared manifests; malformed assets survive."""
    manifest_paths: set[Path] = set()
    missing_roots = []
    for value in prepared_roots:
        root = Path(value).expanduser()
        if root.is_file():
            manifest_paths.add(root.resolve())
        elif root.is_dir():
            manifest_paths.update(p.resolve() for p in root.rglob("manifest.json"))
        else:
            missing_roots.append({"asset_id": None, "asset_root": str(root),
                                  "exclusion_reasons": ["PREPARED_ROOT_MISSING"]})
    inventory = []
    for path in sorted(manifest_paths):
        try:
            meta = json.loads(path.read_text())
        except (OSError, ValueError) as exc:
            inventory.append({"asset_id": None, "manifest": str(path),
                              "exclusion_reasons": ["INVALID_PREPARED_MANIFEST"], "diagnostic": str(exc)})
            continue
        if "asset_id" not in meta:
            continue  # Robot collider manifests are not dataset object records.
        root = path.parent
        asset_id = str(meta["asset_id"])
        urdf = _resolve_urdf(root, meta)
        row = {
            "asset_id": asset_id, "category": str(meta.get("category") or meta.get("object_name") or "UNKNOWN"),
            "object_name": str(meta.get("object_name") or meta.get("category") or "UNKNOWN"),
            "asset_root": str(root), "manifest": str(path), "manifest_sha256": _sha(path),
            "urdf": str(urdf) if urdf else None, "urdf_sha256": _sha(urdf) if urdf else None,
            "dataset": meta.get("source_url", "prepared_PhysX_Mobility_source_not_recorded"),
            "license": meta.get("license"), "prepared_geometry_sha256": meta.get("prepared_geometry_sha256"),
            "geometry_is_approximation": bool(meta.get("interaction_geometry")),
            "geometry_preparation_note": meta.get("physics_note"),
            "source_scale_to_meters": meta.get("scale_source_to_meters"),
            "initial_scene": copy.deepcopy(meta.get("initial_scene", meta.get("scene", {}))),
            "joints": [], "handle_candidates": [], "geometry_inputs": [], "exclusion_reasons": [],
        }
        if urdf is None:
            row["exclusion_reasons"].append("PREPARED_URDF_MISSING")
        else:
            try:
                row["joints"] = _joints(urdf)
                for mesh in ET.parse(urdf).getroot().findall(".//geometry/mesh"):
                    filename = mesh.get("filename", "")
                    mesh_path = (urdf.parent / filename).resolve()
                    if mesh_path.is_file():
                        row["geometry_inputs"].append({"path": str(mesh_path), "sha256": _sha(mesh_path)})
                    else:
                        row["exclusion_reasons"].append("MISSING_REFERENCED_GEOMETRY")
                row["exclusion_reasons"] = sorted(set(row["exclusion_reasons"]))
            except (OSError, ValueError, ET.ParseError, AttributeError) as exc:
                row["exclusion_reasons"].append("INVALID_PREPARED_URDF")
                row["diagnostic"] = str(exc)
        proxy = meta.get("interaction_geometry", {})
        if isinstance(proxy.get("selection"), dict):
            selection = copy.deepcopy(proxy["selection"])
            selection.update(asset_id=asset_id, asset_root=str(root), geometry_source="semantic_proxy_visual_selection")
            row["handle_candidates"].append(selection)
        inventory.append(row)
    return inventory + missing_roots


def scan_source(source_roots: Iterable[str | Path]) -> list[dict]:
    """Inventory raw dataset URDFs for automatic preparation, without changing them."""
    rows = []
    seen: set[Path] = set()
    for root_value in source_roots:
        root = Path(root_value).expanduser()
        if not root.exists():
            rows.append({"source_root": str(root), "asset_id": None,
                         "exclusion_reasons": ["SOURCE_ROOT_MISSING"]})
            continue
        paths = [root] if root.is_file() else sorted(root.rglob("*.urdf"))
        for urdf in paths:
            urdf = urdf.resolve()
            if urdf in seen:
                continue
            seen.add(urdf)
            asset_id = urdf.stem
            category = "UNKNOWN"
            metadata = urdf.parent.parent / "finaljson" / f"{asset_id}.json"
            if metadata.is_file():
                try:
                    meta = json.loads(metadata.read_text())
                    if isinstance(meta, dict):
                        category = str(meta.get("category") or meta.get("object_name") or meta.get("model_cat") or "UNKNOWN")
                except (OSError, ValueError):
                    pass
            row = {"asset_id": asset_id, "category": category, "source_root": str(root),
                   "source_urdf": str(urdf), "source_urdf_sha256": _sha(urdf),
                   "exclusion_reasons": [], "requires_preparation": True}
            try:
                row["joints"] = _joints(urdf)
                if not any(j["type"] == "revolute" for j in row["joints"]):
                    row["exclusion_reasons"].append("NO_REVOLUTE_JOINT")
            except (OSError, ValueError, ET.ParseError, AttributeError) as exc:
                row["exclusion_reasons"].append("INVALID_SOURCE_URDF")
                row["diagnostic"] = str(exc)
            rows.append(row)
    return rows


def geometry_rankings(prepared_roots: Iterable[str | Path]) -> tuple[list[dict], list[dict]]:
    """Reuse the existing geometric scanner, optionally on the simulation server.

    This helper imports NumPy/trimesh only when requested. Scanner errors are
    inventory diagnostics, never grounds to delete an already selected episode.
    Old scanner raw/cooked acceptance prose is intentionally not propagated.
    """
    script = Path(__file__).resolve().parents[1] / "scripts/rank_prepared_handle_assets.py"
    spec = importlib.util.spec_from_file_location("interactive_twin_existing_ranker", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assets = [r for r in scan_prepared(prepared_roots) if r.get("manifest") and r.get("asset_id")]
    rows, errors = [], []
    for asset in assets:
        original = Path(asset["manifest"]).parent
        try:
            # Existing ranker scans one level. A single read-only asset symlink
            # isolates malformed meshes and supports recursive prepared layouts.
            with tempfile.TemporaryDirectory(prefix="interactive-twin-rank-") as temporary:
                (Path(temporary) / "asset").symlink_to(original, target_is_directory=True)
                found = module.scan(Path(temporary))["ranking"]
                for row in found:
                    row["asset_root"] = str(original)
                rows.extend(found)
        except Exception as exc:
            errors.append({"asset_id": asset["asset_id"], "directory": str(original),
                           "reason": "GEOMETRY_SCANNER_ERROR", "diagnostic": str(exc)})
    return rows, errors


def _handle(row: dict, policy: dict) -> dict:
    dims = [_positive(x) for x in row.get("dimensions_m", [])]
    reasons = []
    if len(dims) != 3 or any(x <= 0 for x in dims):
        reasons.append("INVALID_HANDLE_DIMENSIONS")
        dims = (dims + [0, 0, 0])[:3]
    sections = row.get("sections", [])
    gap = min((_positive(s.get("visual_rear_gap_m")) for s in sections), default=0.0)
    if dims[0] < policy["min_handle_length_m"]:
        reasons.append("HANDLE_SHORTER_THAN_OFFICIAL_PAD")
    if dims[1] >= policy["max_handle_width_m"]:
        reasons.append("HANDLE_EXCEEDS_OFFICIAL_OPENING")
    if dims[0] / max(dims[1], 1e-12) < policy["min_handle_aspect_ratio"]:
        reasons.append("NOT_BAR_SIDE_PINCH_GEOMETRY")
    if gap < policy["min_visual_rear_clearance_m"]:
        reasons.append("INSUFFICIENT_VISUAL_REAR_CLEARANCE")
    if not row.get("axis_root") or not row.get("outward_normal_root") or not sections:
        reasons.append("INCOMPLETE_VISUAL_HANDLE_FRAME")
    return {key: copy.deepcopy(row.get(key)) for key in (
        "joint_name", "joint_type", "moving_link", "handle_link", "mesh", "panel_mesh",
        "axis_root", "outward_normal_root", "sections", "geometry_source") } | {
        "dimensions_m": dims, "minimum_visual_rear_gap_m": gap,
        "geometric_exclusion_reasons": reasons,
        # This score is fixed geometry-only ordering, not an execution metric.
        "selection_score": min(gap, 0.05) * 100 + min(dims[0], 0.15),
    }


def validate_policy(policy: dict) -> dict:
    merged = copy.deepcopy(DEFAULT_POLICY)
    for key, value in policy.items():
        if key not in merged:
            raise ValueError(f"unknown selection policy key: {key}")
        if key in ("budgets", "candidate_grid"):
            if set(value) - set(merged[key]):
                raise ValueError(f"unknown {key} entries")
            merged[key].update(copy.deepcopy(value))
        else:
            merged[key] = copy.deepcopy(value)
    if merged["test_asset_count"] < 4 or merged["episodes_per_asset"] < 3:
        raise ValueError("benchmark requires >=4 unseen assets and >=3 configurations")
    if merged["minimum_joint_margin_rad"] < 0.05:
        raise ValueError("joint margin cannot be lowered")
    budget = merged["budgets"]
    if not 1 <= budget["grasp_candidates"] <= 12 or not 1 <= budget["probe_directions"] <= 4:
        raise ValueError("candidate/probe budget exceeds user limits")
    if budget["sysid_simulations"] <= 0 or budget["wall_clock_seconds"] <= 0:
        raise ValueError("finite positive per-episode budgets required")
    if merged["required_joint_type"] != "revolute":
        raise ValueError("this TEST protocol is revolute; other types need a new protocol")
    if merged["selection_order"] != DEFAULT_POLICY["selection_order"]:
        raise ValueError("unsupported selection order")
    if len(merged["initial_articulation_offsets_deg"]) < merged["episodes_per_asset"]:
        raise ValueError("initial articulation offsets must cover every configuration")
    _canonical(merged)
    return merged


def _episodes(asset: dict, role: str, policy: dict) -> list[dict]:
    rows = []
    joint = next(j for j in asset["joints"] if j["name"] == asset["selected_handle"]["joint_name"])
    lower, upper = joint["limits"]["lower"], joint["limits"]["upper"]
    closed = max(lower, min(0.0, upper)) if lower is not None and upper is not None else 0.0
    available = upper - closed if upper is not None else math.inf
    handle = asset["selected_handle"]
    for index in range(policy["episodes_per_asset"]):
        seed_bytes = hashlib.sha256(f'{policy["seed"]}:{asset["asset_id"]}:{index}'.encode()).digest()
        seed = int.from_bytes(seed_bytes[:8], "big")
        rng = random.Random(seed)
        angle_offset = math.radians(policy["initial_articulation_offsets_deg"][index])
        initial_q = closed + angle_offset
        reason = "INITIAL_STATE_OUTSIDE_SOURCE_LIMIT" if available < angle_offset else None
        bound = min(policy["along_handle_offset_bound_m"], max(0.0, (handle["dimensions_m"][0] - policy["min_handle_length_m"]) / 2))
        translation = [rng.uniform(-limit, limit) if index else 0.0 for limit in policy["relative_translation_bound_m"]]
        rows.append({
            "episode_id": f'{role.lower()}_{asset["asset_id"]}_{index:02d}',
            "asset_id": asset["asset_id"], "split": role, "configuration_index": index,
            "seed": seed, "asset_root": asset["asset_root"],
            "visual_handle": copy.deepcopy(handle),
            "initialization_only": {"joint_name": joint["name"], "joint_position_rad": initial_q,
                                    "reference_and_twin_initialize_once": True},
            "deployment": {
                "rule": policy["placement_rule"], "base_reposition_allowed": False,
                "initial_scene": copy.deepcopy(asset["initial_scene"]),
                "translation_in_initial_handle_frame_m": translation,
                "yaw_perturbation_deg": rng.uniform(-policy["relative_yaw_bound_deg"], policy["relative_yaw_bound_deg"]) if index else 0.0,
                "along_handle_offset_m": (bound if index % 2 else -bound) if index else 0.0,
                "requires_scene_anchor_resolution": not bool(asset["initial_scene"]),
            },
            "grasp_family": policy["grasp_family"], "candidate_grid": copy.deepcopy(policy["candidate_grid"]),
            "budgets": copy.deepcopy(policy["budgets"]),
            "minimum_joint_margin_rad": policy["minimum_joint_margin_rad"],
            "preflight_exclusion_reason": reason,
            "included_in_denominator": role == "TEST", "status": "NOT_RUN",
        })
    return rows


def build_manifest(prepared_roots: Iterable[str | Path], *, ranking_paths: Iterable[str | Path] = (),
                   ranking_rows: Iterable[dict] = (), source_roots: Iterable[str | Path] = (),
                   policy: dict | None = None, frozen_algorithm_files: Iterable[str | Path] = ()) -> dict:
    """Build a deterministic inventory and selection before any TEST execution."""
    policy = validate_policy(policy or {})
    inventory = scan_prepared(prepared_roots)
    ranking = list(ranking_rows)
    ranking_sources = []
    for value in ranking_paths:
        path = Path(value).resolve()
        data = json.loads(path.read_text())
        ranking.extend(data["ranking"] if isinstance(data, dict) else data)
        ranking_sources.append({"path": str(path), "sha256": _sha(path)})
    for asset in inventory:
        if not asset.get("asset_id"):
            continue
        own = [r for r in ranking if str(r.get("asset_id")) == asset["asset_id"] and
               (not r.get("asset_root") or Path(r["asset_root"]).resolve() == Path(asset["asset_root"]).resolve())]
        asset["handle_candidates"] = [_handle(r, policy) for r in asset["handle_candidates"] + own]
        valid_joints = {j["name"] for j in asset["joints"] if j["type"] == policy["required_joint_type"]}
        if not valid_joints:
            asset["exclusion_reasons"].append("NO_REVOLUTE_JOINT")
        valid = [h for h in asset["handle_candidates"] if not h["geometric_exclusion_reasons"] and h["joint_name"] in valid_joints]
        if not valid:
            asset["exclusion_reasons"].append("NO_SUPPORTED_VISUAL_BAR_HANDLE")
        asset["selected_handle"] = max(valid, key=lambda h: (h["selection_score"], str(h["mesh"]))) if valid else None
    # Duplicate prepared variants of one dataset ID cannot become independent TEST objects.
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in inventory:
        if row.get("asset_id"):
            grouped[row["asset_id"]].append(row)
    canonical = []
    for asset_id, variants in sorted(grouped.items()):
        valid = [v for v in variants if not v["exclusion_reasons"]]
        if not valid:
            continue
        chosen = min(valid, key=lambda v: (not v["geometry_is_approximation"], -v["selected_handle"]["selection_score"], v["asset_root"]))
        canonical.append(chosen)
        for variant in variants:
            if variant is not chosen and not variant["exclusion_reasons"]:
                variant["exclusion_reasons"].append("DUPLICATE_ASSET_ID_PREPARATION")
    dev = next((r for r in canonical if r["asset_id"] == str(policy["dev_asset_id"])), None)
    groups: dict[str, list[dict]] = defaultdict(list)
    for row in canonical:
        if row is not dev:
            groups[(row["category"].casefold(), row["object_name"].casefold())].append(row)
    for group in groups.values():
        group.sort(key=lambda r: (-r["selected_handle"]["selection_score"], r["asset_id"]))
    ordered = []
    while any(groups.values()):
        for category in sorted(groups):
            if groups[category]:
                ordered.append(groups[category].pop(0))
    selected = ordered[:policy["test_asset_count"]]
    for row in ordered[policy["test_asset_count"]:]:
        row["exclusion_reasons"].append("NOT_SELECTED_BY_FROZEN_GEOMETRY_ORDER")
    episodes = []
    if dev:
        episodes.extend(_episodes(dev, "DEV", policy))
    for row in selected:
        episodes.extend(_episodes(row, "TEST", policy))
    result = {
        "schema": SCHEMA, "frozen": True, "selection_before_test_outcomes": True,
        "policy": policy, "inventory": inventory, "source_inventory": scan_source(source_roots),
        "ranking_sources": ranking_sources,
        "algorithm_files": [{"path": str(Path(p).resolve()), "sha256": _sha(Path(p))} for p in frozen_algorithm_files],
        "dev_asset_id": str(policy["dev_asset_id"]), "dev_prepared": dev is not None,
        "test_asset_ids": [r["asset_id"] for r in selected],
        "selection_complete": len(selected) == policy["test_asset_count"] and dev is not None,
        "episodes": episodes, "test_denominator_assets": len(selected),
        "test_denominator_episodes": sum(e["included_in_denominator"] for e in episodes),
        "missing_test_assets": max(0, policy["test_asset_count"] - len(selected)),
        "claims": {"grasp_feasibility_certified": False, "geometry_improvement_measured": False,
                   "real_robot_data_present": False, "mode": "SIM_TO_SIM_BLIND_SYSID"},
        "runtime_data_boundary": "controller_episode whitelist; initialization and GT evaluation excluded",
    }
    result["manifest_sha256"] = hashlib.sha256(_canonical(result)).hexdigest()
    return result


def verify_frozen_manifest(manifest: dict | str | Path, *, verify_inputs: bool = False) -> dict:
    """Reject changed selection/configuration; existing experiment logs stay separate."""
    data = json.loads(Path(manifest).read_text()) if not isinstance(manifest, dict) else copy.deepcopy(manifest)
    digest = data.pop("manifest_sha256", None)
    if data.get("schema") != SCHEMA or not data.get("frozen"):
        raise ValueError("not a frozen interactive twin manifest")
    if digest != hashlib.sha256(_canonical(data)).hexdigest():
        raise ValueError("FROZEN_MANIFEST_HASH_MISMATCH")
    data["manifest_sha256"] = digest
    if verify_inputs:
        records = [(r["path"], r["sha256"]) for r in data["algorithm_files"] + data["ranking_sources"]]
        for row in data["inventory"]:
            if row.get("manifest_sha256"):
                records.append((row["manifest"], row["manifest_sha256"]))
            if row.get("urdf_sha256"):
                records.append((row["urdf"], row["urdf_sha256"]))
            records.extend((g["path"], g["sha256"]) for g in row.get("geometry_inputs", []))
        for path, expected in records:
            if not Path(path).is_file() or _sha(Path(path)) != expected:
                raise ValueError(f"FROZEN_INPUT_CHANGED: {path}")
    return data


def save_frozen_manifest(manifest: dict, path: str | Path) -> Path:
    path = Path(path)
    verify_frozen_manifest(manifest)
    if path.exists():
        previous = verify_frozen_manifest(path)
        if previous["manifest_sha256"] != manifest["manifest_sha256"]:
            raise FileExistsError("refusing to overwrite a different frozen benchmark; use a new version/path")
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    return path


def controller_episode(manifest: dict, episode_id: str) -> dict:
    """Allowed pre-run visual/deployment metadata, never GT articulation fields.

    The returned path is still an environment resource; the controller must not
    open the asset URDF. Physics initialization is performed by a separate host.
    """
    data = verify_frozen_manifest(manifest)
    episode = next(e for e in data["episodes"] if e["episode_id"] == episode_id)
    handle = episode["visual_handle"]
    return {key: copy.deepcopy(episode[key]) for key in (
        "episode_id", "asset_id", "seed", "grasp_family", "candidate_grid", "budgets", "minimum_joint_margin_rad") } | {
        "deployment": {key: copy.deepcopy(episode["deployment"][key]) for key in (
            "rule", "base_reposition_allowed", "translation_in_initial_handle_frame_m",
            "yaw_perturbation_deg", "along_handle_offset_m", "requires_scene_anchor_resolution")},
        "visual_handle": {key: copy.deepcopy(handle[key]) for key in (
            "mesh", "axis_root", "outward_normal_root", "sections", "dimensions_m", "minimum_visual_rear_gap_m")},
        "manifest_sha256": data["manifest_sha256"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepared", type=Path, action="append", required=True)
    parser.add_argument("--ranking", type=Path, action="append", default=[])
    parser.add_argument("--source", type=Path, action="append", default=[])
    parser.add_argument("--policy", type=Path)
    parser.add_argument("--scan-geometry", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rankings, errors = geometry_rankings(args.prepared) if args.scan_geometry else ([], [])
    policy = json.loads(args.policy.read_text()) if args.policy else None
    result = build_manifest(args.prepared, ranking_paths=args.ranking, ranking_rows=rankings,
                            source_roots=args.source, policy=policy, frozen_algorithm_files=[__file__])
    save_frozen_manifest(result, args.output)
    print(json.dumps({"manifest": str(args.output), "sha256": result["manifest_sha256"],
                      "test_asset_ids": result["test_asset_ids"], "test_episodes": result["test_denominator_episodes"],
                      "selection_complete": result["selection_complete"], "geometry_scan_errors": errors}, indent=2))
    return 0 if result["selection_complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
