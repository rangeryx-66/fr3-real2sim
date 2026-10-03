"""Prepare dataset revolute assets at their metadata scale, never grasp-fit scale.

Runs the existing preparation implementation into a separate experiment output.
The source dimension prior (centimetres, third component) sets asset height. No
missing dimension is guessed; all skipped/failed assets remain in inventory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def metadata_height(meta: dict) -> float:
    value = meta.get("dimension")
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError:
            value = value.strip("[]() ").replace(",", " ").replace("*", " ").replace("×", " ").split()
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("MISSING_SOURCE_DIMENSION_CM_PRIOR")
    dimensions = [float(x) for x in value]
    if any(not math.isfinite(x) or x <= 0 for x in dimensions):
        raise ValueError("INVALID_SOURCE_DIMENSION_CM_PRIOR")
    return dimensions[2] / 100.0


def _category(meta: dict) -> str:
    return str(meta.get("category") or meta.get("object_name") or meta.get("model_cat") or meta.get("name") or "UNKNOWN")


def _mass_by_link(root: ET.Element) -> dict:
    return {link.get("name"): float(link.find("inertial/mass").get("value"))
            for link in root.findall("link") if link.find("inertial/mass") is not None}


def _joint_signature(root: ET.Element) -> list:
    # Origin translation is scaled by the documented source metric conversion.
    return [{"name": j.get("name"), "type": j.get("type"),
             "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
             "axis": j.find("axis").attrib if j.find("axis") is not None else None,
             "limit": j.find("limit").attrib if j.find("limit") is not None else None,
             "dynamics": j.find("dynamics").attrib if j.find("dynamics") is not None else None}
            for j in root.findall("joint")]


def prepare_assets(source: Path, output: Path, *, timeout_s: float = 180.0) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    assets = output / "assets"
    assets.mkdir(exist_ok=True)
    rows = []
    for urdf in sorted((source / "urdf").glob("*.urdf")):
        asset_id = urdf.stem
        destination = assets / asset_id
        row = {"asset_id": asset_id, "source_urdf": str(urdf.resolve()),
               "source_urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
               "output_asset_root": str(destination.resolve()), "status": "NOT_PREPARED", "exclusion_reasons": []}
        rows.append(row)
        try:
            root = ET.parse(urdf).getroot()
            moving = [j for j in root.findall("joint") if j.get("type") != "fixed"]
            row["joints"] = [{"name": j.get("name"), "type": j.get("type")} for j in moving]
            if len(moving) != 1 or moving[0].get("type") != "revolute":
                raise ValueError("PREPARER_REQUIRES_SINGLE_REVOLUTE_JOINT")
            meta_file = source / "finaljson" / f"{asset_id}.json"
            if not meta_file.exists():
                raise ValueError("SOURCE_METADATA_MISSING")
            meta = json.loads(meta_file.read_text())
            height_m = metadata_height(meta)
            row.update(category=_category(meta), metadata_dimension_cm=meta["dimension"],
                       height_m=height_m, source_metadata_sha256=hashlib.sha256(meta_file.read_bytes()).hexdigest())
            manifest_file = destination / "manifest.json"
            if manifest_file.exists():
                existing = json.loads(manifest_file.read_text())
                if (existing.get("source_urdf_sha256") != row["source_urdf_sha256"] or
                        existing.get("benchmark_preparation", {}).get("source_metadata_sha256") != row["source_metadata_sha256"]):
                    raise ValueError("EXISTING_PREPARATION_INPUT_MISMATCH_USE_NEW_OUTPUT")
                row["status"] = "REUSED_VERIFIED_PREPARATION"
                continue
            if destination.exists() and any(destination.iterdir()):
                raise ValueError("PARTIAL_PREPARATION_EXISTS_USE_NEW_OUTPUT_OR_INSPECT")
            command = [sys.executable, str(ROOT / "scripts/prepare_physx_microwave.py"),
                       "--source", str(source), "--asset-id", asset_id,
                       "--height-m", str(height_m), "--output", str(destination)]
            row["command"] = command
            log = output / f"prepare_{asset_id}.log"
            with log.open("w") as stream:
                result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                        timeout=timeout_s, check=False)
            row["log"] = str(log.resolve())
            if result.returncode:
                raise ValueError(f"PREPARATION_EXIT_{result.returncode}")
            prepared_urdf = destination / "urdf" / urdf.name
            prepared = ET.parse(prepared_urdf).getroot()
            if _mass_by_link(prepared) != _mass_by_link(root):
                raise ValueError("SOURCE_MASS_CHANGED")
            if _joint_signature(prepared) != _joint_signature(root):
                raise ValueError("SOURCE_JOINT_DEFINITION_CHANGED")
            metadata = json.loads(manifest_file.read_text())
            child = moving[0].find("child").get("link")
            metadata.update(object_name=str(meta.get("object_name") or _category(meta)), category=_category(meta), door_link=child,
                            prepared_urdf=str(prepared_urdf.resolve()))
            metadata["benchmark_preparation"] = {
                "method": "existing_prepare_physx_microwave_generic_single_revolute_import",
                "scale_policy": "source finaljson.dimension[2] in centimetres / 100; never grasp-fitted",
                "metadata_dimension_cm": meta["dimension"], "source_metadata_sha256": row["source_metadata_sha256"],
                "source_mass_verified_unchanged": True, "source_joint_verified_unchanged": True,
                "geometry_note": "visuals use source units prior; semantic interaction proxy prepared separately",
                "inertia_note": "existing importer box-inertia approximation retained and explicitly not GT",
                "category_source": "source finaljson metadata", "no_execution_outcomes_used": True,
            }
            manifest_file.write_text(json.dumps(metadata, indent=2) + "\n")
            row["status"] = "PREPARED"
        except subprocess.TimeoutExpired:
            row.update(status="EXCLUDED", exclusion_reasons=["PREPARATION_TIMEOUT"])
        except Exception as exc:
            row.update(status="EXCLUDED", exclusion_reasons=[str(exc)], error_type=type(exc).__name__)
        finally:
            # Incremental inventory survives interruptions and records every failure.
            (output / "preparation_inventory.json").write_text(json.dumps({
                "schema": "interactive-twin-preparation-v1", "source_root": str(source.resolve()),
                "output_root": str(output.resolve()), "scale_policy_frozen_before_execution": True,
                "assets": rows}, indent=2) + "\n")
    return {"assets": rows, "prepared_root": str(assets.resolve())}


def refresh_source_metadata(source: Path, prepared: Path, report_path: Path) -> dict:
    """Fix descriptive category/name fields only, before benchmark freeze.

    Source metadata bytes, URDFs, meshes, dynamics, scales and preparation fields
    remain unchanged. The caller must not use this to mutate a frozen dataset.
    """
    rows = []
    for manifest_file in sorted(Path(prepared).rglob("manifest.json")):
        metadata = json.loads(manifest_file.read_text())
        asset_id = metadata.get("asset_id")
        if asset_id is None:
            continue
        source_file = Path(source) / "finaljson" / f"{asset_id}.json"
        row = {"asset_id": str(asset_id), "manifest": str(manifest_file), "status": "NOT_CHANGED"}
        rows.append(row)
        if not source_file.is_file():
            row.update(status="SOURCE_METADATA_MISSING")
            continue
        original = json.loads(source_file.read_text())
        category = _category(original)
        object_name = str(original.get("object_name") or category)
        row["before"] = {key: metadata.get(key) for key in ("category", "object_name")}
        row["after"] = {"category": category, "object_name": object_name}
        if row["before"] == row["after"]:
            row["status"] = "ALREADY_CORRECT"
            continue
        preserved = {k: v for k, v in metadata.items() if k not in ("category", "object_name")}
        metadata.update(category=category, object_name=object_name)
        assert preserved == {k: v for k, v in metadata.items() if k not in ("category", "object_name")}
        manifest_file.write_text(json.dumps(metadata, indent=2) + "\n")
        row["status"] = "DESCRIPTIVE_METADATA_REFRESHED"
    result = {"schema": "interactive-twin-metadata-refresh-v1", "source_root": str(source),
              "prepared_root": str(prepared), "assets": rows,
              "changed_fields_only": ["category", "object_name"],
              "geometry_urdf_physics_files_touched": False, "must_run_before_manifest_freeze": True}
    report_path = Path(report_path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(result, indent=2) + "\n")
    return result

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=180)
    parser.add_argument("--rank", action="store_true")
    parser.add_argument("--refresh-metadata-only", type=Path, metavar="PREPARED_ROOT")
    args = parser.parse_args()
    if args.refresh_metadata_only:
        result = refresh_source_metadata(args.source, args.refresh_metadata_only, args.output / "metadata_refresh.json")
        print(json.dumps({"assets": len(result["assets"]), "updated": sum(r["status"] == "DESCRIPTIVE_METADATA_REFRESHED" for r in result["assets"]), "report": str(args.output / "metadata_refresh.json")}, indent=2))
        return 0
    result = prepare_assets(args.source, args.output, timeout_s=args.timeout_s)
    if args.rank:
        sys.path.insert(0, str(ROOT))
        from interactive_twin.manifest import geometry_rankings
        rows, errors = geometry_rankings([Path(result["prepared_root"])])
        ranking_file = args.output / "ranking.json"
        ranking_file.write_text(json.dumps({"ranking": rows, "errors": errors,
                                           "geometry_only": True, "acceptance": "physical contact baseline, raw intersections diagnostic only"}, indent=2) + "\n")
        result["ranking"] = str(ranking_file.resolve())
    print(json.dumps({"prepared_root": result["prepared_root"],
                      "prepared": sum(r["status"] in ("PREPARED", "REUSED_VERIFIED_PREPARATION") for r in result["assets"]),
                      "excluded": sum(r["status"] == "EXCLUDED" for r in result["assets"]),
                      "ranking": result.get("ranking")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
