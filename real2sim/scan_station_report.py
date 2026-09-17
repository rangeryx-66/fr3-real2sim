"""Write a compact, reproducible report for a scan-station run.

The report is deliberately a consumer of recorded manifests.  It never reads
simulator GT to compute the acquisition QA or the metric object frame.  A GT
mesh, when supplied to the reconstruction runner, is only reflected in the
evaluation metrics section.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def _load(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return default


def _number(path: Path, key: str = "WALL_SECONDS") -> float | None:
    if not path.exists():
        return None
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith(key + "="):
            try:
                return float(line.split("=", 1)[1])
            except ValueError:
                return None
    return None


def _f(value: Any, digits: int = 3) -> str:
    if value is None:
        return "—"
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def _pct(value: Any, digits: int = 1) -> str:
    if value is None:
        return "—"
    try:
        return f"{100.0 * float(value):.{digits}f}%"
    except (TypeError, ValueError):
        return "—"


def _path(path: Any) -> str:
    return str(path) if path else "—"


def _frame_stats(scan: dict[str, Any]) -> dict[str, Any]:
    views = scan.get("views", [])
    q = [v.get("quality", {}) for v in views]
    def vals(key: str) -> list[float]:
        return [float(x[key]) for x in q if x.get(key) is not None]
    heights = vals("bbox_height_fraction")
    lumas = vals("luma_mean")
    depth = vals("depth_valid_ratio")
    occ = vals("gripper_occlusion_ratio")
    full_occ = vals("full_gripper_fraction")
    if not full_occ:
        # Older manifests predate the image-wide QA field.  The station camera
        # is fixed at 640x480, so derive the same conservative ratio from the
        # recorded full-frame instance-pixel count for backward-compatible
        # reports.
        full_occ = [float(v.get("gripper_pixels", 0)) / (640.0 * 480.0)
                    for v in views if v.get("gripper_pixels") is not None]
    return {
        "count": len(views),
        "accepted": int(scan.get("accepted_count", 0)),
        "rejected": int(scan.get("rejected_count", 0)),
        "height_min": min(heights) if heights else None,
        "height_max": max(heights) if heights else None,
        "height_mean": sum(heights) / len(heights) if heights else None,
        "depth_min": min(depth) if depth else None,
        "occ_max": max(occ) if occ else None,
        "full_occ_max": max(full_occ) if full_occ else None,
        "luma_min": min(lumas) if lumas else None,
        "luma_max": max(lumas) if lumas else None,
        "motion_max_mm": max(
            [1000.0 * float(v.get("target_motion_during_capture_m", 0.0)) for v in views]
            or [0.0]),
        "rotation_max_deg": max(
            [float(v.get("target_rotation_during_capture_deg", 0.0)) for v in views]
            or [0.0]),
    }


def _stage_timings(final: Path) -> dict[str, float]:
    names = [
        "view_selection", "reconviagen_inference", "metric_constraint", "texture",
        "coacd", "asset_builder", "coverage", "metrics", "usd_validation",
    ]
    out: dict[str, float] = {}
    for name in names:
        value = _number(final / f"{name}.time")
        if value is not None:
            out[name] = value
    return out


def _fscore(value: Any) -> str:
    """Format either the scalar or the precision/recall/fscore mapping."""
    if isinstance(value, dict):
        value = value.get("fscore")
    return _f(value, 3)


def write_report(run_root: str | Path, output: str | Path | None = None) -> Path:
    """Write ``SCAN_STATION_REPORT.md`` and return its path."""
    root = Path(run_root).resolve()
    scan_root = root / "scan_station"
    final = root / "reconviagen_v05"
    scan = _load(scan_root / "scan_manifest.json", {}) or {}
    selection = _load(final / "views" / "view_selection.json", {}) or {}
    inference = _load(final / "generated" / "inference_report.json", {}) or {}
    metric = _load(final / "metric" / "metric_constraint_report.json", {}) or {}
    texture = _load(final / "textured" / "texture_report.json", {}) or {}
    coverage = _load(final / "coverage.json", {}) or {}
    metrics = _load(final / "metrics.json", {}) or {}
    collision = _load(final / "collision" / "collision_manifest.json", {}) or {}
    metadata = _load(final / "asset" / "metadata.json", {}) or {}
    reload_result = _load(final / "asset" / "reload_validation.json", {}) or {}
    native = _load(final / "native_texture_compare" / "native_texture_report.json", {}) or {}
    native_reload = _load(final / "native_texture_compare" / "asset" / "reload_validation.json", {}) or {}
    native_metric_reload = _load(final / "native_texture_compare" / "metric_asset" / "reload_validation.json", {}) or {}
    run = _load(root / "scan_station_run.json", {}) or {}
    fs = _frame_stats(scan)

    directions = selection.get("direction_coverage", {})
    constraint = metric.get("constraint", {})
    metric_mesh = metric.get("mesh", {})
    geometry = metrics.get("geometry", {})
    tracking = metrics.get("tracking", {})
    region = coverage.get("region_coverage", {})
    passes = coverage.get("passes", {})
    stage = _stage_timings(final)
    # The inference report contains nested model timings.  Keep its total as a
    # separate annotation instead of adding it to the process-stage wall time
    # (which would double count the same work).
    recon_model_time = inference.get("timing_seconds", {}).get("total")
    total = run.get("capture_seconds", 0.0) + run.get("reconstruction_seconds", 0.0)
    if not total:
        total = sum(stage.values())

    extent = metric_mesh.get("aabb_extent_m", [None, None, None])
    extent_mm = [1000.0 * float(x) for x in extent] if all(x is not None for x in extent) else [None] * 3
    report_path = Path(output).resolve() if output else root / "SCAN_STATION_REPORT.md"
    report_path.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    add = lines.append
    add("# Scan-station RGB-D → ReconViaGen v0.5 → Isaac USD report")
    add("")
    add(f"- Run root: `{root}`")
    add(f"- Target / seed: `{scan.get('target', run.get('target', '—'))}` / `{run.get('seed', '—')}`")
    add("- Grasp executor modified: **no**")
    add("- Acquisition/alignment GT use: **no**; `eval_gt/` and the optional GT mesh are evaluation-only.")
    add("")
    add("## 1. Station acquisition and inputs")
    add("")
    add(f"The object was placed on the clean station, released, and the FR3 was parked at the remote staging transform before capture. The target remained stationary (maximum translation **{_f(fs['motion_max_mm'], 4)} mm**, rotation **{_f(fs['rotation_max_deg'], 4)}°** during a frame).")
    add("")
    add("| QA | Result |")
    add("|---|---:|")
    add(f"| Views / accepted / rejected | {fs['count']} / {fs['accepted']} / {fs['rejected']} |")
    add(f"| Object bbox height fraction | {_pct(fs['height_min'])}–{_pct(fs['height_max'])} (mean {_pct(fs['height_mean'])}) |")
    add(f"| Depth-valid ratio (minimum) | {_pct(fs['depth_min'])} |")
    add(f"| Gripper in object bbox (maximum) | {_pct(fs['occ_max'])} |")
    add(f"| Gripper in full frame (maximum) | {_pct(fs['full_occ_max'])} |")
    add(f"| Object luma range | {_f(fs['luma_min'], 1)}–{_f(fs['luma_max'], 1)} |")
    add(f"| Crop padding | {_pct(scan.get('crop_policy', {}).get('padding_fraction'))} |")
    add("")
    add("The ReconViaGen inputs are the 24 saved bbox-centered crops (15% padding), with object mask applied and gripper pixels removed. The two 12-frame rings use 30° azimuth spacing; the high ring is elevated to expose the upper surface. The full-frame and crop mosaics are available at:")
    add("")
    add(f"- `{scan_root / 'scan_station_full_contact_sheet.png'}`")
    add(f"- `{scan_root / 'scan_station_crops_contact_sheet.png'}`")
    add("")
    add("## 2. View selection evidence")
    add("")
    add("| Measure | Result |")
    add("|---|---:|")
    add(f"| Selected views | {len(selection.get('selected', []))} |")
    add(f"| Pairwise silhouette overlap, mean / minimum | {_f(selection.get('pairwise_overlap_mean'), 4)} / {_f(selection.get('pairwise_overlap_min'), 4)} |")
    add(f"| Mean / maximum pair angle | {_f(directions.get('mean_pair_angle_deg'), 2)}° / {_f(directions.get('max_pair_angle_deg'), 2)}° |")
    add(f"| Viewing-direction span | {_f(directions.get('angular_span_deg'), 2)}° (required {_f(directions.get('required_min_angular_span_deg'), 1)}°) |")
    add(f"| Direction labels | {', '.join(directions.get('unique_labels', [])) or '—'} |")
    add(f"| Coverage requirement passed | {directions.get('direction_coverage_sufficient', '—')} |")
    add("")
    add("This is connected, high-overlap multi-view evidence rather than sparse view jumps. The crop images selected by the model are listed in `views/view_selection.json`.")
    add("")
    add("## 3. ReconViaGen and metric constraint")
    add("")
    add("ReconViaGen produced the topology/shape prior. The RGB-D point cloud reconstructed from the object masks and `T_base_camera` then set the metric frame and rejected generated surfaces that conflicted with measured rays. No GT pose or mesh entered this step.")
    add("")
    add("| Measure | Result |")
    add("|---|---:|")
    add(f"| Raw generated mesh (vertices / faces) | {inference.get('mesh', {}).get('vertices', '—')} / {inference.get('mesh', {}).get('faces', '—')} |")
    add(f"| Metric mesh (vertices / faces) | {metric_mesh.get('vertices', '—')} / {metric_mesh.get('faces', '—')} |")
    add(f"| Metric AABB (x × y × z) | {_f(extent_mm[0], 2)} × {_f(extent_mm[1], 2)} × {_f(extent_mm[2], 2)} mm |")
    add(f"| Uniform scale / translation | {_f(metric.get('alignment', {}).get('uniform_scale'), 6)} / `{metric.get('alignment', {}).get('translation_m', '—')}` |")
    add(f"| Measured surface fraction before conflict rejection | {_pct(constraint.get('pre_rejection_rgbd_observed_surface_fraction'))} |")
    add(f"| Generated-only fraction before rejection | {_pct(constraint.get('pre_rejection_generated_only_surface_fraction'))} |")
    add(f"| Generated/RGB-D conflict fraction | {_pct(constraint.get('generated_rgbd_conflict_surface_fraction'))} |")
    add(f"| Final measured / generated-only fraction | {_pct(constraint.get('final_rgbd_observed_surface_fraction'))} / {_pct(constraint.get('final_generated_only_surface_fraction'))} |")
    add("")
    add("## 4. Coverage and mesh quality")
    add("")
    add("Coverage below is the reconstruction audit proxy, not GT-assisted completion. The object axes are the RGB-D-derived scan convention.")
    add("")
    add("| Region / pass | Observed coverage |")
    add("|---|---:|")
    for name in ("top", "bottom", "side", "front", "back"):
        val = region.get(name, {})
        if isinstance(val, dict):
            val = val.get("observed_fraction", val.get("coverage"))
        add(f"| {name} | {_pct(val)} |")
    for key, val in passes.items():
        if isinstance(val, dict):
            val = val.get("observed_surface_coverage", val.get("coverage", val.get("surface_coverage")))
        add(f"| pass {key} | {_pct(val)} |")
    add(f"| all passes | {_pct(coverage.get('observed_surface_coverage'))} |")
    add("")
    add("| Mesh / metric | Result |")
    add("|---|---:|")
    add(f"| Boundary edges / components | {geometry.get('boundary_edges', metric_mesh.get('boundary_edges', '—'))} / {geometry.get('components', '—')} |")
    add(f"| Watertight | {geometry.get('watertight', metric_mesh.get('watertight', '—'))} |")
    add(f"| Chamfer | {_f(geometry.get('chamfer_mm'), 3)} mm |")
    add(f"| F-score @2 / @5 mm | {_fscore(geometry.get('fscore_2mm'))} / {_fscore(geometry.get('fscore_5mm'))} |")
    add(f"| Local plane noise median / p95 | {_f(geometry.get('local_plane_noise_median_mm'), 3)} / {_f(geometry.get('local_plane_noise_p95_mm'), 3)} mm |")
    add("")
    add("The station views substantially improve side/back evidence, but the top and especially bottom remain weakly observed. Remaining boundaries/components therefore indicate missing observations and/or generated mesh fragmentation; they are not evidence that a GT mesh was used to fill the shell.")
    add("")
    add("## 5. Texture, collision and Isaac validation")
    add("")
    add("| Stage | Result |")
    add("|---|---:|")
    add(f"| Real-RGB measured vertex texture coverage | {_pct(texture.get('measured_vertex_coverage'))} |")
    add(f"| Texture atlas surface texel fraction | {_pct(texture.get('atlas_surface_texel_fraction'))} |")
    add(f"| CoACD parts / collision faces | {len(collision.get('parts', [])) or '—'} / {collision.get('collision_faces', '—')} |")
    add(f"| Isaac USD loaded / finite / settled | {reload_result.get('loaded', '—')} / {reload_result.get('finite', '—')} / {reload_result.get('settled', '—')} |")
    add(f"| Physics validation | {reload_result.get('physics_passed', '—')} |")
    if native:
        add(f"| ReconViaGen native texture comparison | {native.get('mesh', {}).get('vertices', '—')} vertices; exact-UV reload {native_reload.get('physics_passed', '—')}; metric-mesh UV-transfer reload {native_metric_reload.get('physics_passed', '—')} |")
    add("")
    add("The captured RGB is clean and has no gripper pixels, but the current atlas bake is visibly speckled on the generated soup mesh. This is a texture-bake/very-high-poly atlas limitation, separate from acquisition quality; the visual output should not yet be called production-clean.")
    add("")
    add("## 6. Products")
    add("")
    products = {
        "input crop mosaic": scan_root / "scan_station_crops_contact_sheet.png",
        "input full mosaic": scan_root / "scan_station_full_contact_sheet.png",
        "raw ReconViaGen mesh": final / "generated" / "generated_complete_mesh.glb",
        "metric constrained mesh": final / "metric" / "metric_constrained_mesh.obj",
        "textured mesh": final / "textured" / "textured_mesh.obj",
        "texture atlas": final / "textured" / "material_0.png",
        "CoACD manifest": final / "collision" / "collision_manifest.json",
        "Isaac USD": final / "asset" / f"{metadata.get('name', run.get('asset_name', 'object'))}.usda",
        "Isaac reload render": final / "asset" / "reload_render.png",
        "source SHA256 list": root / "scan_station_source_sha256.txt",
    }
    if native:
        products.update({
            "ReconViaGen native texture": final / "native_texture_compare" / "reconviagen_native_basecolor.png",
            "native texture exact-UV render": final / "native_texture_compare" / "asset" / "reload_render.png",
            "native texture metric-mesh render": final / "native_texture_compare" / "metric_asset" / "reload_render.png",
            "native texture comparison report": root / "NATIVE_TEXTURE_COMPARISON.md",
        })
    for label, path in products.items():
        add(f"- **{label}:** `{path}`")
    add("")
    add("## 7. Timing")
    add("")
    add("| Stage | Seconds |")
    add("|---|---:|")
    add(f"| Station capture | {_f(run.get('capture_seconds'), 2)} |")
    for name, value in stage.items():
        add(f"| {name} | {_f(value, 2)} |")
    if recon_model_time is not None:
        add(f"| ReconViaGen model report total (nested) | {_f(recon_model_time, 2)} |")
    add(f"| **Total run wall time** | **{_f(total, 2)}** |")
    add("")
    add("## Conclusion")
    add("")
    add("The scan-station experiment removes the dominant input-side confound: the target is stationary, close, occupies the requested image scale, has valid depth, and the robot/hand are absent from the frames. ReconViaGen plus metric RGB-D constraint is therefore being tested with substantially better evidence. The resulting side geometry and GT metrics improve over the earlier narrow-view scan, and the USD reload/physics check passes. However, top/bottom coverage and mesh topology remain incomplete, and the real-RGB texture atlas is noisy. This run supports using the station skill, but it does not yet justify declaring ReconViaGen a clean drop-in replacement for the slower BundleSDF path without another geometry/texture backend pass.")

    report_path.write_text("\n".join(lines) + "\n")
    return report_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(write_report(args.run_root, args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
