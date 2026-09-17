"""Build the three-way scan experiment report from copied run manifests.

This is a read-only report generator.  It never runs reconstruction, changes a
scan, or reads GT data for anything other than the evaluation fields already
written by the pipeline.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


def load(path: Path):
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def fmt(v, digits=3):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def mv_rows(root: Path):
    rows = []
    # The local copy keeps static outputs under static_outputs and held outputs
    # under held_preflight_outputs; final held copies may be added later.
    for group, base in (("static", root / "static_outputs"),
                        ("held", root / "held_outputs"),
                        ("held_preflight", root / "held_preflight_outputs")):
        if not base.exists():
            continue
        for glb in sorted(base.rglob("*.glb")):
            m = re.search(r"views_(\d+)", str(glb)) or re.search(r"result_(\d+)v", glb.name)
            n = int(m.group(1)) if m else None
            rows.append(dict(group=group, views=n, path=str(glb), size=glb.stat().st_size))
    return rows


def mv_timings(root: Path):
    rows = []
    for log in sorted(root.glob("mv_*static*.log")) + sorted(root.glob("mv_*held*.log")):
        text = log.read_text(errors="replace")
        out = re.search(r"Output directory: .*?(views_(\d+)_soup_[^\s]+)", text)
        finish = re.findall(r"run_multi_view:.*?Finished!", text)
        coords = re.findall(r"Generated coordinates:\s*(\d+)", text)
        stamps = re.findall(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+).*?(?:Output directory:|Finished!)", text, re.M)
        elapsed = None
        if len(stamps) >= 2:
            from datetime import datetime
            try:
                elapsed = (datetime.fromisoformat(stamps[-1]) - datetime.fromisoformat(stamps[0])).total_seconds()
            except ValueError:
                pass
        rows.append(dict(log=log.name, views=int(out.group(2)) if out else None,
                         output=out.group(1) if out else None,
                         finished=bool(finish), coordinates=int(coords[-1]) if coords else None,
                         elapsed_seconds=elapsed))
    return rows


def render_report(root: Path, out: Path):
    official = load(root / "real2sim_official_soup_1800_v2/official_pipeline/scalable_real2sim_pipeline_manifest.json") or {}
    stages = official.get("stages", {})
    cov = stages.get("coverage_audit", {})
    bundle = stages.get("bundlesdf", {})
    gt = stages.get("gt_evaluation", {})
    fresh = stages.get("fresh_isaac", {})
    held_dirs = sorted(root.glob("real2sim_soup_held_regrasp_v*/scan/held_regrasp.json"))
    held = load(held_dirs[-1]) if held_dirs else None
    held_root = held_dirs[-1].parents[1] if held_dirs else None
    held_runs = sorted(root.glob("real2sim_soup_held_regrasp_v*"))
    latest_held_run = held_runs[-1] if held_runs else held_root
    held_official = None
    if held_root:
        p = root / ("real2sim_held_official_soup_1800_" + held_root.name) / "official_pipeline/scalable_real2sim_pipeline_manifest.json"
        held_official = load(p)
    mvroot = root / "mv_sam3d_soup_v1"
    lines = []
    add = lines.append
    add("# Soup scan reconstruction experiments")
    add("")
    add("Generated from the frozen FR3 handoff and the recorded remote artifacts. "
        "The grasp executor, AnyGrasp model/weights/inference and grasp parameters were not changed.")
    add("")
    add("## Status")
    add("")
    add(f"- Stationary official run: **complete**, 1800 frames, two camera rings, no physical regrasp.")
    if held:
        result = held.get("result", "unknown")
        if result == "REGRASP_FAILED" and latest_held_run and latest_held_run.name != held_root.name and not (latest_held_run / "scan/scan_manifest.json").exists():
            add(f"- Held scan: {held_root.name} is archived as **REGRASP_FAILED** at {held.get('first_pass_frames','—')} partial frames; {latest_held_run.name} is running with the scan-only settled-reorientation guard.")
        else:
            add(f"- Held scan: **{result}**, first={held.get('first_pass_frames','—')}, second={held.get('second_pass_frames','—')}, physical_regrasp={held.get('physical_regrasp')}." )
    else:
        add("- Held scan: no completed held_regrasp.json has been copied yet; partial runs remain archived.")
    add("- MV-SAM3D: static 2/4/8-view runs complete; held preflight 2/4/8 runs are retained; full held-v8 runs will be listed after the 1800-frame scan handoff.")
    add("")
    add("## Three result sets")
    add("")
    add("| set | input / motion | reconstruction | result |")
    add("|---|---|---|---|")
    add("| stationary | clean station, 900 + 900 side rings, no regrasp | official Scalable Real2Sim BundleSDF (tracking + global refinement + texture) | metric geometry is valid but top/bottom are unobserved |")
    if held:
        add("| held | 900 frames, physical lower/release/regrasp, 900 frames | official BundleSDF after scan handoff | two physically different occlusion configurations |")
    else:
        add("| held | latest retry active: 900 + physical regrasp + 900 | queued official BundleSDF | pending completion |")
    add("| MV-SAM3D | stationary 2/4/8 selected views | official MV-SAM3D `run_inference_weighted.py`, `gaussian,mesh` | static outputs complete; held preflight retained |")
    add("")
    add("## Stationary official BundleSDF")
    add("")
    add(f"- accepted frames: {cov.get('accepted_frames','—')}; BundleSDF elapsed: {fmt(bundle.get('elapsed_seconds'),1)} s; total pipeline: {fmt(stages.get('timing',{}).get('total_seconds'),1)} s.")
    if cov.get("pass_stats"):
        for p in cov["pass_stats"]:
            add(f"- pass {p.get('pass_id')}: {p.get('frames')} frames, new depth-supported area={fmt(p.get('new_depth_supported_area_fraction'),3)}, cumulative={fmt(p.get('cumulative_depth_supported_area_fraction'),3)}, new faces={p.get('new_faces')}.")
    topo = cov.get("mesh_topology", {}) or {}
    tex = cov.get("texture", {}) or {}
    add(f"- mesh extents (canonical): {fmt(topo.get('extents_m'))}; vertices={topo.get('vertices','—')}, faces={topo.get('faces','—')}, watertight={topo.get('watertight','—')}; boundary edges={topo.get('boundary_edges','—')}.")
    add(f"- texture coverage proxy: nonzero pixels={fmt(tex.get('nonzero_pixel_fraction'),3)}, UV vertex fraction={fmt(tex.get('uv_vertex_fraction'),3)}; source={tex.get('path','—')}.")
    add(f"- GT-only evaluation: Chamfer={fmt((gt.get('scan',{}) or {}).get('chamfer_m'),4)} m, F@2mm={fmt((gt.get('scan',{}) or {}).get('fscore_2mm',{}).get('fscore'),3)}, F@5mm={fmt((gt.get('scan',{}) or {}).get('fscore_5mm',{}).get('fscore'),3)}.")
    collision_parts = len((stages.get("asset", {}) or {}).get("collision_parts", []) or [])
    add(f"- fresh Isaac reload: loaded={fresh.get('loaded')}, physics_passed={fresh.get('physics_passed')}, inertia_readback={fresh.get('inertia_readback')}, collision prims={fresh.get('collision_prim_count', collision_parts or '—')}.")
    add("")
    add("The tracking-based direction audit for the stationary capture contains side directions only (no top/bottom elevation bands). This explains the open shell and is the reason for the held scan; it is not repaired with GT geometry.")
    add("")
    add("## Held-object scan")
    add("")
    if held:
        attempts = held.get("regrasp_attempts") or []
        successful = [a for a in attempts if (a.get("stability") or {}).get("passed")]
        add(f"- result: `{held.get('result')}`; first pass={held.get('first_pass_frames')}, second pass={held.get('second_pass_frames')}; physical regrasp={held.get('physical_regrasp')}; successful regrasp attempts={len(successful)}/{len(attempts)}.")
        if attempts:
            a = attempts[0]
            add(f"- selected regrasp: angle={fmt(a.get('angle_deg'),1)}°, MoveIt category={(a.get('moveit') or {}).get('category','—')}, planning={fmt((a.get('moveit') or {}).get('planning_seconds'),2)} s, stability={(a.get('stability') or {}).get('category','—')}.")
        add(f"- held scan artifact: `{held_root}`.")
        if held_official:
            hs = held_official.get("stages", {})
            hc = hs.get("coverage_audit", {})
            hb = hs.get("bundlesdf", {})
            hgt = hs.get("gt_evaluation", {})
            hf = hs.get("fresh_isaac", {})
            add(f"- held official accepted frames={hc.get('accepted_frames','—')}; BundleSDF elapsed={fmt(hb.get('elapsed_seconds'),1)} s; total={fmt(hs.get('timing',{}).get('total_seconds'),1)} s.")
            add(f"- held GT-only metrics: Chamfer={fmt((hgt.get('scan',{}) or {}).get('chamfer_m'),4)} m, F@2mm={fmt((hgt.get('scan',{}) or {}).get('fscore_2mm',{}).get('fscore'),3)}, F@5mm={fmt((hgt.get('scan',{}) or {}).get('fscore_5mm',{}).get('fscore'),3)}; watertight={((hc.get('mesh',{}) or {}).get('watertight'))}, boundary edges={((hc.get('mesh',{}) or {}).get('boundary_edges'))}.")
            add(f"- held fresh Isaac reload: loaded={hf.get('loaded')}, physics_passed={hf.get('physics_passed')}, collision prims={hf.get('collision_prim_count','—')}.")
        else:
            add("- held official BundleSDF: waiting for the v7 scan manifest.")
    else:
        add("The v6 run is archived as a scan safety timeout at frame 826 (`FREE_SPACE_SETTLE`, bilateral contact retained, no DROP); v7 is archived as an outer one-hour timeout at 1105/1800 frames; v8 is archived as `REGRASP_FAILED` after a settled 15.7° reorientation; v9 is archived as a joint tracking abort; the latest retry uses the scan-only settled-reorientation guard and slower held-scan execution.")
    add("")
    add("## MV-SAM3D 2/4/8")
    add("")
    add("The adapter preserves the upstream single-object layout (`images/*.png` and RGBA alpha masks) and uses the same deterministic ring selection. It excludes no measured RGB pixels beyond the stored object mask; gripper masks and poses remain in the scan audit. The official runs use `run_inference_weighted.py --decode_formats gaussian,mesh` with default stage settings.")
    add("")
    add("| group | views | output | status |")
    add("|---|---:|---|---|")
    for r in mv_rows(mvroot):
        add(f"| {r['group']} | {r.get('views','—')} | `{r['path']}` | {r['size']/1e6:.1f} MB GLB |")
    add("")
    for r in mv_timings(mvroot):
        add(f"- `{r['log']}`: views={r.get('views','—')}, finished={r.get('finished')}, elapsed={fmt(r.get('elapsed_seconds'),1)} s, generated_coordinates={r.get('coordinates','—')}; output={r.get('output','—')}.")
    add("")
    add("Static visual previews:")
    add("")
    add("- `fr3_moveit_grasp/results/mv_sam3d_soup_v1/mv_soup_02v.png`")
    add("- `fr3_moveit_grasp/results/mv_sam3d_soup_v1/mv_soup_04v.png`")
    add("- `fr3_moveit_grasp/results/mv_sam3d_soup_v1/mv_soup_08v.png`")
    add("- `fr3_moveit_grasp/results/mv_sam3d_soup_v1/renders_held_preflight/mv_soup_8v.png`")
    add("")
    add("## Inputs and provenance")
    add("")
    add("- static MV-SAM3D input montage: `fr3_moveit_grasp/results/mv_sam3d_soup_v1/static_inputs/views_08/input_mask_overlay.png`")
    add("- held preflight input montage: `fr3_moveit_grasp/results/mv_sam3d_soup_v1/inputs_held_preflight/input_mask_overlay.png`")
    add("- stationary official input: `/data1/home/rangeryx/fr3_moveit_grasp/results/real2sim_official_soup_1800_v2/official_pipeline/official_input/soup`")
    add("- held input and official output paths are recorded in the run manifests; all poses come from the robot/scan records.")
    add("- official commits: Scalable Real2Sim `a8e4d97cbb0c3ea887a69fa313bcd3a252c5a8a3`, BundleSDF `4029bb7504b5aa9af2e9bc7161704b9e82df3d32`, robot_payload_id `c52e31cf26c83b33aee5e56f805e1d4d710fd549`.")
    add("- MV-SAM3D commit: `abb04b5e8af5bc33b0265bdf19937e76bbb6bcdd`; checkpoint source is the user-requested ModelScope `facebook/sam-3d-objects` snapshot.")
    add("")
    add("## Interpretation")
    add("")
    add("The stationary run demonstrates the official reconstruction path and fresh Isaac packaging, but its side-only direction coverage cannot recover the underside or the original hand-occluded region. The physical held pass is the required experiment for those surfaces. MV-SAM3D provides a separate multi-view prior comparison at 2/4/8 views; its outputs are not used to alter the official BundleSDF asset or the frozen grasp executor.")
    add("")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    render_report(a.root, a.output)
    print(a.output)


if __name__ == "__main__":
    main()
