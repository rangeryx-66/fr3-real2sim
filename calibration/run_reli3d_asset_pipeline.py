#!/usr/bin/env python3
"""Run the independent ReLi3D -> metric -> CoACD -> Isaac asset chain.

The script is intentionally adjacent to the existing pipelines.  It never
imports the grasp executor and never reads ``eval_gt`` unless an explicit GT
mesh is supplied for the final audit.  The acquisition, BundleSDF and VGGT
backends remain untouched and can be used as fallbacks/auditors.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from real2sim.collision_mesh import decompose
from real2sim.coverage_audit import audit as coverage_audit
from real2sim.reli3d_backend import (
    ReLi3DInputConfig,
    extract_texture_and_obj,
    metric_align_and_constrain,
    prepare_input,
)


def _resolve(path: Path, base: Path = ROOT) -> Path:
    path = Path(path).expanduser()
    return path if path.is_absolute() else (base / path).resolve()


def _stage(output: Path, name: str, command: list[str], env: dict[str, str] | None = None,
           cwd: Path | None = None) -> dict[str, object]:
    started = time.time()
    log = output / f"{name}.log"
    merged = os.environ.copy()
    if env:
        merged.update(env)
    with log.open("w", encoding="utf-8") as stream:
        stream.write("$ " + " ".join(command) + "\n")
        stream.flush()
        proc = subprocess.run(command, cwd=str(cwd) if cwd else None, env=merged,
                              stdout=stream, stderr=subprocess.STDOUT, check=False)
    elapsed = time.time() - started
    (output / f"{name}.time").write_text(f"WALL_SECONDS={elapsed:.6f}\n")
    if proc.returncode:
        raise RuntimeError(f"{name} failed with exit code {proc.returncode}; see {log}")
    return {"stage": name, "seconds": elapsed, "log": str(log), "command": command}


def _mesh_topology(mesh_path: Path) -> dict[str, object]:
    import trimesh
    import numpy as np
    mesh = trimesh.load(mesh_path, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError(f"mesh topology requires a single mesh: {mesh_path}")

    def _stats(cur):
        edges = np.sort(cur.edges, axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        return {
            "vertices": int(len(cur.vertices)), "faces": int(len(cur.faces)),
            "watertight": bool(cur.is_watertight),
            "connected_components": int(len(cur.split(only_watertight=False))),
            "boundary_edges": int(np.sum(counts == 1)),
        }

    raw = _stats(mesh)
    # ReLi3D's UV baking duplicates vertex records along seams.  Report the
    # raw glTF counts for provenance and welded topology for meaningful
    # boundary/component numbers.
    welded = mesh.copy()
    welded.merge_vertices(digits_vertex=6)
    welded.remove_unreferenced_vertices()
    welded_stats = _stats(welded)
    return {
        **welded_stats,
        "raw_glb": raw,
        "welded": welded_stats,
    }


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Independent ReLi3D scan-station asset pipeline")
    p.add_argument("dataset", type=Path, help="scan-station directory")
    p.add_argument("tracking", type=Path, help="tracking directory (audit only; robot poses come from dataset)")
    p.add_argument("inertial", type=Path, help="frozen PayloadID/physics JSON")
    p.add_argument("output", type=Path)
    p.add_argument("gt_mesh", type=Path, nargs="?", default=None, help="optional final-evaluation-only GT mesh")
    p.add_argument("--reli3d-root", type=Path, default=Path("/data1/home/rangeryx/ReLi3D"))
    p.add_argument("--python", dest="python", type=Path, default=Path("/data1/home/rangeryx/.conda/envs/robotics/bin/python3.10"))
    p.add_argument("--isaac-python", type=Path, default=None)
    p.add_argument("--gpu", type=int, default=0)
    p.add_argument("--num-views", type=int, default=4,
                   help="conditioning views passed to the released checkpoint (trained max is four)")
    p.add_argument("--texture-size", type=int, default=1024)
    p.add_argument("--image-size", type=int, default=512)
    p.add_argument("--crop-padding", type=float, default=.12)
    p.add_argument("--model-camera-radius", type=float, default=2.4,
                   help="model-space camera radius; zero disables translation normalization")
    p.add_argument("--asset-name", default="object_reli3d")
    p.add_argument("--overwrite", action="store_true")
    p.add_argument("--skip-inference", action="store_true", help="reuse output/reli3d/scan_object/mesh.glb")
    p.add_argument("--skip-coacd", action="store_true")
    p.add_argument("--skip-isaac", action="store_true")
    return p.parse_args()


def main() -> None:
    args = _args()
    dataset, tracking, inertial = _resolve(args.dataset), _resolve(args.tracking), _resolve(args.inertial)
    output = _resolve(args.output)
    if output.exists() and args.overwrite:
        # Do not remove source scan data; only the requested output directory is
        # owned by this independent backend.
        import shutil
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)
    reli_root = _resolve(args.reli3d_root, Path("/"))
    python = _resolve(args.python, Path("/"))
    stages: list[dict[str, object]] = []
    started_all = time.time()

    input_root = output / "input"
    input_cfg = ReLi3DInputConfig(image_size=args.image_size, crop_padding=args.crop_padding,
                                  max_views=args.num_views, model_camera_radius=args.model_camera_radius)
    t0 = time.time(); input_report = prepare_input(dataset, input_root, input_cfg)
    stages.append({"stage": "input_prepare", "seconds": time.time() - t0, "log": None})

    official_out = output / "reli3d"
    raw_glb = official_out / "scan_object" / "mesh.glb"
    if not args.skip_inference:
        config = reli_root / "artifacts/model/config.yaml"
        checkpoint = reli_root / "artifacts/model/reli3d_final.ckpt"
        if not config.exists() or not checkpoint.exists():
            raise FileNotFoundError(f"ReLi3D artifacts missing: {config}, {checkpoint}")
        cmd = [str(python), str(reli_root / "demos/reli3d/infer_from_transforms.py"),
               "--input-root", str(input_root), "--objects", "scan_object",
               "--output-root", str(official_out), "--config", str(config),
               "--checkpoint", str(checkpoint), "--num-views", str(args.num_views),
               "--texture-size", str(args.texture_size), "--remesh", "none",
               "--overwrite", "--disable-hf-download"]
        stages.append(_stage(output, "reli3d_inference", cmd,
                             {"CUDA_VISIBLE_DEVICES": str(args.gpu), "PYTHONPATH": f"{reli_root}:{ROOT}",
                              "HF_HOME": os.environ.get("HF_HOME", "/data1/home/rangeryx/.cache/huggingface"),
                              "HF_HUB_OFFLINE": "1", "TRANSFORMERS_OFFLINE": "1",
                              # The shared robotics environment has Flash-Attention
                              # 2.8.3 while this inference-only release's import
                              # guard stops at 2.8.2.  The CUDA ABI is compatible;
                              # bypass only the version guard for this subprocess.
                              "XFORMERS_IGNORE_FLASH_VERSION_CHECK": "1"}, reli_root))
    if not raw_glb.exists():
        raise FileNotFoundError(f"ReLi3D mesh not found: {raw_glb}")

    t0 = time.time()
    metric_report = metric_align_and_constrain(raw_glb, input_root / "input_report.json", output / "metric")
    stages.append({"stage": "metric_alignment", "seconds": time.time() - t0, "log": None})
    t0 = time.time()
    texture_report = extract_texture_and_obj(Path(metric_report["metric_glb"]), output / "textured")
    stages.append({"stage": "texture_export", "seconds": time.time() - t0, "log": None})

    collision_dir = output / "collision"
    collision_manifest = collision_dir / "collision_manifest.json"
    if not args.skip_coacd:
        t0 = time.time(); collision = decompose(Path(texture_report["mesh"]), collision_dir)
        stages.append({"stage": "coacd", "seconds": time.time() - t0, "log": None})
    else:
        collision = json.loads(collision_manifest.read_text())

    # The existing asset builder is a packaging utility only.  Its input is the
    # ReLi3D metric mesh and the frozen inertial JSON, so physics semantics are
    # unchanged by this backend.
    asset_dir = output / "asset"
    cmd = [str(python), "-m", "real2sim.asset_builder", "--name", args.asset_name,
           "--visual", str(texture_report["mesh"]), "--texture", str(texture_report["texture"]),
           "--collision", str(collision_manifest), "--inertial", str(inertial), "--output", str(asset_dir)]
    stages.append(_stage(output, "asset_builder", cmd, {"PYTHONPATH": str(ROOT)}))

    coverage = None
    try:
        t0 = time.time(); coverage = coverage_audit(Path(metric_report["metric_mesh"]), dataset, tracking, output / "coverage.json")
        stages.append({"stage": "coverage_audit", "seconds": time.time() - t0, "log": None})
    except Exception as exc:
        coverage = {"available": False, "error": str(exc), "gt_used": False}
        (output / "coverage.json").write_text(json.dumps(coverage, indent=2))

    reload_validation = None
    isaac_python = args.isaac_python or (Path("/data1/home/rangeryx/isaaclab-arena/.venv/bin/python") if Path("/data1/home/rangeryx/isaaclab-arena/.venv/bin/python").exists() else None)
    usd = asset_dir / f"{args.asset_name}.usda"
    if not args.skip_isaac and isaac_python and usd.exists():
        cmd = [str(isaac_python), "calibration/validate_real2sim_usd.py", "--asset", str(usd),
               "--output", str(asset_dir / "reload_render.png"), "--gpu", str(args.gpu)]
        # The validator asks once whether to run the drop test.
        reload_validation_stage = output / "isaac_validation.log"
        t0 = time.time()
        with reload_validation_stage.open("w", encoding="utf-8") as stream:
            proc = subprocess.run(cmd, cwd=str(ROOT), input="Yes\n", text=True, stdout=stream,
                                  stderr=subprocess.STDOUT, check=False)
        stages.append({"stage": "isaac_reload", "seconds": time.time() - t0, "log": str(reload_validation_stage)})
        if proc.returncode:
            raise RuntimeError(f"Isaac validation failed ({proc.returncode}); see {reload_validation_stage}")
        validation_json = asset_dir / "reload_validation.json"
        if validation_json.exists():
            reload_validation = json.loads(validation_json.read_text())

    gt_metrics = None
    if args.gt_mesh:
        from real2sim.scan_metrics import evaluate
        gt_metrics = evaluate(Path(metric_report["metric_mesh"]), _resolve(args.gt_mesh), dataset, tracking,
                              texture_report=output / "textured/texture_report.json")
        (output / "gt_metrics.json").write_text(json.dumps(gt_metrics, indent=2))

    manifest = {
        "schema": "fr3_reli3d_asset_pipeline/v1", "backend": "ReLi3D official inference",
        "gt_used_for_generation_or_alignment": False, "grasp_executor_modified": False,
        "dataset": str(dataset), "tracking_audit_only": str(tracking), "inertial": str(inertial),
        "reli3d_root": str(reli_root), "reli3d_model": str(reli_root / "artifacts/model/reli3d_final.ckpt"),
        "products": {"input": str(input_root), "raw_reli3d_glb": str(raw_glb),
                     "metric_mesh": metric_report["metric_mesh"], "metric_glb": metric_report["metric_glb"],
                     "textured_mesh": texture_report["mesh"], "texture": texture_report["texture"],
                     "collision_manifest": str(collision_manifest), "isaac_usd": str(usd),
                     "isaac_render": str(asset_dir / "reload_render.png")},
        "input_report": input_report, "metric_report": metric_report, "texture_report": texture_report,
        "collision": collision, "coverage": coverage, "isaac_reload": reload_validation,
        "gt_metrics": gt_metrics, "raw_mesh_topology": _mesh_topology(raw_glb),
        "timing": {"stages": stages, "total_seconds": time.time() - started_all},
        "fallback_backends": ["BundleSDF", "VGGT", "ReconViaGen"],
    }
    (output / "pipeline_manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
