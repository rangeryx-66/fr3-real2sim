"""ReconViaGen v0.5 multi-view completion adapter.

This module is deliberately separate from the frozen grasp executor.  It feeds
quality-filtered, alpha-masked RGB views to the official ReconViaGen v0.5
hybrid pipeline (VGGT sparse stage + TRELLIS.2 shape/texture stages), then
exports the raw complete mesh for metric constraint.  No simulator mesh,
ground-truth pose, or ground-truth physical parameter is read here.
"""
from __future__ import annotations

import argparse
import gc
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _configure_imports(recon_root: Path, trellis2_root: Path, extra_path: str | None) -> None:
    paths = [str(recon_root), str(trellis2_root), str(recon_root / "wheels" / "vggt"), "/data1/home/rangeryx/vggt"]
    if extra_path:
        paths.insert(0, extra_path)
    for path in reversed(paths):
        if path and path not in sys.path:
            sys.path.insert(0, path)
    os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")
    os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")
    os.environ.setdefault("XFORMERS_DISABLED", "1")
    os.environ.setdefault("ATTN_BACKEND", "sdpa")
    # The lab robotics environment ships xformers against flash-attn 2.8.x;
    # its xformers loader rejects that combination.  The official v0.5 code
    # supports the flash-attn backend directly, so select it by default while
    # allowing an explicit override for a matching installation.
    os.environ.setdefault("SPARSE_ATTN_BACKEND", os.environ.get("RECONVIAGEN_SPARSE_ATTN_BACKEND", "flash_attn"))
    os.environ.setdefault("SPCONV_ALGO", "native")
    # nvdiffrast JIT-compiles a small CUDA rasterizer during the SS stage and
    # locates Ninja through PATH rather than the Python module import.
    python_bin = str(Path(sys.executable).resolve().parent)
    if python_bin not in os.environ.get("PATH", "").split(os.pathsep):
        os.environ["PATH"] = python_bin + os.pathsep + os.environ.get("PATH", "")


def _ensure_dreamsim_cache(cache: Path) -> None:
    """The upstream v0.5 loader uses ``weights/dreamsim`` relative to cwd."""
    cache = cache.expanduser().resolve()
    local = Path.cwd() / "weights" / "dreamsim"
    if local.exists():
        return
    local.parent.mkdir(parents=True, exist_ok=True)
    try:
        local.symlink_to(cache, target_is_directory=True)
    except OSError:
        # A copy is unnecessary for normal runs, but retaining this fallback
        # makes the adapter usable on filesystems that disallow symlinks.
        import shutil
        shutil.copytree(cache, local)


def _samplers(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    ss = {
        "steps": int(args.ss_steps),
        "cfg_strength": float(args.ss_guidance),
        "cfg_interval": [0.6, 1.0],
        "guidance_rescale": float(args.ss_guidance_rescale),
        "rescale_t": float(args.ss_rescale_t),
    }
    shape = {
        "steps": int(args.shape_steps),
        "guidance_strength": float(args.shape_guidance),
        "guidance_rescale": float(args.shape_guidance_rescale),
        "rescale_t": float(args.shape_rescale_t),
    }
    tex = {
        "steps": int(args.tex_steps),
        "guidance_strength": 1.0,
        "guidance_rescale": 0.0,
        "rescale_t": float(args.tex_rescale_t),
    }
    return ss, shape, tex


def _load_inputs(manifest_path: Path):
    from PIL import Image

    manifest = json.loads(manifest_path.read_text())
    images = []
    for item in manifest["input_views"]:
        path = Path(item)
        if not path.is_absolute():
            path = Path.cwd() / path
        if not path.exists():
            raise FileNotFoundError(path)
        with Image.open(path) as image:
            # ReconViaGen accepts RGBA and uses the alpha channel as the object
            # mask.  The selector already removed gripper pixels.
            images.append(image.convert("RGBA"))
    if len(images) < 2:
        raise RuntimeError("ReconViaGen v0.5 requires at least two selected views")
    return manifest, images


def _configure_vggt(vggt: Any, torch: Any, device: str, low_vram: bool) -> None:
    vggt.low_vram = low_vram
    vggt._device = torch.device(device)
    # Gaussian decoding is not needed for the SS stage and can consume several
    # GB while the TRELLIS.2 models are resident.
    vggt.models.pop("slat_decoder_gs", None)
    if low_vram:
        for model in vggt.models.values():
            model.cpu()
        for attr in ("VGGT_model", "birefnet_model"):
            model = getattr(vggt, attr, None)
            if model is not None:
                model.cpu()
    else:
        vggt.to(torch.device(device))


def _configure_trellis2(t2: Any, torch: Any, device: str, low_vram: bool) -> None:
    t2.low_vram = low_vram
    t2._device = torch.device(device)
    # The official hybrid wrapper supplies sparse coordinates from ReconViaGen
    # SS, so the duplicate TRELLIS.2 sparse models are intentionally removed.
    t2.models.pop("sparse_structure_decoder", None)
    t2.models.pop("sparse_structure_flow_model", None)
    if not low_vram:
        t2.to(torch.device(device))
    else:
        for model in t2.models.values():
            model.cpu()


def _numpy_mesh(mesh: Any) -> tuple[np.ndarray, np.ndarray]:
    def arr(value: Any) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        return np.asarray(value)

    vertices = arr(mesh.vertices).astype(np.float64)
    faces = arr(mesh.faces).astype(np.int64)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or faces.ndim != 2 or faces.shape[1] != 3:
        raise RuntimeError(f"unexpected ReconViaGen mesh arrays: {vertices.shape}, {faces.shape}")
    return vertices, faces


def _write_mesh_products(mesh: Any, output: Path, args: argparse.Namespace, torch: Any) -> dict[str, str]:
    import trimesh

    vertices, faces = _numpy_mesh(mesh)
    raw = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    obj = output / "generated_complete_mesh.obj"
    ply = output / "generated_complete_mesh.ply"
    raw.export(obj)
    raw.export(ply)
    products = {"obj": str(obj), "ply": str(ply)}

    # Preserve the PBR result produced by TRELLIS.2 as an audit artifact.  The
    # final texture stage later rebakes from real RGB; generated PBR is never
    # allowed to overwrite measured labels/colours.
    try:
        import o_voxel

        grid_size = getattr(mesh, "voxel_size", None)
        if grid_size is not None:
            if hasattr(grid_size, "detach"):
                grid_size = grid_size.detach().float().cpu().numpy()
            grid_size = np.asarray(grid_size)
            grid_size = int(round(1.0 / float(grid_size))) if grid_size.ndim == 0 else np.rint(1.0 / grid_size).astype(int).tolist()
        else:
            grid_size = int(getattr(mesh, "res", 1024))
        glb = o_voxel.postprocess.to_glb(
            vertices=mesh.vertices,
            faces=mesh.faces,
            attr_volume=mesh.attrs,
            coords=mesh.coords,
            attr_layout=getattr(mesh, "layout", {
                "base_color": slice(0, 3), "metallic": slice(3, 4),
                "roughness": slice(4, 5), "alpha": slice(5, 6),
            }),
            grid_size=grid_size,
            aabb=[[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]],
            decimation_target=int(args.decimation_target),
            texture_size=int(args.generated_texture_size),
            remesh=True,
            remesh_band=1,
            remesh_project=0,
            use_tqdm=True,
        )
        glb_path = output / "generated_complete_mesh.glb"
        glb.export(str(glb_path), extension_webp=True)
        products["glb"] = str(glb_path)
    except Exception as exc:  # raw OBJ remains useful if optional PBR export fails
        (output / "generated_glb_error.txt").write_text(f"{type(exc).__name__}: {exc}\n")
        products["glb_error"] = str(output / "generated_glb_error.txt")
    return products


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    manifest, images = _load_inputs(Path(args.view_manifest))
    recon_root = Path(args.recon_root)
    trellis2_root = Path(args.trellis2_root)
    _configure_imports(recon_root, trellis2_root, args.extra_path)
    _ensure_dreamsim_cache(Path(args.dreamsim_cache))

    import torch
    from trellis.pipelines import TrellisVGGTTo3DPipeline
    from trellis.pipelines.trellis_hybrid_pipeline import TrellisHybridPipeline
    from trellis2.pipelines import Trellis2ImageTo3DPipeline

    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable for ReconViaGen v0.5")
    t0 = time.perf_counter()
    print("loading ReconViaGen v0.5 SS pipeline", flush=True)
    vggt = TrellisVGGTTo3DPipeline.from_pretrained(args.pretrained)
    _configure_vggt(vggt, torch, args.device, args.low_vram)
    gc.collect()
    torch.cuda.empty_cache()
    t_load_ss = time.perf_counter()
    print("loading TRELLIS.2 shape/texture pipeline", flush=True)
    t2 = Trellis2ImageTo3DPipeline.from_pretrained(args.trellis2_pretrained)
    _configure_trellis2(t2, torch, args.device, args.low_vram)
    gc.collect()
    torch.cuda.empty_cache()
    hybrid = TrellisHybridPipeline(vggt, t2, low_vram=args.low_vram)
    t_load_all = time.perf_counter()

    ss, shape, tex = _samplers(args)
    print(f"running official ReconViaGen v0.5 multi-image ({len(images)} views)", flush=True)
    with torch.inference_mode():
        meshes, _latent = hybrid.run_multi_image(
            images,
            strategy=args.strategy,
            seed=int(args.seed),
            ss_sampler_params=ss,
            slat_sampler_params=ss,
            shape_slat_sampler_params=shape,
            tex_slat_sampler_params=tex,
            pipeline_type=args.pipeline_type,
            preprocess_image=False,
            return_latent=True,
            max_num_tokens=int(args.max_num_tokens),
        )
    if not meshes:
        raise RuntimeError("ReconViaGen v0.5 returned no mesh")
    mesh = meshes[0]
    t_infer = time.perf_counter()
    products = _write_mesh_products(mesh, output, args, torch)
    t_export = time.perf_counter()
    vertices, faces = _numpy_mesh(mesh)
    extent = (vertices.max(axis=0) - vertices.min(axis=0)).tolist()
    report = {
        "schema": "fr3_reconviagen_v05_inference/v1",
        "backend": "ReconViaGen v0.5 official hybrid",
        "gt_used": False,
        "view_manifest": str(Path(args.view_manifest)),
        "selected_views": manifest["input_views"],
        "view_count": len(images),
        "strategy": args.strategy,
        "pipeline_type": args.pipeline_type,
        "seed": int(args.seed),
        "samplers": {"ss": ss, "shape": shape, "texture": tex},
        "mesh": {"vertices": int(len(vertices)), "faces": int(len(faces)), "normalized_aabb_extent": extent, "normalized_aabb_min": vertices.min(axis=0).tolist(), "normalized_aabb_max": vertices.max(axis=0).tolist()},
        "products": products,
        "timing_seconds": {"load_ss": t_load_ss - t0, "load_all": t_load_all - t_load_ss, "inference": t_infer - t_load_all, "export": t_export - t_infer, "total": t_export - t0},
        "runtime": {"torch": torch.__version__, "cuda": torch.version.cuda, "device": args.device, "low_vram": bool(args.low_vram), "recon_root": str(recon_root), "trellis2_root": str(trellis2_root), "extra_path": args.extra_path},
        "policy": {"rgbd_metric_constraint_pending": True, "generated_texture_is_audit_only": True, "gripper_mask_excluded_at_input": True},
    }
    (output / "inference_report.json").write_text(json.dumps(report, indent=2))
    return report


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("view_manifest", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--recon-root", default="/data1/home/rangeryx/ReconViaGen")
    p.add_argument("--trellis2-root", default="/data1/home/rangeryx/ReconViaGen/wheels/TRELLIS.2")
    p.add_argument("--extra-path", default=os.environ.get("RECONVIAGEN_EXTRA_PATH"))
    p.add_argument("--pretrained", default="Stable-X/trellis-vggt-v0-2")
    p.add_argument("--trellis2-pretrained", default="microsoft/TRELLIS.2-4B")
    p.add_argument("--dreamsim-cache", default="/data1/home/rangeryx/ReconViaGen/weights/dreamsim")
    p.add_argument("--device", default="cuda")
    p.add_argument("--pipeline-type", choices=("512", "1024", "1536"), default="512")
    p.add_argument("--strategy", choices=("average_right", "weighted_average", "adaptive_guidance_weight", "fixed_guidance_rescale", "average", "sequential"), default="adaptive_guidance_weight")
    p.add_argument("--seed", type=int, default=20260914)
    p.add_argument("--max-num-tokens", type=int, default=49152)
    p.add_argument("--ss-steps", type=int, default=30)
    p.add_argument("--shape-steps", type=int, default=12)
    p.add_argument("--tex-steps", type=int, default=12)
    p.add_argument("--ss-guidance", type=float, default=7.5)
    p.add_argument("--shape-guidance", type=float, default=7.5)
    p.add_argument("--ss-guidance-rescale", type=float, default=0.7)
    p.add_argument("--shape-guidance-rescale", type=float, default=0.5)
    p.add_argument("--ss-rescale-t", type=float, default=5.0)
    p.add_argument("--shape-rescale-t", type=float, default=3.0)
    p.add_argument("--tex-rescale-t", type=float, default=3.0)
    p.add_argument("--decimation-target", type=int, default=500000)
    p.add_argument("--generated-texture-size", type=int, default=2048)
    p.add_argument("--low-vram", action=argparse.BooleanOptionalAction, default=True)
    return p


if __name__ == "__main__":
    args = build_parser().parse_args()
    print(json.dumps(run(args), indent=2))
