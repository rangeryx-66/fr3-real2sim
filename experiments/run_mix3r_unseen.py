#!/usr/bin/env python3
"""Run the official Mix3R V2 pipeline on the frozen 8-view scan inputs.

This experiment deliberately leaves the grasp executor untouched.  The input
views are the same RGBA crops used by the static MV-SAM3D run; the alpha
channel is the object mask and all outputs stay under a separate results tree.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import typing
from pathlib import Path

# cvcg_utils uses typing.Self while the robotics environment is Python 3.10.
try:
    from typing_extensions import Self  # type: ignore
    typing.Self = Self  # type: ignore[attr-defined]
except Exception:
    pass

import numpy as np
import torch


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--repo", default="/data1/home/rangeryx/mix3r")
    p.add_argument("--cvcg", default="/data1/home/rangeryx/cvcg_utils")
    p.add_argument("--input-root", required=True)
    p.add_argument("--output-root", required=True)
    p.add_argument("--objects", nargs="+", required=True)
    p.add_argument("--trellis", default="/data1/home/rangeryx/TRELLIS/checkpoints/TRELLIS-image-large")
    p.add_argument("--dinov2", default="/data1/home/rangeryx/.cache/torch/hub/facebookresearch_dinov2_main")
    p.add_argument("--pi3", default="yyfz233/Pi3")
    p.add_argument("--seed", type=int, default=20260915)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    repo = Path(args.repo)
    cvcg = Path(args.cvcg)
    sys.path.insert(0, str(repo))
    sys.path.insert(0, str(cvcg))

    from cvcg_utils.image import read_rgba
    from mix3r_model.pipelines.pipeline_original import Pipeline

    out_root = Path(args.output_root)
    out_root.mkdir(parents=True, exist_ok=True)
    run_meta = {
        "method": "Mix3R V2 official pipeline",
        "checkpoint": str(repo / "checkpoints" / "mix3r_v2.safetensors"),
        "trellis": args.trellis,
        "dinov2": args.dinov2,
        "pi3": args.pi3,
        "objects": args.objects,
        "views": 8,
        "recenter": False,
        "seed": args.seed,
        "input_root": args.input_root,
        "grasp_executor_changed": False,
    }
    (out_root / "run_meta.json").write_text(json.dumps(run_meta, indent=2), encoding="utf-8")

    print("[mix3r] loading official V2 pipeline", flush=True)
    load_t0 = time.time()
    pipe = Pipeline.from_pretrained(
        mix3r_path=str(repo / "checkpoints" / "mix3r_v2.safetensors"),
        trellis_path=args.trellis,
        dinov2_path=args.dinov2,
        pi3_path=args.pi3,
    )
    pipe.cuda()
    print(f"[mix3r] pipeline ready in {time.time() - load_t0:.1f}s", flush=True)

    rows = []
    for obj in args.objects:
        # The object-named directory contains RGBA crops with clean alpha masks.
        in_dir = Path(args.input_root) / obj / "mv_inputs" / "views_08" / obj
        img_paths = [in_dir / f"{i}.png" for i in range(8)]
        missing = [str(p) for p in img_paths if not p.exists()]
        if missing:
            raise FileNotFoundError("missing Mix3R input(s): " + ", ".join(missing))
        images = [read_rgba(str(p)) for p in img_paths]
        alpha = np.concatenate([im[..., 3:4] for im in images], axis=2)
        out_dir = out_root / obj / "views_08"
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"[mix3r] {obj}: running 8 views", flush=True)
        t0 = time.time()
        status = "complete"
        error = None
        try:
            pipe.run(images, str(out_dir), seed=args.seed, recenter=False)
        except ModuleNotFoundError as exc:
            # The official pipeline writes the mesh/GS/SLAT before its optional
            # 360-view Gaussian preview.  Keep those valid artifacts when a
            # renderer extension is unavailable, and continue with the next
            # object instead of discarding the reconstruction.
            if (out_dir / "sample_mesh_perviewbias.ply").exists():
                status = "mesh_only"
                error = repr(exc)
                print(f"[mix3r] {obj}: mesh written; optional render unavailable: {exc}", flush=True)
            else:
                raise
        elapsed = time.time() - t0
        row = {
            "object": obj,
            "input_views": [str(p) for p in img_paths],
            "mask_area_fraction_min": float((alpha > 0).mean(axis=(0, 1)).min()),
            "mask_area_fraction_max": float((alpha > 0).mean(axis=(0, 1)).max()),
            "elapsed_s": elapsed,
            "output_dir": str(out_dir),
            "status": status,
        }
        if error is not None:
            row["error"] = error
        rows.append(row)
        (out_dir / "run_result.json").write_text(json.dumps(row, indent=2), encoding="utf-8")
        print(f"[mix3r] {obj}: finished in {elapsed:.1f}s", flush=True)
        torch.cuda.empty_cache()

    (out_root / "results.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print("[mix3r] all objects complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
