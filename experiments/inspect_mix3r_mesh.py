#!/usr/bin/env python3
"""Convert Mix3R vertex-colored PLY meshes to GLB and report basic stats."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
import trimesh

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--objects", nargs="+", required=True)
    args = ap.parse_args()
    root = Path(args.root)
    rows = []
    for obj in args.objects:
        d = root / obj / "views_08"
        ply = d / "sample_mesh_perviewbias.ply"
        if not ply.exists():
            rows.append({"object": obj, "status": "missing", "ply": str(ply)})
            continue
        mesh = trimesh.load(str(ply), process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(tuple(g for g in mesh.geometry.values()))
        glb = d / "mix3r_v2_8v.glb"
        mesh.export(str(glb))
        ext = (mesh.bounds[1] - mesh.bounds[0]).astype(float).tolist()
        rows.append({
            "object": obj,
            "status": "ok",
            "ply": str(ply),
            "glb": str(glb),
            "vertices": int(len(mesh.vertices)),
            "faces": int(len(mesh.faces)),
            "watertight": bool(mesh.is_watertight),
            "components": int(mesh.split(only_watertight=False).__len__()),
            "extent_model_units": ext,
            "bounds_model_units": mesh.bounds.astype(float).tolist(),
        })
        print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
    (root / "mesh_stats.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    return 0
if __name__ == "__main__":
    raise SystemExit(main())
