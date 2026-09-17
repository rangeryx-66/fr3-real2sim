"""Create deterministic convex collision parts from the reconstructed visual mesh."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh


def decompose(mesh_path: Path, output: Path, threshold=0.05, max_convex_hull=24) -> dict:
    import coacd
    mesh_path, output = Path(mesh_path), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    loaded = trimesh.load(mesh_path, force="mesh", process=True)
    if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) < 4:
        raise RuntimeError("visual mesh is empty or invalid")
    source = coacd.Mesh(np.asarray(loaded.vertices, dtype=np.float64),
                        np.asarray(loaded.faces, dtype=np.int32))
    parts = coacd.run_coacd(source, threshold=threshold,
                            max_convex_hull=max_convex_hull, seed=20260912)
    paths, volumes = [], []
    for index, (vertices, faces) in enumerate(parts):
        part = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        path = output / f"collision_{index:03d}.obj"
        part.export(path)
        paths.append(str(path)); volumes.append(float(abs(part.volume)))
    if not paths:
        raise RuntimeError("CoACD produced no collision geometry")
    result = {
        "method": "CoACD", "threshold": threshold,
        "max_convex_hull": max_convex_hull, "parts": paths,
        "source_faces": int(len(loaded.faces)), "collision_faces": int(sum(
            len(trimesh.load(p, force="mesh").faces) for p in paths)),
        "part_volumes_m3": volumes,
    }
    (output / "collision_manifest.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("mesh", type=Path); p.add_argument("output", type=Path)
    p.add_argument("--threshold", type=float, default=.05); p.add_argument("--max-parts", type=int, default=24)
    a = p.parse_args(); print(json.dumps(decompose(a.mesh, a.output, a.threshold, a.max_parts), indent=2))
