"""Export ReconViaGen/TRELLIS.2's native generated texture for comparison.

The normal Real2Sim product intentionally rebakes real RGB onto the metric
mesh.  This module is an audit/alternative path: it extracts the PBR texture
that the official ReconViaGen v0.5 ``tex_slat`` stage put in the generated
GLB, applies the RGB-D-derived metric transform, and writes a self-contained
OBJ/PNG pair.  It never calls the grasp executor and never uses GT for scale or
alignment.

The optional metric-mesh export transfers generated UVs by nearest raw-mesh
vertex.  The primary comparison is the aligned native GLB geometry because it
preserves the exact UV seams produced by ReconViaGen; the existing
RGB-D-constrained mesh remains the collision/physics source.
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np


# ``o_voxel.postprocess.to_glb`` uses the glTF Y-up convention while the
# ReconViaGen mesh arrays/metric constraint use the reconstruction Z-up
# convention.  This is the fixed exporter conversion (GLB -> ReconViaGen
# model frame), independent of object category.  Without it, the official
# texture comparison is rotated onto its side even though its scale is right.
GLB_TO_RECON = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
], dtype=float)


def _scene_geometry(path: Path):
    import trimesh

    scene = trimesh.load(path, force="scene", process=False)
    if not hasattr(scene, "geometry") or not scene.geometry:
        raise RuntimeError(f"no geometry in generated GLB: {path}")
    names = list(scene.geometry)
    if len(names) != 1:
        # ReconViaGen v0.5 currently exports one render mesh.  Concatenating
        # several geometries would destroy independent material/UV seams, so
        # fail loudly if a future model changes that contract.
        raise RuntimeError(f"expected one ReconViaGen GLB geometry, got {names}")
    name = names[0]
    geom = scene.geometry[name].copy()
    transform, _ = scene.graph.get(name)
    geom.apply_transform(np.asarray(transform, dtype=float))
    # Keep the exact UV/image data while converting only the GLB geometry back
    # to the frame used by the saved RGB-D metric transform.
    glb_to_recon = np.eye(4)
    glb_to_recon[:3, :3] = GLB_TO_RECON
    geom.apply_transform(glb_to_recon)
    uv = getattr(geom.visual, "uv", None)
    material = getattr(geom.visual, "material", None)
    image = getattr(material, "baseColorTexture", None)
    if uv is None or image is None:
        raise RuntimeError("official GLB has no UV/baseColorTexture")
    return geom, np.asarray(uv, dtype=np.float64), image


def _write_obj(mesh: Any, uv: np.ndarray, obj_path: Path, texture_name: str) -> None:
    """Write an OBJ with one UV per mesh vertex, preserving the GLB layout."""
    v = np.asarray(mesh.vertices, dtype=float)
    f = np.asarray(mesh.faces, dtype=np.int64)
    if len(uv) != len(v):
        raise RuntimeError(f"UV/vertex count mismatch: {len(uv)} vs {len(v)}")
    obj_path.parent.mkdir(parents=True, exist_ok=True)
    mtl_name = obj_path.with_suffix(".mtl").name
    with obj_path.open("w") as out:
        out.write(f"mtllib {mtl_name}\n")
        for p in v:
            out.write(f"v {p[0]:.9g} {p[1]:.9g} {p[2]:.9g}\n")
        for q in uv:
            out.write(f"vt {q[0]:.9g} {q[1]:.9g}\n")
        out.write("usemtl material_0\n")
        for tri in f:
            # The GLB loader gives a vertex-indexed UV array.  OBJ's
            # v/vt/vn triplets can therefore share the same index here.
            out.write("f " + " ".join(f"{int(i)+1}/{int(i)+1}" for i in tri) + "\n")
    obj_path.with_suffix(".mtl").write_text(
        "newmtl material_0\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\n"
        f"map_Kd {texture_name}\n"
    )


def _write_metric_transfer(raw_mesh: Any, raw_uv: np.ndarray, metric_path: Path,
                           scale: float, transform: np.ndarray,
                           output: Path, texture_name: str) -> dict[str, Any]:
    """Attach the native texture to the constrained mesh for a diagnostic.

    The nearest-neighbour UV transfer intentionally does not pretend to repair
    seams.  It is useful to answer whether the native texture is itself
    coherent on the metric surface; the exact-UV aligned raw variant remains
    the authoritative official-method export.
    """
    import trimesh
    from scipy.spatial import cKDTree

    metric = trimesh.load(metric_path, force="mesh", process=False)
    rv = np.asarray(raw_mesh.vertices, dtype=float)
    mv = np.asarray(metric.vertices, dtype=float)
    # metric = scale * R * raw + t
    R = np.asarray(transform[:3, :3], dtype=float)
    t = np.asarray(transform[:3, 3], dtype=float)
    raw_query = ((mv - t) @ R) / float(scale)
    nearest = cKDTree(rv).query(raw_query, workers=-1)[1]
    uv = raw_uv[np.asarray(nearest, dtype=np.int64)]
    path = output / "metric_constrained_official_texture.obj"
    _write_obj(metric, uv, path, texture_name)
    return {
        "path": str(path),
        "vertices": int(len(metric.vertices)),
        "faces": int(len(metric.faces)),
        "uv_transfer": "nearest raw generated vertex in RGB-D metric inverse transform",
    }


def export_variant(generated_glb: str | Path, alignment_report: str | Path,
                   output: str | Path, metric_mesh: str | Path | None = None) -> dict[str, Any]:
    generated_glb = Path(generated_glb).resolve()
    alignment_report = Path(alignment_report).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = json.loads(alignment_report.read_text())
    align = report["alignment"]
    scale = float(align["uniform_scale"])
    X = np.eye(4)
    X[:3, :3] = np.asarray(align["rotation"], dtype=float)
    X[:3, 3] = np.asarray(align["translation_m"], dtype=float)

    mesh, uv, image = _scene_geometry(generated_glb)
    texture_name = "reconviagen_native_basecolor.png"
    texture_path = output / texture_name
    image.convert("RGBA").save(texture_path)
    aligned = mesh.copy()
    aligned.apply_scale(scale)
    aligned.apply_transform(X)
    obj_path = output / "reconviagen_native_texture_metric_aligned.obj"
    _write_obj(aligned, uv, obj_path, texture_name)

    result: dict[str, Any] = {
        "schema": "fr3_reconviagen_v05_native_texture_variant/v1",
        "backend": "ReconViaGen v0.5 official TRELLIS.2 tex_slat",
        "source_glb": str(generated_glb),
        "gt_used": False,
        "metric_alignment_source": str(alignment_report),
        "uniform_scale": scale,
        "metric_transform": X.tolist(),
        "native_texture": str(texture_path),
        "aligned_native_obj": str(obj_path),
        "mesh": {"vertices": int(len(aligned.vertices)), "faces": int(len(aligned.faces))},
        "policy": {
            "generated_texture_is_comparison": True,
            "rgbd_used_for_scale_and_pose": True,
            "rgbd_face_conflict_filter_applied_to_native_variant": False,
            "existing_metric_mesh_remains_physics_source": True,
        },
    }
    if metric_mesh is not None:
        result["metric_mesh_transfer"] = _write_metric_transfer(
            mesh, uv, Path(metric_mesh).resolve(), scale, X, output, texture_name
        )
    (output / "native_texture_report.json").write_text(json.dumps(result, indent=2))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("generated_glb", type=Path)
    parser.add_argument("alignment_report", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--metric-mesh", type=Path)
    args = parser.parse_args()
    print(json.dumps(export_variant(args.generated_glb, args.alignment_report,
                                    args.output, args.metric_mesh), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
