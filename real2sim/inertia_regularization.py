"""Create simulation-safe inertial parameters without consulting ground truth.

The paired dynamics estimate is kept verbatim.  When its regressor is poorly
conditioned, COM and inertia are taken from the reconstructed convex hull at
the dynamically identified mass.  This is an explicit observability fallback,
not a hidden replacement of the PayloadID result.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh


def regularize(dynamic_path: Path, mesh_path: Path, output: Path,
               max_condition: float = 500.0) -> dict:
    dynamic = json.loads(Path(dynamic_path).read_text())
    mesh = trimesh.load(mesh_path, force="mesh", process=True)
    hull = mesh.convex_hull
    mass = float(dynamic["mass"])
    if not np.isfinite(hull.volume) or hull.volume <= 0:
        raise RuntimeError("reconstructed convex hull has invalid volume")
    density_scale = mass / float(hull.volume)
    geometry_com = np.asarray(hull.center_mass, dtype=float)
    geometry_inertia = np.asarray(hull.moment_inertia, dtype=float) * density_scale

    condition = float(dynamic.get("condition_number", np.inf))
    raw_com = np.asarray(dynamic["center_of_mass"], dtype=float)
    raw_inertia = np.asarray(dynamic["inertia_matrix"], dtype=float)
    raw_eigenvalues = np.linalg.eigvalsh((raw_inertia + raw_inertia.T) / 2)
    extent = np.maximum(np.asarray(hull.extents, dtype=float), 1e-6)
    com_plausible = bool(np.all(np.abs(raw_com - geometry_com) <= extent))
    inertia_scale = mass * float(np.dot(extent, extent))
    inertia_plausible = bool(
        np.all(raw_eigenvalues > 1e-7 * inertia_scale)
        and np.all(raw_eigenvalues < inertia_scale)
    )
    dynamics_observable = bool(
        condition <= max_condition and com_plausible and inertia_plausible
    )

    if dynamics_observable:
        com = raw_com
        inertia = (raw_inertia + raw_inertia.T) / 2
        source = "paired_payload_id"
    else:
        com = geometry_com
        inertia = geometry_inertia
        source = "reconstructed_convex_hull_at_identified_mass"

    result = {
        **dynamic,
        "center_of_mass": com.tolist(),
        "inertia_matrix": inertia.tolist(),
        "estimator": "payload_id_with_geometry_observability_fallback_v1",
        "simulation_inertia_source": source,
        "payload_dynamics_observable": dynamics_observable,
        "observability_max_condition": max_condition,
        "raw_payload_id": dynamic,
        "geometry_prior": {
            "mesh": str(mesh_path),
            "convex_hull_volume_m3": float(hull.volume),
            "center_of_mass": geometry_com.tolist(),
            "inertia_matrix": geometry_inertia.tolist(),
        },
        "checks": {
            "condition_ok": condition <= max_condition,
            "com_plausible": com_plausible,
            "inertia_plausible": inertia_plausible,
        },
    }
    Path(output).write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("dynamic", type=Path)
    parser.add_argument("mesh", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-condition", type=float, default=500.0)
    args = parser.parse_args()
    print(json.dumps(regularize(args.dynamic, args.mesh, args.output,
                                args.max_condition), indent=2))
