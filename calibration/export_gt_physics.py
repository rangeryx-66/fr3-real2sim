"""Evaluation-only export of Isaac GT physics after identification completes."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh


def export(calibration_result: Path, mesh_npz: Path, output_dir: Path) -> dict:
    result = json.loads(Path(calibration_result).read_text())
    material = result["runtime_material"]["material"]
    physics = {
        "mass": float(np.asarray(material["target_mass_kg"]).reshape(-1)[0]),
        "center_of_mass": np.asarray(material["target_COM_local"], dtype=float)
        .reshape(-1, 7)[0, :3].tolist(),
        "inertia_matrix": np.asarray(material["target_inertia"], dtype=float)
        .reshape(-1, 3, 3)[0].tolist(),
        "evaluation_only": True,
        "read_after_estimation": True,
    }
    data = np.load(mesh_npz)
    mesh = trimesh.Trimesh(vertices=data["vertices"], faces=data["triangles"],
                           process=False)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "physics.json").write_text(json.dumps(physics, indent=2))
    mesh.export(output_dir / "mesh.obj")
    return physics


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("calibration_result", type=Path)
    parser.add_argument("mesh_npz", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(export(args.calibration_result, args.mesh_npz, args.output),
                     indent=2))
