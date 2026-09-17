"""Headless multi-angle preview for a textured mesh/GLB.

This is a display-only utility for reconstruction outputs.  It does not touch
the grasp or Real2Sim execution paths and does not use ground-truth geometry.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import pyrender
import trimesh


def look_at(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - position
    forward /= max(np.linalg.norm(forward), 1e-12)
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, up))) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right /= max(np.linalg.norm(right), 1e-12)
    up = np.cross(right, forward)
    up /= max(np.linalg.norm(up), 1e-12)
    # OpenGL camera basis: right, up, and -forward.
    pose = np.eye(4, dtype=float)
    pose[:3, :3] = np.column_stack((right, up, -forward))
    pose[:3, 3] = position
    return pose


def render(mesh_path: Path, output: Path, size: int = 640, untextured: bool = False) -> None:
    loaded = trimesh.load(mesh_path, force="scene", process=False)
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"unsupported mesh type: {type(mesh)!r}")
    mesh = mesh.copy()
    center = mesh.bounds.mean(axis=0)
    mesh.apply_translation(-center)
    extent = float(np.max(mesh.extents))
    mesh.apply_scale(1.0 / max(extent, 1e-9))
    if untextured:
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=[0.56, 0.68, 0.82, 1.0],
            metallicFactor=0.0,
            roughnessFactor=0.75,
        )
        primitive = pyrender.Mesh.from_trimesh(mesh, material=material, smooth=False)
    else:
        primitive = pyrender.Mesh.from_trimesh(mesh, smooth=False)
    tiles = []
    renderer = pyrender.OffscreenRenderer(viewport_width=size, viewport_height=size)
    try:
        for idx, az in enumerate(np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)):
            scene = pyrender.Scene(
                bg_color=np.array([0.025, 0.035, 0.05, 1.0]),
                ambient_light=np.array([0.30, 0.30, 0.30]),
            )
            scene.add(primitive)
            scene.add(pyrender.PerspectiveCamera(yfov=np.deg2rad(45.0)),
                      pose=look_at(np.array([1.75 * np.cos(az), 1.75 * np.sin(az), 0.72]), np.zeros(3)))
            for light_pose, intensity in [
                (np.array([2.0, -1.5, 2.5]), 12.0),
                (np.array([-1.5, 1.0, 1.0]), 6.0),
            ]:
                scene.add(pyrender.PointLight(color=np.ones(3), intensity=intensity),
                          pose=look_at(light_pose, np.zeros(3)))
            color, _ = renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
            tile = cv2.cvtColor(color, cv2.COLOR_RGBA2BGR)
            cv2.putText(tile, f"az {np.rad2deg(az):.0f} deg", (16, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            tiles.append(tile)
    finally:
        renderer.delete()
    rows = []
    for i in range(0, len(tiles), 4):
        row = tiles[i:i + 4]
        if len(row) < 4:
            row += [np.zeros_like(tiles[0])] * (4 - len(row))
        rows.append(np.concatenate(row, axis=1))
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), np.concatenate(rows, axis=0))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--size", type=int, default=640)
    parser.add_argument("--untextured", action="store_true")
    args = parser.parse_args()
    render(args.mesh, args.output, args.size, args.untextured)


if __name__ == "__main__":
    main()
