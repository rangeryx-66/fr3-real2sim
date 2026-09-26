"""Stage accepted continuous-loop wrist captures for reconstruction tools."""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np


def rotation_wxyz(q):
    w, x, y, z = map(float, q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wrist-scan", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--minimum-target-pixels", type=int, default=5000)
    args = ap.parse_args()
    for name in ("rgb", "depth", "masks", "poses"):
        (args.output / name).mkdir(parents=True, exist_ok=True)
    rows = []
    for meta_path in sorted(args.wrist_scan.glob("*.json")):
        meta = json.loads(meta_path.read_text())
        pixels = int(meta.get("target_pixels", 0))
        if pixels < args.minimum_target_pixels:
            continue
        stem = meta_path.stem
        paths = {
            "rgb": args.wrist_scan / f"{stem}.png",
            "depth": args.wrist_scan / f"{stem}.depth.png",
            "mask": args.wrist_scan / f"{stem}.mask.png",
        }
        if not all(path.exists() for path in paths.values()):
            continue
        frame_id = len(rows)
        out_name = f"{frame_id:06d}.png"
        shutil.copy2(paths["rgb"], args.output / "rgb" / out_name)
        shutil.copy2(paths["depth"], args.output / "depth" / out_name)
        shutil.copy2(paths["mask"], args.output / "masks" / out_name)
        T = np.eye(4)
        T[:3, :3] = rotation_wxyz(meta["camera_quaternion_world_wxyz"])
        T[:3, 3] = np.asarray(meta["camera_position_world"], float)
        pose = {
            "frame_id": frame_id,
            "source_frame": stem,
            "T_B_camera": T.tolist(),
            "intrinsics": meta.get("intrinsics"),
            "timestamp_s": meta.get("timestamp_s"),
            "target_pixels": pixels,
        }
        (args.output / "poses" / f"{frame_id:06d}.json").write_text(json.dumps(pose, indent=2))
        rows.append({
            **pose,
            "rgb": str(args.output / "rgb" / out_name),
            "depth": str(args.output / "depth" / out_name),
            "mask": str(args.output / "masks" / out_name),
        })
    manifest = {
        "schema": "fr3_continuous_wrist_scan/v1",
        "source": str(args.wrist_scan),
        "minimum_target_pixels": args.minimum_target_pixels,
        "frames": rows,
    }
    (args.output / "scan_manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps({"output": str(args.output), "accepted": len(rows),
                      "target_pixels": [row["target_pixels"] for row in rows]}))


if __name__ == "__main__":
    main()
