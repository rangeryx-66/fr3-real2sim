"""Split a combined PayloadID static record into evaluator-compatible poses."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def split(source: Path, output: Path, prefix: str):
    output.mkdir(parents=True, exist_ok=True)
    with np.load(source, allow_pickle=True) as loaded:
        values = {key: np.asarray(loaded[key]) for key in loaded.files}
    count = len(values["t"])
    pose = np.asarray(values["static_pose_id"]).reshape(-1)
    hold = np.asarray(values["static_hold"], bool).reshape(-1)
    rows = []
    for pose_id in sorted(set(map(int, pose[hold & (pose >= 0)]))):
        keep = hold & (pose == pose_id)
        selected = {}
        for key, value in values.items():
            selected[key] = value[keep] if value.ndim and len(value) == count else value
        path = output / f"{prefix}_pose{pose_id:02d}.npz"
        np.savez_compressed(path, **selected)
        rows.append({"pose_id": pose_id, "path": str(path), "samples": int(keep.sum())})
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--payload", type=Path, required=True)
    parser.add_argument("--empty", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = split(args.payload, args.output / "payload", "payload")
    empty = split(args.empty, args.output / "empty", "baseline")
    by_empty = {row["pose_id"]: row for row in empty}
    pairs = [{"pose_id": row["pose_id"], "payload_path": row["path"],
              "empty_path": by_empty[row["pose_id"]]["path"]}
             for row in payload if row["pose_id"] in by_empty]
    (args.output / "pairs.json").write_text(json.dumps(pairs, indent=2))
    print(json.dumps({"pairs": len(pairs), "output": str(args.output)}))


if __name__ == "__main__":
    main()
