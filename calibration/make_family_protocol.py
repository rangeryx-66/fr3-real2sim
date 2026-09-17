"""Freeze the family expansion A/B protocol before physical grasp trials."""
from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets/family_v1"
OLD = ROOT / "assets/arena_complex"
OUT = ROOT / "assets/family_formal"
OUT.mkdir(parents=True, exist_ok=True)

UNSEEN = [
    "vomp__plate_large__plate_large",
    "vomp__plate_small__plate_small",
    "vomp__serving_bowl__serving_bowl",
    "vomp__bin_b03__bin_b03",
    "vomp__plasticpail_a02__plasticpail_a02",
    "handal__measuring_cups",
    "handal__measuring_spoon",
    "hot3d__pitcher",
    "hot3d__coffee_pot",
    "ycb__pitcher",
]
REGRESSION = ["sugar", "soup", "banana", "mug", "mustard"]
CLUTTER = ["raisin", "soup", "mustard", "sugar", "hidden_tuna"]

new_inventory = json.loads((SOURCE / "inventory.json").read_text())
old_inventory = json.loads((OLD / "inventory.json").read_text())
inventory = {"table": old_inventory["table"]}
for name in UNSEEN:
    inventory[name] = new_inventory[name]
    shutil.copy2(SOURCE / f"{name}_mesh.npz", OUT / f"{name}_mesh.npz")
for name in sorted(set(REGRESSION + CLUTTER)):
    inventory[name] = old_inventory[name]
    shutil.copy2(OLD / f"{name}_mesh.npz", OUT / f"{name}_mesh.npz")
(OUT / "inventory.json").write_text(json.dumps(inventory, indent=2))


def upright_spec(name, center, yaw):
    bounds = np.asarray(inventory[name]["bounds"], dtype=float)
    local_center = bounds.mean(axis=0)
    rotation = Rotation.from_euler("z", yaw).as_matrix()
    root = np.asarray(center, dtype=float) - rotation @ local_center
    root[2] = -bounds[0, 2] + 0.001
    quaternion = Rotation.from_euler("z", yaw).as_quat()
    return {
        "asset": name,
        "position": root.tolist(),
        "quaternion_wxyz": np.roll(quaternion, 1).tolist(),
    }


def planar_radius(name):
    bounds = np.asarray(inventory[name]["bounds"], dtype=float)
    return float(np.linalg.norm(np.ptp(bounds, axis=0)[:2]) / 2.0)


rng = np.random.default_rng(20260914)
episodes = []
seed = 5000
for cohort, targets, repeats in [("unseen", UNSEEN, 3), ("regression", REGRESSION, 3)]:
    for target in targets:
        for repeat in range(repeats):
            target_center = np.array([rng.uniform(0.47, 0.52), rng.uniform(-0.025, 0.025), 0.0])
            target_yaw = rng.uniform(-np.pi, np.pi)
            target_spec = upright_spec(target, target_center, target_yaw)
            obstacle_names = [name for name in CLUTTER if name != target][:3]
            objects = [target_spec]
            target_radius = planar_radius(target)
            phase = rng.uniform(-0.20, 0.20)
            for index, obstacle in enumerate(obstacle_names):
                angle = phase + index * 2.0 * np.pi / 3.0
                distance = target_radius + planar_radius(obstacle) + 0.035
                center = target_center + np.array([distance * np.cos(angle), distance * np.sin(angle), 0.0])
                objects.append(upright_spec(obstacle, center, rng.uniform(-0.25, 0.25)))
            episodes.append(
                {
                    "seed": seed,
                    "cohort": cohort,
                    "repeat": repeat,
                    "target": target,
                    "objects": objects,
                    "paired_modes": ["BASELINE", "FAMILY"],
                }
            )
            seed += 1

protocol = {
    "version": "grasp_family_formal_v2",
    "created_before_physical_trials": True,
    "runtime_scene_snapshots": "per-episode diagnostic outputs; never scene inputs or sealed assets",
    "selection_rule": "ten preregistered targets selected for geometry diversity and tabletop scale before physical trials; excludes the 0.51 m bin and redundant tall/box controls; no grasp outcomes used",
    "algorithm_development_assets": ["hot3d__wooden_bowl", "hot3d__clay_plates"],
    "unseen_targets": UNSEEN,
    "regression_targets": REGRESSION,
    "scene_pairs": len(episodes),
    "physical_episodes": len(episodes) * 2,
    "random_seed": 20260914,
    "episodes": episodes,
}
(ROOT / "FAMILY_FORMAL_PROTOCOL.json").write_text(json.dumps(protocol, indent=2))

hashes = {}
for path in sorted(OUT.iterdir()):
    # *_scene.usdc files are per-episode flattened diagnostics written by the
    # simulator.  They are outputs, never scene inputs, and therefore are not
    # part of the immutable asset manifest.
    if path.is_file() and not path.name.endswith("_scene.usdc"):
        hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
(ROOT / "FAMILY_ASSET_HASHES.json").write_text(json.dumps(hashes, indent=2))
print(json.dumps({"scene_pairs": len(episodes), "physical_episodes": len(episodes) * 2, "assets": len(inventory) - 1}))
