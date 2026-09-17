"""Seal the final AnyGrasp-first fallback implementation and frozen 90-episode protocol."""
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NAMES = [
    "src/infer.py", "src/frames.py", "src/backend.py", "src/clutter_backend.py",
    "src/sim_server.py", "src/plant.py", "src/contact_trace.py", "src/arena_scene.py",
    "src/clutter_scene.py", "src/mesh_hand_geometry.py", "src/hand_geometry.py",
    "src/grasp_refinement.py", "src/grasp_family_expansion.py", "src/moveit.launch.py",
    "calibration/stability_backend.py", "calibration/sim_calibration.py",
    "calibration/hand_force_control.py", "calibration/settling_gate.py",
    "calibration/support_aware.py", "calibration/unseen_geometry.py",
    "calibration/unseen_scene.py", "calibration/sim_unseen.py",
    "calibration/unseen_backend.py", "calibration/run_unseen.py",
    "FAMILY_FINAL_PROTOCOL.json", "FAMILY_FINAL_ASSET_HASHES.json",
    "calibration/make_family_final_protocol.py",
    "config/fr3.urdf", "config/fr3.srdf", "assets/asset_path.txt",
]
paths = [ROOT / name for name in NAMES]
paths += [
    path for path in (ROOT / "assets/family_final_formal").iterdir()
    if path.is_file() and not path.name.endswith("_scene.usdc")
]
paths += list((ROOT / "franka_description/meshes").rglob("*.stl"))
robot = Path((ROOT / "assets/asset_path.txt").read_text().strip())
paths += list(robot.parent.rglob("*.usd")) + list(robot.parent.rglob("*.usda")) + list(robot.parent.rglob("*.usdc"))
sdk = Path("/data1/home/rangeryx/anygrasp_sdk/grasp_detection")
paths += list(sdk.glob("*.py")) + list(sdk.glob("*.so")) + [sdk / "log/checkpoint_detection.tar"]
seal = {}
for path in sorted(set(paths)):
    key = str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)
    seal[key] = hashlib.sha256(path.read_bytes()).hexdigest()
destination = ROOT / "FAMILY_FINAL_SEAL.json"
if destination.exists():
    raise RuntimeError("refuse to overwrite FAMILY_FINAL_SEAL.json")
destination.write_text(json.dumps(seal, indent=2))
print("SEALED", len(seal), "files", hashlib.sha256(destination.read_bytes()).hexdigest())
