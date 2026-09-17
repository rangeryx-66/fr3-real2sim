"""Export geometry for the final AnyGrasp-first frozen validation pool.

The list is frozen before grasp execution.  Structural eligibility uses only
USD rigid-body layout and dimensions, never grasp outcomes.
"""
import argparse
import hashlib
import json
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument("--gpu", type=int, default=5)
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "active_gpu": args.gpu, "physics_gpu": args.gpu, "multi_gpu": False})

import numpy as np
import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom, UsdPhysics


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets/family_final_v1"
OUT.mkdir(parents=True, exist_ok=True)
PREFIX = (
    "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/"
    "IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/"
)
CANDIDATES = [
    "hot3d/megaphone",
    "hot3d/potato_masher",
    "hot3d/storage_box",
    "ycb/coffee_can",
    "ycb/scissors",
    "vomp/milkjug_a01/milkjug_a01",
    "vomp/utilityjug_a03/utilityjug_a03",
    "vomp/whitepackerbottle_a01/whitepackerbottle_a01",
]

old = json.loads((ROOT / "assets/arena_complex/inventory.json").read_text())
inventory = {"table": old["table"]}
stage = omni.usd.get_context().get_stage()
(OUT / "candidate_pool.json").write_text(
    json.dumps(
        {
            "assets": CANDIDATES,
            "selection_time_rule": "USD root/body validity and tabletop dimensions only; before grasp trials",
            "excluded_all_prior_targets": [
                "mustard", "raisin", "hidden_tuna", "bowl", "banana", "sugar", "soup", "mug",
                "hope/ketchup_bottle", "hope/mayonnaise_bottle", "hope/milk_carton", "hope/yogurt_cup",
                "hot3d/wooden_bowl", "hot3d/clay_plates", "hot3d/ceramic_mug",
                "fruits_veggies/avocado01", "ycb/spam_can", "fruits_veggies/red_onion",
            ],
        },
        indent=2,
    )
)

for relative in CANDIDATES:
    name = relative.replace("/", "__")
    path = PREFIX + relative + ".usd"
    prim = add_reference_to_stage(path, "/World/" + name)
    for item in list(Usd.PrimRange(prim)):
        if item.IsInstance():
            item.SetInstanceable(False)
    vertices, triangles, bodies, colliders = [], [], [], []
    for item in Usd.PrimRange(prim):
        if item.HasAPI(UsdPhysics.RigidBodyAPI):
            mass = UsdPhysics.MassAPI(item)
            bodies.append(
                {
                    "path": str(item.GetPath()),
                    "mass_kg": mass.GetMassAttr().Get(),
                    "center_of_mass": None
                    if mass.GetCenterOfMassAttr().Get() is None
                    else list(mass.GetCenterOfMassAttr().Get()),
                }
            )
        if item.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append(
                {
                    "path": str(item.GetPath()),
                    "approximation": UsdPhysics.MeshCollisionAPI(item).GetApproximationAttr().Get(),
                }
            )
        if not item.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(item).ComputeVisibility() == "invisible":
            continue
        mesh = UsdGeom.Mesh(item)
        points = mesh.GetPointsAttr().Get()
        if points is None:
            continue
        transform = UsdGeom.Xformable(item).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        offset = len(vertices)
        vertices.extend([list(transform.Transform(Gf.Vec3d(*map(float, point)))) for point in points])
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        start = 0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face = indices[start : start + count]
            start += count
            triangles.extend([[offset + face[0], offset + face[j], offset + face[j + 1]] for j in range(1, count - 1)])
    reason = None
    if not vertices or not colliders:
        reason = "MISSING_VISIBLE_MESH_OR_COLLIDER"
    elif len(bodies) != 1 or bodies[0]["path"] != "/World/" + name:
        reason = "NESTED_OR_MULTIPLE_RIGID_ROOT_UNSUPPORTED_BY_EXISTING_BRIDGE"
    if reason:
        (OUT / (name + "_excluded.json")).write_text(
            json.dumps({"reason": reason, "usd_path": path, "bodies": bodies, "colliders": colliders}, indent=2)
        )
        print("ASSET_EXCLUDED", name, reason, flush=True)
        continue
    vertices = np.asarray(vertices)
    mesh_path = OUT / (name + "_mesh.npz")
    np.savez_compressed(mesh_path, vertices=vertices, triangles=triangles)
    inventory[name] = {
        "usd_path": path,
        "registry_relative": relative,
        "scale": [1, 1, 1],
        "bounds": [vertices.min(0).tolist(), vertices.max(0).tolist()],
        "bodies": bodies,
        "colliders": colliders,
        "vertices": len(vertices),
        "triangles": len(triangles),
        "mesh_sha256": hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
    }
    (OUT / "inventory.json").write_text(json.dumps(inventory, indent=2))
    print("ASSET_READY", name, "dimensions", np.ptp(vertices, axis=0), flush=True)

app.close()
