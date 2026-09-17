"""Isaac Sim plant for the independent post-placement scan-station skill.

This process is intentionally separate from ``sim_real2sim.py`` and the
frozen grasp executor.  It places the selected Arena asset on a clean station,
parks the robot, and exposes only camera-pose/capture commands over HTTP.
"""
from __future__ import annotations

import argparse
import json
import queue
import threading
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser()
p.add_argument("--gpu", type=int, default=3)
p.add_argument("--port", type=int, default=18930)
p.add_argument("--target", default="soup")
p.add_argument("--seed", type=int, default=1030)
p.add_argument("--station-x", type=float, default=.70)
p.add_argument("--station-y", type=float, default=.25)
p.add_argument("--station-z", type=float, default=None)
p.add_argument("--robot-stage-x", type=float, default=-3.00)
p.add_argument("--robot-stage-y", type=float, default=0.0)
a = p.parse_args()

ROOT = Path(__file__).resolve().parents[1]
# arena_scene reads these during import.  The scene is used only to load the
# existing Arena table/object assets; no GT pose is exported to the estimator.
import os
os.environ["FR3_ARENA_TARGET"] = a.target
os.environ["CALIBRATION_SCENE_SEED"] = str(a.seed)
os.environ.setdefault("FR3_ASSET_DIRECTORY", str(ROOT / "assets/arena_complex"))

from isaacsim import SimulationApp

app = SimulationApp({
    "headless": True,
    "active_gpu": a.gpu,
    "physics_gpu": a.gpu,
    "multi_gpu": False,
})

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim, SingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from isaacsim.core.api.materials import PhysicsMaterial
from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom, Gf
import omni.usd

import arena_scene

DT = 1 / 240.0
world = World(
    stage_units_in_meters=1.0,
    physics_dt=DT,
    rendering_dt=1 / 30.0,
    backend="numpy",
    device="cpu",
)
world.scene.add_default_ground_plane(z_position=-.76)
mat = PhysicsMaterial(
    # arena_scene.table/spawn bind this shared physics material by path.
    "/World/grasp_material",
    static_friction=.7,
    dynamic_friction=.65,
    restitution=0.0,
)
box = arena_scene.table(world, omni.usd.get_context().get_stage(), mat)
stage = omni.usd.get_context().get_stage()

extent = np.asarray(arena_scene.INVENTORY[a.target]["bounds"], dtype=float)
half_height = float(-extent[0, 2])
station_z = float(a.station_z) if a.station_z is not None else half_height + .001
station = np.array([a.station_x, a.station_y, station_z], dtype=float)
box.set_world_pose(station, [1.0, 0.0, 0.0, 0.0])
box.set_linear_velocity([0.0, 0.0, 0.0])
box.set_angular_velocity([0.0, 0.0, 0.0])

# Reuse the generated FR3 USD from the existing plant, importing it only if
# this checkout has not generated it yet.
asset_dir = ROOT / "assets"
asset_dir.mkdir(exist_ok=True)
asset_path = asset_dir / "asset_path.txt"
if asset_path.exists() and Path(asset_path.read_text().strip()).exists():
    robot_usd = asset_path.read_text().strip()
else:
    robot_usd = URDFImporter(URDFImporterConfig(
        urdf_path=str(ROOT / "config/fr3.urdf"),
        usd_path=str(asset_dir),
        fix_base=True,
        allow_self_collision=True,
        merge_fixed_joints=False,
        joint_drive_type="force",
        joint_target_type="position",
        override_joint_stiffness={".*joint[1-7]": 10000., ".*finger_joint.*": 1000.},
        override_joint_damping={".*joint[1-7]": 400., ".*finger_joint.*": 40.},
    )).import_urdf()
    asset_path.write_text(robot_usd)
from isaacsim.core.utils.stage import add_reference_to_stage
add_reference_to_stage(robot_usd, "/World/FR3")
stage = omni.usd.get_context().get_stage()
# The physical station procedure has already released the payload.  Stage the
# fixed-base robot at a remote, collision-free parking bay so neither the arm
# nor the hand enters the close-range scan frustum.  This is a scene-level
# station placement, separate from the frozen grasp executor.
UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/FR3")).SetTranslate(
    Gf.Vec3d(float(a.robot_stage_x), float(a.robot_stage_y), 0.0)
)
robot = world.scene.add(SingleArticulation("/World/FR3", name="fr3"))
# Isaac Sim resolves the articulation DOF names during ``World.reset``.  Read
# them only after initialization; querying before reset returns ``None`` on
# some Isaac Sim builds and would make the station process exit before its
# HTTP endpoint becomes ready.
world.reset()
camera = None
names = robot.dof_names
arm_indices = [names.index(f"fr3_joint{i}") for i in range(1, 8)]
finger_indices = [names.index("fr3_finger_joint1"), names.index("fr3_finger_joint2")]
tcp_paths = [str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "fr3_hand_tcp"]
if len(tcp_paths) != 1:
    raise RuntimeError(f"fr3_hand_tcp not found: {tcp_paths}")
tcp = SingleXFormPrim(tcp_paths[0])

# This is a parking pose, not a grasp pose.  It keeps the arm on the far side
# of the station while the near-field camera circles the object.
# Fold the arm beside the fixed base, below the near-field camera frustum.
# The previous overhead pose put the hand in the upper part of several RGB
# frames even though the target bbox itself was clean.  This pose is chosen
# from the robot joint limits and is checked by the per-frame instance mask.
PARK = np.array([0.0, 1.50, 0.0, -2.00, 0.0, 1.00, .785398], dtype=float)
HOME_Q = np.zeros(len(names), dtype=float)
HOME_Q[arm_indices] = PARK
HOME_Q[finger_indices] = .04
robot.set_joint_positions(HOME_Q)
robot.set_joint_velocities(np.zeros(len(names)))
robot.apply_action(ArticulationAction(joint_positions=HOME_Q))

camera = Camera(
    "/World/scan_station_camera",
    position=[station[0] + .30, station[1], station[2] + .08],
    resolution=(640, 480),
    frequency=30,
)
camera.set_focal_length(1.8)
camera.set_horizontal_aperture(1.44)
camera.set_vertical_aperture(1.08)
camera.set_clipping_range(.03, 2.0)

camera.initialize()
camera.add_distance_to_image_plane_to_frame()
camera.add_instance_id_segmentation_to_frame()
for _ in range(480):
    robot.set_joint_positions(HOME_Q)
    robot.set_joint_velocities(np.zeros(len(names)))
    robot.apply_action(ArticulationAction(joint_positions=HOME_Q))
    world.step(render=(_ % 8 == 0))
print("PARK_ACTUAL", json.dumps({
    "q": np.asarray(robot.get_joint_positions()).tolist(),
    "tcp_position": np.asarray(tcp.get_world_pose()[0]).tolist(),
}), flush=True)

T_B_CAMERA = np.eye(4)
tick = 480
commands: queue.Queue = queue.Queue()
results: dict[str, dict] = {}
lock = threading.Lock()
active_camera_pose = None


def _camera_rotation(position, target):
    z = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
    z /= max(float(np.linalg.norm(z)), 1e-12)
    up = np.array([0., 0., 1.])
    if abs(float(np.dot(z, up))) > .98:
        up = np.array([0., 1., 0.])
    x = np.cross(z, up)
    x /= max(float(np.linalg.norm(x)), 1e-12)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _pose_matrix(position, quaternion_wxyz):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(np.roll(np.asarray(quaternion_wxyz), -1)).as_matrix()
    out[:3, 3] = np.asarray(position, dtype=float)
    return out


def _station_pose():
    pos, quat = box.get_world_pose()
    return _pose_matrix(pos, quat)


def _capture_station_frame(
    output: Path,
    frame_id: int,
    pass_id: int,
    ring: str,
    azimuth_deg: float,
    elevation_deg: float,
    position: np.ndarray,
    look_at: np.ndarray,
    settle_steps: int = 8,
) -> dict:
    """Move the scan camera and atomically write one RGB-D/pose tuple.

    This helper is called only by the Isaac simulation thread.  Keeping the
    camera pose update, physics settling and sensor read in one function is
    what makes the high-rate batch equivalent to the official ImageSaver
    stream: every frame has one timestamp, one mask pair and one measured
    ``T_base_camera``.
    """
    global T_B_CAMERA, tick
    output = Path(output)
    for folder in ["rgb", "depth", "masks", "masks_hand", "gripper_masks", "poses", "eval_gt"]:
        (output / folder).mkdir(parents=True, exist_ok=True)
    position = np.asarray(position, dtype=float)
    look_at = np.asarray(look_at, dtype=float)
    R = _camera_rotation(position, look_at)
    camera.set_world_pose(
        position=position,
        orientation=np.roll(Rotation.from_matrix(R).as_quat(), 1),
        camera_axes="ros",
    )
    T_B_CAMERA = np.eye(4)
    T_B_CAMERA[:3, :3] = R
    T_B_CAMERA[:3, 3] = position
    before = _station_pose()
    # Advance at the nominal 30 Hz sensor cadence, but render only the final
    # physics sub-step.  Rendering all eight 240 Hz sub-steps produces the same
    # sensor sample while making a 1800-frame run needlessly slow.
    steps = max(int(settle_steps), 1)
    for _ in range(steps - 1):
        world.step(render=False)
        tick += 1
    world.step(render=True)
    tick += 1
    after = _station_pose()
    rgba = np.asarray(camera.get_rgba())
    depth = np.asarray(camera.get_depth())
    if rgba.size == 0 or depth.size == 0:
        raise RuntimeError("RGBD_UNAVAILABLE")
    object_mask, gripper_mask = _ids_and_masks(depth.shape)
    # Isaac's first post-camera-move render can expose an empty instance
    # buffer while the RGB/depth buffers are already valid.  Retry one rendered
    # step only for that frame; later frames stay at the nominal 30 Hz cadence.
    if not object_mask.any():
        for _ in range(4):
            world.step(render=True)
            tick += 1
            rgba = np.asarray(camera.get_rgba())
            depth = np.asarray(camera.get_depth())
            object_mask, gripper_mask = _ids_and_masks(depth.shape)
            if object_mask.any():
                break
    if not object_mask.any():
        raise RuntimeError("OBJECT_INSTANCE_MASK_EMPTY")
    stem = f"{int(frame_id):06d}"
    rgb_path = output / "rgb" / f"{stem}.png"
    depth_path = output / "depth" / f"{stem}.png"
    mask_path = output / "masks" / f"{stem}.png"
    hand_path = output / "masks_hand" / f"{stem}.png"
    gripper_path = output / "gripper_masks" / f"{stem}.png"
    import cv2

    cv2.imwrite(str(rgb_path), rgba[..., :3][..., [2, 1, 0]])
    depth_mm = np.where(
        np.isfinite(depth) & (depth > 0),
        np.clip(np.rint(depth * 1000), 0, 65535),
        0,
    ).astype(np.uint16)
    cv2.imwrite(str(depth_path), depth_mm)
    cv2.imwrite(str(mask_path), object_mask.astype(np.uint8) * 255)
    cv2.imwrite(str(hand_path), gripper_mask.astype(np.uint8) * 255)
    cv2.imwrite(str(gripper_path), gripper_mask.astype(np.uint8) * 255)
    K = camera.get_intrinsics_matrix()
    np.savetxt(output / "cam_K.txt", K)
    tcp_pos, tcp_quat = tcp.get_world_pose()
    T_B_TCP = _pose_matrix(tcp_pos, tcp_quat)
    # This simulator pose is isolated under eval_gt and is never consumed by
    # BundleSDF, metric alignment or payload identification.
    np.savetxt(output / "eval_gt" / f"{stem}_T_B_object.txt", after, fmt="%.10f")
    record = {
        "ok": True,
        "timestamp_s": tick * DT,
        "rgb": str(rgb_path),
        "depth": str(depth_path),
        "object_mask": str(mask_path),
        "gripper_mask": str(gripper_path),
        "T_B_camera": T_B_CAMERA.tolist(),
        "T_base_camera": T_B_CAMERA.tolist(),
        "T_B_TCP": T_B_TCP.tolist(),
        "intrinsics": K.tolist(),
        "pass_id": int(pass_id),
        "frame_id": int(frame_id),
        "ring": str(ring),
        "azimuth_deg": float(azimuth_deg),
        "elevation_deg": float(elevation_deg),
        "target_motion_during_capture_m": float(np.linalg.norm(after[:3, 3] - before[:3, 3])),
        "target_rotation_during_capture_deg": float(np.degrees(Rotation.from_matrix(before[:3, :3].T @ after[:3, :3]).magnitude())),
        "object_pixels": int(object_mask.sum()),
        "gripper_pixels": int(gripper_mask.sum()),
        "gripper_excluded_from_estimator": True,
    }
    (output / "poses" / f"{stem}.json").write_text(json.dumps(record, indent=2))
    return record


def _ids_and_masks(depth_shape):
    seg = camera.get_current_frame().get("instance_id_segmentation")
    if seg is None:
        raise RuntimeError("instance segmentation unavailable")
    raw = np.asarray(seg["data"]).reshape(depth_shape)
    labels = seg["info"]["idToLabels"]
    object_ids = [
        int(k) for k, v in labels.items()
        if str(v) == "/World/box" or str(v).startswith("/World/box/")
    ]
    robot_ids = [
        int(k) for k, v in labels.items()
        if str(v).startswith("/World/FR3/")
    ]
    if not object_ids:
        raise RuntimeError(f"target instance id missing: {labels}")
    return np.isin(raw, object_ids), np.isin(raw, robot_ids)


class Handler(__import__("http.server").server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        with lock:
            # Never call a PhysX-backed prim accessor from the HTTP thread.
            # Isaac requires all scene reads/writes to be serialized with the
            # simulation thread.  The runner only needs readiness and command
            # results, so expose the immutable station pose and the latest
            # camera matrix maintained by the simulation loop instead.
            payload = {
                "ready": True,
                "t": tick * DT,
                "target": a.target,
                "box": station.tolist(),
                "box_quat": [1.0, 0.0, 0.0, 0.0],
                "camera": np.asarray(T_B_CAMERA).tolist(),
                "results": results.copy(),
                "busy": bool(not commands.empty()),
            }
            if self.path == "/control":
                pass
            elif self.path == "/calibration":
                payload = {k: payload[k] for k in ["ready", "t", "target"]}
        self.send_response(200)
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode())

    def do_POST(self):
        try:
            data = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            token = str(time.time_ns())
            commands.put((token, data))
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps({"id": token}).encode())
        except Exception as exc:
            self.send_response(400)
            self.end_headers()
            self.wfile.write(str(exc).encode())


from http.server import ThreadingHTTPServer
server = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
print("SCAN_STATION_READY", json.dumps({"target": a.target, "port": a.port}), flush=True)

try:
    while app.is_running():
        while not commands.empty():
            token, command = commands.get_nowait()
            try:
                op = command.get("op")
                if op == "scan_station_prepare":
                    # The placement state is explicit.  The station runner
                    # begins after release; no force/grasp command is exposed.
                    box.set_world_pose(station, [1., 0., 0., 0.])
                    box.set_linear_velocity([0., 0., 0.])
                    box.set_angular_velocity([0., 0., 0.])
                    robot.set_joint_positions(HOME_Q)
                    robot.set_joint_velocities(np.zeros(len(names)))
                    robot.apply_action(ArticulationAction(joint_positions=HOME_Q))
                    for _ in range(480):
                        world.step(render=(_ % 8 == 0))
                        tick += 1
                    p0, q0 = box.get_world_pose()
                    results[token] = {
                        "ok": True,
                        "phase": "PLACED_RELEASED_ARM_PARKED",
                        "target_pose_base": _pose_matrix(p0, q0).tolist(),
                        "settle_seconds": 2.0,
                        "station": station.tolist(),
                    }
                elif op == "scan_station_view":
                    record = _capture_station_frame(
                        Path(command["output"]),
                        int(command["frame_id"]),
                        int(command["pass_id"]),
                        str(command.get("ring", "unknown")),
                        float(command.get("azimuth_deg", 0.0)),
                        float(command.get("elevation_deg", 0.0)),
                        np.asarray(command["position"], dtype=float),
                        np.asarray(command["look_at"], dtype=float),
                        settle_steps=12,
                    )
                    results[token] = record
                elif op == "scan_station_batch":
                    # One command runs the complete 30 Hz trajectory on the
                    # Isaac simulation thread.  This mirrors the official
                    # ImageSaver stream and avoids 1800 HTTP round trips.
                    output = Path(command["output"])
                    n = int(command["num_frames"])
                    horizontal_n = int(command["horizontal_frames"])
                    if n < 8 or not 4 <= horizontal_n <= n - 4:
                        raise ValueError("invalid continuous scan frame split")
                    center = np.asarray(command.get("center", station), dtype=float)
                    radius = float(command.get("radius_m", 0.30))
                    horizontal_elevation = float(command.get("horizontal_elevation_deg", 14.0))
                    high_elevation = float(command.get("high_elevation_deg", 34.0))
                    high_offset = float(command.get("high_phase_offset_deg", 0.4))
                    records = []
                    for i in range(n):
                        if i < horizontal_n:
                            ring = "horizontal"
                            pass_id = 0
                            j = i
                            count = horizontal_n
                            elevation = horizontal_elevation
                            phase = 0.0
                        else:
                            ring = "high"
                            pass_id = 1
                            j = i - horizontal_n
                            count = n - horizontal_n
                            elevation = high_elevation
                            phase = high_offset
                        azimuth = 360.0 * float(j) / float(count) + phase
                        er = np.deg2rad(elevation)
                        ar = np.deg2rad(azimuth)
                        horizontal = radius * np.cos(er)
                        position = np.array([
                            center[0] + horizontal * np.cos(ar),
                            center[1] + horizontal * np.sin(ar),
                            center[2] + radius * np.sin(er),
                        ])
                        records.append(_capture_station_frame(
                            output, i, pass_id, ring, azimuth, elevation,
                            position, center, settle_steps=8,
                        ))
                    results[token] = {
                        "ok": True,
                        "frames": len(records),
                        "horizontal_frames": horizontal_n,
                        "high_frames": n - horizontal_n,
                        "nominal_fps": 30.0,
                        "capture_start_t": records[0]["timestamp_s"] if records else None,
                        "capture_end_t": records[-1]["timestamp_s"] if records else None,
                    }
                elif op == "stop":
                    results[token] = {"ok": True}
                    app.close()
                else:
                    raise ValueError(f"unknown scan-station operation: {op}")
            except Exception as exc:
                results[token] = {"ok": False, "reason": str(exc)}
        # Keep the parked articulation deterministic while the camera moves;
        # position drives alone can ring at the joint limits and reintroduce a
        # moving hand into a near-field frame.
        robot.set_joint_positions(HOME_Q)
        robot.set_joint_velocities(np.zeros(len(names)))
        robot.apply_action(ArticulationAction(joint_positions=HOME_Q))
        world.step(render=False)
        tick += 1
finally:
    server.shutdown()
    app.close()
