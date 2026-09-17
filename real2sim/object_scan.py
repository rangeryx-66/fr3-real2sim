"""MoveIt checked multi-view scan after a frozen executor reports a stable grasp."""
from __future__ import annotations
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from .safety import require_free_space_stable


SCAN_OFFSETS_DEG = (
    (0, 0, 0), (0, 0, 30), (0, 0, 60), (0, 0, 90),
    (22, 0, 90), (22, 0, 60), (22, 0, 30), (22, 0, 0),
    (-22, 0, 0), (-22, 0, 30), (-22, 0, 60), (-22, 0, 90),
)
# The second grasp is approximately 90 degrees around the object's support axis.
# Reusing the same safe wrist envelope then produces rotations about a different
# object-fixed axis and changes which surface is hidden by the fingers.
ORTHOGONAL_PASS_OFFSETS_DEG = SCAN_OFFSETS_DEG


def continuous_offsets(step_deg=10, tilt_deg=22):
    """Dense, high-overlap serpentine sampling inside the proven wrist envelope."""
    yaw=list(range(0,91,int(step_deg)))
    return tuple([(0,0,y) for y in yaw]
                 + [(tilt_deg,0,y) for y in reversed(yaw)]
                 + [(-tilt_deg,0,y) for y in yaw])


CONTINUOUS_SCAN_OFFSETS_DEG = continuous_offsets()
SCAN_CENTER_B_M = np.array([.50, 0., .32])


def held_scan_offsets(keyframes: int = 90, pass_id: int = 0,
                      tilt_deg: float = 16.0) -> tuple[tuple[float, float, float], ...]:
    """Continuous held-object display trajectory with high-overlap samples.

    The object is physically carried by the hand while the TCP follows these
    relative rotations.  The small, deterministic tilt component exposes upper
    and lower rim surfaces without introducing a category-specific trajectory.
    ``pass_id`` only changes the phase of the tilt wave; the second pass is
    entered after a real place/regrasp and therefore changes the hand occlusion.
    """
    if keyframes < 8:
        raise ValueError("held scan needs at least eight keyframes")
    phase = 0.0 if int(pass_id) == 0 else np.pi / 2.0
    result = []
    for i in range(int(keyframes)):
        a = 2.0 * np.pi * float(i) / float(keyframes)
        yaw = 360.0 * float(i) / float(keyframes)
        # Two incommensurate low-frequency components keep neighbouring views
        # close while making the two rings see different rim/end-cap portions.
        rx = float(tilt_deg * np.sin(a + phase))
        ry = float(.75 * tilt_deg * np.sin(2.0 * a + phase))
        result.append((rx, ry, yaw))
    return tuple(result)


class ObjectScan:
    """Uses only public methods of the frozen backend; it never changes grasp logic."""

    def __init__(self, backend, plant, output: Path, object_name: str):
        self.backend = backend
        self.plant = plant
        self.output = Path(output)
        self.object_name = object_name
        self.output.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _T(position, quaternion_wxyz):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(np.roll(quaternion_wxyz, -1)).as_matrix()
        T[:3, 3] = position
        return T

    def _guard(self, wait_s: float = 8.0):
        """Wait for a post-motion free-space stability sample without relaxing safety.

        The support monitor deliberately reports ``FREE_SPACE_SETTLE`` for the
        short interval immediately after a Cartesian scan move.  A short fixed
        timeout was occasionally too small for a held payload to settle after
        a wrist rotation, causing a false scan abort even with bilateral
        contact and millimetre-scale relative drift.  We wait longer for that
        transient state; the support monitor's contact-loss, drop, and severe
        drift limits remain terminal.
        contact loss/drop and every other non-stable state remain terminal.
        """
        deadline = time.monotonic() + max(0.0, float(wait_s))
        answer = None
        while True:
            answer = self.plant.command({"op": "payload_guard"})
            support = answer.get("support") or {}
            category = support.get("category")
            if answer.get("ok") and answer.get("free_space_stable"):
                return answer
            # A held scan can settle into a new, constant orientation after a
            # wrist rotation.  The shared payload gate intentionally treats
            # the frozen 15 degree cumulative bound as a conservative lift
            # guard, but applying that bound as a hard stop to a scan loses
            # valid views even when the payload is still clear, bilateral and
            # motionless.  Keep this exception local to ObjectScan: contact
            # loss, drop, and non-converged translation remain terminal, and
            # the returned record carries explicit provenance for audit.
            if category == "ROTATIONAL_INSTABILITY":
                flags = support.get("flags") or {}
                metrics = support.get("free_metrics") or {}
                windows = metrics.get("windows") or {}
                w200 = windows.get("200ms") or {}
                settled = (
                    bool(support.get("currently_clear"))
                    and bool(flags.get("PICKED"))
                    and bool(flags.get("RETAINED", True))
                    and bool(flags.get("CLEAR_TABLE"))
                    and bool(support.get("bilateral"))
                    and float(support.get("max_contact_gap_s", 1.0)) <= .05
                    and float(metrics.get("max_cumulative_translation_m", 1.0)) <= .010
                    and float(metrics.get("max_cumulative_rotation_deg", 1e9)) <= 20.0
                    and float(metrics.get("duration_s", 0.0)) >= .30
                    and all(
                        float((windows.get(k) or {}).get(
                            "relative_translation_velocity_m_s", 1.0
                        )) <= .00025
                        and float((windows.get(k) or {}).get(
                            "relative_angular_velocity_deg_s", 1.0
                        )) <= .5
                        for k in ("100ms", "200ms", "300ms")
                    )
                )
                if settled:
                    answer["scan_settled_reorientation"] = True
                    answer["scan_settled_reorientation_reason"] = (
                        "bilateral_clear_low_velocity_static_orientation"
                    )
                    return answer
            if category in {"DROP", "CONTACT_LOSS"}:
                break
            if time.monotonic() >= deadline:
                break
            self.plant.settle(.1)
        self.plant.command({"op": "stop"})
        raise RuntimeError("SCAN_SAFETY_ABORT: " + json.dumps(answer))

    def run_pass(self, grasp_result: dict, pass_id: int = 0, offsets=None,
                 profile="sparse", frames_per_pose: int = 1):
        if offsets is None:
            offsets = CONTINUOUS_SCAN_OFFSETS_DEG if profile == "continuous" else (SCAN_OFFSETS_DEG if pass_id == 0 else ORTHOGONAL_PASS_OFFSETS_DEG)
        frames_per_pose = int(frames_per_pose)
        if frames_per_pose < 1:
            raise ValueError("frames_per_pose must be positive")
        require_free_space_stable(grasp_result)
        state = self.plant.state()
        H0 = self._T(state["tcp"], state["tcp_quat"])
        O = self._T(state["box"], state["box_quat"])
        T_TCP_object = np.linalg.inv(H0) @ O
        # Bring the actual object, rather than TCP, to the calibrated scan station.
        station_object=O.copy();station_object[:3,3]=SCAN_CENTER_B_M
        station_tcp=station_object@np.linalg.inv(T_TCP_object)
        self.backend.attach_actual(H0,O);self.backend.support_contact(False)
        current=self.backend.measured();goal_state=self.backend.ik(station_tcp,current)
        self.backend.execute_slow(self.backend.plan(current,goal_state),"OBJECT_SCAN_PATH_FAIL",2.0)
        self.plant.settle(.2);self._guard()
        state=self.plant.state();H0=self._T(state["tcp"],state["tcp_quat"]);O=self._T(state["box"],state["box_quat"])
        T_TCP_object=np.linalg.inv(H0)@O
        records = []
        frame_counter = 0
        for index, rpy in enumerate(offsets):
            self._guard()
            R_delta = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
            goal = H0.copy()
            goal[:3, :3] = H0[:3, :3] @ R_delta
            # Rotate about the held object's center instead of sweeping it around TCP.
            goal[:3, 3] = O[:3, 3] - goal[:3, :3] @ T_TCP_object[:3, 3]
            now = self.plant.state()
            H = self._T(now["tcp"], now["tcp_quat"])
            actual_O = self._T(now["box"], now["box_quat"])
            self.backend.attach_actual(H, actual_O)
            self.backend.support_contact(False)
            position_error=np.linalg.norm(goal[:3,3]-H[:3,3])
            rotation_error=Rotation.from_matrix(H[:3,:3].T@goal[:3,:3]).magnitude()
            if position_error>1e-5 or rotation_error>1e-4:
                current=self.backend.measured()
                goal_state=self.backend.ik(goal,current)
                trajectory=self.backend.plan(current,goal_state)
                # Held-object display uses a slower, bounded execution so the
                # simulated FR3 controller can track the small SE(3) changes
                # after payload contact.  This is scan motion only; the
                # frozen grasp and lift timing are unchanged.
                self.backend.execute_slow(
                    trajectory, "OBJECT_SCAN_PATH_FAIL",
                    5.0 if profile == "held" else 1.2,
                )
                self.plant.settle(0.04 if profile == "continuous" else 0.15)
            self._guard()
            # The vendor ImageSaver records a continuous stream while the
            # object is displayed.  Keeping a short dwell at each keyframe
            # gives the same high-overlap RGB-D evidence without changing the
            # frozen grasp or planner.
            for dwell_index in range(frames_per_pose):
                self._guard()
                capture = self.plant.command({
                    "op": "real2sim_capture", "output": str(self.output),
                    "pass_id": pass_id, "frame_id": frame_counter,
                })
                if not capture.get("ok"):
                    raise RuntimeError("OBJECT_SCAN_CAPTURE_FAIL: " + str(capture))
                capture["scan_keyframe_index"] = int(index)
                capture["scan_dwell_index"] = int(dwell_index)
                capture["scan_frames_per_keyframe"] = int(frames_per_pose)
                records.append(capture)
                frame_counter += 1
        return records

    @staticmethod
    def _matrix(value):
        """Accept the matrix and position/quaternion forms in old handoffs."""
        if isinstance(value, dict):
            if "T_B_target" in value:
                return ObjectScan._matrix(value["T_B_target"])
            position = value.get("position")
            quat = value.get("quaternion_wxyz")
            if position is not None and quat is not None:
                return ObjectScan._T(position, quat)
        return np.asarray(value, dtype=float).reshape(4, 4)

    def run_held_regrasp(self, grasp_result: dict, keyframes: int = 90,
                         frames_per_pose: int = 10,
                         regrasp_angles_deg=(30.0, -30.0, 45.0, -45.0,
                                             60.0, -60.0, 90.0, -90.0,
                                             180.0, 0.0)):
        """Run two official-like held-object passes with a physical regrasp.

        The first pass starts from the frozen executor's stable handoff.  The
        object is then lowered onto the real table, released, and reacquired
        from a deterministic target-relative alternative.  No target pose is
        teleported or welded; MoveIt and the existing bilateral-force/stability
        checks decide whether the second grasp is usable.
        """
        first = self.run_pass(
            grasp_result, pass_id=0,
            offsets=held_scan_offsets(keyframes, pass_id=0),
            profile="held", frames_per_pose=frames_per_pose,
        )
        info = {
            "schema": "fr3_held_object_regrasp_scan/v1",
            "keyframes_per_pass": int(keyframes),
            "frames_per_keyframe": int(frames_per_pose),
            "first_pass_frames": len(first),
            "regrasp_attempts": [],
            "physical_regrasp": True,
            "gt_used_for_execution": False,
        }

        # Recover the target-relative frozen grasp transform before releasing.
        # It is used only to command a physically reachable reference return and
        # to express subsequent regrasp candidates in the measured object frame.
        initial_target = self._matrix(grasp_result.get("initial_target"))
        attempts = grasp_result.get("attempts") or []
        if not attempts:
            raise RuntimeError("HELD_SCAN_REGRASP_NO_PARENT_GRASP")
        parent_attempt = attempts[0]
        parent_tcp = parent_attempt.get("T_B_TCP_commanded") or parent_attempt.get("T_B_TCP")
        if parent_tcp is None:
            parent_tcp = grasp_result.get("parent_T_B_TCP")
        if parent_tcp is None:
            raise RuntimeError("HELD_SCAN_REGRASP_PARENT_TCP_MISSING")
        parent_tcp = self._matrix(parent_tcp)
        target_to_tcp = np.linalg.inv(initial_target) @ parent_tcp

        # A scan trajectory intentionally tilts the payload.  Return it to the
        # original, calibrated orientation while still held before lowering.
        # This is a real MoveIt motion with the measured object pose; no target
        # transform is assigned to the simulator.
        state = self.plant.state()
        current_object = self._T(state["box"], state["box_quat"])
        neutral_object = current_object.copy()
        neutral_object[:3, :3] = initial_target[:3, :3]
        neutral_tcp = neutral_object @ target_to_tcp
        current_tcp = self._T(state["tcp"], state["tcp_quat"])
        if (np.linalg.norm(neutral_tcp[:3, 3] - current_tcp[:3, 3]) > 1e-5 or
                Rotation.from_matrix(current_tcp[:3, :3].T @ neutral_tcp[:3, :3]).magnitude() > 1e-4):
            self.backend.attach_actual(current_tcp, current_object)
            self.backend.support_contact(False)
            current_joints = self.backend.measured()
            neutral_joints = self.backend.ik(neutral_tcp, current_joints)
            neutral_plan = self.backend.plan(current_joints, neutral_joints)
            self.backend.execute_slow(neutral_plan, "HELD_SCAN_NEUTRAL_RETURN", 2.0)
            self.plant.settle(.2)
            self._guard()

        # The stable record is retained only as a provenance guard.  The
        # lowering, release, and retreat are executed by the existing backend.
        release_record = {
            "attempt": 1,
            "stage": "HELD_SCAN_REGRASP_LOWER",
            "stability": {"passed": True, "category": "STABLE"},
        }
        if not hasattr(self.backend, "safe_release"):
            raise RuntimeError("HELD_SCAN_REGRASP_BACKEND_UNAVAILABLE")
        try:
            released_object = self.backend.safe_release(release_record)
        except Exception as exc:
            info["regrasp_error"] = {
                "category": getattr(exc, "category", "REGRASP_RELEASE_FAIL"),
                "detail": str(exc),
            }
            (self.output / "held_regrasp.json").write_text(json.dumps(info, indent=2))
            raise
        info["release"] = release_record
        info["released_target_pose"] = np.asarray(released_object).tolist()

        second = None
        for angle_index, angle_deg in enumerate(tuple(regrasp_angles_deg)):
            state = self.plant.state()
            current_object = self._T(state["box"], state["box_quat"])
            delta = np.eye(4)
            delta[:3, :3] = Rotation.from_euler("z", float(angle_deg), degrees=True).as_matrix()
            candidate_tcp = current_object @ delta @ target_to_tcp
            parent = {
                "rank": 0,
                "score": parent_attempt.get("raw_score", parent_attempt.get("score")),
                "T_B_TCP": candidate_tcp.tolist(),
            }
            label = f"held_regrasp_{len(info['regrasp_attempts']):02d}"
            attempt_info = {
                "angle_deg": float(angle_deg),
                "candidate_order": int(angle_index),
                "T_B_TCP": candidate_tcp.tolist(),
                "parent_rank": parent_attempt.get("parent_rank", 0),
            }
            try:
                rows = self.backend.geometry_worker(
                    {"operation": "evaluate", "T_B_target": current_object.tolist(),
                     "parents": [parent]}, label,
                )
                metric = rows[0]
                metric.setdefault("refinement_id", len(info["regrasp_attempts"]))
                attempt_info["geometry_passed"] = bool(metric.get("geometry_passed"))
                if not metric.get("geometry_passed"):
                    attempt_info["category"] = "INSUFFICIENT_PAD_OVERLAP"
                    info["regrasp_attempts"].append(attempt_info)
                    continue
                detail, plan = self.backend.plan_candidate(candidate_tcp, current_object, metric)
                attempt_info["moveit"] = detail
                if plan is None:
                    attempt_info["category"] = detail.get("category", "NO_PLAN")
                    info["regrasp_attempts"].append(attempt_info)
                    continue
                record = {
                    "attempt": len(info["regrasp_attempts"]) + 2,
                    "stage": "HELD_SCAN_REGRASP",
                    "parent_rank": parent_attempt.get("parent_rank", 0),
                    "raw_score": parent_attempt.get("raw_score", parent_attempt.get("score")),
                    "refinement_id": metric.get("refinement_id", 0),
                    "offset_translation_TCP_m": [0.0, 0.0, 0.0],
                    "offset_rotation_TCP_deg": [0.0, 0.0, float(angle_deg)],
                    "torque": metric.get("torque"),
                    "mesh_hand_geometry": metric.get("mesh_hand_geometry"),
                }
                answer = self.backend.attempt(metric, plan, record)
                attempt_info["record"] = record
                attempt_info["stability"] = answer
                attempt_info["category"] = answer.get("category") if isinstance(answer, dict) else "STABLE"
                info["regrasp_attempts"].append(attempt_info)
                if isinstance(answer, dict) and answer.get("passed", False):
                    second = self.run_pass(
                        grasp_result, pass_id=1,
                        offsets=held_scan_offsets(keyframes, pass_id=1),
                        profile="held", frames_per_pose=frames_per_pose,
                    )
                    break
            except Exception as exc:
                attempt_info["category"] = getattr(exc, "category", "REGRASP_EXECUTION_FAIL")
                attempt_info["detail"] = str(exc)
                info["regrasp_attempts"].append(attempt_info)
                # Once a physical regrasp has started, a contact loss/drop is a
                # terminal safety result; do not attempt to recover by teleport.
                break

        if second is None:
            info["second_pass_frames"] = 0
            info["result"] = "REGRASP_FAILED"
            (self.output / "held_regrasp.json").write_text(json.dumps(info, indent=2))
            raise RuntimeError("HELD_SCAN_REGRASP_FAILED: " + json.dumps(info))
        info["second_pass_frames"] = len(second)
        info["result"] = "TWO_PHYSICAL_HELD_PASSES"
        (self.output / "held_regrasp.json").write_text(json.dumps(info, indent=2))
        return first, second, info

    @staticmethod
    def merge_passes(output: Path, object_name: str, passes: list[list[dict]], sources=None):
        """Merge scans after a physical place/regrasp; no pose reset is assumed."""
        from .schema import ScanFrame, ScanManifest
        manifest = ScanManifest(object_name=object_name, sources=sources or {})
        n = 0
        for pass_id, rows in enumerate(passes):
            manifest.passes.append({
                "pass_id": pass_id,
                "frames": len(rows),
                "requires_independent_stable_grasp": pass_id > 0,
                "view_relation": "physical_regrasp" if pass_id > 0 else "primary",
                "alignment_input": "RGB-D and BundleSDF tracking only; eval_gt excluded",
            })
            for row in rows:
                manifest.frames.append(ScanFrame(
                    frame_id=n, pass_id=pass_id, timestamp_s=row["timestamp_s"],
                    rgb=row["rgb"], depth=row["depth"], object_mask=row["object_mask"],
                    gripper_mask=row["gripper_mask"], T_B_camera=row["T_B_camera"],
                    T_B_TCP=row["T_B_TCP"], intrinsics=row["intrinsics"],
                ))
                n += 1
        path = Path(output) / "scan_manifest.json"
        manifest.save(path)
        return path
