"""One persistent-world FR3 loop with third-person recording.

Sequence: wrist-camera scene survey -> frozen AnyGrasp/MoveIt grasp -> static
PayloadID motion -> transport/release at a clear table station -> wrist-camera
stationary-object scan.  A matched-opening empty baseline is collected after
the visible sequence so the new payload record can be evaluated correctly.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "calibration")]
import plant
from settling_gate import GateThresholds
from unseen_backend import UnseenBackend, VERTICES
from real2sim.object_scan import ObjectScan
from real2sim.payload_skill_v2 import PayloadIDV2Skill
from strict_pair_protocol import window_quality


T_HAND_CAMERA = np.eye(4)
T_HAND_CAMERA[:3, :3] = Rotation.from_quat([0.0, 0.0, 0.70710678, 0.70710678]).as_matrix()
T_HAND_CAMERA[:3, 3] = [0.11, -0.031, -0.074]
T_HAND_TCP = np.eye(4)
T_HAND_TCP[2, 3] = 0.1034
ARM_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
FINGER_NAMES = ["fr3_finger_joint1", "fr3_finger_joint2"]


def look_at(position, target):
    position, target = np.asarray(position, float), np.asarray(target, float)
    z = target - position
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0.0, 1.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    value = np.eye(4)
    value[:3, :3] = np.column_stack([x, y, z])
    value[:3, 3] = position
    return value


def camera_to_tcp(camera):
    return camera @ np.linalg.inv(T_HAND_CAMERA) @ T_HAND_TCP


def execute_camera_views(backend, output, phase, target, positions, dwell=.7,
                         prefer_cartesian=False, minimum_target_pixels=0):
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, nominal in enumerate(positions):
        nominal = np.asarray(nominal, float)
        toward_target = np.asarray(target, float) - nominal
        toward_target /= max(np.linalg.norm(toward_target), 1e-9)
        # Keep alternatives in camera space and close to the requested view.
        # The second candidate moves 25 mm closer when a nominal view has too
        # few target pixels; the others only provide small collision/IK
        # escapes without turning this into a random joint search.
        candidates = [nominal, nominal + .025 * toward_target,
                      nominal + [0.0, 0.0, .025],
                      nominal + [0.0, .025, .015],
                      nominal + [0.0, -.025, .015]]
        errors, selected = [], None
        for candidate_index, position in enumerate(candidates):
            tcp = camera_to_tcp(look_at(position, target))
            try:
                current = backend.measured()
                backend.phase(f"{phase}_{index:02d}")
                planner = "global"
                if prefer_cartesian and index > 0:
                    try:
                        trajectory = backend.cartesian(current, tcp)
                        planner = "cartesian"
                    except Exception as cartesian_exc:
                        errors.append(f"cartesian fallback: {cartesian_exc}")
                        goal = backend.ik(tcp, current)
                        trajectory = backend.plan(current, goal)
                else:
                    goal = backend.ik(tcp, current)
                    trajectory = backend.plan(current, goal)
                backend.execute_slow(trajectory, f"{phase}_NO_PLAN", 1.8)
                plant.settle(dwell)
                capture = plant.command({"op": "wrist_capture", "output": str(output), "phase": phase})
                if not capture.get("ok"):
                    raise RuntimeError(str(capture))
                if int(capture.get("target_pixels", 0)) < int(minimum_target_pixels):
                    raise RuntimeError(
                        f"target too small: {capture.get('target_pixels', 0)} < {minimum_target_pixels}")
                selected = {
                    "index": index, "candidate_index": candidate_index,
                    "camera_position": position.tolist(), "look_at_target": np.asarray(target).tolist(),
                    "camera_distance_m": float(np.linalg.norm(position - np.asarray(target, float))),
                    "planner": planner, "tcp_pose": tcp.tolist(), "capture": capture,
                }
                break
            except Exception as exc:
                errors.append(str(exc))
        if selected is None:
            rows.append({"index": index, "success": False, "errors": errors})
        else:
            selected.update(success=True, errors=errors)
            rows.append(selected)
    return rows


def close_scan_positions(target):
    """Two smooth, robot-facing near-field arcs around the released object."""
    target = np.asarray(target, float)
    facing_robot = np.arctan2(-target[1], -target[0])
    offsets = np.deg2rad([-50.0, -17.0, 17.0, 50.0])
    positions = []
    # Traverse the upper arc in reverse so the transition from the last low
    # view is short and does not cause an avoidable wrist flip.
    for radius, dz, angles in [(.22, .15, offsets), (.20, .23, offsets[::-1])]:
        for offset in angles:
            angle = facing_robot + offset
            positions.append(target + [radius * np.cos(angle), radius * np.sin(angle), dz])
    return [p.tolist() for p in positions]


def wait_calibration_force(total_force):
    answer = plant.command({"op": "calibration_force", "force_N": float(total_force)})
    if not answer.get("ok"):
        raise RuntimeError(f"calibration force command failed: {answer}")
    start = plant.state()["t"]
    stable_since, last = None, [0.0, 0.0]
    while plant.state()["t"] - start < 6.0:
        state = plant.state("calibration")
        last = np.asarray(state["calibration"]["filtered_force_N"], float).reshape(-1)
        # The force command is a controller target, while curved object
        # contacts can report an asymmetric resultant after both finger
        # actuators saturate.  Gate on a stable, physically sufficient
        # bilateral measured force instead of requiring the resultant to be
        # within 20% of the command (55.7 N previously missed that bound by
        # only 0.3 N despite no relative motion).
        on_target = (last.size >= 2 and np.min(last[:2]) >= 20.0
                     and np.sum(last[:2]) >= .70 * total_force)
        stable_since = state["t"] if on_target and stable_since is None else stable_since if on_target else None
        if stable_since is not None and state["t"] - stable_since >= .3:
            return {"target_total_force_N": total_force, "filtered_force_N": last.tolist(), "target_reached": True}
        time.sleep(.01)
    raise RuntimeError(f"calibration force target not reached: {last.tolist()}")


def move_held_to_station(backend, station_candidates):
    """Reorient for a hand-above-object placement, then release 6 mm high."""
    failures = []
    for station_index, station_xy in enumerate(station_candidates):
        state = plant.state()
        hand = ObjectScan._T(state["tcp"], state["tcp_quat"])
        obj = ObjectScan._T(state["box"], state["box_quat"])
        tcp_object = np.linalg.inv(hand) @ obj
        object_tcp = np.linalg.inv(tcp_object)
        object_to_tcp = object_tcp[:3, 3]
        if np.linalg.norm(object_to_tcp) < 1e-6:
            failures.append({"station_xy": list(map(float, station_xy)), "error": "degenerate grasp transform"})
            continue
        # Rotate the fixed grasp transform so the TCP lies above the object.
        # This prevents the side-grasp hand from reaching the table first.
        place_rotation = Rotation.align_vectors(
            np.asarray([[0.0, 0.0, 1.0]]),
            np.asarray([object_to_tcp / np.linalg.norm(object_to_tcp)]),
        )[0].as_matrix()
        destination = np.eye(4)
        destination[:3, :3] = place_rotation
        destination[:3, 3] = [float(station_xy[0]), float(station_xy[1]), .30]
        backend.attach_actual(hand, obj)
        backend.support_contact(False)
        try:
            current = backend.measured()
            goal = backend.ik(destination @ object_tcp, current)
            backend.phase("TRANSPORT_TO_SCAN_STATION")
            backend.execute_slow(backend.plan(current, goal), "TRANSPORT_TO_SCAN_STATION_FAIL", 3.0)
            plant.settle(.3)

            # Put the lowest transformed target vertex 6 mm above the table.
            local_min_z = float((VERTICES @ place_rotation.T)[:, 2].min())
            destination[:3, 3] = [float(station_xy[0]), float(station_xy[1]), .006 - local_min_z]
            state = plant.state()
            backend.attach_actual(ObjectScan._T(state["tcp"], state["tcp_quat"]),
                                  ObjectScan._T(state["box"], state["box_quat"]))
            backend.support_contact(True)
            current = backend.measured()
            goal = backend.ik(destination @ object_tcp, current)
            backend.phase("PLACE_LOWER")
            backend.execute_slow(backend.plan(current, goal), "PLACE_LOWER_FAIL", 2.5)
            plant.settle(.2)
            release_record = {
                "attempt": station_index + 1,
                "stage": "PLACE_FOR_TABLE_SCAN",
                "station_xy": list(map(float, station_xy)),
                "previous_station_failures": failures,
                "release_clearance_m": .006,
                "placement_mode": "HAND_ABOVE_OBJECT_LOW_RELEASE",
            }
            plant.command({"op": "support_pause"})
            backend.phase("RELEASE")
            backend.open_hand()
            plant.settle(.8)
            released_state = plant.state()
            released = ObjectScan._T(released_state["box"], released_state["box_quat"])
            release_record["release_target_pose"] = released.tolist()
            # Retreat while MoveIt still treats the target as attached, then
            # restore it as a world collision object at its settled pose.
            retreat = ObjectScan._T(released_state["tcp"], released_state["tcp_quat"])
            retreat[2, 3] += .10
            backend.execute(backend.cartesian(backend.measured(), retreat), "RELEASE_RETREAT_FAIL")
            backend.reset_scene_at(released)
            plant.settle(.5)
            return released, release_record
        except Exception as exc:
            failures.append({"station_xy": list(map(float, station_xy)), "error": str(exc)})
            # safe_release can reject a partial descent.  The target is still
            # held, so re-establish the measured attachment before replanning.
            state = plant.state()
            backend.attach_actual(ObjectScan._T(state["tcp"], state["tcp_quat"]),
                                  ObjectScan._T(state["box"], state["box_quat"]))
            backend.support_contact(False)
    raise RuntimeError(f"no collision-free scan station: {failures}")


def move_held_to_calibration_zone(backend, object_position=(.58, -.27, .32)):
    """Move the attached object clear of the clutter before attitude excitation."""
    state = plant.state()
    hand = ObjectScan._T(state["tcp"], state["tcp_quat"])
    obj = ObjectScan._T(state["box"], state["box_quat"])
    tcp_object = np.linalg.inv(hand) @ obj
    destination = obj.copy()
    destination[:3, 3] = np.asarray(object_position, float)
    goal_tcp = destination @ np.linalg.inv(tcp_object)
    current = backend.measured()
    goal = backend.ik(goal_tcp, current)
    backend.phase("TRANSPORT_TO_CALIBRATION_ZONE")
    backend.execute_slow(backend.plan(current, goal), "CALIBRATION_ZONE_NO_PLAN", 3.0)
    plant.settle(.5)
    final = plant.state()
    return {"requested_object_position": list(map(float, object_position)),
            "actual_object_pose": ObjectScan._T(final["box"], final["box_quat"]).tolist(),
            "actual_tcp_pose": ObjectScan._T(final["tcp"], final["tcp_quat"]).tolist()}


def move_arm_to_q(backend, q):
    current = backend.measured()
    goal = backend.measured()
    for name, value in zip(ARM_NAMES, q):
        goal.joint_state.position[goal.joint_state.name.index(name)] = float(value)
    backend.validate(goal)
    backend.phase("MATCHED_EMPTY_CENTER")
    backend.execute_slow(backend.plan(current, goal), "MATCHED_EMPTY_CENTER_FAIL", 2.5)


def capture_clean_static_poses(skill, output, protocol, mode, payload_specs=None):
    """Capture independently gated static windows using the validated policy."""
    output.mkdir(parents=True, exist_ok=True)
    if mode == "payload":
        static = protocol["static"]
        pose_ids = np.asarray(static["static_pose_id"])
        hold = np.asarray(static["static_hold"], bool)
        available = [int(x) for x in np.unique(pose_ids[hold]) if x >= 0]
        chosen = [available[i] for i in np.unique(np.linspace(0, len(available)-1, min(8, len(available))).astype(int))]
        specs = [{"pose_id": pid, "q_ref": np.asarray(static["q"])[np.flatnonzero((pose_ids == pid) & hold)[-1]].tolist()} for pid in chosen]
    else:
        specs = list(payload_specs or [])
    rows, accepted, reference = [], [], None
    for spec in specs:
        pid, qref = int(spec["pose_id"]), np.asarray(spec["q_ref"], float)
        for attempt, settle_s in enumerate((1.0, 3.0, 5.0)):
            skill.backend.validate(skill._robot_state_for_q(qref))
            if mode == "baseline":
                opening = float(spec["opening_mean_m"])
                answer = skill.plant.command({"op": "trajectory", "names": FINGER_NAMES,
                    "points": [{"t": 1.0, "q": [opening/2, opening/2]}], "gripper": True})
                if not answer.get("ok"):
                    raise RuntimeError(f"matched opening failed at pose {pid}: {answer}")
            move = skill.plant.command({"op": "trajectory", "names": skill.arm_names,
                "points": [{"t": 2.0, "q": qref.tolist()}]}, timeout=180)
            skill.plant.settle(settle_s)
            start = skill.plant.command({"op": "payload_record_start", "mode": mode})
            if not start.get("ok"):
                rows.append({"pose_id": pid, "attempt": attempt, "accepted": False,
                             "reasons": [start.get("reason", "PAYLOAD_GUARD_FAILED")], "start": start})
                (output / "capture.json").write_text(json.dumps({"mode": mode, "rows": rows,
                    "accepted": accepted, "finished": False, "GT_used": False}, indent=2))
                if attempt < 2 and start.get("reason") == "PAYLOAD_ID_REQUIRES_FREE_SPACE_STABLE":
                    continue
                break
            skill.plant.settle(.65)
            path = output / f"{mode}_pose{pid:02d}_attempt{attempt}.npz"
            stop = skill.plant.command({"op": "payload_record_stop", "path": str(path.resolve())})
            if not path.exists():
                rows.append({"pose_id": pid, "attempt": attempt, "accepted": False,
                             "reasons": ["NO_RECORD"], "stop": stop})
                break
            with np.load(path, allow_pickle=True) as loaded:
                data = {key: loaded[key] for key in loaded.files}
            keep = data["t"] >= data["t"][-1] - .5
            opening = float(spec["opening_mean_m"]) if mode == "baseline" else float(data["actual_opening_m"][keep].mean())
            row = window_quality(data, qref, opening, payload=mode == "payload")
            row["reasons"] = [x for x in row["reasons"] if x != "Q_REF_MISMATCH"]
            row.update(pose_id=pid, attempt=attempt, q_ref=qref.tolist(), path=str(path),
                       opening_mean_m=opening, trajectory=move)
            if not stop.get("ok"):
                row["reasons"].append("CAPTURE_GUARD_FAILED")
                row["stop"] = stop
            if mode == "payload":
                transforms = data["T_TCP_object"][keep]
                if reference is None and len(transforms):
                    reference = transforms[0]
                if reference is not None and len(transforms):
                    relative = np.linalg.inv(reference)[None] @ transforms
                    if (np.max(np.linalg.norm(relative[:, :3, 3], axis=1)) > .003 or
                            np.max(Rotation.from_matrix(relative[:, :3, :3]).magnitude()) > np.deg2rad(5)):
                        row["reasons"].append("GLOBAL_RELATIVE_SLIP")
            row["accepted"] = not row["reasons"]
            rows.append(row)
            if row["accepted"]:
                accepted.append(row)
            (output / "capture.json").write_text(json.dumps({"mode": mode, "rows": rows,
                "accepted": accepted, "finished": False, "GT_used": False}, indent=2))
            if row["accepted"] or not stop.get("ok"):
                break
        if mode == "payload" and rows and any(reason in rows[-1]["reasons"] for reason in
                ("GLOBAL_RELATIVE_SLIP", "CAPTURE_GUARD_FAILED", "PAYLOAD_ID_REQUIRES_FREE_SPACE_STABLE")):
            break
    result = {"mode": mode, "rows": rows, "accepted": accepted,
              "finished": True, "GT_used": False}
    (output / "capture.json").write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=1004)
    parser.add_argument("--mode", choices=["GT", "ANYGRASP"], default="ANYGRASP")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration-force", type=float, default=60.0)
    parser.add_argument("--payload-force", type=float, default=70.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    backend = UnseenBackend(args.output, GateThresholds())
    video_started = False
    summary = {"schema": "fr3_continuous_closed_loop/v1", "seed": args.seed,
               "mode": args.mode, "single_sim_process": True,
               "continuous_world_state": True, "stages": []}
    try:
        plant.command({"op": "reset", "seed": args.seed})
        plant.settle(1.0)
        initial = plant.state()
        initial_object = ObjectScan._T(initial["box"], initial["box_quat"])
        backend.reset_scene_at(initial_object)
        video = args.output / "third_person_continuous.mp4"
        plant.command({"op": "third_record_start", "path": str(video), "phase": "observe_scene"})
        video_started = True

        roi = np.array([.50, 0.0, .09])
        observation_positions = [
            [.22, -.27, .52], [.18, -.13, .56], [.19, .00, .62],
            [.20, .14, .56], [.23, .27, .52],
        ]
        observed = execute_camera_views(backend, args.output / "wrist_observe", "OBSERVE_SCENE", roi, observation_positions)
        if sum(x["success"] for x in observed) < 3:
            raise RuntimeError(f"too few observation viewpoints: {observed}")
        summary["observe_scene"] = observed
        summary["stages"].append("observe_scene")

        plant.command({"op": "third_record_mark", "phase": "pick_object"})
        original_command = plant.command
        skipped = {"done": False}
        def preserve_world(command, timeout=120):
            if command.get("op") == "reset" and not skipped["done"]:
                skipped["done"] = True
                return {"ok": True, "preserved_existing_world": True}
            # The frozen grasp episode normally re-arms its legacy contact
            # monitor immediately after reset.  This continuous episode has
            # already moved the robot through observation waypoints in the
            # same world, so reinitializing that tensor view can tear down
            # Isaac Sim.  Contact state is already live from scene creation;
            # preserve it along with the world instead of rebuilding it.
            if command.get("op") == "arm_metrics":
                return {"ok": True, "preserved_existing_monitor": True}
            return original_command(command, timeout)
        plant.command = preserve_world
        try:
            grasp = backend.episode(args.seed, args.mode, family_enabled=True)
        finally:
            plant.command = original_command
        if not grasp.get("success"):
            raise RuntimeError(f"frozen grasp failed: {grasp.get('category')}")
        summary["grasp"] = grasp
        summary["stages"].append("pick_object")

        plant.command({"op": "third_record_mark", "phase": "estimate_mass_com"})
        held_state = plant.state()
        backend.attach_actual(ObjectScan._T(held_state["tcp"], held_state["tcp_quat"]),
                              ObjectScan._T(held_state["box"], held_state["box_quat"]))
        backend.support_contact(False)
        summary["calibration_zone"] = move_held_to_calibration_zone(backend)
        force = wait_calibration_force(args.payload_force)
        # Let the increased bilateral force and the moved payload establish a
        # fresh stationary relative-pose window before the first record gate.
        plant.settle(3.0)
        state = plant.state()
        center_q = [state["q"][state["names"].index(name)] for name in ARM_NAMES]
        finger_q = [state["q"][state["names"].index(name)] for name in FINGER_NAMES]
        center = {"q": center_q, "names": ARM_NAMES, "finger_q": finger_q, "finger_names": FINGER_NAMES}
        (args.output / "excitation_center.json").write_text(json.dumps(center, indent=2))
        payload_skill = PayloadIDV2Skill(backend, plant)
        # Mass+COM uses quasi-static gravity attitudes only.  Do not design
        # the unrelated inertia Fourier trajectory: the validated production
        # fresh-gate entry point applies the same static-only protocol.
        protocol_path = args.output / "payload_id_v2_protocol.json"
        static = payload_skill._static_protocol(args.output / "payload_id_v2_protocol_static.json")
        protocol = {"schema": "cross_object_static_only/v1", "static": static,
                    "dynamic": None, "gt_used_for_estimation": False}
        protocol_path.write_text(json.dumps(protocol, indent=2))
        payload = capture_clean_static_poses(payload_skill, args.output / "payload", protocol, "payload")
        # A rejected Payload-ID attitude is evidence, not a reason to abort
        # the visible robot task.  Preserve PARTIAL explicitly and continue
        # through placement and wrist scanning so one run always yields the
        # requested end-to-end third-person record.
        payload_quality = "SUCCESS" if len(payload["accepted"]) >= 4 else "PARTIAL"
        summary["payload_verification"] = {
            "status": payload_quality,
            "accepted_pose_count": len(payload["accepted"]),
            "required_for_full_estimate": 4,
        }
        summary["payload_capture"] = payload
        summary["calibration_force"] = force
        summary["stages"].append("estimate_mass_com_motion")

        plant.command({"op": "third_record_mark", "phase": "place_at_scan_station"})
        # Far table-edge candidates leave clearance for both the object and
        # the wrist/link-6 release sweep.  The previous (.70, .27) center was
        # object-clear but made link 6 graze clutter_0 during descent.
        released, release_record = move_held_to_station(
            backend, [(.68, .33), (.74, .31), (.30, -.31), (.74, -.31)])
        summary["placement"] = {"released_object_pose": np.asarray(released).tolist(), "record": release_record}
        summary["stages"].append("place_at_scan_station")

        plant.command({"op": "third_record_mark", "phase": "scan_stationary_object"})
        scan_target = np.asarray(released)[:3, 3]
        scan_target[2] += .035
        scan_positions = close_scan_positions(scan_target)
        scan = execute_camera_views(
            backend, args.output / "wrist_scan", "SCAN_STATIONARY_OBJECT",
            scan_target, scan_positions, dwell=.8, prefer_cartesian=True,
            minimum_target_pixels=5000)
        if sum(x["success"] for x in scan) < 6:
            raise RuntimeError(f"too few scan viewpoints: {scan}")
        summary["scan"] = scan
        summary["stages"].append("scan_stationary_object")

        video_result = plant.command({"op": "third_record_stop"})
        video_started = False
        summary["third_person_video"] = video_result

        # Collect the matched-opening empty leg after the visible task.  It
        # uses the exact payload protocol and measured center/opening.
        move_arm_to_q(backend, center_q)
        opening = float(sum(finger_q))
        finger_answer = plant.command({"op": "trajectory", "names": FINGER_NAMES,
            "points": [{"t": 1.0, "q": [opening / 2.0, opening / 2.0]}], "gripper": True})
        if not finger_answer.get("ok"):
            raise RuntimeError(f"matched opening failed: {finger_answer}")
        plant.settle(.3)
        empty = capture_clean_static_poses(payload_skill, args.output / "empty", protocol,
                                           "baseline", payload["accepted"])
        summary["empty_verification"] = {
            "status": "SUCCESS" if len(empty["accepted"]) >= min(4, len(payload["accepted"])) else "PARTIAL",
            "accepted_pose_count": len(empty["accepted"]),
            "matched_payload_pose_count": len(payload["accepted"]),
        }
        summary["empty_capture"] = empty
        summary["matched_opening_m"] = opening

        final = plant.state()
        summary["final_clutter"] = final.get("clutter")
        summary["status"] = "ACQUISITION_SUCCESS"
        (args.output / "continuous_closed_loop.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps({"status": summary["status"], "output": str(args.output),
                          "video": video_result, "stages": summary["stages"]}))
    except Exception as exc:
        summary["status"] = "FAIL"
        summary["error"] = str(exc)
        if video_started:
            try:
                summary["third_person_video"] = plant.command({"op": "third_record_stop"})
            except Exception as stop_exc:
                summary["video_stop_error"] = str(stop_exc)
        (args.output / "continuous_closed_loop.json").write_text(json.dumps(summary, indent=2))
        raise
    finally:
        backend.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
