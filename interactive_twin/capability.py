"""Read-only PiPER capabilities and a hardware-observable data boundary.

This module never imports or instantiates the SDK, opens CAN, or queries a robot.
An API found in source is not evidence that the installed robot/firmware supplies
valid measurements. Native PhysX contact data belongs in a separate safety log.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

OFFICIAL_COMMIT = "c9e8a28174e71eeaac448593cb65f8ab258a92fe"
OFFICIAL_BASE = f"https://github.com/agilexrobotics/piper_sdk/blob/{OFFICIAL_COMMIT}"
SOURCE_FILES = {
    "interface": "piper_sdk/interface/piper_interface.py",
    "joint": "piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_joint_states.py",
    "motor": "piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_high_spd.py",
    "gripper": "piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_gripper.py",
    "protocol": "piper_sdk/protocol/protocol_v2/piper_protocol_v2.py",
    "fk": "piper_sdk/kinematics/piper_fk.py",
    "readme": "README.MD",
}


def _signal(api: str | None, units: str, limitation: str, source: str,
            *, supported: bool = True) -> dict[str, Any]:
    return {"official_api": api, "documented_by_audited_sdk": supported,
            "hardware_verified": False, "runtime_availability": "UNKNOWN_NOT_CONNECTED",
            "units_and_conversion": units, "limitation": limitation,
            "source": f"{OFFICIAL_BASE}/{SOURCE_FILES[source]}"}


def inspect_local_sdk(sdk_root: str | Path | None = None) -> dict[str, Any]:
    """Inspect files/metadata without executing SDK code or touching hardware."""
    result: dict[str, Any] = {"present": False, "path": None, "version": None,
                             "commit": None, "api_methods": [], "source_sha256": {},
                             "hardware_connected": False, "firmware": "UNKNOWN"}
    if sdk_root is None:
        try:
            spec = importlib.util.find_spec("piper_sdk")
        except (ImportError, ValueError):
            spec = None
        if spec and spec.submodule_search_locations:
            sdk_root = next(iter(spec.submodule_search_locations))
        try:
            result["version"] = importlib.metadata.version("piper_sdk")
        except importlib.metadata.PackageNotFoundError:
            pass
    if sdk_root is None:
        result["reason"] = "No local SDK detected; official source evidence remains separate."
        return result
    root = Path(sdk_root).expanduser().resolve()
    package = root / "piper_sdk" if (root / "piper_sdk").is_dir() else root
    interface = package / "interface" / "piper_interface.py"
    legacy = package / "interface" / "piper_interface_v2.py"
    if not interface.exists() and legacy.exists():
        interface = legacy
    if not interface.exists():
        result.update(path=str(root), reason="SDK interface source not found; no APIs assumed.")
        return result
    result.update(present=True, path=str(package))
    try:
        parsed = ast.parse(interface.read_text(encoding="utf-8"))
        result["api_methods"] = sorted({n.name for n in ast.walk(parsed)
                                        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                                        and not n.name.startswith("_")})
    except (SyntaxError, UnicodeError) as error:
        result["source_parse_error"] = str(error)
    version_file = package / "version.py"
    if version_file.exists() and result["version"] is None:
        match = re.search(r"PIPER_SDK_CURRENT_VERSION\s*=\s*PIPER_SDK_VERSION_(\d+)_(\d+)_(\d+)",
                          version_file.read_text(encoding="utf-8"))
        if match:
            result["version"] = ".".join(match.groups())
    for relative in SOURCE_FILES.values():
        source = package.parent / relative
        if source.is_file():
            result["source_sha256"][relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    # rev-parse alone could accidentally report an enclosing project repository.
    if (package.parent / ".git").exists():
        try:
            result["commit"] = subprocess.check_output(
                ["git", "-C", str(package.parent), "rev-parse", "HEAD"],
                text=True, stderr=subprocess.DEVNULL, timeout=3).strip()
        except (OSError, subprocess.SubprocessError):
            pass
    result["same_as_official_audit"] = result["commit"] == OFFICIAL_COMMIT
    return result


def calibration_eligible(calibration: Mapping[str, Any] | None) -> bool:
    """A flag alone cannot promote simulator effort to calibrated hardware torque.

    This only checks an evidence record's declared completeness. The calibration
    experiment and its held-out validation still require review; no run is made.
    """
    if not calibration:
        return False
    required_true = ("robot_only_baseline_frozen", "units_verified", "bias_identified",
                     "synchronization_validated", "heldout_validation_passed")
    required_nonempty = ("robot_serial", "firmware", "sdk_version", "evidence_path",
                        "signal_name", "calibration_sha256")
    return (all(calibration.get(k) is True for k in required_true)
            and all(calibration.get(k) not in (None, "", "UNKNOWN") for k in required_nonempty)
            and calibration.get("signal_name") in ("motor_current_a", "sdk_effort_raw")
            and calibration.get("source") == "REAL_HARDWARE")


def build_capability_report(sdk_root: str | Path | None = None,
                            calibration: Mapping[str, Any] | None = None) -> dict[str, Any]:
    local = inspect_local_sdk(sdk_root)
    signals = {
        "joint_position": _signal("GetArmJointMsgs().joint_state.joint_1..6",
            "raw 0.001 degree; q_rad = raw * pi / 180000", "Three CAN frames are not one atomic sample; reject initial zero/stale frames.", "joint"),
        "joint_velocity": _signal("GetArmHighSpdInfoMsgs().motor_1..6.motor_speed",
            "raw 0.001 rad/s; SI = raw * 0.001", "SDK labels motor speed; compare against timestamped joint-position differences and firmware convention before treating it as joint velocity.", "motor"),
        "fk_tcp_pose": _signal("GetFK('feedback') / GetArmEndPoseMsgs()",
            "GetFK: mm, degree. EndPose: 0.001 mm, 0.001 degree. Normalize to m/rad SE(3).",
            "FK is encoder/model-derived, not independent Cartesian sensing. Verify DH generation, encoder zeros, base transform and actual TCP; firmware-dependent 2-degree DH offsets exist. No arbitrary TCP offset inferred.", "fk"),
        "motor_current": _signal("GetArmHighSpdInfoMsgs().motor_1..6.current",
            "raw 0.001 A; SI = raw * 0.001", "Motor current includes servo/gravity/inertia/friction; not external joint torque. Doc says uint16 while audited parser decodes signed int16: retain raw CAN and verify sign on hardware.", "protocol"),
        "sdk_effort": _signal("GetArmHighSpdInfoMsgs().motor_1..6.effort",
            "SDK raw current * coefficient: J1-3=1.18125, J4-6=0.95844; documented 0.001 N/m (ambiguous torque typography).",
            "Derived from current, not an independent torque measurement. Keep sdk_effort_raw; do not label external_torque_nm or infer a physical torque scale before independent calibration.", "motor"),
        "gripper_opening": _signal("GetArmGripperMsgs().gripper_state.grippers_angle",
            "raw stroke 0.001 mm; gripper_stroke_m = raw * 1e-6", "Verify homing and stroke-to-physical-opening mapping; field is not independent left/right finger position.", "gripper"),
        "gripper_feedback": _signal("GetArmGripperMsgs(): grippers_effort, foc_status",
            "effort documented 0.001 N m, integer; status bitfield", "One total actuator estimate; not fingertip force in N, not two pad loads, and not proof of bilateral contact. Preserve raw value unless independently calibrated.", "gripper"),
        "feedback_timestamp": _signal("message.time_stamp; CAN Message.timestamp",
            "seconds", "Audited parser copies python-can receive timestamp; group timestamp updates per arriving frame. This is not a robot execution timestamp. Record monotonic receive/send stamps and per-frame age; older SDKs may use host time instead.", "protocol"),
        "command_timestamp": _signal("application send log + send status",
            "monotonic seconds / integer ns", "Record before/after host send. Send success is not an execution acknowledgement; actual command latency requires robot-only calibration.", "interface"),
        "wrist_ft": _signal(None, "unavailable", "No six-axis wrist F/T sensor verified; not required by motion-only fitting.", "readme", supported=False),
        "external_joint_torque": _signal(None, "unavailable", "No calibrated external joint torque sensing verified.", "motor", supported=False),
        "bilateral_tactile": _signal(None, "unavailable", "No independently measured left/right tactile arrays verified in stock feedback.", "gripper", supported=False),
        "contact_manifold": _signal(None, "simulator-only", "Exact PhysX points/normals, ownership, separations and contact impulses are simulation diagnostics/safety oracle, not PiPER estimator inputs.", "gripper", supported=False),
    }
    methods = set(local["api_methods"])
    for entry in signals.values():
        api = entry["official_api"]
        entry["api_found_in_local_source"] = bool(api and any(name in api for name in methods))
    eligible = calibration_eligible(calibration)
    return {
        "schema": "interactive-twin-capabilities-v1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "audit_only_no_hardware_connection": True,
        "local_sdk": local,
        "official_sdk_audit": {"commit": OFFICIAL_COMMIT, "version": "0.6.2",
                               "source": "https://github.com/agilexrobotics/piper_sdk"},
        "firmware": "UNKNOWN", "robot_serial": "UNKNOWN", "signals": signals,
        "control_interface": {"documented": ["JointCtrl", "EndPoseCtrl", "JointMitCtrl", "GripperCtrl"],
            "hardware_control_modes_verified": False,
            "limitation": "MIT command API does not establish calibrated physical torque, bandwidth or safe timing. Existing Isaac direct-effort control has not been validated as an equivalent PiPER hardware servo."},
        "B3": {"status": "EVIDENCE_DECLARED_REVIEW_REQUIRED" if eligible else "UNAVAILABLE",
               "enabled": False,
               "reason": "No independently verified current/effort calibration loaded; cannot substitute simulator joint/contact forces.",
               "calibration_record_complete": eligible},
        "allowed_estimator_inputs": sorted(OBSERVATION_ALIASES),
        "simulator_only_baseline_dependencies": ["bilateral fingertip contact load safety",
            "dangerous collision oracle", "contact-plane retention drift proxy"],
        "claim_boundary": {"simulation_result_label": "SIM_TO_SIM_BLIND_SYSID",
            "real_to_sim_success_proven": False, "full_relative_slip_observable": False,
            "physx_contacts_may_be_estimator_or_policy_inputs": False,
            "physx_contacts_may_enforce_simulation_safety": True,
            "physics_parameter_units_without_torque_calibration": "effective simulator resistance parameters or identifiable intervals"},
        "required_before_real_deployment": ["record robot serial, firmware, SDK commit and frame conventions",
            "verify qdot/sign/time synchronization and measure robot-only latency/response",
            "calibrate TCP and gripper stroke mapping without changing official geometry",
            "provide an actual hardware grasp-retention/safety measurement path; aggregate gripper effort is insufficient for bilateral verification",
            "validate actuator current baseline, scale and held-out response before enabling B3"],
    }


OBSERVATION_ALIASES = {
    "t_s": ("t_s", "t"), "q_rad": ("q_rad", "q"),
    "qdot_rad_s": ("qdot_rad_s", "qdot", "encoder_difference_joint_velocity"),
    "T_world_tcp": ("T_world_tcp", "T_tcp"),
    "gripper_stroke_m": ("gripper_stroke_m",),
    "gripper_opening_m": ("gripper_opening_m", "aperture_m"),
    "gripper_status": ("gripper_status",), "command": ("command",),
    "command_send_monotonic_ns": ("command_send_monotonic_ns",),
    "receive_monotonic_ns": ("receive_monotonic_ns",),
}
OPTIONAL_LOG_SIGNALS = {"motor_current_a", "sdk_effort_raw", "gripper_effort_raw"}
COMMAND_FIELDS = {"q_target_rad", "qdot_target_rad_s", "joint_effort_command",
                  "gripper_stroke_target_m", "gripper_effort_command", "direction_world",
                  "task_drive_reference_world_m", "task_drive_speed_m_s", "mode",
                  "send_monotonic_ns", "send_success", "units"}


def validate_controller_inputs(signal_names: Iterable[str], report: Mapping[str, Any] | None = None) -> None:
    """Fail closed: annotations containing GT/contact oracle are not policy inputs."""
    allowed = set(OBSERVATION_ALIASES) | {"initial_visual_handle_frame", "attempt_history", "estimated_articulation"}
    if report and report.get("B3", {}).get("enabled") is True:
        allowed |= OPTIONAL_LOG_SIGNALS
    forbidden = sorted(set(signal_names) - allowed)
    if forbidden:
        raise ValueError("NON_HARDWARE_POLICY_INPUT: " + ", ".join(forbidden))


def project_observation(row: Mapping[str, Any], mode: str = "SIM_TO_SIM_BLIND_SYSID",
                        allow_calibrated_effort: bool = False,
                        report: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Export only robot-observable SI fields from a richer simulation safety row.

    Units are NOT inferred or converted here; source adapters must normalize them.
    Current/effort are omitted by default. Keep their uncalibrated raw data in a
    separate diagnostic log. A simulator effort signal is not measured current.
    """
    if mode not in ("SIM_TO_SIM_BLIND_SYSID", "REAL_LOG_TO_SIM"):
        raise ValueError("Unknown experiment mode: " + mode)
    if allow_calibrated_effort and not (report and report.get("B3", {}).get("enabled") is True):
        raise ValueError("B3_UNAVAILABLE: independently reviewed calibration required")
    projected: dict[str, Any] = {}
    for canonical, aliases in OBSERVATION_ALIASES.items():
        for alias in aliases:
            if alias in row:
                projected[canonical] = row[alias]
                break
    if "command" in projected:
        command = projected["command"]
        if not isinstance(command, Mapping) or set(command) - COMMAND_FIELDS:
            raise ValueError("COMMAND_SCHEMA_REJECTED: unrecognized fields could leak GT")
        # Commands are flat numeric/vector fields except explicitly named labels.
        for key, value in command.items():
            if isinstance(value, Mapping):
                raise ValueError("COMMAND_SCHEMA_REJECTED: nested metadata is not a command")
    if allow_calibrated_effort:
        for name in OPTIONAL_LOG_SIGNALS:
            if name in row:
                projected[name] = row[name]
    _check_finite(projected)
    if "q_rad" in projected and len(projected["q_rad"]) != 6:
        raise ValueError("PiPER requires six arm joint positions; gripper is separate")
    if "qdot_rad_s" in projected and len(projected["qdot_rad_s"]) != 6:
        raise ValueError("PiPER requires six arm joint velocities")
    if "T_world_tcp" in projected:
        T = projected["T_world_tcp"]
        if len(T) != 4 or any(len(line) != 4 for line in T) or any(abs(T[3][i]-v) > 1e-8 for i, v in enumerate((0, 0, 0, 1))):
            raise ValueError("T_world_tcp must be a homogeneous 4x4 matrix")
    return projected


def _check_finite(value: Any) -> None:
    if isinstance(value, Mapping):
        for child in value.values():
            _check_finite(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            _check_finite(child)
    elif isinstance(value, (float, int)) and not math.isfinite(value):
        raise ValueError("NONFINITE_SENSOR_DATA")


def report_markdown(report: Mapping[str, Any]) -> str:
    lines = ["# PiPER capability report", "", "Read-only source audit; no CAN connection or hardware validation.", "",
             f"Firmware: **{report['firmware']}**. B3: **{report['B3']['status']}**.", "",
             f"Official SDK: `{OFFICIAL_COMMIT}` (0.6.2); local SDK: `{report['local_sdk']['version']}`.", "",
             "| Signal | Documented API | Units | Limitation |", "|---|---|---|---|"]
    for name, signal in report["signals"].items():
        cells = [name, signal["official_api"] or "unavailable", signal["units_and_conversion"], signal["limitation"]]
        lines.append("| " + " | ".join(str(c).replace("|", "/") for c in cells) + " |")
    lines += ["", "## Claim boundary", "", "Simulation contact manifolds and bilateral pad loads may enforce a simulation safety oracle; they are excluded from system-identification observations and are not stock PiPER sensors.", "",
              "The frozen baseline's tactile retention observer has no verified hardware replacement. Full in-plane slip is not observable. SIM_TO_SIM_BLIND_SYSID is not a demonstrated real-to-sim result.", "",
              "## Before hardware deployment", ""]
    lines.extend("- " + item for item in report["required_before_real_deployment"])
    lines += ["", "## Official source evidence", ""]
    lines.extend(f"- [{key}]({OFFICIAL_BASE}/{path})" for key, path in SOURCE_FILES.items())
    return "\n".join(lines) + "\n"


def write_capability_report(output_dir: str | Path, sdk_root: str | Path | None = None,
                            calibration: Mapping[str, Any] | None = None) -> dict[str, Any]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    report = build_capability_report(sdk_root, calibration)
    (output / "capability_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    (output / "capability_report.md").write_text(report_markdown(report), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sdk-root", type=Path)
    parser.add_argument("--calibration", type=Path)
    args = parser.parse_args()
    calibration = json.loads(args.calibration.read_text()) if args.calibration else None
    report = write_capability_report(args.output, args.sdk_root, calibration)
    print(json.dumps({"output": str(args.output), "firmware": report["firmware"], "B3": report["B3"]["status"]}))


if __name__ == "__main__":
    main()
