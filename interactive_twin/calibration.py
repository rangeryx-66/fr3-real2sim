"""Robot-only repeat calibration using observable logs, before object fitting."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from interactive_twin.sysid import InvalidLog, NoiseScales, _interp, _rms, command_hash, validate_log


def calibrate_robot_response(logs, *, noise_floors, output_dir=None):
    """Freeze noise and robot response summaries from >=2 no-contact repeats.

    ``noise_floors`` is an explicit physical-unit dictionary matching NoiseScales
    except calibration_id. Floors include quantization/known FK propagation;
    repeat disagreement is added conservatively by taking their maximum.

    Torque/current commands alone cannot identify servo gain, communication
    latency and mechanical response delay separately; those estimates remain
    null. Current means are robot-only bias baselines, NOT torque calibration.
    A SIM result never claims hardware calibration.
    """
    if len(logs) < 2:
        raise InvalidLog("Robot-only calibration needs at least two repeats")
    floors = dict(noise_floors)
    floors.pop("calibration_id", None)
    # Validate all explicit floors before looking at repeat variability.
    NoiseScales(**floors, calibration_id="pending_robot_calibration").validate()
    reference = logs[0]
    for log in logs:
        validate_log(log, independent_sim=log["provenance"]["source"] == "isaac_physics")
        if log["split"] != "calibration":
            raise InvalidLog("Object-interaction samples cannot calibrate robot response")
        if log["provenance"].get("no_contact_supervisor_certified") is not True:
            raise InvalidLog("Robot-only rollout lacks no-contact supervisor certification")
        if command_hash(log["commands"]) != command_hash(reference["commands"]):
            raise InvalidLog("Robot-only repeats did not receive identical commands")
        for key in ("mode",):
            if log[key] != reference[key]:
                raise InvalidLog("Cannot mix simulation and real calibration")
        for key in ("source", "robot_model_id", "controller_id"):
            if log["provenance"][key] != reference["provenance"][key]:
                raise InvalidLog(f"Robot-only calibration mismatch: {key}")
    t = np.asarray(reference["time_s"], float)
    signals = []
    for log in logs:
        own_t = np.asarray(log["time_s"], float)
        if own_t[0] > t[0] + 1e-6 or own_t[-1] < t[-1] - 1e-6:
            raise InvalidLog("Calibration repeat has incomplete command coverage")
        signals.append({key: _interp(t, own_t, value) for key, value in log["signals"].items()})
    samples = {key: [] for key in ("position_m", "rotation_rad", "joint_rad",
                                   "joint_velocity_rad_s", "ee_velocity_m_s", "tracking_m")}
    auxiliary = {"current_a": "motor_current_a", "effort": "sdk_effort"}
    for key, signal in auxiliary.items():
        if all(signal in s for s in signals):
            samples[key] = []
    for i, a in enumerate(signals):
        for b in signals[i + 1:]:
            pa, pb = a["ee_T_world_tcp"][:, :3, 3], b["ee_T_world_tcp"][:, :3, 3]
            samples["position_m"].append(_rms(pa - pb) / np.sqrt(2.))
            samples["tracking_m"].append(_rms(pa - pb) / np.sqrt(2.))
            samples["joint_rad"].append(_rms(a["q_rad"] - b["q_rad"]) / np.sqrt(2.))
            samples["joint_velocity_rad_s"].append(_rms(a["qdot_rad_s"] - b["qdot_rad_s"]) / np.sqrt(2.))
            samples["ee_velocity_m_s"].append(_rms(np.gradient(pa, t, axis=0) - np.gradient(pb, t, axis=0)) / np.sqrt(2.))
            Ra, Rb = a["ee_T_world_tcp"][:, :3, :3], b["ee_T_world_tcp"][:, :3, :3]
            relative = Ra @ np.transpose(Rb, (0, 2, 1))
            angles = np.arccos(np.clip((np.trace(relative, axis1=1, axis2=2) - 1.) / 2., -1., 1.))
            samples["rotation_rad"].append(_rms(angles) / np.sqrt(2.))
            for key, signal in auxiliary.items():
                if key in samples:
                    samples[key].append(_rms(a[signal] - b[signal]) / np.sqrt(2.))
    observed_noise = {key: max(value, default=0.) for key, value in samples.items()}
    scales = dict(floors)
    for key, measured in observed_noise.items():
        if key in auxiliary and floors.get(key) is None:
            # A positive declared floor is necessary before using current/effort.
            scales[key] = None
        else:
            scales[key] = max(floors[key], measured)
    # Timestamp sampling alone limits observed onset precision.
    scales["start_time_s"] = max(floors["start_time_s"], float(np.max(np.diff(t))))
    response = {
        "command_latency_s": None,
        "command_latency_reason": "Observed motion onset combines communication and mechanics; no separate timing calibration supplied",
        "response_gain": None,
        "response_gain_reason": "No independently identifiable command-to-servo gain in this protocol",
        "joint_velocity_rms_rad_s": [_rms(s["qdot_rad_s"]) for s in signals],
        "joint_velocity_peak_rad_s": [float(np.max(np.abs(s["qdot_rad_s"]))) for s in signals],
        "ee_velocity_peak_m_s": [float(np.max(np.linalg.norm(np.gradient(s["ee_T_world_tcp"][:, :3, 3], t, axis=0), axis=1))) for s in signals],
    }
    # Report a descriptive position-response gain only when joint reference
    # commands exist and have >10 sensor-floor excitation. Never rescale gains.
    fields = reference["commands"]["fields"]
    q = signals[0]["q_rad"]
    position_fields = [f"joint_{i + 1}_position_rad" for i in range(q.shape[1])]
    if all(name in fields for name in position_fields):
        values = np.asarray(reference["commands"]["values"])
        target = _interp(t, reference["commands"]["time_s"], values[:, [fields.index(name) for name in position_fields]])
        x = (target - target[0]).ravel()
        if _rms(x) > 10. * scales["joint_rad"]:
            gains = [float(np.dot(x, (s["q_rad"] - s["q_rad"][0]).ravel()) / np.dot(x, x)) for s in signals]
            response["response_gain"] = float(np.mean(gains))
            response["response_gain_repeat_values"] = gains
            response["response_gain_reason"] = "Descriptive scalar position response over this protocol; not a replacement servo model"
    current = {}
    for key, signal in auxiliary.items():
        if all(signal in s for s in signals):
            # Baseline is explicitly protocol-dependent, not external torque.
            current[signal] = {
                "available": True,
                "robot_only_protocol_mean": np.mean([s[signal].mean(0) for s in signals], axis=0).tolist(),
                "initial_sample_mean": np.mean([s[signal][0] for s in signals], axis=0).tolist(),
                "independently_calibrated_to_external_torque": False,
                "B3_comparable": False,
                "note": "A bias summary does not establish motor-current to external-torque conversion",
            }
        else:
            current[signal] = {"available": False, "robot_only_protocol_mean": None, "B3_comparable": False}
    report = {
        "schema_version": 1,
        "mode": reference["mode"],
        "source": reference["provenance"]["source"],
        "scope": "simulation robot response calibration" if reference["provenance"]["source"] == "isaac_physics" else "real robot log response calibration",
        "hardware_calibration": reference["provenance"]["source"] == "real_robot_log",
        "robot_model_id": reference["provenance"]["robot_model_id"],
        "controller_id": reference["provenance"]["controller_id"],
        "repeats": len(logs), "no_contact_supervisor_certified": True,
        "command_sha256": command_hash(reference["commands"]),
        "source_log_sha256": [hashlib.sha256(json.dumps(log, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest() for log in logs],
        "noise_floor_config": floors, "repeat_noise_estimates": observed_noise,
        "noise_scales": scales, "response": response, "current_or_effort": current,
        "robot_parameters_modified": False,
        "limitations": ["Repeatability is not sensor accuracy", "No object resistance fitted during robot-only calibration",
                        "Servo latency/gain remain unidentifiable unless an appropriate independent command protocol exists"],
    }
    digest = hashlib.sha256(json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    report["calibration_id"] = "robot_" + digest[:20]
    report["noise_scales"]["calibration_id"] = report["calibration_id"]
    NoiseScales(**report["noise_scales"]).validate()
    if output_dir is not None:
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        (output / "robot_calibration.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report
