"""Blind input/response analysis for external, freely evolving Isaac rollouts.

This module never advances a reduced door ODE, calls Isaac, or replays observed
joint states. The caller supplies independently executed simulator rollouts.
SIM_TO_SIM_BLIND_SYSID is explicitly a simulation surrogate, not real-to-sim.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import csv
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


SCHEMA_VERSION = 1
MODES = {"SIM_TO_SIM_BLIND_SYSID", "REAL_LOG_TO_SIM"}
TRAIN_PROBES = ("P1", "P2", "P3")
HELDOUT_PROBES = ("P4",)
SIGNALS = {"q_rad", "qdot_rad_s", "ee_T_world_tcp", "gripper_opening_m", "gripper_effort",
           "motor_current_a", "sdk_effort"}
FORBIDDEN_TOKENS = ("ground_truth", "gt_", "hinge_axis", "joint_origin",
                    "door_angle", "object_joint", "moving_link", "contact_manifold",
                    "contact_point", "external_torque", "object_force")
REQUIRED_LOG_KEYS = {"schema_version", "mode", "episode_id", "probe_id", "split",
                     "time_s", "commands", "signals", "provenance"}
OPTIONAL_LOG_KEYS = {"sensor_calibration"}
PROVENANCE_KEYS = {"source", "robot_model_id", "robot_calibration_id", "controller_id",
                   "simulator", "complete", "initialization_only", "robot_state_replayed",
                   "object_state_replayed", "direct_object_actuation", "attachment",
                   "command_applied_sha256", "source_log_sha256", "no_contact_supervisor_certified"}


class InvalidLog(ValueError):
    """Data violate the hardware-observable or independent replay contract."""


@dataclass(frozen=True)
class NoiseScales:
    """Positive physical noise scales, frozen using robot-only/repeat calibration.

    Values are never inferred from a TEST object's fitting residual. A scale is
    not a claim that the stock PiPER sensor has that accuracy.
    """
    position_m: float
    rotation_rad: float
    joint_rad: float
    joint_velocity_rad_s: float
    ee_velocity_m_s: float
    start_time_s: float
    tracking_m: float
    current_a: float | None = None
    effort: float | None = None
    calibration_id: str = ""

    def validate(self):
        for key, value in asdict(self).items():
            if key == "calibration_id" or value is None:
                continue
            if not np.isfinite(value) or value <= 0:
                raise InvalidLog(f"Nonpositive/nonfinite noise scale: {key}")
        if not self.calibration_id:
            raise InvalidLog("Noise scales require a frozen calibration_id")


def _reject_forbidden(value, location="log"):
    if isinstance(value, Mapping):
        for key, item in value.items():
            lower = str(key).lower()
            if any(token in lower for token in FORBIDDEN_TOKENS):
                raise InvalidLog(f"Forbidden estimator input: {location}.{key}")
            _reject_forbidden(item, f"{location}.{key}")
    elif isinstance(value, list):
        for item in value:
            if isinstance(item, (Mapping, list)):
                _reject_forbidden(item, location)


def command_hash(commands: Mapping) -> str:
    return hashlib.sha256(json.dumps(commands, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def _array(value, name, shape=None):
    array = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(array)):
        raise InvalidLog(f"{name} contains nonfinite values")
    if shape is not None and array.shape != shape:
        raise InvalidLog(f"{name} shape {array.shape}; expected {shape}")
    return array


def validate_log(log: Mapping, *, independent_sim=False) -> Mapping:
    """Reject privileged fields and validate an observable time-series log.

    ``independent_sim`` also checks replay attestations. These are an auditable
    runner contract, not cryptographic proof that the simulator obeyed them.
    """
    _reject_forbidden(log)
    unknown = set(log) - REQUIRED_LOG_KEYS - OPTIONAL_LOG_KEYS
    missing = REQUIRED_LOG_KEYS - set(log)
    if unknown or missing:
        raise InvalidLog(f"Log keys: missing={sorted(missing)}, unknown={sorted(unknown)}")
    if log["schema_version"] != SCHEMA_VERSION or log["mode"] not in MODES:
        raise InvalidLog("Unsupported schema_version or mode")
    if log["split"] not in {"train", "heldout", "calibration", "sensitivity"}:
        raise InvalidLog("Unknown data split")
    t = _array(log["time_s"], "time_s")
    if t.ndim != 1 or len(t) < 5 or t[0] < 0 or np.any(np.diff(t) <= 0):
        raise InvalidLog("Need at least five monotonic nonnegative timestamps")
    s = log["signals"]
    required = {"q_rad", "qdot_rad_s", "ee_T_world_tcp", "gripper_opening_m"}
    if required - set(s) or set(s) - SIGNALS:
        raise InvalidLog("Signals must use the documented PiPER-observable schema")
    q = _array(s["q_rad"], "q_rad")
    if q.ndim != 2 or q.shape[0] != len(t):
        raise InvalidLog("q_rad must be N by joint_count")
    _array(s["qdot_rad_s"], "qdot_rad_s", q.shape)
    pose = _array(s["ee_T_world_tcp"], "ee_T_world_tcp", (len(t), 4, 4))
    if not np.allclose(pose[:, 3], [0, 0, 0, 1], atol=1e-6):
        raise InvalidLog("EE poses must be homogeneous SE(3) matrices")
    R = pose[:, :3, :3]
    if (not np.allclose(R @ np.transpose(R, (0, 2, 1)), np.eye(3), atol=2e-4)
            or not np.allclose(np.linalg.det(R), 1., atol=2e-4)):
        raise InvalidLog("EE rotations are not in SO(3)")
    _array(s["gripper_opening_m"], "gripper_opening_m", (len(t),))
    if "gripper_effort" in s:
        _array(s["gripper_effort"], "gripper_effort", (len(t),))
    for name in ("motor_current_a", "sdk_effort"):
        if name in s:
            _array(s[name], name, q.shape)
    commands = log["commands"]
    if set(commands) != {"time_s", "values", "fields", "kind"}:
        raise InvalidLog("Commands require time_s, values, fields, kind")
    ct = _array(commands["time_s"], "commands.time_s")
    if ct.ndim != 1 or not len(ct) or np.any(np.diff(ct) <= 0) or ct[0] < 0:
        raise InvalidLog("Commands need monotonic nonnegative timestamps")
    values = _array(commands["values"], "commands.values")
    if values.shape != (len(ct), len(commands["fields"])):
        raise InvalidLog("Command values do not match timestamps/fields")
    if ct[-1] > t[-1] + 1e-6:
        raise InvalidLog("Incomplete observation horizon for applied commands")
    provenance = log["provenance"]
    if set(provenance) - PROVENANCE_KEYS:
        raise InvalidLog("Undocumented provenance fields; put GT in a separate evaluation file")
    for key in ("source", "robot_model_id", "robot_calibration_id", "controller_id"):
        if not provenance.get(key):
            raise InvalidLog(f"Missing provenance.{key}")
    if provenance["source"] not in {"isaac_physics", "real_robot_log"}:
        raise InvalidLog("Source must identify Isaac physics or a real robot log")
    if log["mode"] == "SIM_TO_SIM_BLIND_SYSID" and provenance["source"] != "isaac_physics":
        raise InvalidLog("Simulation surrogate cannot be relabeled as a real robot log")
    if independent_sim:
        expected = {"source": "isaac_physics", "simulator": "isaac_physx", "complete": True,
                    "initialization_only": True, "robot_state_replayed": False,
                    "object_state_replayed": False, "direct_object_actuation": False,
                    "attachment": False, "command_applied_sha256": command_hash(commands)}
        for key, value in expected.items():
            if provenance.get(key) != value:
                raise InvalidLog(f"Independent simulator replay violation: {key}")
    return log


def _interp(t, source_t, array):
    array = np.asarray(array, float)
    flat = array.reshape(len(source_t), -1)
    out = np.column_stack([np.interp(t, source_t, flat[:, i]) for i in range(flat.shape[1])])
    return out.reshape((len(t),) + array.shape[1:])


def _aligned(reference, predicted):
    validate_log(reference)
    if reference["provenance"].get("complete") is not True:
        raise InvalidLog("Reference probe is incomplete; censored response cannot be fitted as a complete action")
    if reference["provenance"]["source"] == "isaac_physics":
        validate_log(reference, independent_sim=True)
    validate_log(predicted, independent_sim=True)
    if reference["mode"] != predicted["mode"]:
        raise InvalidLog("Reference and twin modes differ")
    if reference["mode"] == "REAL_LOG_TO_SIM" and reference["provenance"]["source"] != "real_robot_log":
        raise InvalidLog("REAL_LOG_TO_SIM requires an actual real_robot_log reference")
    if reference["probe_id"] != predicted["probe_id"]:
        raise InvalidLog("Cannot compare different probe protocols")
    if command_hash(reference["commands"]) != command_hash(predicted["commands"]):
        raise InvalidLog("Twin did not receive the identical timed command sequence")
    for key in ("robot_model_id", "robot_calibration_id", "controller_id"):
        if reference["provenance"][key] != predicted["provenance"][key]:
            raise InvalidLog(f"Frozen robot/control mismatch: {key}")
    t = np.asarray(reference["time_s"], float)
    pt = np.asarray(predicted["time_s"], float)
    if pt[0] > t[0] + 1e-6 or pt[-1] < t[-1] - 1e-6:
        raise InvalidLog("Twin horizon cannot truncate a difficult part of a reference probe")
    # No time warping or per-trial latency alignment: onset/delay are part of D.
    rs = {k: np.asarray(v, float) for k, v in reference["signals"].items()}
    ps = {k: _interp(t, pt, v) for k, v in predicted["signals"].items()}
    # Projection makes linear interpolation of nearby rotation matrices valid.
    U, _, Vt = np.linalg.svd(ps["ee_T_world_tcp"][:, :3, :3])
    R = U @ Vt
    U[np.linalg.det(R) < 0, :, -1] *= -1
    ps["ee_T_world_tcp"][:, :3, :3] = U @ Vt
    return t, rs, ps


def _start_time(t, position, threshold_m):
    distance = np.linalg.norm(position - position[0], axis=1)
    # At least 50 ms above a 3-sigma distance threshold; a single jitter spike
    # is not motion onset. No object angle or estimated hinge is needed.
    for i in np.flatnonzero(distance >= threshold_m):
        j = np.searchsorted(t, t[i] + .05)
        if j < len(t) and np.all(distance[i:j + 1] >= threshold_m):
            return float(t[i])
    return None


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def _current_comparable(reference, predicted, signal):
    a = reference.get("sensor_calibration", {}).get(signal, {})
    b = predicted.get("sensor_calibration", {}).get(signal, {})
    return (a.get("independently_calibrated") is True and
            b.get("independently_calibrated") is True and
            bool(a.get("calibration_id")) and a.get("calibration_id") == b.get("calibration_id"))


def noise_normalized_distance(reference: Mapping, predicted: Mapping, noise: NoiseScales,
                              *, use_current=False) -> dict:
    """Time-aligned motion loss, preserving onset, velocity and tracking error.

    Each component receives equal weight after physical noise normalization.
    Current/SDK effort is optional, never called external joint torque.
    """
    noise.validate()
    t, r, p = _aligned(reference, predicted)
    if noise.calibration_id != reference["provenance"]["robot_calibration_id"]:
        raise InvalidLog("Noise/calibration IDs differ; TEST assets cannot retune robot noise")
    rp = r["ee_T_world_tcp"][:, :3, 3]
    pp = p["ee_T_world_tcp"][:, :3, 3]
    rv = np.gradient(rp, t, axis=0)
    pv = np.gradient(pp, t, axis=0)
    relative_R = p["ee_T_world_tcp"][:, :3, :3] @ np.transpose(r["ee_T_world_tcp"][:, :3, :3], (0, 2, 1))
    angles = np.arccos(np.clip((np.trace(relative_R, axis1=1, axis2=2) - 1.) / 2., -1., 1.))
    measured = {
        "ee_position_rmse_m": _rms(pp - rp),
        "ee_rotation_rmse_rad": _rms(angles),
        "q_rmse_rad": _rms(p["q_rad"] - r["q_rad"]),
        "qdot_rmse_rad_s": _rms(p["qdot_rad_s"] - r["qdot_rad_s"]),
        "ee_velocity_rmse_m_s": _rms(pv - rv),
        "final_displacement_error_m": float(np.linalg.norm((pp[-1] - pp[0]) - (rp[-1] - rp[0]))),
    }
    scales = {"ee_position_rmse_m": noise.position_m, "ee_rotation_rmse_rad": noise.rotation_rad,
              "q_rmse_rad": noise.joint_rad, "qdot_rmse_rad_s": noise.joint_velocity_rad_s,
              "ee_velocity_rmse_m_s": noise.ee_velocity_m_s,
              "final_displacement_error_m": noise.position_m}
    rs = _start_time(t, rp, 3. * noise.position_m)
    ps = _start_time(t, pp, 3. * noise.position_m)
    measured["start_time_error_s"] = (0. if rs is None and ps is None else
                                      abs((t[-1] if rs is None else rs) - (t[-1] if ps is None else ps)))
    scales["start_time_error_s"] = noise.start_time_s
    commands = reference["commands"]
    fields = commands["fields"]
    cartesian_fields = ["ee_reference_x_m", "ee_reference_y_m", "ee_reference_z_m"]
    tracking_source = "joint_command"
    if all(f in fields for f in cartesian_fields):
        target = _interp(t, commands["time_s"], np.asarray(commands["values"])[:, [fields.index(f) for f in cartesian_fields]])
        measured["tracking_error_profile_rmse_m"] = _rms(np.linalg.norm(target - pp, axis=1) - np.linalg.norm(target - rp, axis=1))
        scales["tracking_error_profile_rmse_m"] = noise.tracking_m
        tracking_source = "cartesian_reference"
    else:
        # Position commands are optional for compliant velocity/effort probes.
        # The full response to the identical timed input is still compared.
        tracking_source = "no_position_reference_in_command; trajectory_response_used"
    motion_metric_names = list(scales)
    used_auxiliary = []
    if use_current:
        for signal, scale in (("motor_current_a", noise.current_a), ("sdk_effort", noise.effort)):
            if signal not in r or signal not in p or not _current_comparable(reference, predicted, signal):
                continue
            if scale is None:
                raise InvalidLog(f"Calibrated {signal} requires a frozen noise scale")
            key = signal + "_rmse"
            measured[key] = _rms(p[signal] - r[signal])
            scales[key] = scale
            used_auxiliary.append(signal)
        if not used_auxiliary:
            raise InvalidLog("B3 unavailable: no independently calibrated, comparable current/effort")
    normalized = {key: measured[key] / scale for key, scale in scales.items()}
    return {"normalized_loss": float(np.mean(np.square(list(normalized.values())))),
            "motion_only_normalized_loss": float(np.mean([normalized[key] ** 2 for key in motion_metric_names])),
            "metrics": measured, "noise_normalized_metrics": normalized,
            "reference_start_s": rs, "predicted_start_s": ps,
            "tracking_source": tracking_source, "auxiliary_signals_used": used_auxiliary,
            "comparison_samples": len(t), "time_warping": False}


def _validate_inertia_prior(prior):
    """Accept a known scalar or a frozen model reference without fake units."""
    if isinstance(prior, str):
        if not prior.strip():
            raise ValueError("Fixed inertia-model reference cannot be empty")
        return prior
    if isinstance(prior, Mapping):
        allowed = {"kind", "sha256", "scalar_J_eff", "frozen"}
        if (set(prior) != allowed or prior["kind"] != "fixed_articulated_model" or
                prior["frozen"] is not True or prior["scalar_J_eff"] is not None or
                not isinstance(prior["sha256"], str) or len(prior["sha256"]) != 64 or
                any(c not in "0123456789abcdef" for c in prior["sha256"].lower())):
            raise ValueError("Malformed frozen articulated inertia-model handle")
        return dict(prior)
    if isinstance(prior, (bool, np.bool_)) or not np.isfinite(prior) or prior <= 0:
        raise ValueError("J_eff must be a positive fixed prior or an immutable model handle")
    return float(prior)


def candidate_grid(tau_c_values: Sequence[float], b_values: Sequence[float], *,
                   J_eff_prior, budget: int) -> list[dict]:
    """Deterministic bounded grid; these are effective simulator parameters."""
    J_eff_prior = _validate_inertia_prior(J_eff_prior)
    tau = sorted(set(float(x) for x in tau_c_values))
    damping = sorted(set(float(x) for x in b_values))
    if len(tau) < 2 or len(damping) < 2 or any(not np.isfinite(x) or x < 0 for x in tau + damping):
        raise ValueError("Both nonnegative parameter axes need at least two finite values")
    if len(tau) * len(damping) > budget:
        raise ValueError("Frozen system-identification simulation budget exceeded")
    return [{"candidate_id": f"phi_{i:03d}", "tau_c": c, "b": d,
             "J_eff_prior": J_eff_prior} for i, (c, d) in enumerate((c, d) for c in tau for d in damping)]


def _signature(log, noise, count=64):
    validate_log(log)
    t = np.asarray(log["time_s"])
    tt = np.linspace(t[0], t[-1], count)
    s = log["signals"]
    p = np.asarray(s["ee_T_world_tcp"])[:, :3, 3]
    velocity = np.gradient(p, t, axis=0)
    return np.r_[_interp(tt, t, p).ravel() / noise.position_m,
                 _interp(tt, t, s["q_rad"]).ravel() / noise.joint_rad,
                 _interp(tt, t, s["qdot_rad_s"]).ravel() / noise.joint_velocity_rad_s,
                 _interp(tt, t, velocity).ravel() / noise.ee_velocity_m_s]


def sensitivity_report(conditions: Mapping[str, Sequence[Mapping]], noise: NoiseScales,
                       *, parameter_values: Mapping[str, Mapping] | None = None) -> dict:
    """LOW/MEDIUM/HIGH observability against repeated signal variability.

    Hidden reference parameters are not needed. ``parameter_values`` is only
    for a *twin candidate grid* whose parameters belong to the estimator, never
    the hidden reference. Identifying two parameters requires rank two.
    """
    noise.validate()
    if len(conditions) < 2 or any(not logs for logs in conditions.values()):
        raise InvalidLog("Sensitivity needs at least two nonempty conditions")
    anchor = next(iter(conditions.values()))[0]
    if anchor["provenance"]["robot_calibration_id"] != noise.calibration_id:
        raise InvalidLog("Sensitivity noise/calibration IDs differ")
    means, variability = [], []
    conditions_json = {}
    names = list(conditions)
    for name, logs in conditions.items():
        signatures = []
        for log in logs:
            _aligned(anchor, log)
            signatures.append(_signature(log, noise))
        signatures = np.asarray(signatures)
        mean = signatures.mean(0)
        means.append(mean)
        variability.extend((_rms(row - mean) for row in signatures))
        conditions_json[name] = {"repeats": len(logs), "within_condition_rms_noise_units": _rms(signatures - mean)}
    means = np.asarray(means)
    repeat_floor = max(1., max(variability, default=0.))
    pairwise = [{"a": names[i], "b": names[j],
                 "response_difference_noise_units": _rms(means[i] - means[j]),
                 "difference_over_repeat_floor": _rms(means[i] - means[j]) / repeat_floor}
                for i in range(len(names)) for j in range(i + 1, len(names))]
    observable = max(row["difference_over_repeat_floor"] for row in pairwise) > 1.
    rank = None
    singular = []
    if parameter_values is not None:
        X = np.asarray([[parameter_values[name]["tau_c"], parameter_values[name]["b"]] for name in names], float)
        ranges = np.ptp(X, axis=0)
        if np.any(ranges <= 0):
            rank = 0
        else:
            X = (X - X.mean(0)) / ranges
            design_rank = int(np.linalg.matrix_rank(X))
            jacobian = np.linalg.lstsq(X, (means - means.mean(0)) / repeat_floor, rcond=None)[0]
            singular = (np.linalg.svd(jacobian, compute_uv=False) / np.sqrt(means.shape[1])).tolist()
            rank = min(design_rank, sum(x > 1. for x in singular))
    return {"status": "OBSERVABLE_RESPONSE" if observable else "UNIDENTIFIABLE_UNDER_CURRENT_PROBE",
            "parameter_rank": rank, "normalized_singular_values": singular,
            "repeat_floor_noise_units": repeat_floor, "conditions": conditions_json,
            "pairwise": pairwise, "repeat_variability_measured": all(len(logs) >= 2 for logs in conditions.values()),
            "robot_calibration_id": noise.calibration_id,
            "robot_model_id": anchor["provenance"]["robot_model_id"],
            "controller_id": anchor["provenance"]["controller_id"],
            "rank_scope": "two-parameter rank applies only to estimator-owned twin grid"}


def fit_resistance(reference_train: Mapping[str, Mapping], rollouts: Sequence[Mapping],
                   noise: NoiseScales, *, J_eff_prior, budget: int,
                   use_current=False, profile_loss_delta=1., sensitivity_evidence=None,
                   expected_grid=None, censored_candidates=(), planned_grid=None) -> dict:
    """Fit tau_c/b from actual cached simulator rollouts, never held-out P4.

    Each rollout is {candidate_id,tau_c,b,J_eff_prior,probes:{P1:log,...}}.
    Profile intervals are *grid support sets*, not statistical 95% CIs.
    A flat/rank-deficient result keeps T2 unaccepted rather than inventing a fit.
    """
    J_eff_prior = _validate_inertia_prior(J_eff_prior)
    if expected_grid is not None and planned_grid is not None:
        raise InvalidLog("Specify expected_grid or planned_grid, not both")
    expected_grid = expected_grid if expected_grid is not None else planned_grid
    grid_verified = expected_grid is not None
    if expected_grid is not None:
        if not expected_grid or len(expected_grid) > budget:
            raise InvalidLog("Frozen full grid is empty or exceeds the candidate budget")
        if len({c["candidate_id"] for c in expected_grid}) != len(expected_grid):
            raise InvalidLog("Frozen full grid has duplicate candidate IDs")
        for candidate in expected_grid:
            if candidate["J_eff_prior"] != J_eff_prior:
                raise InvalidLog("Frozen full grid changed the fixed inertia prior")
            if not np.isfinite([candidate["tau_c"], candidate["b"]]).all() or min(candidate["tau_c"], candidate["b"]) < 0:
                raise InvalidLog("Invalid frozen-grid resistance candidate")
    if set(reference_train) != set(TRAIN_PROBES):
        raise InvalidLog("Physics fit requires frozen P1/P2/P3 only; P4 is held out")
    if not rollouts or len(rollouts) > budget:
        raise InvalidLog("No independently simulated candidates or budget exceeded")
    for probe, log in reference_train.items():
        validate_log(log)
        if log["probe_id"] != probe or log["split"] != "train":
            raise InvalidLog("Held-out data leakage into fitting")
    rows, completed = [], []
    for candidate in rollouts:
        if candidate["J_eff_prior"] != J_eff_prior:
            raise InvalidLog("J_eff prior changed while fitting resistance")
        c, b = float(candidate["tau_c"]), float(candidate["b"])
        if not np.isfinite([c, b]).all() or min(c, b) < 0:
            raise InvalidLog("Invalid resistance candidate")
        if set(candidate["probes"]) != set(TRAIN_PROBES):
            raise InvalidLog("Every candidate requires all P1/P2/P3; failed probes cannot be dropped")
        distances = {}
        for probe in TRAIN_PROBES:
            log = candidate["probes"][probe]
            if log["split"] != "train":
                raise InvalidLog("Held-out rollout used during physics fitting")
            distances[probe] = noise_normalized_distance(reference_train[probe], log, noise, use_current=use_current)
        row = {"candidate_id": candidate["candidate_id"], "tau_c": c, "b": b,
               "loss": float(np.mean([v["normalized_loss"] for v in distances.values()])), "probes": distances}
        rows.append(row)
        completed.append(candidate)
    if len({r["candidate_id"] for r in rows}) != len(rows):
        raise InvalidLog("Duplicate candidate ids")
    full_grid = list(expected_grid) if expected_grid is not None else completed
    full_by_id = {row["candidate_id"]: row for row in full_grid}
    completed_ids = {row["candidate_id"] for row in completed}
    for row in completed:
        expected = full_by_id.get(row["candidate_id"])
        if expected is None or any(row[key] != expected[key] for key in ("tau_c", "b", "J_eff_prior")):
            raise InvalidLog("Executed candidate differs from the frozen full grid")
    censored_by_id = {row["candidate_id"]: dict(row) for row in censored_candidates}
    if set(censored_by_id) - set(full_by_id):
        raise InvalidLog("Censored candidate is outside the frozen full grid")
    if completed_ids & set(censored_by_id):
        raise InvalidLog("A candidate cannot be both fully observed and censored")
    for cid in set(full_by_id) - completed_ids:
        censored_by_id.setdefault(cid, {"candidate_id": cid, "status": "NOT_RUN_OR_INCOMPLETE"})
    best = min(rows, key=lambda row: row["loss"])
    plausible = [row for row in rows if row["loss"] <= best["loss"] + profile_loss_delta]
    intervals = {key: [min(row[key] for row in plausible), max(row[key] for row in plausible)] for key in ("tau_c", "b")}
    diagnostic_profile = dict(intervals)
    searched_ranges = {key: [min(row[key] for row in full_grid), max(row[key] for row in full_grid)] for key in ("tau_c", "b")}
    if censored_by_id:
        # Missing/unsafe responses carry unknown likelihood. They cannot be
        # silently excluded to manufacture tight support or parameter rank.
        intervals = dict(searched_ranges)
    boundary_limited = {key: (intervals[key][1] == bounds[1] or
                              (intervals[key][0] == bounds[0] and bounds[0] > 0.))
                        for key, bounds in searched_ranges.items()}
    # Stack all three independently excited protocols before taking rank.
    vectors = np.asarray([np.concatenate([_signature(c["probes"][probe], noise) for probe in TRAIN_PROBES]) for c in completed])
    X = np.asarray([[c["tau_c"], c["b"]] for c in completed])
    ranges = np.ptp(X, axis=0)
    singular = []
    rank = 0
    repeat_floor = max(1., float((sensitivity_evidence or {}).get("repeat_floor_noise_units", 1.)))
    if not np.isfinite(repeat_floor):
        raise InvalidLog("Sensitivity repeat floor must be finite")
    if np.all(ranges > 0):
        centered = (X - X.mean(0)) / ranges
        J = np.linalg.lstsq(centered, vectors - vectors.mean(0), rcond=None)[0]
        singular = (np.linalg.svd(J, compute_uv=False) / np.sqrt(vectors.shape[1]) / repeat_floor).tolist()
        rank = min(int(np.linalg.matrix_rank(centered)), sum(x > 1. for x in singular))
    wide = any((intervals[key][1] - intervals[key][0]) >= .8 * ranges[i]
               for i, key in enumerate(("tau_c", "b")))
    sensitivity_passed = (sensitivity_evidence is not None and
                          sensitivity_evidence.get("status") == "OBSERVABLE_RESPONSE" and
                          sensitivity_evidence.get("repeat_variability_measured") is True)
    if sensitivity_evidence is not None:
        provenance = next(iter(reference_train.values()))["provenance"]
        if any(sensitivity_evidence.get(key) != provenance[key]
               for key in ("robot_model_id", "robot_calibration_id", "controller_id")):
            raise InvalidLog("Sensitivity evidence is not from the same frozen robot/calibration/controller")
    identifiable = (rank == 2 and not wide and sensitivity_passed and not any(boundary_limited.values())
                    and grid_verified and not censored_by_id)
    status = ("SENSITIVITY_NOT_VALIDATED" if sensitivity_evidence is None else
              "GRID_COVERAGE_NOT_VALIDATED" if not grid_verified else
              "IDENTIFIABLE_ON_FROZEN_GRID" if identifiable else "UNIDENTIFIABLE_UNDER_CURRENT_PROBE")
    return {"status": status,
            "mode": next(iter(reference_train.values()))["mode"],
            "parameter_semantics": "effective simulator resistance parameters; not calibrated real hinge torque",
            "accepted_parameters": {key: best[key] for key in ("tau_c", "b")} if identifiable else None,
            "best_grid_candidate_diagnostic": {key: best[key] for key in ("candidate_id", "tau_c", "b", "loss")},
            "parameter_intervals": intervals, "interval_semantics": "discrete profile-loss support; not a confidence interval",
            "searched_parameter_ranges": searched_ranges, "grid_boundary_limited": boundary_limited,
            "full_grid_coverage_verified": grid_verified, "grid_censored": bool(censored_by_id),
            "censored_candidates": [censored_by_id[key] for key in sorted(censored_by_id)],
            "diagnostic_observed_grid_profile_intervals": diagnostic_profile,
            "best_fit_rms_noise_units": float(np.sqrt(best["loss"])),
            "model_adequacy": {"accepted_as_noise_consistent": None,
                               "diagnostic": "Best normalized residual is reported without a post-hoc TEST adequacy threshold; narrow grid support does not exclude robot/kinematic/contact model mismatch"},
            "profile_loss_delta": profile_loss_delta, "sensitivity_rank": rank,
            "rank_repeat_floor_noise_units": repeat_floor,
            "normalized_singular_values": singular, "J_eff_prior_fixed": J_eff_prior,
            "train_probe_ids": list(TRAIN_PROBES), "heldout_probe_ids": list(HELDOUT_PROBES),
            "simulator_candidates_used": len(rows), "simulation_budget": budget,
            "budget_units": "parameter candidates, each requires all three training probe segments",
            "probe_segments_evaluated": len(rows) * len(TRAIN_PROBES),
            "physics_rollouts_used": None,
            "rollout_count_note": "Count actual Isaac jobs in runner; three sequential probe segments may belong to one independently evolving rollout",
            "reference_sensitivity_passed": sensitivity_passed,
            "current_or_effort_used": bool(use_current), "candidate_losses": rows}


def heldout_comparison(reference: Mapping, predictions: Mapping[str, Mapping],
                       noise: NoiseScales) -> dict:
    """Compare B0/T0, B1/T1, B2/T2, optional B3, and diagnostic Oracle."""
    if reference["probe_id"] not in HELDOUT_PROBES or reference["split"] != "heldout":
        raise InvalidLog("Held-out validation requires the predeclared P4 action")
    validate_log(reference, independent_sim=reference["provenance"]["source"] == "isaac_physics")
    if reference["provenance"].get("complete") is not True:
        raise InvalidLog("Held-out reference action is incomplete")
    unknown = set(predictions) - {"B0", "B1", "B2", "B3", "Oracle"}
    if unknown:
        raise InvalidLog(f"Unknown ablation labels: {unknown}")
    rows = {}
    for name in ("B0", "B1", "B2", "B3", "Oracle"):
        if name not in predictions:
            rows[name] = {"status": "NOT_RUN", "reason": "No independent held-out rollout supplied"}
            continue
        predicted = predictions[name]
        if predicted["split"] != "heldout":
            raise InvalidLog("Training rollout passed as held-out prediction")
        try:
            result = noise_normalized_distance(reference, predicted, noise, use_current=name == "B3")
        except InvalidLog as error:
            if name == "B3" and str(error).startswith("B3 unavailable"):
                rows[name] = {"status": "UNAVAILABLE", "reason": str(error)}
                continue
            raise
        rows[name] = {"status": "EVALUATED", **result}
    improvements = {}
    if rows["B2"]["status"] == "EVALUATED":
        for baseline in ("B0", "B1"):
            if rows[baseline]["status"] == "EVALUATED":
                old, new = rows[baseline]["normalized_loss"], rows["B2"]["normalized_loss"]
                improvements[baseline] = {"absolute_loss_reduction": old - new,
                                          "relative_loss_reduction": None if old <= 1e-12 else (old - new) / old}
    return {"mode": reference["mode"], "probe_id": "P4", "fitting_used_this_action": False,
            "methods": rows, "B2_improvement": improvements,
            "twin_mapping": {"B0": "T0 initial", "B1": "T1 kinematics only", "B2": "T2 kinematics + motion-only physics"},
            "method_scope": "internal project baselines; not official Act2See results",
            "oracle_scope": "diagnostic upper bound only; excluded from estimator inputs"}


def write_analysis(output_dir, name: str, report: Mapping):
    """Write strict JSON and a compact candidate/ablation CSV, without raw GT."""
    path = Path(output_dir)
    path.mkdir(parents=True, exist_ok=True)
    (path / f"{name}.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    rows = report.get("candidate_losses", [])
    if rows:
        with (path / f"{name}.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=["candidate_id", "tau_c", "b", "loss"])
            writer.writeheader()
            writer.writerows({key: row[key] for key in writer.fieldnames} for row in rows)
