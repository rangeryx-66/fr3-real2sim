"""Scalable Real2Sim mass/CoM adapter for the measured FR3 records.

The grasp executor and the scan/asset pipeline are deliberately outside this
module.  It consumes an already recorded robot-alone/payload pair and only
identifies the four gravity parameters ``[m, m*cx, m*cy, m*cz]``.  The
rotational inertia is kept fixed through the upstream
``MassAndComInertialParameter`` parameterization; it is not identified here.

The official project identifies an IIWA with a Drake plant.  That plant cannot
be substituted for this repository's FR3 model without changing the robot
model.  This adapter therefore reuses the official, model-independent pieces
(Butterworth filtering and the mass/CoM-only physical parameterization) and
uses the existing FR3 TCP gravity regressor plus measured robot-alone torque
subtraction.  The distinction is written into every output report.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .payload_id_v2 import (
    STATIC_PARAMETER_ORDER,
    _static_mask,
    _time,
    align_baseline,
    load_record,
    static_gravity_regressor,
    robust_linear_fit,
)


SCHEMA = "real2sim/scalable_payload_mass_com/v1"
OFFICIAL_COMMIT = "c52e31cf26c83b33aee5e56f805e1d4d710fd549"
DEFAULT_OFFICIAL_ROOT = Path(
    "/data1/home/rangeryx/fr3_moveit_grasp/third_party/"
    "scalable_real2sim_robot_payload_id_upstream_c52e31c"
)


@dataclass
class _OfficialJointData:
    """The five fields consumed by the upstream filtering module.

    Loading the official ``dataclasses.py`` imports the whole upstream utility
    package (including wandb).  A small structural shim lets us execute the
    exact upstream ``filtering.py`` without importing unrelated utilities or
    changing the vendor checkout.
    """

    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_accelerations: np.ndarray
    joint_torques: np.ndarray
    sample_times_s: np.ndarray


def _load_official_filtering(
    official_root: Path,
) -> tuple[Callable[..., np.ndarray], Callable[..., _OfficialJointData]]:
    """Load the upstream filtering functions without importing wandb-heavy utils."""

    filtering_path = Path(official_root) / "robot_payload_id" / "utils" / "filtering.py"
    if not filtering_path.exists():
        raise FileNotFoundError(f"official filtering.py not found: {filtering_path}")

    package_name = "_scalable_real2sim_payload_utils"
    data_name = f"{package_name}.dataclasses"
    filtering_name = f"{package_name}.filtering"
    package = types.ModuleType(package_name)
    package.__path__ = []  # type: ignore[attr-defined]
    data_module = types.ModuleType(data_name)
    data_module.JointData = _OfficialJointData  # type: ignore[attr-defined]
    # Replace stale modules from a previous call so tests can use another
    # checkout in the same Python process deterministically.
    sys.modules[package_name] = package
    sys.modules[data_name] = data_module
    sys.modules.pop(filtering_name, None)
    spec = importlib.util.spec_from_file_location(filtering_name, filtering_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load official filtering module: {filtering_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[filtering_name] = module
    spec.loader.exec_module(module)
    return module.filter_time_series_data, module.process_joint_data


def _official_mass_com_class(official_root: Path):
    """Import the exact upstream mass/CoM-only torch module."""

    root = str(Path(official_root).resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    try:
        from robot_payload_id.eric_id.drake_torch_dynamics import (  # type: ignore
            MassAndComInertialParameter,
        )
    except Exception as exc:  # pragma: no cover - exercised on missing optional env
        raise RuntimeError(
            "official MassAndComInertialParameter is unavailable; run with the "
            "Scalable Real2Sim/Drake environment (official_payload_env)"
        ) from exc
    return MassAndComInertialParameter


def _sampling_rate(t: np.ndarray) -> float:
    dt = np.diff(np.asarray(t, dtype=float))
    if len(dt) == 0 or not np.all(np.isfinite(dt)) or np.any(dt <= 0):
        raise ValueError("record has no valid sample period")
    return float(1.0 / np.median(dt))


def _official_process_torque(
    record: dict[str, np.ndarray],
    process_joint_data: Callable[..., _OfficialJointData],
    filter_time_series_data: Callable[..., np.ndarray],
) -> tuple[np.ndarray, dict[str, Any]]:
    """Apply the upstream process/filter settings to a v2 capture.

    ``process_joint_data`` is the primary path and uses the official defaults
    (torque order 12, cutoff 1.6 Hz).  The direct function is a small audit
    fallback for legacy records whose timestamps are too short for the
    derivative side of the upstream helper.  Both paths still use the exact
    upstream Butterworth implementation.
    """

    t = _time(record)
    q = np.asarray(record["q"], dtype=float)
    dq = np.asarray(record.get("dq", np.zeros_like(q)), dtype=float)
    ddq = np.asarray(record.get("ddq_ref", np.zeros_like(q)), dtype=float)
    tau = np.asarray(record["tau"], dtype=float)
    fs = _sampling_rate(t)
    try:
        data = _OfficialJointData(
            joint_positions=q,
            joint_velocities=dq,
            joint_accelerations=ddq,
            joint_torques=tau,
            sample_times_s=t,
        )
        processed = process_joint_data(
            joint_data=data,
            num_endpoints_to_remove=0,
            compute_velocities=False,
            filter_positions=False,
            vel_filter_order=20,
            vel_cutoff_freq_hz=2.0,
            acc_filter_order=20,
            acc_cutoff_freq_hz=2.0,
            torque_filter_order=12,
            torque_cutoff_freq_hz=1.6,
        )
        filtered = np.asarray(processed.joint_torques, dtype=float)
        return filtered, {
            "implementation": "official.process_joint_data",
            "torque_filter_order": 12,
            "torque_cutoff_hz": 1.6,
            "sample_hz": fs,
        }
    except Exception as exc:
        filtered = filter_time_series_data(
            tau, order=12, cutoff_freq_hz=1.6, fs_hz=fs, visualize=False
        )
        return np.asarray(filtered, dtype=float), {
            "implementation": "official.filter_time_series_data_fallback",
            "torque_filter_order": 12,
            "torque_cutoff_hz": 1.6,
            "sample_hz": fs,
            "process_joint_data_fallback_reason": str(exc),
        }


def _interp_columns(values: np.ndarray, source_t: np.ndarray, target_t: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [np.interp(target_t, source_t, values[:, j]) for j in range(values.shape[1])]
    )


def _contiguous_post_settle_indices(
    idx: np.ndarray, *, rate_hz: float, settle_s: float
) -> np.ndarray:
    """Discard the settling prefix while retaining a complete static window."""

    idx = np.asarray(idx, dtype=int)
    if len(idx) == 0:
        return idx
    # A pose id is generated by the recorder's interpolated protocol.  Keep
    # only the longest contiguous run so a transition sample cannot leak into a
    # static average.
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks + 1, len(idx)]
    run = int(np.argmax(ends - starts))
    longest = idx[starts[run] : ends[run]]
    trim = int(round(max(0.0, settle_s) * rate_hz))
    if len(longest) - trim < 8:
        # Keep a short window for diagnostics, but never include the first
        # sample when an actual settle prefix is available.
        trim = min(max(0, len(longest) - 8), trim)
    return longest[trim:]


def _relative_motion_guard(record: dict[str, np.ndarray]) -> dict[str, Any]:
    """Audit per-sample object/TCP motion when the recorder provides it.

    Existing v2 captures expose a capture-level ``guard_passed`` flag but not a
    full ``T_TCP_object`` stream.  We do not reconstruct that stream from GT;
    the report states the evidence level explicitly.  Future captures can add
    ``T_TCP_object`` and will automatically receive the stricter audit.
    """

    for key in ("T_TCP_object", "T_TCP_target", "T_TCP_payload"):
        if key not in record:
            continue
        transforms = np.asarray(record[key], dtype=float)
        if transforms.ndim != 3 or transforms.shape[1:] != (4, 4):
            return {"status": "INVALID_RELATIVE_POSE_STREAM", "key": key}
        p = transforms[:, :3, 3]
        # Rotation angle from the first sample, only for an audit—not a GT
        # estimate.  A guard failure is intentionally fail-closed.
        from scipy.spatial.transform import Rotation

        r0 = Rotation.from_matrix(transforms[0, :3, :3])
        angles = np.asarray(
            [np.linalg.norm((r0.inv() * Rotation.from_matrix(R[:3, :3])).as_rotvec())
             for R in transforms]
        )
        translation = np.linalg.norm(p - p[0], axis=1)
        return {
            "status": "PASS" if float(np.max(translation)) < 0.002 and float(np.max(angles)) < np.deg2rad(3.0) else "FAIL",
            "key": key,
            "max_translation_m": float(np.max(translation)),
            "max_rotation_rad": float(np.max(angles)),
            "samples": int(len(transforms)),
        }
    return {
        "status": "CAPTURE_FLAG_ONLY",
        "detail": "v2 NPZ has guard_passed but no per-sample T_TCP_object stream",
    }


def _relative_window_mask(
    record: dict[str, np.ndarray],
    pose_id: np.ndarray,
    hold: np.ndarray,
    *,
    max_translation_m: float = 0.002,
    max_rotation_rad: float = np.deg2rad(3.0),
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """Mark static windows whose grasp transform remains rigid.

    The capture-level guard is necessary but not sufficient for a static
    pairing: a short slip can occur during one orientation and then settle.
    When the simulator provides ``T_TCP_object`` we reject that orientation
    only, preserving the other poses for diagnostics.  No GT pose is used.
    """

    n = len(pose_id)
    mask = np.ones(n, dtype=bool)
    if "T_TCP_object" not in record:
        return mask, [{"status": "MISSING_STREAM", "used": False}]
    transforms = np.asarray(record["T_TCP_object"], dtype=float)
    if transforms.ndim != 3 or transforms.shape != (n, 4, 4):
        return np.zeros(n, dtype=bool), [{"status": "INVALID_STREAM", "used": False}]
    details: list[dict[str, Any]] = []
    for pid in sorted(int(x) for x in np.unique(pose_id[hold]) if x >= 0):
        idx = np.flatnonzero(hold & (pose_id == pid))
        if len(idx) == 0:
            continue
        ref = transforms[idx[0]]
        rel = np.linalg.inv(ref)[None, ...] @ transforms[idx]
        translation = np.linalg.norm(rel[:, :3, 3], axis=1)
        from scipy.spatial.transform import Rotation

        rotation = np.asarray([Rotation.from_matrix(x[:3, :3]).magnitude() for x in rel])
        max_t = float(np.max(translation))
        max_r = float(np.max(rotation))
        passed = bool(max_t <= max_translation_m and max_r <= max_rotation_rad)
        if not passed:
            mask[idx] = False
        details.append({
            "pose_id": pid,
            "samples": int(len(idx)),
            "max_translation_m": max_t,
            "max_rotation_rad": max_r,
            "used": passed,
            "reason": None if passed else "RELATIVE_POSE_WINDOW",
        })
    return mask, details


def _actual_pose_pairing_mask(
    baseline: dict[str, np.ndarray],
    payload: dict[str, np.ndarray],
    pose_id: np.ndarray,
    hold: np.ndarray,
    *,
    max_q_error_rad: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Require empty-arm and payload captures to occupy the same measured q.

    The command-side ``q_ref`` check catches protocol mixups.  This second
    check catches load-dependent tracking offsets, which otherwise look like
    a torque residual.  Windows outside static holds are ignored.
    """

    n = len(pose_id)
    mask = np.ones(n, dtype=bool)
    tb, tp = _time(baseline), _time(payload)
    q0 = _interp_columns(np.asarray(baseline["q"], dtype=float), tb, tp)
    q1 = np.asarray(payload["q"], dtype=float)
    error = np.linalg.norm(q1 - q0, axis=1)
    details: list[dict[str, Any]] = []
    for pid in sorted(int(x) for x in np.unique(pose_id[hold]) if x >= 0):
        idx = np.flatnonzero(hold & (pose_id == pid))
        if len(idx) == 0:
            continue
        max_error = float(np.max(error[idx]))
        p95 = float(np.quantile(error[idx], 0.95))
        passed = bool(max_error <= max_q_error_rad)
        if not passed:
            mask[idx] = False
        details.append({
            "pose_id": pid,
            "samples": int(len(idx)),
            "max_q_error_rad": max_error,
            "p95_q_error_rad": p95,
            "used": passed,
            "reason": None if passed else "ACTUAL_POSE_MISMATCH",
        })
    return mask, {
        "max_q_error_rad": float(max_q_error_rad),
        "max_error_rad": float(np.max(error[hold])) if np.any(hold) else None,
        "p95_error_rad": float(np.quantile(error[hold], 0.95)) if np.any(hold) else None,
        "postures": details,
    }


def _build_static_observations(
    baseline: dict[str, np.ndarray],
    payload: dict[str, np.ndarray],
    protocol: dict[str, Any] | None,
    baseline_tau: np.ndarray,
    payload_tau: np.ndarray,
    *,
    settle_s: float,
    min_postures: int,
    max_pair_q_error_rad: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Synchronize filtered torques and average settled orientation holds."""

    # This performs the existing protocol/q_ref pairing check.  Its returned
    # raw baseline torque is intentionally ignored; each stream is filtered
    # separately below to match the upstream processing contract.
    align_baseline(baseline, payload)
    tb, tp = _time(baseline), _time(payload)
    baseline_on_payload = _interp_columns(baseline_tau, tb, tp)
    delta = np.asarray(payload_tau, dtype=float) - baseline_on_payload
    from .payload_jacobian import tcp_jacobian_record
    Yall = static_gravity_regressor(tcp_jacobian_record(payload))
    hold, pose_id = _static_mask(payload, protocol)
    rate = _sampling_rate(tp)
    valid = (
        hold
        & (pose_id >= 0)
        & np.isfinite(delta).all(axis=1)
        & np.isfinite(Yall).all(axis=(1, 2))
    )
    relative_mask, relative_details = _relative_window_mask(payload, pose_id, hold)
    pairing_mask, pairing_details = _actual_pose_pairing_mask(
        baseline, payload, pose_id, hold, max_q_error_rad=max_pair_q_error_rad
    )
    valid &= relative_mask & pairing_mask

    # The empty-arm and payload captures are separate Isaac processes, so
    # their recorder clocks have unrelated origins and their transition
    # durations can differ by a few samples.  Interpolating the baseline by
    # relative time (the historical path above) then leaks a transition or
    # the neighbouring hold into a static payload window.  When both records
    # carry the protocol's static pose IDs, pair the *settled means by pose
    # ID* instead.  This is the strict identical-pose pairing contract: the
    # measured-q check above still rejects a load-dependent tracking mismatch,
    # while the torque subtraction itself is no longer clock-dependent.
    baseline_hold, baseline_pose_id = _static_mask(baseline, protocol)
    baseline_ids = np.asarray(baseline_pose_id, dtype=int)
    pose_id_pairing = bool(
        len(baseline_hold) == len(baseline_tau)
        and len(baseline_ids) == len(baseline_tau)
        and np.any(baseline_hold)
        and np.any(baseline_ids[np.asarray(baseline_hold, dtype=bool)] >= 0)
    )

    Ys: list[np.ndarray] = []
    ds: list[np.ndarray] = []
    details: list[dict[str, Any]] = []
    for pid in sorted(int(x) for x in np.unique(pose_id[valid]) if x >= 0):
        raw_idx = np.flatnonzero(hold & (pose_id == pid))
        all_idx = np.flatnonzero(valid & (pose_id == pid))
        idx = _contiguous_post_settle_indices(all_idx, rate_hz=rate, settle_s=settle_s)
        baseline_idx = np.flatnonzero(
            np.asarray(baseline_hold, dtype=bool) & (baseline_ids == pid)
        ) if pose_id_pairing else np.asarray([], dtype=int)
        baseline_idx = _contiguous_post_settle_indices(
            baseline_idx, rate_hz=_sampling_rate(tb), settle_s=settle_s
        )
        if len(idx) < 8:
            details.append(
                {
                    "pose_id": pid,
                    "raw_samples": int(len(raw_idx)),
                    "settled_samples": int(len(idx)),
                    "used": False,
                    "reason": "SHORT_AFTER_SETTLE" if len(all_idx) >= 8 else "PAIR_OR_RELATIVE_POSE_REJECT",
                }
            )
            continue
        # Require a complete settled empty-arm window when pose-ID pairing is
        # available.  Falling back to the interpolated stream is reserved for
        # legacy captures without protocol IDs; current calibration runs fail
        # closed instead of silently using a mismatched baseline.
        if pose_id_pairing and len(baseline_idx) < 8:
            details.append(
                {
                    "pose_id": pid,
                    "raw_samples": int(len(raw_idx)),
                    "settled_samples": int(len(idx)),
                    "baseline_samples": int(len(baseline_idx)),
                    "used": False,
                    "reason": "BASELINE_POSE_WINDOW_MISSING",
                }
            )
            continue
        Ys.append(np.mean(Yall[idx], axis=0))
        baseline_mean = (
            np.mean(baseline_tau[baseline_idx], axis=0)
            if pose_id_pairing
            else np.mean(baseline_on_payload[idx], axis=0)
        )
        ds.append(np.mean(payload_tau[idx], axis=0) - baseline_mean)
        paired_residual = payload_tau[idx] - baseline_mean
        details.append(
            {
                "pose_id": pid,
                "raw_samples": int(len(raw_idx)),
                "settled_samples": int(len(idx)),
                "baseline_samples": int(len(baseline_idx)) if pose_id_pairing else None,
                "settle_discarded": int(len(all_idx) - len(idx)),
                "used": True,
                "delta_std_Nm": np.std(paired_residual, axis=0).tolist(),
            }
        )
    if len(Ys) < min_postures:
        raise ValueError(
            f"INSUFFICIENT_STATIC_POSTURES: {len(Ys)} < {min_postures}"
        )
    return (
        np.concatenate(Ys, axis=0),
        np.concatenate(ds, axis=0),
        {
            "rate_hz": rate,
            "posture_count": len(Ys),
            "postures": details,
            "relative_pose_windows": relative_details,
            "actual_pose_pairing": pairing_details,
            "torque_pairing": "static_pose_id_settled_mean" if pose_id_pairing else "relative_time_interpolation_legacy",
            "max_pair_q_error_rad": float(max_pair_q_error_rad),
            "settle_s": float(settle_s),
            "protocol_source": "record.static_hold/static_pose_id",
        },
    )


def _torch_fit_mass_com(
    X: np.ndarray,
    y: np.ndarray,
    official_root: Path,
) -> dict[str, Any]:
    """Fit mass and CoM with the upstream physical parameterization.

    The adapter deliberately uses two stages.  A robust four-parameter linear
    fit first supplies the payload mass.  That scalar is then held fixed while
    only the first moment/CoM is optimized through the upstream
    ``MassAndComInertialParameter`` class.  Keeping mass out of the second
    optimization prevents a poorly observed first moment from trading against
    mass, and keeps the static estimator separate from the (fallback-only)
    rotational inertia prior.
    """

    return _torch_fit_mass_com_fixed_mass(X, y, official_root)


def _torch_fit_mass_com_fixed_mass(
    X: np.ndarray,
    y: np.ndarray,
    official_root: Path,
) -> dict[str, Any]:
    """Two-stage robust mass then fixed-mass first-moment fit.

    ``X`` contains one 7-joint gravity block per settled pose and columns
    ``[m, m*cx, m*cy, m*cz]``.  The first robust fit is used only to determine
    a positive mass.  The second robust fit sees ``y - X[:,0] * mass`` and
    estimates the three first-moment columns with that mass held constant.
    The official nonlinear parameterization is still used for the second
    stage so its physical mass/CoM parameter semantics remain unchanged.
    """

    try:
        import torch
    except Exception as exc:  # pragma: no cover - optional environment
        raise RuntimeError("PyTorch is required for the official parameterization") from exc

    MassAndComInertialParameter = _official_mass_com_class(official_root)
    X = np.asarray(X, dtype=float)
    y = np.asarray(y, dtype=float).reshape(-1)
    if X.ndim != 2 or X.shape[1] != 4 or len(X) != len(y):
        raise ValueError("mass+COM fit expects an N x 4 gravity regressor")

    # Stage 1: robustly determine mass.  The moment columns are retained in
    # this fit so the mass estimate is not biased by an assumed COM; their
    # result is intentionally discarded before stage 2.
    mass_stage = robust_linear_fit(X, y)
    theta_mass = np.asarray(mass_stage["theta"], dtype=float)
    mass0 = float(theta_mass[0])
    if not np.isfinite(mass0) or mass0 <= 0.0:
        theta_ols = np.linalg.lstsq(X, y, rcond=1.0e-10)[0]
        mass0 = float(theta_ols[0])
    if not np.isfinite(mass0) or mass0 <= 0.0:
        raise ValueError("ROBUST_MASS_NONPOSITIVE")

    # Stage 2: robustly fit first moment with mass fixed.  This is a separate
    # regression, so moment noise cannot change the reported mass.
    Xh = X[:, 1:4]
    yh = y - X[:, 0] * mass0
    moment_stage = robust_linear_fit(Xh, yh)
    h0 = np.asarray(moment_stage["theta"], dtype=float)
    com0 = h0 / mass0
    if not np.isfinite(com0).all() or np.linalg.norm(com0) > 0.20:
        com0 = np.zeros(3, dtype=float)
    # A fixed, deliberately conservative rotational inertia prior is only
    # needed to instantiate the official class.  Static gravity has no
    # rotational-inertia columns, and the value is never reported as an
    # identified inertia estimate.
    radius_prior_m = 0.05
    rot0 = np.eye(3, dtype=float) * (mass0 * radius_prior_m**2 / 6.0)
    dtype = torch.double
    module = MassAndComInertialParameter(
        torch.tensor([mass0], dtype=dtype),
        torch.tensor(np.asarray(com0, dtype=float)[None, :], dtype=dtype),
        torch.tensor(np.asarray(rot0, dtype=float)[None, ...], dtype=dtype),
    )
    # ``MassAndComInertialParameter`` exposes a log-mass parameter by design;
    # freeze it explicitly for the second stage.  The rotational inertia is
    # already a fixed prior in this class and remains fallback-only.
    module.log_masses.requires_grad_(False)
    Xt = torch.tensor(np.asarray(X, dtype=float), dtype=dtype)
    yt = torch.tensor(np.asarray(y, dtype=float), dtype=dtype)
    theta0 = np.r_[mass0, h0]
    initial_residual = np.asarray(X @ theta0 - y, dtype=float)
    mad = float(np.median(np.abs(initial_residual - np.median(initial_residual))))
    huber_scale = max(1.0e-4, 1.4826 * mad)
    huber_k = 1.5 * huber_scale

    def closure():
        optimizer.zero_grad()
        masses, coms, _ = module()
        theta = torch.cat((masses, (masses[:, None] * coms).reshape(-1)))
        residual = Xt @ theta - yt
        a = torch.abs(residual)
        loss = torch.where(
            a <= huber_k,
            0.5 * residual * residual,
            huber_k * (a - 0.5 * huber_k),
        ).mean()
        loss.backward()
        return loss

    optimizer = torch.optim.LBFGS(
        [module.coms],
        lr=0.5,
        max_iter=160,
        tolerance_grad=1.0e-10,
        tolerance_change=1.0e-12,
        line_search_fn="strong_wolfe",
    )
    optimizer.step(closure)
    with torch.no_grad():
        masses, coms, fixed_rot = module()
        mass = float(masses[0].cpu().item())
        com = np.asarray(coms[0].cpu().numpy(), dtype=float)
        fixed_rot_np = np.asarray(fixed_rot[0].cpu().numpy(), dtype=float)
    theta = np.r_[mass, mass * com]
    residual = np.asarray(X @ theta - y, dtype=float)

    # Covariance in the official optimization coordinates [log(m), c].  Mass
    # uncertainty comes from stage 1; the CoM covariance comes from the
    # fixed-mass stage 2.  Cross covariance is conservatively set to zero
    # because the second stage explicitly conditions on the stage-1 mass.
    mad_final = float(np.median(np.abs(residual - np.median(residual))))
    scale = max(1.0e-4, 1.4826 * mad_final, huber_scale,
                float(moment_stage["robust_scale_Nm"]))
    u = np.abs(residual) / (1.5 * scale)
    weights = np.where(u <= 1.0, 1.0, 1.0 / np.maximum(u, 1.0e-12))
    J_com = mass * X[:, 1:4]
    normal_com = J_com.T @ (weights[:, None] * J_com)
    inv_normal_com = np.linalg.pinv(normal_com, rcond=1.0e-10)
    psi = np.clip(residual / scale, -1.5, 1.5)
    meat_com = J_com.T @ ((psi * psi)[:, None] * J_com)
    cov_com = inv_normal_com @ meat_com @ inv_normal_com * scale * scale
    cov_com = (cov_com + cov_com.T) / 2.0
    mass_var = float(max(np.asarray(mass_stage["covariance"])[0, 0], 0.0))
    covariance_phi = np.zeros((4, 4), dtype=float)
    covariance_phi[0, 0] = mass_var / max(mass * mass, 1.0e-24)
    covariance_phi[1:, 1:] = cov_com
    std_phi = np.sqrt(np.maximum(np.diag(covariance_phi), 0.0))
    mass_sigma = float(np.sqrt(mass_var))
    com_sigma = float(np.sqrt(max(np.trace(cov_com), 0.0)))
    # Diagnostics are reported for both the full [m,h] observability and the
    # fixed-mass [h] stage.  The former retains the rank-4 acceptance check;
    # the latter exposes whether the COM-only second stage is well-conditioned.
    J = np.empty((len(X), 4), dtype=float)
    J[:, 0] = X[:, 0] * mass + X[:, 1:4] @ (mass * com)
    J[:, 1:] = mass * X[:, 1:4]
    sv_x = np.linalg.svd(X, compute_uv=False)
    sv_j = np.linalg.svd(J, compute_uv=False)
    sv_h = np.linalg.svd(J_com, compute_uv=False)
    sigma_min_x = float(sv_x[-1]) if len(sv_x) else 0.0
    sigma_max_x = float(sv_x[0]) if len(sv_x) else 0.0
    sigma_min_j = float(sv_j[-1]) if len(sv_j) else 0.0
    sigma_max_j = float(sv_j[0]) if len(sv_j) else 0.0
    sigma_min_h = float(sv_h[-1]) if len(sv_h) else 0.0
    sigma_max_h = float(sv_h[0]) if len(sv_h) else 0.0
    inliers = np.abs(residual) <= max(3.0 * scale, 1.0e-8)
    rank = int(np.linalg.matrix_rank(J, tol=max(sigma_max_j, 1.0) * 1.0e-10))
    condition = (
        float(sigma_max_j / sigma_min_j) if sigma_min_j > 1.0e-14 else float("inf")
    )
    physical = bool(
        np.isfinite(mass)
        and mass > 0.0
        and np.isfinite(com).all()
        and np.linalg.norm(com) < 0.25
    )
    uncertainty_ok = bool(
        mass_sigma <= 0.15 * mass and com_sigma <= 0.012
    )
    accepted = bool(rank >= 4 and condition <= 1.0e4 and physical and uncertainty_ok)

    def _stage_summary(stage: dict[str, Any]) -> dict[str, Any]:
        """Keep stage diagnostics JSON-safe without dumping sample arrays."""
        return {
            "theta": np.asarray(stage["theta"], dtype=float).tolist(),
            "covariance": np.asarray(stage["covariance"], dtype=float).tolist(),
            "sigma_min": float(stage["sigma_min"]),
            "sigma_max": float(stage["sigma_max"]),
            "condition_number": float(stage["condition_number"]),
            "rank": int(stage["rank"]),
            "residual_rms_Nm": float(stage["residual_rms_Nm"]),
            "robust_scale_Nm": float(stage["robust_scale_Nm"]),
            "outlier_fraction": float(stage["outlier_fraction"]),
            "inlier_count": int(stage["inlier_count"]),
            "sample_count": int(stage["sample_count"]),
        }

    return {
        "accepted": accepted,
        "source": "STATIC_GRAVITY_ID_OFFICIAL_MASS_COM" if accepted else "UNOBSERVABLE",
        "reason": "OK" if accepted else "STATIC_UNCERTAIN",
        "frame": "fr3_hand_tcp",
        "mass_kg": mass,
        "first_moment_kg_m": (mass * com).tolist(),
        "center_of_mass_m": com.tolist(),
        "parameter_order": list(STATIC_PARAMETER_ORDER),
        "parameterization": "official.MassAndComInertialParameter",
        "fixed_rotational_inertia_for_parameterization_kg_m2": fixed_rot_np.tolist(),
        "fixed_rotational_inertia_source": "isotropic_0.05m_prior_only_for_official_module",
        "initial_theta": theta0.tolist(),
        "linear_fit_selection": {
            "selected": "FIXED_MASS_HUBER_IRLS",
            "selection_without_gt": True,
            "mass_stage": _stage_summary(mass_stage),
            "first_moment_stage": _stage_summary(moment_stage),
        },
        "identification_stages": {
            "mass": {
                "method": "ROBUST_HUBER_IRLS_4_PARAMETER_INITIALIZATION",
                "mass_frozen_for_com": True,
                "mass_kg": mass,
                "sigma_kg": mass_sigma,
            },
            "first_moment": {
                "method": "FIXED_MASS_ROBUST_HUBER_IRLS_PLUS_OFFICIAL_COM_OPTIMIZATION",
                "mass_kg": mass,
                "sigma_min": sigma_min_h,
                "sigma_max": sigma_max_h,
                "condition_number": float(sigma_max_h / sigma_min_h)
                if sigma_min_h > 1.0e-14 else float("inf"),
            },
        },
        "final_theta": theta.tolist(),
        "parameter_covariance_optimized_logm_com": covariance_phi.tolist(),
        "parameter_std_optimized_logm_com": std_phi.tolist(),
        "center_of_mass_covariance_m2": cov_com.tolist(),
        "mass_sigma_kg": mass_sigma,
        "center_of_mass_sigma_m": com_sigma,
        "observability": {
            "sigma_min": sigma_min_j,
            "sigma_max": sigma_max_j,
            "condition_number": condition,
            "rank": rank,
            "linear_theta_sigma_min": sigma_min_x,
            "linear_theta_sigma_max": sigma_max_x,
            "linear_theta_condition_number": float(sigma_max_x / sigma_min_x)
            if sigma_min_x > 1.0e-14
            else float("inf"),
            "fixed_mass_com_sigma_min": sigma_min_h,
            "fixed_mass_com_sigma_max": sigma_max_h,
            "fixed_mass_com_condition_number": float(sigma_max_h / sigma_min_h)
            if sigma_min_h > 1.0e-14 else float("inf"),
        },
        "residual_rms_Nm": float(np.sqrt(np.mean(residual**2))),
        "robust_scale_Nm": float(scale),
        "outlier_fraction": float(1.0 - np.mean(inliers)),
        "inlier_count": int(np.sum(inliers)),
        "samples": int(len(y)),
        "physical_checks": {
            "mass_positive": bool(mass > 0.0),
            "com_finite": bool(np.isfinite(com).all()),
            "com_within_0.25m": bool(np.linalg.norm(com) < 0.25),
            "uncertainty_ok": uncertainty_ok,
        },
        "uncertainty_gate": {
            "mass_relative_sigma_limit": 0.15,
            "com_sigma_limit_m": 0.012,
            "mass_relative_sigma": float(mass_sigma / max(mass, 1.0e-12)),
        },
    }


def identify_mass_com(
    baseline: dict[str, np.ndarray],
    payload: dict[str, np.ndarray],
    protocol: dict[str, Any] | None = None,
    *,
    official_root: Path = DEFAULT_OFFICIAL_ROOT,
    settle_s: float = 0.0,
    min_postures: int = 5,
    max_pair_q_error_rad: float = 0.03,
) -> dict[str, Any]:
    """Identify mass/CoM from a paired, guarded FR3 static capture."""

    result: dict[str, Any] = {
        "schema": SCHEMA,
        "stage": "STATIC_GRAVITY_ID_MASS_COM",
        "official_commit": OFFICIAL_COMMIT,
        "official_root": str(Path(official_root).resolve()),
        "accepted": False,
        "source": "UNOBSERVABLE",
        "frame": "fr3_hand_tcp",
        "gt_used_for_estimation": False,
    }
    guard_passed = bool(np.asarray(payload.get("guard_passed", False)).reshape(-1)[-1]) if "guard_passed" in payload else False
    result["guard_passed"] = guard_passed
    result["relative_motion_guard"] = _relative_motion_guard(payload)
    result["attachment_mode"] = str(np.asarray(payload.get("attachment_mode", ["UNKNOWN"])).reshape(-1)[0])
    result["pairing_policy"] = {
        "actual_measured_joint_pose_required": True,
        "max_q_error_rad": float(max_pair_q_error_rad),
        "command_q_ref_checked": True,
    }
    if not guard_passed:
        result.update(reason="PAYLOAD_GUARD_ABORT", data_usable=False)
        return result
    try:
        filter_time_series_data, process_joint_data = _load_official_filtering(
            Path(official_root)
        )
        baseline_tau, baseline_filter = _official_process_torque(
            baseline, process_joint_data, filter_time_series_data
        )
        payload_tau, payload_filter = _official_process_torque(
            payload, process_joint_data, filter_time_series_data
        )
        protocol_static = protocol.get("static", protocol) if protocol else None
        X, y, observation = _build_static_observations(
            baseline,
            payload,
            protocol_static,
            baseline_tau,
            payload_tau,
            settle_s=settle_s,
            min_postures=min_postures,
            max_pair_q_error_rad=max_pair_q_error_rad,
        )
        fit = _torch_fit_mass_com(X, y, Path(official_root))
        result.update(
            fit,
            data_usable=True,
            baseline_subtraction={
                "method": (
                    "official-filtered robot-alone torque; static_pose_id "
                    "settled-window means for torque subtraction"
                ),
                "baseline_samples": int(len(baseline["t"])),
                "payload_samples": int(len(payload["t"])),
                "time_alignment": (
                    "q_ref path checked by payload_id_v2.align_baseline; "
                    "torque paired by static_pose_id"
                ),
                "baseline_filter": baseline_filter,
                "payload_filter": payload_filter,
            },
            observation=observation,
            gripper_opening="paired robot-alone/payload capture at same executor opening; see optional excitation_center.json",
        )
    except Exception as exc:
        result.update(
            reason=("NO_VALID_CAPTURE" if "PAYLOAD_GUARD_ABORT" in str(exc) else "MASS_COM_ESTIMATOR_ERROR"),
            error=str(exc),
            data_usable=False,
        )
    return result


def _recursive_value(obj: Any, wanted: str) -> Any | None:
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key == wanted:
                return value
            found = _recursive_value(value, wanted)
            if found is not None:
                return found
    elif isinstance(obj, list):
        for value in obj:
            found = _recursive_value(value, wanted)
            if found is not None:
                return found
    return None


def _trial_evaluation(path: Path) -> dict[str, Any]:
    """Read trial GT only after identification, for an audit report."""

    data = json.loads(Path(path).read_text())
    mass = _recursive_value(data, "target_mass_kg")
    com_tcp = _recursive_value(data, "target_COM_TCP_m")
    if mass is not None:
        mass_arr = np.asarray(mass, dtype=float).reshape(-1)
        mass = float(mass_arr[0]) if len(mass_arr) else None
    if com_tcp is not None:
        com_arr = np.asarray(com_tcp, dtype=float).reshape(-1)[:3]
        com_tcp = com_arr.tolist()
    return {
        "source": str(Path(path).resolve()),
        "mass_kg": mass,
        "center_of_mass_tcp_m": com_tcp,
        "frame": "fr3_hand_tcp" if com_tcp is not None else None,
        "evaluation_only": True,
    }


def _error_summary(estimate: dict[str, Any], gt: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    if estimate.get("mass_kg") is not None and gt.get("mass_kg") is not None:
        m, mg = float(estimate["mass_kg"]), float(gt["mass_kg"])
        out["mass_relative_error_pct"] = abs(m - mg) / max(abs(mg), 1.0e-12) * 100.0
    if estimate.get("center_of_mass_m") is not None and gt.get("center_of_mass_tcp_m") is not None:
        c = np.asarray(estimate["center_of_mass_m"], dtype=float)
        cg = np.asarray(gt["center_of_mass_tcp_m"], dtype=float)
        out["com_error_xyz_mm"] = ((c - cg) * 1000.0).tolist()
        out["com_error_euclidean_mm"] = float(np.linalg.norm(c - cg) * 1000.0)
    return out


def _find_trial(run: Path) -> Path | None:
    candidates = sorted(run.glob("*_GT.json"))
    return candidates[0] if candidates else None


def evaluate_run(
    run: Path,
    *,
    output: Path | None = None,
    gt_trial: Path | None = None,
    old_report: Path | None = None,
    official_root: Path = DEFAULT_OFFICIAL_ROOT,
    settle_s: float = 0.0,
    max_pair_q_error_rad: float = 0.03,
) -> dict[str, Any]:
    """Run the new estimator and write a side-by-side audit report."""

    run = Path(run).resolve()
    output = Path(output or run / "payload_id_mass_com_official.json").resolve()
    baseline_path = run / "system_id_baseline_static.npz"
    payload_path = run / "system_id_payload_static.npz"
    protocol_path = run / "payload_id_v2_protocol.json"
    protocol = json.loads(protocol_path.read_text()) if protocol_path.exists() else None
    result: dict[str, Any]
    if not baseline_path.exists() or not payload_path.exists():
        result = {
            "schema": SCHEMA,
            "stage": "STATIC_GRAVITY_ID_MASS_COM",
            "accepted": False,
            "source": "UNOBSERVABLE",
            "reason": "NO_RECORD",
            "frame": "fr3_hand_tcp",
            "gt_used_for_estimation": False,
        }
    else:
        result = identify_mass_com(
            load_record(baseline_path),
            load_record(payload_path),
            protocol,
            official_root=official_root,
            settle_s=settle_s,
            max_pair_q_error_rad=max_pair_q_error_rad,
        )
    opening_path = run / "excitation_center.json"
    if opening_path.exists():
        try:
            opening = json.loads(opening_path.read_text())
            result["gripper_opening_metadata"] = {
                "source": str(opening_path),
                "finger_names": opening.get("finger_names"),
                "finger_q_rad": opening.get("finger_q"),
                "paired_baseline_payload": True,
            }
        except Exception as exc:
            result["gripper_opening_metadata"] = {
                "source": str(opening_path),
                "error": str(exc),
            }
    report: dict[str, Any] = {
        "schema": "real2sim/payload_mass_com_audit/v1",
        "run": str(run),
        "estimate": result,
        "gt_used_for_estimation": False,
        "official_reuse": {
            "commit": OFFICIAL_COMMIT,
            "components": [
                "robot_payload_id.eric_id.drake_torch_dynamics.MassAndComInertialParameter",
                "robot_payload_id.utils.filtering.process_joint_data",
                "robot_payload_id.utils.filtering.filter_time_series_data",
            ],
            "fr3_adaptation": [
                "FR3 measured TCP gravity regressor from payload_id_v2",
                "paired robot-alone torque subtraction and q_ref time check",
                "Isaac capture guard_passed / optional T_TCP_object stream",
            ],
        },
    }
    old_path = Path(old_report).resolve() if old_report else run / "payload_id_v2_report.json"
    if old_path.exists():
        old = json.loads(old_path.read_text())
        report["old_v2"] = {
            "source": str(old_path),
            "static": old.get("static", {}),
            "frame": old.get("static", {}).get("frame", "fr3_hand_tcp (v2 contract)"),
        }
    gt_path = Path(gt_trial).resolve() if gt_trial else _find_trial(run)
    if gt_path and gt_path.exists():
        # Deliberately after identify_mass_com: this function is evaluation-only.
        gt = _trial_evaluation(gt_path)
        report["gt_evaluation_only"] = gt
        report["errors"] = _error_summary(result, gt)
        if "old_v2" in report:
            old_static = report["old_v2"]["static"]
            report["old_v2_errors"] = _error_summary(
                {
                    "mass_kg": old_static.get("mass_kg"),
                    "center_of_mass_m": old_static.get("center_of_mass_m"),
                },
                gt,
            )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--gt-trial", type=Path)
    parser.add_argument("--old-report", type=Path)
    parser.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.0,
        help="optional extra settling prefix; protocol holds already follow a validated transition",
    )
    parser.add_argument(
        "--max-pair-q-error-rad",
        type=float,
        default=0.03,
        help="strict measured empty/payload joint-pose mismatch limit per static window",
    )
    args = parser.parse_args()
    report = evaluate_run(
        args.run,
        output=args.output,
        gt_trial=args.gt_trial,
        old_report=args.old_report,
        official_root=args.official_root,
        settle_s=args.settle_s,
        max_pair_q_error_rad=args.max_pair_q_error_rad,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
