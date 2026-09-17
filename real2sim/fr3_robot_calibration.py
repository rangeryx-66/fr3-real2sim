"""FR3 adaptation of the Scalable Real2Sim robot/payload identification path.

The official project assumes an IIWA model with ``iiwa_link_*`` names and a
Drake actuator on every joint.  Isaac's FR3 asset uses different names and
does not expose those actuators in its URDF.  This module creates a temporary,
arm-only FR3 model with the same physical inertials, welds the base, and adds
the seven Drake joint actuators before calling the *upstream* autodiff data
matrix implementation.  The source URDF and the frozen grasp/scan/asset
pipeline are never modified.

The measured simulator trajectory carries analytic Fourier q/dq/ddq sidecars.
Only the official zero-phase Butterworth torque filter is applied to the
measured torque; no noisy finite difference is used for the identification
regressor.  A minimum-norm base-parameter fit is reported because the full
link parameter vector has a structural null space, exactly as in the official
base-parameter reduction.
"""
from __future__ import annotations
import os

import importlib.util
import json
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .payload_id_v2 import _time, robust_linear_fit, spatial_regressor


OFFICIAL_COMMIT = "c52e31cf26c83b33aee5e56f805e1d4d710fd549"
DEFAULT_OFFICIAL_ROOT = Path(os.environ.get('OFFICIAL_PAYLOAD_ROOT', str(Path(__file__).resolve().parents[1]/'third_party/scalable_real2sim_robot_payload_id_upstream_c52e31c')))
DEFAULT_FR3_URDF = Path(os.environ.get('FR3_URDF', str(Path(__file__).resolve().parents[1]/'config/fr3.urdf')))
SCHEMA = "real2sim/scalable_payload_fr3_official/v1"
ARM_JOINTS = 7
OPENINGS_MM = (0, 20, 40, 60, 80)


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [_json_safe(x) for x in value.tolist()]
    if isinstance(value, (list, tuple)):
        return [_json_safe(x) for x in value]
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    return value


def _official_filtering(official_root: Path) -> tuple[Callable, Callable]:
    """Load exact upstream filter functions without importing a ROS stack."""
    path = Path(official_root) / "robot_payload_id" / "utils" / "filtering.py"
    if not path.exists():
        raise FileNotFoundError(path)
    package_name = "_fr3_scalable_payload_utils"
    data_name = f"{package_name}.dataclasses"
    filtering_name = f"{package_name}.filtering"
    package = types.ModuleType(package_name)
    package.__path__ = []  # type: ignore[attr-defined]

    class JointData:
        def __init__(self, joint_positions, joint_velocities,
                     joint_accelerations, joint_torques, sample_times_s):
            self.joint_positions = joint_positions
            self.joint_velocities = joint_velocities
            self.joint_accelerations = joint_accelerations
            self.joint_torques = joint_torques
            self.sample_times_s = sample_times_s

    dm = types.ModuleType(data_name)
    dm.JointData = JointData  # type: ignore[attr-defined]
    sys.modules[package_name] = package
    sys.modules[data_name] = dm
    sys.modules.pop(filtering_name, None)
    spec = importlib.util.spec_from_file_location(filtering_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[filtering_name] = mod
    spec.loader.exec_module(mod)
    return mod.filter_time_series_data, mod.process_joint_data


def _official_data_imports(official_root: Path):
    """Import official Drake data helpers in the mixed official/Isaac env.

    The official environment intentionally does not install torch, while the
    Isaac environment does.  The evaluator prepends both site-packages trees.
    Optional ``wandb`` and ``manipulation`` imports are stubbed only when they
    are absent/broken; neither is used by the numeric data matrix.
    """
    root = str(Path(official_root).resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    if "wandb" not in sys.modules:
        sys.modules["wandb"] = types.SimpleNamespace(run=None)
    # robot_payload_id.utils.utils imports this optional helper only to add
    # package mappings.  Our parser is created directly, so a no-op is enough.
    if "manipulation.utils" not in sys.modules:
        manipulation = sys.modules.get("manipulation") or types.ModuleType("manipulation")
        mutils = types.ModuleType("manipulation.utils")
        mutils.ConfigureParser = lambda parser: None
        manipulation.utils = mutils  # type: ignore[attr-defined]
        sys.modules["manipulation"] = manipulation
        sys.modules["manipulation.utils"] = mutils
    # data_matrix_numeric imports create_arm from environment, but does not
    # call it.  Avoid importing the upstream IIWA/ROS station package.
    if "robot_payload_id.environment" not in sys.modules:
        env = types.ModuleType("robot_payload_id.environment")
        env.create_arm = lambda *args, **kwargs: None
        sys.modules["robot_payload_id.environment"] = env
    from robot_payload_id.data.data_matrix_numeric import (  # type: ignore
        compute_base_param_mapping,
        extract_numeric_data_matrix_autodiff,
    )
    from robot_payload_id.utils import ArmPlantComponents, JointData  # type: ignore
    from robot_payload_id.symbolic.environment import create_autodiff_plant  # type: ignore
    return (extract_numeric_data_matrix_autodiff, compute_base_param_mapping,
            ArmPlantComponents, JointData, create_autodiff_plant)


def make_fr3_arm_urdf(source: Path, output: Path) -> Path:
    """Create an official-name, arm-only FR3 URDF in a calibration cache."""
    source, output = Path(source), Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    root = ET.parse(source).getroot()
    root.attrib["name"] = "arm"
    remove_names = {"fr3_leftfinger", "fr3_rightfinger",
                    "fr3_finger_joint1", "fr3_finger_joint2"}
    for elem in list(root):
        name = elem.attrib.get("name", "")
        if (elem.tag in {"link", "joint"}
                and (name in remove_names or "accelerometer" in name)):
            root.remove(elem)
    # Rename only the seven arm links/joints.  The fixed hand and TCP remain
    # part of the robot mass, matching the Isaac FR3 model at the flange.
    for elem in root.iter():
        if elem.tag in {"link", "joint"}:
            name = elem.attrib.get("name", "")
            if elem.tag == "link" and name.startswith("fr3_link"):
                suffix = name[len("fr3_link"):]
                if suffix.isdigit() and 1 <= int(suffix) <= 7:
                    elem.attrib["name"] = f"link{suffix}"
            if elem.tag == "joint" and name.startswith("fr3_joint"):
                suffix = name[len("fr3_joint"):]
                if suffix.isdigit() and 1 <= int(suffix) <= 7:
                    elem.attrib["name"] = f"joint{suffix}"
        if elem.tag in {"parent", "child"}:
            link = elem.attrib.get("link", "")
            if link.startswith("fr3_link"):
                suffix = link[len("fr3_link"):]
                if suffix.isdigit() and 1 <= int(suffix) <= 7:
                    elem.attrib["link"] = f"link{suffix}"
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return output


def _make_drake_plant(urdf_path: Path):
    from pydrake.all import MultibodyPlant, Parser

    plant = MultibodyPlant(0.0)
    model = Parser(plant).AddModels(str(urdf_path))[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    for i in range(1, ARM_JOINTS + 1):
        plant.AddJointActuator(f"joint{i}", plant.GetJointByName(f"joint{i}"))
    plant.Finalize()
    return plant, model


def _record_arrays(record: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t = _time(record)
    q = np.asarray(record.get("q_ref", record["q"]), dtype=float)
    dq = np.asarray(record.get("dq_ref", record.get("dq")), dtype=float)
    ddq = np.asarray(record.get("ddq_ref"), dtype=float) if "ddq_ref" in record else None
    if ddq is None:
        # Legacy captures are explicitly marked as a fallback.  New captures
        # always carry analytic sidecars.
        ddq = np.gradient(dq, t, axis=0, edge_order=2)
    tau = np.asarray(record["tau"], dtype=float)
    n = min(len(t), len(q), len(dq), len(ddq), len(tau))
    q, dq, ddq, tau, t = q[:n], dq[:n], ddq[:n], tau[:n], t[:n]
    good = (np.isfinite(q).all(axis=1) & np.isfinite(dq).all(axis=1)
            & np.isfinite(ddq).all(axis=1) & np.isfinite(tau).all(axis=1)
            & np.isfinite(t))
    if good.sum() < 100:
        raise ValueError(f"too few finite samples ({int(good.sum())})")
    return t[good], q[good], dq[good], ddq[good], tau[good]


def filter_torque_official(record: dict[str, np.ndarray], official_root: Path) -> tuple[np.ndarray, dict[str, Any]]:
    """Apply the official zero-phase torque filter to a v2 record."""
    filter_time_series_data, _ = _official_filtering(official_root)
    t, q, dq, ddq, tau = _record_arrays(record)
    fs = float(1.0 / np.median(np.diff(t)))
    # The upstream script uses order=10/cutoff=4 Hz.  For the high-rate Isaac
    # capture we retain the upstream production default from process_joint_data
    # (order 12, cutoff 1.6 Hz), which is also the existing v2 audit setting.
    filtered = filter_time_series_data(tau, order=12, cutoff_freq_hz=1.6,
                                       fs_hz=fs, visualize=False)
    return filtered, {"implementation": "official.filter_time_series_data",
                      "order": 12, "cutoff_hz": 1.6, "sample_hz": fs,
                      "analytic_qdqddq": "record q_ref/dq_ref/ddq_ref"}


def _official_regressor(record: dict[str, np.ndarray], official_root: Path,
                        *, payload_only: bool = False) -> dict[str, Any]:
    """Build the exact official FR3 autodiff W/w0 matrix."""
    (extract, mapping_fn, ArmPlantComponents, JointData, create_ad) = _official_data_imports(official_root)
    cache = Path(record.get("_model_cache", "/tmp/fr3_payload_id_models"))
    urdf = make_fr3_arm_urdf(DEFAULT_FR3_URDF, cache / "fr3_arm_official_names.urdf")
    plant, model = _make_drake_plant(urdf)
    t, q, dq, ddq, tau = _record_arrays(record)
    jd = JointData(joint_positions=q, joint_velocities=dq,
                   joint_accelerations=ddq, joint_torques=tau,
                   sample_times_s=t)
    comp = ArmPlantComponents(plant=plant, plant_context=plant.CreateDefaultContext())
    W, w0, model_tau = extract(comp, jd, add_rotor_inertia=False,
                               add_reflected_inertia=True,
                               add_viscous_friction=True,
                               add_dynamic_dry_friction=True,
                               payload_only=payload_only,
                               use_progress_bar=False)
    return {"W": np.asarray(W, float), "w0": np.asarray(w0, float),
            "model_tau": np.asarray(model_tau, float), "time": t,
            "q": q, "dq": dq, "ddq": ddq, "tau": tau,
            "plant": plant, "model": model,
            "mapping_fn": mapping_fn, "ArmPlantComponents": ArmPlantComponents,
            "JointData": JointData, "create_autodiff_plant": create_ad,
            "urdf": str(urdf)}


def fit_robot_record(record: dict[str, np.ndarray], official_root: Path = DEFAULT_OFFICIAL_ROOT,
                     *, opening_mm: float | None = None) -> dict[str, Any]:
    """Fit FR3 base inertial/friction/reflected parameters for one opening."""
    official_root = Path(official_root)
    filtered_tau, filtering = filter_torque_official(record, official_root)
    reg = _official_regressor(record, official_root)
    # _record_arrays drops non-finite rows; records from the simulator are
    # finite, so the same row count is expected.  Align defensively if a legacy
    # trace has a bad sample.
    filtered_tau = np.asarray(filtered_tau, dtype=float)
    if filtered_tau.ndim != 2 or filtered_tau.shape[1] != ARM_JOINTS:
        raise ValueError(f"official torque filter returned shape {filtered_tau.shape}, expected (N,7)")
    n = min(filtered_tau.shape[0], len(reg["time"]))
    W = np.asarray(reg["W"], dtype=float)[: n * ARM_JOINTS, :]
    w0 = np.asarray(reg["w0"], dtype=float).reshape(-1)[: n * ARM_JOINTS]
    if len(W) != len(w0) or len(W) != n * ARM_JOINTS:
        raise ValueError(f"official regressor row mismatch W={W.shape} w0={w0.shape} n={n}")
    y = filtered_tau[:n].reshape(-1) - w0
    # The official ordering is 13 terms/link:
    # m,hx,hy,hz,Ixx,Ixy,Ixz,Iyy,Iyz,Izz,reflected,viscous,dry.
    mapping = reg["mapping_fn"](W, tol=1e-8)
    Wb = W @ mapping
    fit = robust_linear_fit(Wb, y)
    theta_base = np.asarray(fit["theta"], float)
    theta_full = mapping @ theta_base
    pred = Wb @ theta_base + w0
    residual = pred - filtered_tau[:n].reshape(-1)
    per_joint = residual.reshape(-1, 7)
    sv_full = np.linalg.svd(W, compute_uv=False)
    sv_base = np.linalg.svd(Wb, compute_uv=False)
    # Convert the minimum-norm equivalent vector back to named terms.  The
    # individual terms are only accepted if the base mapping is full rank;
    # the report always exposes rank/nullity to prevent overinterpretation.
    labels = []
    names = ("mass", "first_moment_x", "first_moment_y", "first_moment_z",
             "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz",
             "reflected_inertia", "viscous_friction", "coulomb_friction")
    for j in range(7):
        for k, name in enumerate(names):
            labels.append({"joint": j + 1, "parameter": name,
                           "value": float(theta_full[j * 13 + k])})
    return _json_safe({
        "schema": SCHEMA + "/robot_fit",
        "official_commit": OFFICIAL_COMMIT,
        "stage": "FR3_ROBOT_ALONE_CALIBRATION",
        "opening_mm": opening_mm,
        "source": "OFFICIAL_DRAKE_AUTODIFF_BASE_PARAMETER_REGRESSOR",
        "urdf_adapter": {"generated_model": reg["urdf"],
                          "base_welded_to_world": True,
                          "arm_joints": 7, "added_joint_actuators": True,
                          "removed_links": ["leftfinger", "rightfinger", "accelerometer_sensors"],
                          "original_urdf_unchanged": True},
        "filtering": filtering,
        "parameter_order_per_joint": list(names),
        "base_parameter_count": int(Wb.shape[1]),
        "full_parameter_count": int(W.shape[1]),
        "base_mapping": mapping.tolist(),
        "identified_base_parameters": theta_base.tolist(),
        "identified_full_minimum_norm_parameters": theta_full.tolist(),
        "named_minimum_norm_parameters": labels,
        "observability": {
            "rank_full_raw": int(np.linalg.matrix_rank(W, tol=max(sv_full[0], 1.0) * 1e-10)),
            "rank_base": int(fit["rank"]),
            "sigma_min_full_raw": float(sv_full[-1]),
            "sigma_min_base": float(sv_base[-1]),
            "condition_full_raw": float(sv_full[0] / sv_full[-1]) if sv_full[-1] > 1e-14 else float("inf"),
            "condition_base": float(fit["condition_number"]),
            "structural_nullity": int(W.shape[1] - Wb.shape[1]),
        },
        "fit": {"residual_rms_Nm": float(np.sqrt(np.mean(residual ** 2))),
                "per_joint_residual_rms_Nm": np.sqrt(np.mean(per_joint ** 2, axis=0)).tolist(),
                "robust_scale_Nm": fit["robust_scale_Nm"],
                "outlier_fraction": fit["outlier_fraction"],
                "sample_count": int(len(y)),
                "covariance_base": np.asarray(fit["covariance"]).tolist(),
                "std_base": np.sqrt(np.maximum(np.diag(fit["covariance"]), 0)).tolist()},
        "parameter_semantics": {
            "mass_and_inertial_terms": "FR3 link inertial lumped parameters in the generated arm model",
            "reflected_inertia": "official reflected-inertia columns; structurally coupled terms remain in base space",
            "viscous_friction": "official linear velocity friction term",
            "coulomb_friction": "official dynamic dry friction sign term",
            "individual_full_terms": "minimum-norm equivalent only when structural nullity > 0",
        },
        "gt_used_for_estimation": False,
    })


def design_fr3_fourier_protocol(center_q: np.ndarray, *, duration_s: float = 15.0,
                                sample_hz: float = 30.0, seed: int = 20260911) -> dict[str, Any]:
    """Design a deterministic official-style Fourier excitation for FR3.

    The upstream optimizer uses ``q=q0+Σ(a sin(ωlt)+b cos(ωlt))`` and scores
    the numeric data matrix.  Without an FR3 MoveIt collision model in Drake,
    we evaluate a fixed finite candidate set using that same parameterization,
    analytic derivatives, and conservative joint envelopes.  The selected
    candidate depends only on center pose and model-free information, never on
    payload results.
    """
    center_q = np.asarray(center_q, float).reshape(7)
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, duration_s, int(round(duration_s * sample_hz)) + 1)
    # Harmonics and amplitudes stay well inside FR3 velocity/acceleration
    # limits and are fixed before any payload outcome is observed.  The
    # previous 0.7-scale set produced only about 0.046 rad of motion, which
    # made the payload first-moment signal smaller than the torque residual.
    # These larger candidates improve mass/CoM observability without changing
    # the frozen grasp or controller.  The payload identification experiment
    # needs roughly 0.30 rad of joint excitation: the former 1.70 scale only
    # reached about 0.114 rad, so the payload first-moment signal was buried
    # in the torque residual.  We retain the official Fourier parameterization
    # and choose the largest candidate that passes the explicit FR3 limits
    # below; this is an identification-trajectory setting, not a grasp rule.
    omega = 0.30 * np.pi
    terms = 5
    base_amp = np.array([.014, .018, .012, .018, .014, .020, .022])
    # FR3 calibration limits are deliberately below the hardware limits.  The
    # 0.30-rad target gives about 0.70 rad/s and 2.4 rad/s² for this 15 s
    # envelope, while remaining well inside the arm velocity/acceleration
    # limits and the safe joint intervals around the validated center poses.
    joint_lower = np.array([-2.70, -1.65, -2.70, -2.95, -2.70, 0.55, -2.85])
    joint_upper = np.array([ 2.70,  1.65,  2.70, -0.20,  2.70, 4.50,  2.85])
    max_dq_limit = 0.90
    max_ddq_limit = 3.50
    candidates = []
    for scale in (1.0, 1.70, 2.80, 3.60, 4.90):
        for phase_seed in (0.0, 0.37, 0.79, 1.21):
            phases = phase_seed + rng.uniform(-0.15, 0.15, size=(7, terms))
            amp = base_amp[:, None] * scale / np.sqrt(np.arange(1, terms + 1))[None, :]
            a = amp * np.cos(phases)
            b = amp * np.sin(phases)
            l = np.arange(1, terms + 1, dtype=float)
            wt = omega * l
            # Keep the upstream Fourier parameterization, but apply a smooth
            # sin^2 start/stop envelope.  The official hardware pipeline has
            # a separate retimed start trajectory; the Isaac bridge accepts a
            # single position trajectory, so the envelope is the explicit
            # equivalent that guarantees q(0)=center and dq(0)=0.  This is
            # essential for a held payload: the raw cosine term must never
            # teleport the grasp at capture start.
            phase_t = t[:, None] * wt
            raw_offset = np.einsum("jl,tl->tj", a, np.sin(phase_t)) + np.einsum("jl,tl->tj", b, np.cos(phase_t))
            raw_dq = np.einsum("jl,tl->tj", a, wt[None, :] * np.cos(phase_t)) - np.einsum("jl,tl->tj", b, wt[None, :] * np.sin(phase_t))
            raw_ddq = np.einsum("jl,tl->tj", -a, (wt[None, :] ** 2) * np.sin(phase_t)) - np.einsum("jl,tl->tj", b, (wt[None, :] ** 2) * np.cos(phase_t))
            env = np.sin(np.pi * t / duration_s) ** 2
            denv = (np.pi / duration_s) * np.sin(2.0 * np.pi * t / duration_s)
            ddenv = (2.0 * np.pi**2 / duration_s**2) * np.cos(2.0 * np.pi * t / duration_s)
            q = center_q + env[:, None] * raw_offset
            dq = denv[:, None] * raw_offset + env[:, None] * raw_dq
            ddq = ddenv[:, None] * raw_offset + 2.0 * denv[:, None] * raw_dq + env[:, None] * raw_ddq
            # Reject unsafe candidates before information scoring.  Limits are
            # conservative calibration envelopes, not replacements for
            # MoveIt's collision/trajectory checks during execution.
            if (np.any(q < joint_lower[None, :]) or np.any(q > joint_upper[None, :])
                    or np.max(np.abs(dq)) > max_dq_limit
                    or np.max(np.abs(ddq)) > max_ddq_limit):
                continue
            # Official design maximizes a data-matrix information criterion.
            # This lightweight pre-score covers acceleration, velocity and
            # gravity direction diversity before the exact W is built.  The
            # columns are normalized only for comparing candidates; the
            # physical regressor remains unscaled.
            X = np.column_stack([ddq, dq, np.sin(q), np.cos(q)])
            X = X / np.maximum(np.std(X, axis=0, keepdims=True), 1e-9)
            sv = np.linalg.svd(X, compute_uv=False)
            score = float(np.sum(np.log(np.maximum(sv, 1e-12))))
            candidates.append((float(sv[-1]), score, scale, phase_seed, q, dq, ddq, a, b))
    # The normalized proxy is used to choose phase; scale is ordered first so
    # the highest safe excitation is selected when normalized information is
    # otherwise tied across amplitudes.
    if not candidates:
        raise RuntimeError("no FR3 Fourier candidate satisfies calibration limits")
    best = max(candidates, key=lambda z: (z[2], z[0], z[1]))
    sigma_min, score, scale, phase_seed, q, dq, ddq, a, b = best
    return _json_safe({
        "schema": SCHEMA + "/fourier_protocol",
        "source": "OFFICIAL_FOURIER_SERIES_PARAMETERIZATION_FR3_ADAPTED",
        "seed": seed, "omega_rad_s": omega, "num_fourier_terms": terms,
        "duration_s": duration_s, "sample_hz": sample_hz,
        "center_q": center_q, "amplitude_scale": scale,
        "phase_seed": phase_seed, "information_proxy_logdet": score,
        "proxy_sigma_min": sigma_min, "a_coefficients": a, "b_coefficients": b,
        "time": t, "q": q, "dq": dq, "ddq": ddq,
        "candidate_count": len(candidates),
        "candidate_scales": sorted({float(x[2]) for x in candidates}),
        "calibration_joint_limits": {"lower": joint_lower, "upper": joint_upper,
                                     "max_abs_dq_rad_s": max_dq_limit,
                                     "max_abs_ddq_rad_s2": max_ddq_limit},
        "trajectory_limits": {"max_abs_dq_rad_s": float(np.max(np.abs(dq))),
                              "max_abs_ddq_rad_s2": float(np.max(np.abs(ddq))),
                              "max_abs_offset_rad": float(np.max(np.abs(q - center_q))),
                              "initial_offset_rad": float(np.max(np.abs(q[0] - center_q))),
                              "initial_velocity_rad_s": float(np.max(np.abs(dq[0])))},
        "start_stop_envelope": "sin^2 (zero initial/final offset and velocity)",
        "gt_used_for_design": False,
    })


def _guard_mask(record: dict[str, np.ndarray]) -> tuple[np.ndarray, dict[str, Any]]:
    """Return rows allowed by the per-sample TCP/object stability guard."""
    n = len(record["t"])
    mask = np.ones(n, dtype=bool)
    details = {"source": "per_sample_T_TCP_object", "rejected_rows": 0,
               "max_translation_m": 0.0, "max_rotation_rad": 0.0}
    if "T_TCP_object" not in record:
        details.update(source="capture_flag_only", warning="missing_T_TCP_object_stream")
        return mask, details
    T = np.asarray(record["T_TCP_object"], float)
    if T.ndim != 3 or T.shape[1:] != (4, 4) or len(T) != n:
        return np.zeros(n, dtype=bool), {"source": "invalid_stream", "rejected_rows": n}
    from scipy.spatial.transform import Rotation
    ref = T[0]
    rel = np.linalg.inv(ref)[None, ...] @ T
    tr = np.linalg.norm(rel[:, :3, 3], axis=1)
    rr = np.asarray([Rotation.from_matrix(x[:3, :3]).magnitude() for x in rel])
    mask &= np.isfinite(tr) & np.isfinite(rr) & (tr <= .003) & (rr <= np.deg2rad(5.0))
    details.update(rejected_rows=int((~mask).sum()), max_translation_m=float(np.max(tr)),
                   max_rotation_rad=float(np.max(rr)), threshold_translation_m=.003,
                   threshold_rotation_rad=float(np.deg2rad(5.0)))
    return mask, details


def identify_dynamic_mass_com(baseline: dict[str, np.ndarray], payload: dict[str, np.ndarray],
                              official_root: Path = DEFAULT_OFFICIAL_ROOT, *,
                              baseline_fit: dict[str, Any] | None = None,
                              opening_mm: float | None = None) -> dict[str, Any]:
    """Identify payload mass/first moment from strict loaded−unloaded dynamics."""
    result: dict[str, Any] = {"schema": SCHEMA + "/dynamic_mass_com", "stage": "DYNAMIC_MASS_COM_ID",
                              "official_commit": OFFICIAL_COMMIT, "opening_mm": opening_mm,
                              "accepted": False, "source": "UNOBSERVABLE", "gt_used_for_estimation": False}
    try:
        if not bool(np.asarray(payload.get("guard_passed", [False])).reshape(-1)[-1]):
            result["reason"] = "PAYLOAD_GUARD_ABORT"; return result
        mask, guard = _guard_mask(payload)
        if not mask.any():
            result.update(reason="RELATIVE_SLIP_ALL_ROWS", guard=guard); return result
        # Exact official torque filtering for both legs.  Pairing is performed
        # by analytic q_ref path, with time interpolation only for the torque.
        tau0, filtering0 = filter_torque_official(baseline, Path(official_root))
        tau1, filtering1 = filter_torque_official(payload, Path(official_root))
        tb, qb, _, _, _ = _record_arrays(baseline)
        tp, qp, _, _, _ = _record_arrays(payload)
        if "q_ref" in baseline and "q_ref" in payload:
            qbr = np.asarray(baseline["q_ref"], float); qpr = np.asarray(payload["q_ref"], float)
            # Both processes run the same serialized protocol; compare the
            # command path itself rather than load-dependent measured lag.
            nref = min(len(qbr), len(qpr))
            if np.max(np.abs(qbr[:nref] - qpr[:nref])) > .04:
                result["reason"] = "PROTOCOL_Q_REF_MISMATCH"; return result
        n = min(len(tp), len(tau1), len(mask))
        # The two Isaac processes can advance the same serialized trajectory
        # with slightly different wall-clock sample durations (GPU/ROS
        # scheduling). Pair filtered torques by the common commanded
        # reference progress when q_ref is present, instead of treating that
        # duration difference as a physical phase lag. Both legs still use the
        # identical q_ref/dq_ref/ddq_ref path; only measured torque is
        # interpolated. Legacy records without q_ref retain time pairing.
        if "q_ref" in baseline and "q_ref" in payload:
            baseline_progress = np.linspace(0.0, 1.0, len(tb))
            payload_progress = np.linspace(0.0, 1.0, n)
            tau0_on_payload = np.column_stack([
                np.interp(payload_progress, baseline_progress, tau0[:, j])
                for j in range(7)
            ])
            pairing_mode = "normalized_common_q_ref_progress"
        else:
            tau0_on_payload = np.column_stack([np.interp(tp[:n], tb, tau0[:, j]) for j in range(7)])
            pairing_mode = "relative_time_for_torque_only"
        delta = tau1[:n] - tau0_on_payload
        # Build the FR3 TCP external-payload regressor from real robot pose,
        # Jacobian and analytic Fourier derivatives.  Its first four columns
        # are [m, m*cx, m*cy, m*cz]; inertia is intentionally fixed/fallback.
        rec = {k: np.asarray(v)[:n] for k, v in payload.items()
               if isinstance(v, np.ndarray) and len(np.asarray(v).shape) and len(np.asarray(v)) >= n}
        rec["q_ref"] = np.asarray(payload.get("q_ref", qp))[:n]
        rec["dq_ref"] = np.asarray(payload.get("dq_ref", np.gradient(rec["q_ref"], tp[:n], axis=0)))[:n]
        rec["ddq_ref"] = np.asarray(payload.get("ddq_ref", np.gradient(rec["dq_ref"], tp[:n], axis=0)))[:n]
        from .payload_jacobian import tcp_jacobian_record
        rec = tcp_jacobian_record(rec)
        Y = spatial_regressor(rec, kinematics=None).reshape(n * 7, 10)
        y = delta.reshape(-1)
        row_mask = np.repeat(mask[:n], 7) & np.isfinite(Y).all(axis=1) & np.isfinite(y)
        X = Y[row_mask, :4]; yy = y[row_mask]
        if len(yy) < 32 or np.linalg.matrix_rank(X) < 4:
            result.update(reason="PAYLOAD_DYNAMIC_UNOBSERVABLE", guard=guard, valid_rows=int(len(yy)))
            return result
        # Reuse the official MassAndCom parameterization through the existing
        # two-stage implementation: robust mass initialization, mass frozen,
        # official COM optimization.  It never sees GT values.
        from .payload_id_official_mass_com import _torch_fit_mass_com_fixed_mass
        fit = _torch_fit_mass_com_fixed_mass(X, yy, Path(official_root))
        fit["guard"] = guard
        fit["filtering"] = {"baseline": filtering0, "payload": filtering1}
        fit["pairing"] = {"mode": "loaded_minus_unloaded", "same_analytic_protocol": True,
                           "baseline_interpolation": pairing_mode,
                           "valid_rows": int(len(yy)), "rejected_rows": int(len(y) - len(yy))}
        fit["stage"] = "DYNAMIC_MASS_COM_ID"
        fit["official_commit"] = OFFICIAL_COMMIT
        fit["opening_mm"] = opening_mm
        fit["source"] = "DYNAMIC_ID_OFFICIAL_MASS_COM_PARAMETERIZATION"
        fit["gt_used_for_estimation"] = False
        return _json_safe(fit)
    except Exception as exc:
        result.update(reason="DYNAMIC_MASS_COM_ESTIMATOR_ERROR", error=str(exc))
        return result


def choose_opening_baseline(baselines: dict[float, dict[str, Any]], opening_mm: float) -> dict[str, Any]:
    """Select the nearest calibrated opening, with explicit interpolation metadata."""
    keys = np.asarray(sorted(float(k) for k in baselines), float)
    if len(keys) == 0:
        raise ValueError("no opening baselines")
    idx = int(np.argmin(np.abs(keys - float(opening_mm))))
    key = float(keys[idx])
    return {"requested_opening_mm": float(opening_mm), "selected_opening_mm": key,
            "mode": "nearest_baseline", "interpolation": False,
            "baseline": baselines[key]}
