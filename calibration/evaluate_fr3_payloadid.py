"""Official-style FR3 robot/payload calibration evaluator.

This script is intentionally separate from the Isaac runner.  It is executed
with the Scalable Real2Sim Drake environment plus the Isaac environment's
numpy/scipy/torch site-packages, so the exact upstream autodiff data matrix and
MassAndCom parameterization are available without touching the frozen runtime
pipeline.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from real2sim.fr3_robot_calibration import (
    DEFAULT_OFFICIAL_ROOT,
    fit_robot_record,
    identify_dynamic_mass_com,
)


def _load(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=True) as data:
        return {k: np.asarray(data[k]) for k in data.files}


def _trial_eval(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {"evaluation_only": True, "available": False}
    data = json.loads(path.read_text())

    def rec(v: Any, key: str):
        if isinstance(v, dict):
            if key in v:
                return v[key]
            for x in v.values():
                got = rec(x, key)
                if got is not None:
                    return got
        if isinstance(v, list):
            for x in v:
                got = rec(x, key)
                if got is not None:
                    return got
        return None

    mass = rec(data, "target_mass_kg")
    com = rec(data, "target_COM_TCP_m")
    if mass is not None:
        mass = float(np.asarray(mass).reshape(-1)[0])
    if com is not None:
        com = np.asarray(com, float).reshape(-1)[:3].tolist()
    return {"evaluation_only": True, "available": mass is not None and com is not None,
            "source": str(path.resolve()), "mass_kg": mass,
            "center_of_mass_tcp_m": com, "frame": "fr3_hand_tcp" if com is not None else None}


def _errors(estimate: dict[str, Any], gt: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    if estimate.get("mass_kg") is not None and gt.get("mass_kg") is not None:
        m, mg = float(estimate["mass_kg"]), float(gt["mass_kg"])
        out["mass_error_pct"] = abs(m - mg) / max(abs(mg), 1e-12) * 100.0
    if estimate.get("center_of_mass_m") is not None and gt.get("center_of_mass_tcp_m") is not None:
        d = (np.asarray(estimate["center_of_mass_m"]) - np.asarray(gt["center_of_mass_tcp_m"])) * 1000.0
        out["com_error_xyz_mm"] = d.tolist()
        out["com_error_euclidean_mm"] = float(np.linalg.norm(d))
    return out


def main() -> None:
    p = argparse.ArgumentParser()
    # Isaac site-packages contains an old argparse backport; avoid the
    # Python-3.7-only ``required=`` subparser keyword so the official Drake
    # environment can be combined with Isaac scipy/torch packages.
    sub = p.add_subparsers(dest="command")
    r = sub.add_parser("robot")
    r.add_argument("--record", type=Path, required=True)
    r.add_argument("--output", type=Path, required=True)
    r.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
    r.add_argument("--opening-mm", type=float)
    d = sub.add_parser("payload")
    d.add_argument("--baseline", type=Path, required=True)
    d.add_argument("--payload", type=Path, required=True)
    d.add_argument("--output", type=Path, required=True)
    d.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
    d.add_argument("--opening-mm", type=float)
    d.add_argument("--trial", type=Path)
    d.add_argument("--baseline-fit", type=Path)
    args = p.parse_args()
    if args.command is None:
        p.error("a robot or payload subcommand is required")
    if args.command == "robot":
        out = fit_robot_record(_load(args.record), args.official_root, opening_mm=args.opening_mm)
    else:
        baseline = _load(args.baseline)
        payload = _load(args.payload)
        baseline_fit = json.loads(args.baseline_fit.read_text()) if args.baseline_fit and args.baseline_fit.exists() else None
        est = identify_dynamic_mass_com(baseline, payload, args.official_root,
                                        baseline_fit=baseline_fit, opening_mm=args.opening_mm)
        gt = _trial_eval(args.trial)
        out = {"schema": "real2sim/fr3_payload_id_evaluation/v1",
               "estimate": est, "gt_evaluation_only": gt,
               "errors": _errors(est, gt) if gt.get("available") else {},
               "gt_used_for_estimation": False,
               "official_reuse": {"commit": "c52e31cf26c83b33aee5e56f805e1d4d710fd549",
                                  "modules": ["data.extract_numeric_data_matrix_autodiff",
                                              "data.compute_base_param_mapping",
                                              "utils.filtering.filter_time_series_data",
                                              "eric_id.drake_torch_dynamics.MassAndComInertialParameter"],
                                  "fr3_adaptation": ["arm-only renamed FR3 URDF", "base weld + explicit actuators",
                                                      "TCP external payload regressor"]}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=2, allow_nan=False, default=lambda x: x.tolist() if hasattr(x, "tolist") else str(x)))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
