"""Run the FR3 official-style calibration and strict paired payload trials.

This is a calibration-only runner.  It launches the existing isolated Isaac
plant and frozen grasp executor; no scan, MV-SAM3D, CoACD or USD code is
modified.  Robot-alone baselines are collected at five hand openings.  Then
three to five stable Arena objects are attempted with the same analytic
Fourier trajectory and a matching empty-arm capture at the selected opening.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ISAAC_PY = "/data1/home/rangeryx/isaaclab-arena/.venv/bin/python"
OFFICIAL_PY = "/data1/home/rangeryx/official_payload_env/bin/python"
OFFICIAL_ROOT = Path(
    "/data1/home/rangeryx/fr3_moveit_grasp/third_party/"
    "scalable_real2sim_robot_payload_id_upstream_c52e31c"
)
DEFAULT_OUTPUT = ROOT / "results/fr3_official_payloadid_v1"
# Calibration-only contact settings.  The frozen grasp executor is not
# changed; these values are passed only to the isolated PayloadID simulator so
# a 0.30-rad excitation cannot be mistaken for a weak-grasp failure.
CALIBRATION_FORCE_N = 60.0
CALIBRATION_MU = 1.0


def _sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def _json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def _run(cmd: list[str], cwd: Path, env: dict[str, str], log: Path,
         timeout_s: float = 3600.0) -> dict[str, Any]:
    log.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log.open("w") as f:
        try:
            p = subprocess.run(cmd, cwd=cwd, env=env, stdout=f, stderr=subprocess.STDOUT,
                               timeout=timeout_s, check=False)
            return {"returncode": int(p.returncode), "wall_seconds": time.monotonic() - started,
                    "command": cmd, "log": str(log)}
        except subprocess.TimeoutExpired:
            return {"returncode": 124, "wall_seconds": time.monotonic() - started,
                    "command": cmd, "log": str(log), "error": "TIMEOUT"}


def _base_env() -> dict[str, str]:
    env = os.environ.copy()
    env.update({"PYTHONUNBUFFERED": "1", "OPENBLAS_NUM_THREADS": "1",
                "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                "NO_PROXY": "127.0.0.1,localhost", "no_proxy": "127.0.0.1,localhost",
                "OMNI_KIT_ACCEPT_EULA": "YES", "ACCEPT_EULA": "Y"})
    env.pop("PAYLOAD_ATTACHMENT_MODE", None)
    return env


def _launch(*, target: str, seed: int, output: Path, port: int, domain: int,
            baseline_only: bool = False, capture_payload: bool = False,
            center_q: Path | None = None, opening_mm: float | None = None,
            mode: str = "ANYGRASP") -> list[str]:
    cmd = [ISAAC_PY, "-u", "calibration/launch_real2sim.py", "--target", target,
           "--seed", str(seed), "--mode", mode, "--output", str(output), "--gpu",
           os.environ.get("PAYLOAD_OFFICIAL_GPU", "3"), "--port", str(port),
           "--domain", str(domain), "--payload-v2", "--dynamic-only", "--skip-scan",
           "--calibration-force", f"{CALIBRATION_FORCE_N:.1f}",
           "--calibration-mu", f"{CALIBRATION_MU:.2f}"]
    if baseline_only:
        cmd.append("--baseline-only")
    if capture_payload:
        cmd.append("--capture-payload")
    if center_q:
        cmd += ["--center-q", str(center_q)]
    if opening_mm is not None:
        cmd += ["--gripper-opening-mm", f"{opening_mm:.6f}"]
    return cmd


def _eval_robot(record: Path, output: Path, opening: float) -> dict[str, Any]:
    env = _base_env()
    env["PYTHONPATH"] = os.pathsep.join([
        str(ROOT), str(ROOT / "src"), str(ROOT / "calibration"),
        str(OFFICIAL_ROOT),
        "/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages",
        env.get("PYTHONPATH", ""),
    ])
    return _run([OFFICIAL_PY, "-u", "calibration/evaluate_fr3_payloadid.py", "robot",
                 "--record", str(record), "--output", str(output),
                 "--official-root", str(OFFICIAL_ROOT), "--opening-mm", str(opening)],
                ROOT, env, output.with_suffix(".log"), timeout_s=900.0)


def _eval_payload(baseline: Path, payload: Path, output: Path, opening: float,
                  trial: Path | None, baseline_fit: Path | None) -> dict[str, Any]:
    env = _base_env()
    env["PYTHONPATH"] = os.pathsep.join([
        str(ROOT), str(ROOT / "src"), str(ROOT / "calibration"), str(OFFICIAL_ROOT),
        "/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages",
        env.get("PYTHONPATH", ""),
    ])
    cmd = [OFFICIAL_PY, "-u", "calibration/evaluate_fr3_payloadid.py", "payload",
           "--baseline", str(baseline), "--payload", str(payload), "--output", str(output),
           "--official-root", str(OFFICIAL_ROOT), "--opening-mm", str(opening)]
    if trial and trial.exists(): cmd += ["--trial", str(trial)]
    if baseline_fit and baseline_fit.exists(): cmd += ["--baseline-fit", str(baseline_fit)]
    return _run(cmd, ROOT, env, output.with_suffix(".log"), timeout_s=900.0)


def _opening_from_center(center: Path) -> float:
    d = _json(center) or {}
    fq = np.asarray(d.get("finger_q", [0.04, 0.04]), float)
    return float(np.clip(2.0 * float(np.mean(fq)) * 1000.0, 0.0, 80.0))


def _payload_guard_ok(run: Path) -> tuple[bool, str]:
    path = run / "system_id_payload_dynamic.npz"
    if not path.exists():
        return False, "NO_DYNAMIC_RECORD"
    try:
        d = np.load(path, allow_pickle=True)
        ok = bool(np.asarray(d.get("guard_passed", [False])).reshape(-1)[-1])
        if not ok:
            return False, str(np.asarray(d.get("motion_events_json", ["GUARD_ABORT"])).reshape(-1)[-1])
        if len(d["t"]) < 100:
            return False, f"SHORT_DYNAMIC_RECORD:{len(d['t'])}"
        return True, "OK"
    except Exception as exc:
        return False, f"READ_RECORD:{exc}"


def _trial_file(run: Path, mode: str) -> Path | None:
    candidates = sorted(run.glob(f"*_ANYGRASP.json")) + sorted(run.glob(f"*_GT.json"))
    return candidates[0] if candidates else None


def run(args: argparse.Namespace) -> dict[str, Any]:
    out = Path(args.output).resolve(); out.mkdir(parents=True, exist_ok=True)
    env = _base_env()
    manifest: dict[str, Any] = {
        "schema": "real2sim/fr3_official_payloadid_run/v1",
        "official_commit": "c52e31cf26c83b33aee5e56f805e1d4d710fd549",
        "frozen_pipeline": ["grasp", "scan", "MV-SAM3D", "CoACD", "USD"],
        "estimator": "paired_dynamic_mass_com_then_geometry_inertia_fallback",
        "gt_used_for_estimation": False,
        "openings_mm": [0, 20, 40, 60, 80], "robot_baselines": [], "payload_trials": [],
        "started_unix": time.time(),
    }
    # Stage 1: each opening gets a fresh empty-arm dynamic capture.  The
    # target remains in the table scene only to preserve the existing plant
    # startup; no grasp or payload is used.
    for idx, opening in enumerate((0, 20, 40, 60, 80)):
        run_dir = out / "robot_baselines" / f"opening_{opening:03d}"
        run_dir.mkdir(parents=True, exist_ok=True)
        runner = _run(_launch(target="soup", seed=1030, output=run_dir,
                              port=19300 + idx * 3, domain=50 + idx * 3,
                              baseline_only=True, opening_mm=opening),
                      ROOT, env, run_dir / "runner.log", timeout_s=3600.0)
        record = run_dir / "system_id_baseline_dynamic.npz"
        fit_path = run_dir / "robot_fit.json"
        fit_runner = _eval_robot(record, fit_path, opening) if record.exists() else None
        row = {"opening_mm": opening, "run": str(run_dir), "runner": runner,
               "record": str(record), "record_exists": record.exists(),
               "fit": str(fit_path), "fit_runner": fit_runner}
        manifest["robot_baselines"].append(row)
        (out / "progress.json").write_text(json.dumps(manifest, indent=2, default=str))

    # Stage 2: capture payload dynamic data with the frozen executor.  A
    # failed grasp is retained and the next predeclared seed is attempted.
    objects = [("soup", [1030, 1031, 1032, 1033, 1034]),
               ("mustard", [1000, 1001, 1002, 1003, 1004]),
               ("banana", [1020, 1021, 1022, 1023, 1024]),
               ("mug", [1035, 1036, 1037, 1038, 1039]),
               ("bowl", [1015, 1016, 1017, 1018, 1019])]
    for oi, (target, seeds) in enumerate(objects[:args.objects]):
        target_row: dict[str, Any] = {"target": target, "attempts": []}
        for ai, seed in enumerate(seeds[:args.max_attempts]):
            run_dir = out / "payload_trials" / target / f"attempt_{ai:02d}_seed{seed}"
            run_dir.mkdir(parents=True, exist_ok=True)
            runner = _run(_launch(target=target, seed=seed, output=run_dir,
                                  port=19500 + oi * 30 + ai * 3,
                                  domain=100 + oi * 20 + ai * 3,
                                  capture_payload=True), ROOT, env,
                          run_dir / "runner.log", timeout_s=3600.0)
            ok, reason = _payload_guard_ok(run_dir)
            row: dict[str, Any] = {"target": target, "seed": seed, "attempt": ai,
                                   "run": str(run_dir), "runner": runner,
                                   "dynamic_guarded": ok, "reason": reason}
            if not ok:
                target_row["attempts"].append(row)
                continue
            center = run_dir / "excitation_center.json"
            opening = _opening_from_center(center)
            baseline_dir = out / "paired_baselines" / target / f"attempt_{ai:02d}_seed{seed}"
            baseline_dir.mkdir(parents=True, exist_ok=True)
            baseline_runner = _run(_launch(target=target, seed=seed,
                                           output=baseline_dir,
                                           port=19500 + oi * 30 + ai * 3 + 1,
                                           domain=100 + oi * 20 + ai * 3 + 1,
                                           baseline_only=True, center_q=center,
                                           opening_mm=opening), ROOT, env,
                                   baseline_dir / "runner.log", timeout_s=3600.0)
            baseline_record = baseline_dir / "system_id_baseline_dynamic.npz"
            baseline_fit_path = baseline_dir / "robot_fit.json"
            baseline_fit_runner = (_eval_robot(baseline_record, baseline_fit_path, opening)
                                   if baseline_record.exists() else None)
            trial = _trial_file(run_dir, "ANYGRASP")
            eval_path = run_dir / "payload_dynamic_mass_com.json"
            eval_runner = (_eval_payload(baseline_record, run_dir / "system_id_payload_dynamic.npz",
                                         eval_path, opening, trial, baseline_fit_path)
                           if baseline_record.exists() else None)
            row.update({"opening_mm": opening, "center_q": str(center),
                        "payload_record": str(run_dir / "system_id_payload_dynamic.npz"),
                        "baseline_run": str(baseline_dir),
                        "baseline_runner": baseline_runner,
                        "baseline_record": str(baseline_record),
                        "baseline_fit": str(baseline_fit_path),
                        "baseline_fit_runner": baseline_fit_runner,
                        "evaluation": str(eval_path), "evaluation_runner": eval_runner,
                        "protocol_sha256_payload": _sha256(run_dir / "payload_id_v2_protocol.json"),
                        "protocol_sha256_baseline": _sha256(baseline_dir / "payload_id_v2_protocol.json")})
            target_row["attempts"].append(row)
            # First guarded capture is retained as the principal sample; later
            # attempts are still preserved if the user asks for more retries.
            if sum(bool(x.get("dynamic_guarded")) for x in target_row["attempts"]) >= 1:
                break
        manifest["payload_trials"].append(target_row)
        (out / "progress.json").write_text(json.dumps(manifest, indent=2, default=str))
    manifest["finished_unix"] = time.time(); manifest["wall_seconds"] = manifest["finished_unix"] - manifest["started_unix"]
    (out / "fr3_official_payloadid_manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    return manifest


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--objects", type=int, default=5, help="number of predefined object classes (3..5)")
    p.add_argument("--max-attempts", type=int, default=5)
    args = p.parse_args()
    print(json.dumps(run(args), indent=2, default=str))


if __name__ == "__main__":
    main()
