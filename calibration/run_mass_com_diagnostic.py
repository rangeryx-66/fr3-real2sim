"""Run a paired normal-grasp/FixedJoint mass+CoM diagnostic sweep.

This runner is calibration-only.  It never changes the grasp, scan, mesh,
CoACD, or USD pipelines.  A normal payload capture is attempted first for
each Arena target.  If it passes the existing physical guard, the exact
center-q and static protocol are reused for an empty-arm baseline and a
diagnostic FixedJoint payload capture.  Failed attempts and their logs remain
in the output tree; only guarded captures enter the estimator.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TARGETS = [
    "mustard", "raisin", "hidden_tuna", "bowl",
    "banana", "sugar", "soup", "mug",
]
LAUNCH_PY = "/data1/home/rangeryx/isaaclab-arena/.venv/bin/python"
OFFICIAL_PY = "/data1/home/rangeryx/official_payload_env/bin/python"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def _payload_ok(run: Path) -> tuple[bool, str]:
    summary = _json(run / "acquisition_result.json")
    if not summary:
        return False, "NO_ACQUISITION_RESULT"
    payload = summary.get("payload") or {}
    static = payload.get("static") or {}
    rec = static.get("record") or {}
    if not static.get("ok"):
        detail = rec.get("abort") or static.get("detail") or static.get("reason")
        return False, str(detail or "PAYLOAD_STATIC_FAILED")
    if rec.get("abort"):
        return False, str(rec["abort"])
    npz = run / "system_id_payload_static.npz"
    if not npz.exists():
        return False, "NO_PAYLOAD_STATIC_NPZ"
    return True, "OK"


def _run_command(
    args: list[str],
    *,
    cwd: Path,
    env: dict[str, str],
    log_path: Path,
    timeout_s: float = 3600.0,
) -> dict[str, Any]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log_path.open("w") as log:
        try:
            p = subprocess.run(
                args,
                cwd=cwd,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=timeout_s,
                check=False,
            )
            return {
                "returncode": int(p.returncode),
                "wall_seconds": time.monotonic() - started,
                "command": args,
                "log": str(log_path),
            }
        except subprocess.TimeoutExpired:
            return {
                "returncode": 124,
                "wall_seconds": time.monotonic() - started,
                "command": args,
                "log": str(log_path),
                "error": "TIMEOUT",
            }
        except Exception as exc:  # pragma: no cover - environment failure
            return {
                "returncode": 125,
                "wall_seconds": time.monotonic() - started,
                "command": args,
                "log": str(log_path),
                "error": str(exc),
            }


def _launch_args(
    *,
    target: str,
    seed: int,
    output: Path,
    port: int,
    domain: int,
    mode: str = "ANYGRASP",
    gt_path: Path | None = None,
    center_q: Path | None = None,
    baseline_only: bool = False,
    capture_payload: bool = False,
) -> list[str]:
    args = [
        LAUNCH_PY,
        "-u",
        "calibration/launch_real2sim.py",
        "--target", target,
        "--seed", str(seed),
        "--mode", mode,
        "--output", str(output),
        "--gpu", os.environ.get("PAYLOAD_DIAGNOSTIC_GPU", "3"),
        "--port", str(port),
        "--domain", str(domain),
        "--payload-v2",
        "--mass-com-only",
        "--skip-scan",
    ]
    if gt_path is not None:
        args += ["--gt-path", str(gt_path)]
    if center_q is not None:
        args += ["--center-q", str(center_q)]
    if baseline_only:
        args.append("--baseline-only")
    if capture_payload:
        args.append("--capture-payload")
    return args


def _make_pose_replay(normal: Path, target: str, seed: int, mode: str) -> Path | None:
    """Extract the selected pose from a successful run for the fixed-joint pair.

    This keeps the diagnostic pair identical at the grasp-pose level while
    still letting the normal leg use the frozen AnyGrasp-first selection.  The
    replay file contains only the measured target pose and selected TCP pose;
    no GT physical parameter is read or written.
    """
    result_path = normal / f"{target}_seed{seed}_{mode}.json"
    result = _json(result_path)
    if not result:
        return None
    started = [
        attempt for attempt in result.get("attempts", [])
        if attempt.get("actual_grasp_started") and attempt.get("T_B_TCP_commanded")
    ]
    # The executor may physically try lower ranked candidates before the
    # successful one.  Replay the successful attempt, never the first started
    # attempt, so normal and FixedJoint legs have the exact same grasp pose.
    selected = None
    selected_index = None
    for index in range(len(started) - 1, -1, -1):
        attempt = started[index]
        stability = attempt.get("stability") or {}
        if stability.get("category") in {"SUCCESS", "STABLE"} or attempt.get("success"):
            selected = attempt
            selected_index = index
            break
    if selected is None and started:
        selected = started[-1]
        selected_index = len(started) - 1
    if selected is None:
        # Some historical executor records put the pose in the selected
        # candidate metadata rather than the attempt record.
        for attempt in reversed(result.get("attempts", [])):
            if attempt.get("T_B_TCP_commanded"):
                selected = attempt
                break
    if selected is None or not result.get("initial_target"):
        return None
    # A failed first grasp can move the free target before a later candidate
    # succeeds.  The selected TCP is then expressed relative to that latest
    # released target pose, not the episode's original pose.  Preserve the
    # measured pose in the replay so the diagnostic FixedJoint leg evaluates
    # exactly the same object/TCP pair.  This is still replay metadata only;
    # no GT pose or physical parameter enters the estimator.
    replay_target = result["initial_target"]
    if selected_index is not None:
        for previous in started[: selected_index + 1]:
            if previous.get("release_target_pose") is not None:
                replay_target = previous["release_target_pose"]
    if selected.get("release_target_pose") is not None:
        replay_target = selected["release_target_pose"]
    replay = normal / "selected_pose_replay.json"
    replay.write_text(json.dumps({
        "schema": "payload_mass_com/selected_pose_replay/v1",
        "source_result": str(result_path),
        "source_mode": mode,
        "target": target,
        "seed": seed,
        "T_B_target": replay_target,
        "T_B_TCP": selected["T_B_TCP_commanded"],
        "selected_rank": result.get("selected_rank"),
        "raw_score": selected.get("raw_score"),
    }, indent=2))
    return replay


def _safe_domain(domain_base: int, attempt_index: int, phase: int) -> int:
    """Return a ROS 2 domain in the DDS-supported range.

    The sweep is sequential, so a small deterministic domain ring is enough to
    isolate a run while avoiding the Fast-DDS multicast-port limit (domain
    values above 232 are invalid).  ``phase`` is 0=normal, 1=baseline,
    2=FixedJoint.
    """
    return (int(domain_base) + int(attempt_index) * 3 + int(phase)) % 200


def _base_env() -> dict[str, str]:
    env = os.environ.copy()
    env.update({
        "PYTHONUNBUFFERED": "1",
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NO_PROXY": "127.0.0.1,localhost",
        "no_proxy": "127.0.0.1,localhost",
        "OMNI_KIT_ACCEPT_EULA": "YES",
        "ACCEPT_EULA": "Y",
    })
    env.pop("PAYLOAD_ATTACHMENT_MODE", None)
    return env


def _copy_protocol(normal: Path, destination: Path) -> dict[str, Any]:
    """Copy the exact generated protocol into a paired leg.

    Empty-arm and FixedJoint legs must consume the same static pose sequence,
    rather than independently regenerating IK variants from slightly
    different cold-start states.  This is a calibration pairing invariant;
    it does not alter the production grasp/execution pipeline.
    """
    destination.mkdir(parents=True, exist_ok=True)
    copied = []
    for name in (
        "payload_id_v2_protocol.json",
        "payload_id_v2_protocol_static.json",
        "payload_id_v2_protocol_dynamic.json",
    ):
        src = normal / name
        if src.exists():
            dst = destination / name
            shutil.copy2(src, dst)
            copied.append({"name": name, "sha256": _sha256(dst)})
    return {"files": copied}


def _link_source(src: Path, dst: Path) -> None:
    """Expose one capture file in a paired evaluator directory.

    The simulator writes the robot-alone and payload records in separate
    process directories.  The official adapter deliberately consumes a
    single run directory, so a pairing directory contains only absolute
    links to those already recorded files.  No data are copied into the
    estimator and the original attempt tree remains immutable/auditable.
    """
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    dst.symlink_to(src.resolve())


def _prepare_pair_run(
    attempt_root: Path,
    name: str,
    baseline: Path,
    payload: Path,
    protocol_source: Path,
    center_source: Path,
    trial_source: Path | None,
) -> Path:
    """Create a read-only view consumed by the official paired estimator."""

    pair = attempt_root / "pairs" / name
    pair.mkdir(parents=True, exist_ok=True)
    _link_source(baseline / "system_id_baseline_static.npz", pair / "system_id_baseline_static.npz")
    _link_source(payload / "system_id_payload_static.npz", pair / "system_id_payload_static.npz")
    for protocol_name in (
        "payload_id_v2_protocol.json",
        "payload_id_v2_protocol_static.json",
        "payload_id_v2_protocol_dynamic.json",
    ):
        source = protocol_source / protocol_name
        if source.exists():
            _link_source(source, pair / protocol_name)
    if center_source.exists():
        _link_source(center_source, pair / "excitation_center.json")
    if trial_source is not None and trial_source.exists():
        _link_source(trial_source, pair / trial_source.name)
    (pair / "pair_metadata.json").write_text(json.dumps({
        "schema": "real2sim/paired_capture_view/v1",
        "baseline_source": str(baseline.resolve()),
        "payload_source": str(payload.resolve()),
        "protocol_source": str(protocol_source.resolve()),
        "trial_source_evaluation_only": str(trial_source.resolve()) if trial_source else None,
        "gt_used_for_estimation": False,
    }, indent=2))
    return pair


def _evaluate(run: Path, *, official_root: Path, max_pair_q_error: float) -> dict[str, Any]:
    # AnyGrasp normal legs use the frozen proposal result as the trial record;
    # GT is read only by the evaluator for post-fit error reporting.  Fixed
    # diagnostic legs keep the ``*_GT.json`` naming.  Do not feed either file
    # into the estimator itself.
    gt_candidates = sorted(run.glob("*_GT.json")) or sorted(run.glob("*_ANYGRASP.json"))
    gt = next(iter(gt_candidates), None)
    args = [
        OFFICIAL_PY,
        "-u",
        "calibration/evaluate_payload_mass_com_official.py",
        "--run", str(run),
        "--official-root", str(official_root),
        "--max-pair-q-error-rad", str(max_pair_q_error),
    ]
    if gt is not None:
        args += ["--gt-trial", str(gt)]
    log = run / "mass_com_evaluation.log"
    eval_env = _base_env()
    # The evaluator is launched by absolute path from ``calibration/``; add
    # the repository root explicitly so its sibling ``real2sim`` package is
    # importable in the official payload environment.
    # ``official_payload_env`` provides pydrake/torch while the IsaacLab
    # environment provides scipy used by the FR3 adapter.  Both are the same
    # Python 3.12 ABI; expose only the Isaac site-packages directory rather
    # than changing either environment or estimator code.
    eval_env["PYTHONPATH"] = (str(ROOT) + os.pathsep + str(ROOT / "src") +
                              os.pathsep + str(ROOT / "calibration") + os.pathsep +
                              "/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages" +
                              os.pathsep + eval_env.get("PYTHONPATH", ""))
    outcome = _run_command(args, cwd=ROOT, env=eval_env, log_path=log, timeout_s=300.0)
    report = _json(run / "payload_id_mass_com_official.json") or {}
    return {"runner": outcome, "report": report}


def _target_seeds(protocol_path: Path, target: str) -> list[int]:
    data = _json(protocol_path) or {}
    seeds = [int(e["seed"]) for e in data.get("episodes", []) if e.get("target") == target]
    return seeds or list(range(1000, 1040))


def run_target(
    target: str,
    seeds: list[int],
    root_output: Path,
    *,
    max_attempts: int,
    port_base: int,
    domain_base: int,
    official_root: Path,
    max_pair_q_error: float,
    mode: str,
) -> dict[str, Any]:
    target_root = root_output / target
    target_root.mkdir(parents=True, exist_ok=True)
    attempts: list[dict[str, Any]] = []
    selected: dict[str, Any] | None = None
    for attempt_index, seed in enumerate(seeds[:max_attempts]):
        attempt_root = target_root / f"attempt_{attempt_index:02d}_seed{seed}"
        normal = attempt_root / "normal"
        baseline = attempt_root / "baseline"
        fixed = attempt_root / "fixedjoint"
        normal.mkdir(parents=True, exist_ok=True)
        env = _base_env()
        normal_outcome = _run_command(
            _launch_args(target=target, seed=seed, output=normal,
                         port=port_base + attempt_index * 3,
                         mode=mode,
                         domain=_safe_domain(domain_base, attempt_index, 0),
                         capture_payload=True),
            cwd=ROOT, env=env, log_path=normal / "runner.log", timeout_s=3600.0,
        )
        normal_ok, normal_reason = _payload_ok(normal)
        row: dict[str, Any] = {
            "target": target,
            "seed": seed,
            "attempt_index": attempt_index,
            "normal": {"run": str(normal), "outcome": normal_outcome,
                        "guarded_static": normal_ok, "reason": normal_reason},
        }
        if not normal_ok:
            attempts.append(row)
            (attempt_root / "attempt_manifest.json").write_text(json.dumps(row, indent=2))
            continue

        replay_gt = _make_pose_replay(normal, target, seed, mode)
        if replay_gt is None:
            row["normal"]["reason"] = "NO_SELECTED_POSE_REPLAY"
            attempts.append(row)
            (attempt_root / "attempt_manifest.json").write_text(json.dumps(row, indent=2))
            continue
        row["selected_pose_replay"] = str(replay_gt)

        center = normal / "excitation_center.json"
        protocol_copy = {
            "baseline": _copy_protocol(normal, baseline),
            "fixedjoint": _copy_protocol(normal, fixed),
        }
        row["protocol_copy"] = protocol_copy
        if not center.exists():
            row["normal"]["reason"] = "NO_EXCITATION_CENTER"
            attempts.append(row)
            (attempt_root / "attempt_manifest.json").write_text(json.dumps(row, indent=2))
            continue

        baseline.mkdir(parents=True, exist_ok=True)
        baseline_outcome = _run_command(
            _launch_args(target=target, seed=seed, output=baseline,
                         port=port_base + attempt_index * 3 + 1,
                         mode="GT",
                         domain=_safe_domain(domain_base, attempt_index, 1),
                         center_q=center, baseline_only=True),
            cwd=ROOT, env=env, log_path=baseline / "runner.log", timeout_s=3600.0,
        )
        row["baseline"] = {"run": str(baseline), "outcome": baseline_outcome,
                            "recorded": (baseline / "system_id_baseline_static.npz").exists()}

        fixed.mkdir(parents=True, exist_ok=True)
        fixed_env = _base_env()
        fixed_env["PAYLOAD_ATTACHMENT_MODE"] = "FIXED"
        fixed_outcome = _run_command(
            _launch_args(target=target, seed=seed, output=fixed,
                         port=port_base + attempt_index * 3 + 2,
                         mode="GT", gt_path=replay_gt,
                         domain=_safe_domain(domain_base, attempt_index, 2),
                         center_q=center, capture_payload=True),
            cwd=ROOT, env=fixed_env, log_path=fixed / "runner.log", timeout_s=3600.0,
        )
        fixed_ok, fixed_reason = _payload_ok(fixed)
        row["fixedjoint"] = {"run": str(fixed), "outcome": fixed_outcome,
                              "guarded_static": fixed_ok, "reason": fixed_reason}
        if fixed_ok and row["baseline"]["recorded"]:
            # The estimator requires a run directory containing both streams.
            # Keep normal and FixedJoint payload captures separate, while
            # pointing both at the exact same baseline and generated protocol.
            normal_trial = next(iter(sorted(normal.glob("*_ANYGRASP.json"))), None)
            fixed_trial = next(iter(sorted(fixed.glob("*_GT.json"))), None)
            pair_normal = _prepare_pair_run(
                attempt_root, "normal", baseline, normal, normal,
                normal / "excitation_center.json", normal_trial,
            )
            pair_fixed = _prepare_pair_run(
                attempt_root, "fixedjoint", baseline, fixed, normal,
                fixed / "excitation_center.json", fixed_trial,
            )
            row["paired_runs"] = {
                "normal": str(pair_normal),
                "fixedjoint": str(pair_fixed),
                "shared_baseline": str((baseline / "system_id_baseline_static.npz").resolve()),
                "shared_protocol": str((normal / "payload_id_v2_protocol.json").resolve()),
            }
            row["normal_estimate"] = _evaluate(pair_normal, official_root=official_root,
                                                  max_pair_q_error=max_pair_q_error)
            row["fixedjoint_estimate"] = _evaluate(pair_fixed, official_root=official_root,
                                                     max_pair_q_error=max_pair_q_error)
            for run_name, run_path in (("normal", normal), ("fixedjoint", fixed)):
                proto = run_path / "payload_id_v2_protocol.json"
                row.setdefault("protocol_sha256", {})[run_name] = _sha256(proto) if proto.exists() else None
            row["pairing_protocol_identical"] = (
                row["protocol_sha256"].get("normal") == row["protocol_sha256"].get("fixedjoint")
            )
            selected = row
            row["selected"] = True
            attempts.append(row)
            (attempt_root / "attempt_manifest.json").write_text(json.dumps(row, indent=2))
            break
        attempts.append(row)
        (attempt_root / "attempt_manifest.json").write_text(json.dumps(row, indent=2))

    return {"target": target, "attempts": attempts, "selected": selected}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--targets", nargs="+", default=DEFAULT_TARGETS)
    parser.add_argument("--max-attempts", type=int, default=4)
    parser.add_argument("--mode", choices=["ANYGRASP", "GT"], default="ANYGRASP",
                        help="frozen proposal mode for the normal leg; FixedJoint replays its selected pose")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "results/payload_mass_com_diagnostic_v1")
    parser.add_argument("--protocol", type=Path, default=ROOT / "ARENA_COMPLEX_PROTOCOL.json")
    parser.add_argument("--port-base", type=int, default=19100)
    parser.add_argument("--domain-base", type=int, default=20)
    parser.add_argument("--official-root", type=Path,
                        default=Path("/data1/home/rangeryx/fr3_moveit_grasp/third_party/scalable_real2sim_robot_payload_id_upstream_c52e31c"))
    parser.add_argument("--max-pair-q-error-rad", type=float, default=0.03)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema": "real2sim/payload_mass_com_diagnostic_sweep/v1",
        "targets": args.targets,
        "max_attempts": args.max_attempts,
        "grasp_mode": args.mode,
        "pairing": {"actual_measured_joint_pose": True,
                     "max_q_error_rad": args.max_pair_q_error_rad},
        "gt_used_for_estimation": False,
        "frozen_pipeline": ["grasp", "scan", "MV-SAM3D", "CoACD", "USD"],
        "started_unix": time.time(),
        "results": [],
    }
    for i, target in enumerate(args.targets):
        seeds = _target_seeds(args.protocol, target)
        # Use the protocol's fixed seed order; changing seed is only a retry,
        # never a result-dependent parameter choice.
        result = run_target(
            target, seeds, args.output,
            max_attempts=args.max_attempts,
            port_base=args.port_base + i * 20,
            domain_base=args.domain_base + i * 20,
            official_root=args.official_root,
            max_pair_q_error=args.max_pair_q_error_rad,
            mode=args.mode,
        )
        manifest["results"].append(result)
        (args.output / "sweep_progress.json").write_text(json.dumps(manifest, indent=2))
    manifest["finished_unix"] = time.time()
    manifest["wall_seconds"] = manifest["finished_unix"] - manifest["started_unix"]
    (args.output / "mass_com_diagnostic_summary.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
