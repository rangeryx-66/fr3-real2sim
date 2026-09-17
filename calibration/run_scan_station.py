"""Run the independent scan-station acquisition and ReconViaGen asset path."""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
ISAAC_PY = os.environ.get('ISAAC_PYTHON', '/data1/home/rangeryx/isaaclab-arena/.venv/bin/python')
# Running this file directly puts only ``calibration/`` on sys.path.  Add the
# project root explicitly so the independent ``real2sim.scan_station`` skill
# is importable without changing the frozen grasp executor or requiring a
# package install on the remote host.
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _wait_ready(port: int, process: subprocess.Popen, timeout_s: float = 180.0) -> None:
    deadline = time.monotonic() + timeout_s
    url = f"http://127.0.0.1:{port}/calibration"
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"scan-station simulator exited {process.returncode}")
        try:
            with urlopen(url, timeout=2) as f:
                if json.load(f).get("ready"):
                    return
        except Exception:
            pass
        time.sleep(1.0)
    raise TimeoutError("scan-station simulator startup")


def _stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=10)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--target", default="soup")
    p.add_argument("--seed", type=int, default=1030)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--inertial", type=Path,
                   default=ROOT / "results/real2sim_soup_v1_final/inertial_object_simulation.json")
    p.add_argument("--gt-mesh", type=Path)
    p.add_argument("--gpu", type=int, default=3)
    p.add_argument("--port", type=int, default=18930)
    p.add_argument("--views-per-ring", type=int, default=12)
    p.add_argument(
        "--official-frame-count", type=int,
        help="Capture a continuous official-like RGB-D stream (default 1800 when set) instead of sparse per-view commands.",
    )
    p.add_argument(
        "--official-horizontal-fraction", type=float, default=0.5,
        help="Fraction of the continuous stream assigned to the horizontal ring.",
    )
    p.add_argument("--minimum-accepted-views", type=int, default=18)
    p.add_argument("--radius-m", type=float, default=.30)
    p.add_argument("--station-x", type=float, default=.70)
    p.add_argument("--station-y", type=float, default=.25)
    p.add_argument("--station-z", type=float)
    p.add_argument("--robot-stage-x", type=float, default=-3.00)
    p.add_argument("--robot-stage-y", type=float, default=0.0)
    p.add_argument("--crop-padding", type=float, default=.15)
    p.add_argument("--asset-name")
    p.add_argument("--skip-reconstruction", action="store_true")
    a = p.parse_args()
    if a.station_z is None:
        inventory = json.loads((Path(os.environ.get("FR3_ASSET_DIRECTORY",str(ROOT / "assets/arena_complex"))) / "inventory.json").read_text())
        # The clean station height is derived from the asset's local support
        # bound, not from any evaluation pose or GT run-time transform.
        a.station_z = float(-inventory[a.target]["bounds"][0][2] + .001)
    output = a.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    scan_root = output / "scan_station"
    final_root = output / "reconviagen_v05"
    asset_name = a.asset_name or f"{a.target}_scan_station_reconviagen_v05"
    gt_mesh = a.gt_mesh
    if gt_mesh is None:
        candidate = ROOT / f"results/real2sim_soup_scan_v3_coverage/reconviagen_v05/final/gt_eval/{a.target}_gt.obj"
        if candidate.exists():
            gt_mesh = candidate

    log_path = output / "scan_station_sim.log"
    env = {
        **os.environ,
        "PYTHONPATH": f"{ROOT}:{ROOT / 'src'}:{ROOT / 'calibration'}",
        "FR3_ARENA_TARGET": a.target,
        "CALIBRATION_SCENE_SEED": str(a.seed),
        "FR3_ASSET_DIRECTORY": str(ROOT / "assets/arena_complex"),
        "OMNI_KIT_ACCEPT_EULA": "YES",
        "ACCEPT_EULA": "Y",
        "NO_PROXY": "127.0.0.1,localhost",
        "no_proxy": "127.0.0.1,localhost",
        "OPENBLAS_NUM_THREADS": "1",
    }
    t0 = time.monotonic()
    sim = None
    acquisition = None
    try:
        with log_path.open("w") as log:
            sim = subprocess.Popen(
                [ISAAC_PY, "-u", "calibration/sim_scan_station.py",
                 "--gpu", str(a.gpu), "--port", str(a.port),
                 "--target", a.target, "--seed", str(a.seed),
                 "--station-x", str(a.station_x), "--station-y", str(a.station_y),
                 "--station-z", str(a.station_z),
                 "--robot-stage-x", str(a.robot_stage_x), "--robot-stage-y", str(a.robot_stage_y)],
                cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        _wait_ready(a.port, sim)
        from real2sim.scan_station import ScanStationConfig, ScanStationSkill, StationClient
        config = ScanStationConfig(
            center_m=(a.station_x, a.station_y,
                      float(a.station_z) if a.station_z is not None else
                      ScanStationConfig().center_m[2]),
            views_per_ring=a.views_per_ring,
            radius_m=a.radius_m,
            crop_padding=a.crop_padding,
            minimum_accepted_views=a.minimum_accepted_views,
        )
        # Isaac Sim may spend several seconds in a render/fetch cycle before
        # the HTTP handler can return.  Keep this transport timeout separate
        # from the per-command completion timeout used by ``wait``.
        skill = ScanStationSkill(StationClient(a.port, timeout_s=30.0), scan_root, a.target, config)
        if a.official_frame_count is not None:
            acquisition = skill.run_official_continuous(
                frame_count=int(a.official_frame_count),
                horizontal_fraction=float(a.official_horizontal_fraction),
            )
        else:
            acquisition = skill.run()
    finally:
        _stop(sim)
    capture_seconds = time.monotonic() - t0
    summary = {
        "schema": "fr3_scan_station_run/v1",
        "target": a.target,
        "seed": a.seed,
        "scan_root": str(scan_root),
        "capture_seconds": capture_seconds,
        "official_frame_count": a.official_frame_count,
        "acquisition": acquisition,
        "grasp_executor_modified": False,
        "reconstruction_started": False,
    }
    if not a.skip_reconstruction:
        if not a.inertial.exists():
            raise FileNotFoundError(f"inertial JSON not found: {a.inertial}")
        final_root.mkdir(parents=True, exist_ok=True)
        pipeline_env = {
            **os.environ,
            "RECONVIAGEN_NUM_VIEWS": str(a.views_per_ring * 2),
            "RECONVIAGEN_MIN_OVERLAP": ".18",
            "RECONVIAGEN_REUSE_COLLISION": "0",
            "OBSERVED_TSDF_MESH": "",
            "ISAAC_PY": ISAAC_PY,
            "GPU": str(a.gpu),
            "ASSET_NAME": asset_name,
            "HF_HUB_OFFLINE": "1",
            "TRANSFORMERS_OFFLINE": "1",
        }
        command = [
            "bash", "calibration/run_reconviagen_asset_pipeline.sh",
            str(scan_root), str(scan_root / "tracking"), str(a.inertial),
            str(final_root),
        ]
        if gt_mesh is not None:
            command.append(str(gt_mesh))
        stage_start = time.monotonic()
        with (output / "reconviagen_pipeline.log").open("w") as log:
            subprocess.run(command, cwd=ROOT, env=pipeline_env,
                           stdout=log, stderr=subprocess.STDOUT, check=True,
                           timeout=3600)
        summary.update({
            "reconstruction_started": True,
            "reconstruction_seconds": time.monotonic() - stage_start,
            "final_root": str(final_root),
            "asset_name": asset_name,
            "gt_mesh_used_only_for_evaluation": gt_mesh is not None,
        })
    summary_path = output / "scan_station_run.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    # Keep reporting independent from capture/reconstruction.  If a future
    # partial run is inspected before all products exist, the report still
    # contains the station QA and records the missing downstream fields.
    try:
        from real2sim.scan_station_report import write_report
        summary["report"] = str(write_report(output))
    except Exception as exc:  # pragma: no cover - diagnostics must not hide a run
        summary["report_error"] = repr(exc)
    summary_path.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
