"""Launch one persistent Isaac/MoveIt continuous loop and production COM fit."""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ISAAC = "/data1/home/rangeryx/isaaclab-arena/.venv/bin/python"
OFFICIAL = "/data1/home/rangeryx/official_payload_env/bin/python"
OFFICIAL_SITE = "/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages"


def stop(process):
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def wait_sim(port, process):
    for _ in range(360):
        if process.poll() is not None:
            raise RuntimeError(f"simulator exited {process.returncode}")
        try:
            value = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/calibration", timeout=2).read())
            if "calibration" in value:
                return
        except Exception:
            pass
        time.sleep(1)
    raise TimeoutError("simulator startup")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="mustard")
    parser.add_argument("--seed", type=int, default=1004)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gpu", type=int, default=0)
    parser.add_argument("--port", type=int, default=18944)
    parser.add_argument("--domain", type=int, default=144)
    parser.add_argument("--calibration-force", type=float, default=60.0)
    parser.add_argument("--protocol", default=str(ROOT / "ARENA_COMPLEX_PROTOCOL.json"))
    parser.add_argument("--asset-directory", default=str(ROOT / "assets/arena_complex"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    args.output = args.output.resolve()

    os.environ["NO_PROXY"] = "127.0.0.1,localhost"
    os.environ["no_proxy"] = "127.0.0.1,localhost"
    ros = ROOT / "ros_env"
    site = f"{ros}/lib/python312/site-packages:{ros}/lib/python3.12/site-packages"
    env = {
        **os.environ,
        "FR3_ARENA_TARGET": args.target,
        "CALIBRATION_SCENE_SEED": str(args.seed),
        "UNSEEN_PROTOCOL": str(Path(args.protocol).resolve()),
        "FR3_ASSET_DIRECTORY": str(Path(args.asset_directory).resolve()),
        "CALIBRATION_MU": "3.5",
        "CALIBRATION_FORCE_N": str(args.calibration_force),
        "PAYLOAD_MASS_COM_ONLY": "1",
        "CALIBRATION_PORT": str(args.port),
        "ROS_DOMAIN_ID": str(args.domain),
        "ROS_LOCALHOST_ONLY": "1",
        "UNSEEN_GPU": str(args.gpu),
        "OMNI_KIT_ACCEPT_EULA": "YES",
        "ACCEPT_EULA": "Y",
        "NO_PROXY": "127.0.0.1,localhost",
        "no_proxy": "127.0.0.1,localhost",
        "AMENT_PREFIX_PATH": str(ros),
        "PATH": f"{ros}/bin:" + os.environ["PATH"],
        "PYTHONPATH": f"{ROOT}:{ROOT}/src:{ROOT}/calibration:{site}",
        "OPENBLAS_NUM_THREADS": "1",
    }
    env.pop("CUDA_VISIBLE_DEVICES", None)
    moveit = sim = None
    with (args.output / "moveit.log").open("w") as moveit_log, \
         (args.output / "sim.log").open("w") as sim_log, \
         (args.output / "pipeline.log").open("w") as pipeline_log:
        try:
            moveit = subprocess.Popen(
                [str(ros / "bin/ros2"), "launch", str(ROOT / "src/moveit.launch.py")],
                cwd=ROOT, env=env, stdout=moveit_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            time.sleep(8)
            sim = subprocess.Popen(
                [ISAAC, "-u", "calibration/sim_continuous_closed_loop.py",
                 "--gpu", str(args.gpu), "--port", str(args.port)],
                cwd=ROOT, env=env, stdout=sim_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            wait_sim(args.port, sim)
            subprocess.run(
                [str(ros / "bin/python"), "-u", "calibration/run_continuous_closed_loop.py",
                 "--seed", str(args.seed), "--mode", "ANYGRASP",
                 "--output", str(args.output), "--calibration-force", str(args.calibration_force),
                 "--payload-force", "70"],
                cwd=ROOT, env=env, stdout=pipeline_log, stderr=subprocess.STDOUT,
                check=True, timeout=5400,
            )
        finally:
            stop(sim)
            stop(moveit)

    analysis = args.output / "com_analysis"
    analysis.mkdir(exist_ok=True)
    payload_capture = json.loads((args.output / "payload/capture.json").read_text())
    empty_capture = json.loads((args.output / "empty/capture.json").read_text())
    empty_by_id = {int(row["pose_id"]): row for row in empty_capture["accepted"]}
    pairs = [{"pose_id": int(row["pose_id"]), "payload_path": row["path"],
              "empty_path": empty_by_id[int(row["pose_id"])] ["path"]}
             for row in payload_capture["accepted"] if int(row["pose_id"]) in empty_by_id]
    if len(pairs) < 4:
        raise RuntimeError(f"only {len(pairs)} matched clean payload/empty poses")
    cases = [{
        "name": args.target,
        "payload_dir": str(args.output),
        "pairs": pairs,
    }]
    (analysis / "evaluation_cases.json").write_text(json.dumps(cases, indent=2))
    official_env = {**os.environ, "CROSS_OUTPUT": str(analysis),
                    "PYTHONPATH": OFFICIAL_SITE, "OPENBLAS_NUM_THREADS": "1"}
    with (analysis / "evaluate.log").open("w") as log:
        subprocess.run([OFFICIAL, "calibration/evaluate_cross_object.py"], cwd=ROOT,
                       env=official_env, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=1800)


if __name__ == "__main__":
    main()
