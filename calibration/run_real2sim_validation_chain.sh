#!/usr/bin/env bash
set -euo pipefail
cd /data1/home/rangeryx/fr3_moveit_grasp
preflight="$1"
final="$2"
preflight_pid="$(cat "$preflight/launcher.pid")"
while kill -0 "$preflight_pid" 2>/dev/null; do sleep 30; done
test -f "$preflight/acquisition_result.json"
mkdir -p "$final"
/data1/home/rangeryx/isaaclab-arena/.venv/bin/python -u calibration/launch_real2sim.py \
  --target soup --seed 1030 --mode GT --baseline-only \
  --center-q "$preflight/excitation_center.json" --output "$final"
/data1/home/rangeryx/isaaclab-arena/.venv/bin/python -u calibration/launch_real2sim.py \
  --target soup --seed 1030 --mode GT \
  --gt-path results/hand_calibration/gt_path_soup.json \
  --capture-payload --output "$final"
PYTHONPATH=. /data1/home/rangeryx/isaaclab-arena/.venv/bin/python -m real2sim.payload_id \
  "$final/system_id_baseline.npz" "$final/system_id_payload.npz" "$final/inertial_tcp.json"
echo REAL2SIM_ACQUISITION_AND_ID_COMPLETE
