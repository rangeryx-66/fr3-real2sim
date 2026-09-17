#!/usr/bin/env bash
set -euo pipefail

# Restartable official Scalable Real2Sim handoff.  The grasp result and scan are
# explicit inputs; this script never imports or changes the frozen grasp executor.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/data1/home/rangeryx/.conda/envs/robotics/bin/python}"
exec "$PYTHON_BIN" -u "$ROOT/calibration/run_scalable_real2sim_official.py" "$@"

