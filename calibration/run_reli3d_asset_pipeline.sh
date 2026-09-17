#!/usr/bin/env bash
# Independent scan-station ReLi3D backend.  Existing grasp/Real2Sim entrypoints
# are intentionally not called or modified.
set -euo pipefail
if [[ $# -lt 4 ]]; then
  echo "usage: $0 DATASET TRACKING INERTIAL OUTPUT [GT_MESH]" >&2
  exit 2
fi
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${RELI3D_PYTHON:-/data1/home/rangeryx/.conda/envs/robotics/bin/python3.10}"
exec "$PYTHON" "$PROJECT_ROOT/calibration/run_reli3d_asset_pipeline.py" "$@"
