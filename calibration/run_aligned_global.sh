#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export CUDA_VISIBLE_DEVICES=4 PYOPENGL_PLATFORM=egl PYTHONPATH=.
export BUNDLESDF_PYTHON=/data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF/.venv/bin/python
OUT=results/real2sim_soup_scan_compare/dual
/usr/bin/time -v "$BUNDLESDF_PYTHON" -m real2sim.bundlesdf_adapter \
  "$OUT/dataset" "$OUT/bundlesdf" --optimized \
  > "$OUT/aligned_global.log" 2>&1
