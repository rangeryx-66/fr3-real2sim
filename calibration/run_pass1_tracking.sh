#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=results/real2sim_soup_scan_compare/dual
rm -rf "$OUT/pass1_tracking"
export CUDA_VISIBLE_DEVICES=4 PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export LD_PRELOAD=/data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF/native_env/lib/libjpeg.so.8
export LD_LIBRARY_PATH=/data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF/native_env/lib:/usr/local/cuda-12.5/lib64
PY=/data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF/.venv/bin/python
"$PY" real2sim/bundlesdf_optimized_entry.py \
  --bundle-root /data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF \
  --video-dir "$OUT/pass1_dataset" --out-folder "$OUT/pass1_tracking" \
  --tracking-only > "$OUT/pass1_tracking.log" 2>&1
