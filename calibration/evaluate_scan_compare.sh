#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH=.
PY=/data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF/.venv/bin/python
BASE=results/real2sim_soup_scan_compare
GT=results/real2sim_soup_v1_final/eval_gt/mesh.obj
"$PY" -m real2sim.scan_metrics "$BASE/baseline/textured/textured_mesh.obj" "$GT" "$BASE/baseline/dataset" results/real2sim_soup_v1_final/reconstruction/bundlesdf --texture-report "$BASE/baseline/textured/texture_report.json" --output "$BASE/baseline/metrics.json" > "$BASE/baseline/metrics.log"
"$PY" -m real2sim.scan_metrics "$BASE/dual/textured/textured_mesh.obj" "$GT" "$BASE/dual/dataset" "$BASE/dual/bundlesdf" --texture-report "$BASE/dual/textured/texture_report.json" --output "$BASE/dual/metrics.json" > "$BASE/dual/metrics.log"
"$PY" -m real2sim.pass_alignment "$BASE/dual/dataset" "$BASE/dual/bundlesdf" --output "$BASE/dual/pass_alignment.json" > "$BASE/dual/pass_alignment.log"
