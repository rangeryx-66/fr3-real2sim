#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH=.
PY=/data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF/.venv/bin/python
ROOT=results/real2sim_soup_scan_compare/dual
rm -rf "$ROOT/textured_aligned"
"$PY" -m real2sim.texture_bake "$ROOT/bundlesdf/textured_mesh.obj" "$ROOT/dataset" "$ROOT/bundlesdf" "$ROOT/textured_aligned" > "$ROOT/texture_aligned.log"
"$PY" -m real2sim.scan_metrics "$ROOT/textured_aligned/textured_mesh.obj" results/real2sim_soup_v1_final/eval_gt/mesh.obj "$ROOT/dataset" "$ROOT/bundlesdf" --texture-report "$ROOT/textured_aligned/texture_report.json" --output "$ROOT/metrics_aligned.json" > "$ROOT/metrics_aligned.log"
"$PY" -m real2sim.pass_alignment "$ROOT/dataset" "$ROOT/bundlesdf" --output "$ROOT/pass_alignment_aligned.json" > "$ROOT/pass_alignment_aligned.log"
