#!/usr/bin/env bash
# Reproducible VGGT-assisted RGB-D -> Isaac asset pipeline.
# GT is accepted only by the optional final-evaluation step; it is never passed
# to VGGT, metric fusion, TSDF, texture baking, collision decomposition, or USD.
set -euo pipefail

if [[ $# -lt 4 ]]; then
  echo "usage: $0 DATASET TRACKING INERTIAL_JSON OUTPUT_DIR [GT_MESH]" >&2
  exit 2
fi

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET="$1"; TRACKING="$2"; INERTIAL="$3"; OUTPUT="$4"; GT_MESH="${5:-}"
SIMRECON_PY="${SIMRECON_PY:-/data1/home/rangeryx/.conda/envs/simrecon/bin/python}"
ROBOTICS_PY="${ROBOTICS_PY:-/data1/home/rangeryx/.conda/envs/robotics/bin/python3.10}"
VGGT_ROOT="${VGGT_ROOT:-/data1/home/rangeryx/vggt}"
VGGT_CHECKPOINT="${VGGT_CHECKPOINT:-${VGGT_ROOT}/checkpoint/model.pt}"
ISAAC_PY="${ISAAC_PY:-}"
GPU="${GPU:-0}"

cd "$PROJECT_ROOT"
mkdir -p "$OUTPUT"
time_stage() {
  local name="$1"; shift
  /usr/bin/time -f "WALL_SECONDS=%e" -o "$OUTPUT/${name}.time" "$@" >"$OUTPUT/${name}.log" 2>&1
}

time_stage vggt_inference env PYTHONPATH="$VGGT_ROOT:$PROJECT_ROOT" "$SIMRECON_PY" -m real2sim.vggt_inference \
  "$DATASET" "$OUTPUT/vggt" --checkpoint "$VGGT_CHECKPOINT" --chunk-size "${VGGT_CHUNK_SIZE:-12}" --device "${VGGT_DEVICE:-cuda}"
time_stage metric_fusion "$SIMRECON_PY" -m real2sim.vggt_metric_fusion \
  "$DATASET" "$OUTPUT/vggt" "$OUTPUT/vggt_metric_dataset"
time_stage tsdf "$SIMRECON_PY" -m real2sim.tsdf_fusion \
  "$OUTPUT/vggt_metric_dataset" "$TRACKING" "$OUTPUT/tsdf"
time_stage texture "$ROBOTICS_PY" -m real2sim.texture_bake \
  "$OUTPUT/tsdf/mesh.obj" "$OUTPUT/vggt_metric_dataset" "$TRACKING" "$OUTPUT/textured"
time_stage coacd "$ROBOTICS_PY" -m real2sim.collision_mesh \
  "$OUTPUT/tsdf/mesh.obj" "$OUTPUT/collision"
time_stage asset_builder "$ROBOTICS_PY" -m real2sim.asset_builder \
  --name "${ASSET_NAME:-object_vggt_scan}" \
  --visual "$OUTPUT/textured/textured_mesh.obj" \
  --texture "$OUTPUT/textured/material_0.png" \
  --collision "$OUTPUT/collision/collision_manifest.json" \
  --inertial "$INERTIAL" --output "$OUTPUT/asset"
time_stage coverage "$SIMRECON_PY" -m real2sim.coverage_audit \
  "$OUTPUT/tsdf/mesh.obj" "$OUTPUT/vggt_metric_dataset" "$TRACKING" \
  --output "$OUTPUT/coverage.json"

if [[ -n "$GT_MESH" ]]; then
  time_stage metrics "$ROBOTICS_PY" -m real2sim.scan_metrics \
    "$OUTPUT/tsdf/mesh.obj" "$GT_MESH" "$OUTPUT/vggt_metric_dataset" "$TRACKING" \
    --texture-report "$OUTPUT/textured/texture_report.json" --output "$OUTPUT/metrics.json"
fi

if [[ -n "$ISAAC_PY" ]]; then
  # Isaac's first-run EULA prompt is answered explicitly for a headless run.
  printf 'Yes\n' | /usr/bin/time -f "WALL_SECONDS=%e" -o "$OUTPUT/usd_validation.time" \
    "$ISAAC_PY" calibration/validate_real2sim_usd.py \
    --asset "$OUTPUT/asset/${ASSET_NAME:-object_vggt_scan}.usda" \
    --output "$OUTPUT/asset/reload_render.png" --gpu "$GPU" \
    >"$OUTPUT/usd_validation.log" 2>&1
fi

echo "pipeline complete: $OUTPUT"
