#!/usr/bin/env bash
# ReconViaGen v0.5 -> metric RGB-D constraint -> real texture -> CoACD -> USD.
# BundleSDF/VGGT remain upstream tracking/measurement artifacts and fallback
# implementations; they are not the final visual reconstruction backend here.
set -euo pipefail

if [[ $# -lt 4 ]]; then
  echo "usage: $0 DATASET TRACKING INERTIAL_JSON OUTPUT_DIR [GT_MESH]" >&2
  exit 2
fi

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET="$1"; TRACKING="$2"; INERTIAL="$3"; OUTPUT="$4"; GT_MESH="${5:-}"
SIMRECON_PY="${SIMRECON_PY:-/data1/home/rangeryx/.conda/envs/simrecon/bin/python}"
ROBOTICS_PY="${ROBOTICS_PY:-/data1/home/rangeryx/.conda/envs/robotics/bin/python3.10}"
RECONVIAGEN_PY="${RECONVIAGEN_PY:-$ROBOTICS_PY}"
RECON_ROOT="${RECON_ROOT:-/data1/home/rangeryx/ReconViaGen}"
TRELLIS2_ROOT="${TRELLIS2_ROOT:-$RECON_ROOT/wheels/TRELLIS.2}"
RECONVIAGEN_EXTRA_PATH="${RECONVIAGEN_EXTRA_PATH:-/tmp/recon_extra}"
HF_HOME="${HF_HOME:-/data1/home/rangeryx/.cache/huggingface}"
VGGT_SNAPSHOT="${VGGT_SNAPSHOT:-$HF_HOME/hub/models--Stable-X--trellis-vggt-v0-2/snapshots/647659a5ad5fbf67e22793e7b5e2cee4b30c5d13}"
TRELLIS2_SNAPSHOT="${TRELLIS2_SNAPSHOT:-$HF_HOME/hub/models--microsoft--TRELLIS.2-4B/snapshots/af44b45f2e35a493886929c6d786e563ec68364d}"
ISAAC_PY="${ISAAC_PY:-}"
GPU="${GPU:-0}"
ASSET_NAME="${ASSET_NAME:-object_reconviagen_v05}"

cd "$PROJECT_ROOT"
mkdir -p "$OUTPUT"
time_stage() {
  local name="$1"; shift
  /usr/bin/time -f "WALL_SECONDS=%e" -o "$OUTPUT/${name}.time" "$@" >"$OUTPUT/${name}.log" 2>&1
}

if [[ "${RECONVIAGEN_REUSE_GENERATED:-0}" != "1" ]]; then
  time_stage view_selection env PYTHONPATH="$PROJECT_ROOT" "$SIMRECON_PY" -m real2sim.reconviagen_view_selection \
    "$DATASET" "$TRACKING" "$OUTPUT/views" --num-views "${RECONVIAGEN_NUM_VIEWS:-12}" \
    --min-adjacent-overlap "${RECONVIAGEN_MIN_OVERLAP:-0.18}" \
    --min-direction-span-deg "${RECONVIAGEN_MIN_DIRECTION_SPAN_DEG:-90}"
else
  test -f "$OUTPUT/views/view_selection.json"
  echo "reusing existing view selection: $OUTPUT/views/view_selection.json"
fi

# The official ReconViaGen v0.5 code imports a DINOv3-capable Transformers
# package and DreamSim/PEFT.  The existing robotics environment supplies the
# CUDA/compiled stack; RECONVIAGEN_EXTRA_PATH supplies only the missing Python
# packages (no grasp code is imported).
if [[ "${RECONVIAGEN_REUSE_GENERATED:-0}" != "1" ]]; then
time_stage reconviagen_inference env CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-3}" \
  HF_HOME="$HF_HOME" HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
  PYTHONPATH="$RECONVIAGEN_EXTRA_PATH:$RECON_ROOT:$TRELLIS2_ROOT:$RECON_ROOT/wheels/vggt:/data1/home/rangeryx/vggt:$PROJECT_ROOT" \
  "$RECONVIAGEN_PY" -u -m real2sim.reconviagen_inference \
  "$OUTPUT/views/view_selection.json" "$OUTPUT/generated" \
  --recon-root "$RECON_ROOT" --trellis2-root "$TRELLIS2_ROOT" \
  --pretrained "$VGGT_SNAPSHOT" --trellis2-pretrained "$TRELLIS2_SNAPSHOT" \
  --pipeline-type "${RECONVIAGEN_PIPELINE_TYPE:-1024}" \
  --strategy "${RECONVIAGEN_STRATEGY:-adaptive_guidance_weight}" \
  --ss-steps "${RECONVIAGEN_SS_STEPS:-30}" --shape-steps "${RECONVIAGEN_SHAPE_STEPS:-12}" \
  --tex-steps "${RECONVIAGEN_TEX_STEPS:-12}" --seed "${RECONVIAGEN_SEED:-20260914}" \
  --decimation-target "${RECONVIAGEN_DECIMATION_TARGET:-500000}" \
  --generated-texture-size "${RECONVIAGEN_GENERATED_TEXTURE_SIZE:-2048}"
else
  test -f "$OUTPUT/generated/generated_complete_mesh.obj"
  echo "reusing existing ReconViaGen output: $OUTPUT/generated/generated_complete_mesh.obj"
fi

METRIC_ARGS=("$OUTPUT/generated/generated_complete_mesh.obj" "$DATASET" "$TRACKING" "$OUTPUT/metric" \
  --support-mm "${RECONVIAGEN_SUPPORT_MM:-6}" --conflict-mm "${RECONVIAGEN_CONFLICT_MM:-10}" \
  --blend "${RECONVIAGEN_MEASURED_BLEND:-0.85}")
if [[ -n "${OBSERVED_TSDF_MESH:-}" ]]; then
  METRIC_ARGS+=(--observed-mesh "$OBSERVED_TSDF_MESH")
fi
time_stage metric_constraint "$ROBOTICS_PY" -m real2sim.reconviagen_metric_constraint "${METRIC_ARGS[@]}"

# Keep the measured RGB bake as the default product.  ReconViaGen's official
# tex_slat output is available as an explicit comparison mode; it must never
# silently replace the measured texture in an existing run.
TEXTURE_BACKEND="real_rgb_bake"
TEXTURED_MESH="$OUTPUT/textured/textured_mesh.obj"
TEXTURE_IMAGE="$OUTPUT/textured/material_0.png"
TEXTURE_REPORT="$OUTPUT/textured/texture_report.json"
if [[ "${RECONVIAGEN_TEXTURE_MODE:-real_rgb}" == "native" ]]; then
  time_stage native_texture "$ROBOTICS_PY" -m real2sim.reconviagen_texture_variant \
    "$OUTPUT/generated/generated_complete_mesh.glb" \
    "$OUTPUT/metric/metric_constraint_report.json" \
    "$OUTPUT/native_texture_compare" \
    --metric-mesh "$OUTPUT/metric/metric_constrained_mesh.obj"
  TEXTURE_BACKEND="reconviagen_v05_native_tex_slat"
  TEXTURED_MESH="$OUTPUT/native_texture_compare/metric_constrained_official_texture.obj"
  TEXTURE_IMAGE="$OUTPUT/native_texture_compare/reconviagen_native_basecolor.png"
  TEXTURE_REPORT="$OUTPUT/native_texture_compare/native_texture_report.json"
else
  time_stage texture "$ROBOTICS_PY" -m real2sim.texture_bake \
    "$OUTPUT/metric/metric_constrained_mesh.obj" "$DATASET" "$TRACKING" "$OUTPUT/textured"
fi

if [[ "${RECONVIAGEN_REUSE_COLLISION:-0}" != "1" ]]; then
  time_stage coacd "$ROBOTICS_PY" -m real2sim.collision_mesh \
    "$OUTPUT/metric/metric_constrained_mesh.obj" "$OUTPUT/collision"
else
  test -f "$OUTPUT/collision/collision_manifest.json"
  echo "reusing existing CoACD collision manifest: $OUTPUT/collision/collision_manifest.json"
fi

time_stage asset_builder "$ROBOTICS_PY" -m real2sim.asset_builder \
  --name "$ASSET_NAME" --visual "$TEXTURED_MESH" \
  --texture "$TEXTURE_IMAGE" \
  --collision "$OUTPUT/collision/collision_manifest.json" --inertial "$INERTIAL" \
  --output "$OUTPUT/asset"

time_stage coverage env PYTHONPATH="$PROJECT_ROOT" "$SIMRECON_PY" -m real2sim.coverage_audit \
  "$OUTPUT/metric/metric_constrained_mesh.obj" "$DATASET" "$TRACKING" \
  --output "$OUTPUT/coverage.json"

if [[ -n "$GT_MESH" ]]; then
  time_stage metrics "$ROBOTICS_PY" -m real2sim.scan_metrics \
    "$OUTPUT/metric/metric_constrained_mesh.obj" "$GT_MESH" "$DATASET" "$TRACKING" \
    --texture-report "$TEXTURE_REPORT" --output "$OUTPUT/metrics.json"
fi

if [[ -n "$ISAAC_PY" ]]; then
  printf 'Yes\n' | /usr/bin/time -f "WALL_SECONDS=%e" -o "$OUTPUT/usd_validation.time" \
    "$ISAAC_PY" calibration/validate_real2sim_usd.py \
    --asset "$OUTPUT/asset/${ASSET_NAME}.usda" --output "$OUTPUT/asset/reload_render.png" --gpu "$GPU" \
    >"$OUTPUT/usd_validation.log" 2>&1
fi

cat >"$OUTPUT/pipeline_manifest.json" <<EOF
{
  "schema": "fr3_reconviagen_v05_asset_pipeline/v1",
  "backend": "ReconViaGen v0.5 official hybrid",
  "gt_used_for_generation_or_alignment": false,
  "grasp_executor_modified": false,
  "dataset": "$DATASET",
  "tracking": "$TRACKING",
  "inertial": "$INERTIAL",
  "output": "$OUTPUT",
  "asset_name": "$ASSET_NAME",
  "texture_backend": "$TEXTURE_BACKEND",
  "products": {
    "raw_generated_mesh": "$OUTPUT/generated/generated_complete_mesh.obj",
    "raw_generated_glb": "$OUTPUT/generated/generated_complete_mesh.glb",
    "metric_constrained_mesh": "$OUTPUT/metric/metric_constrained_mesh.obj",
    "textured_mesh": "$TEXTURED_MESH",
    "texture": "$TEXTURE_IMAGE",
    "collision_manifest": "$OUTPUT/collision/collision_manifest.json",
    "isaac_usd": "$OUTPUT/asset/${ASSET_NAME}.usda"
  },
  "fallback_backends": ["BundleSDF", "VGGT"],
  "model_snapshots": {"reconviagen": "$VGGT_SNAPSHOT", "trellis2": "$TRELLIS2_SNAPSHOT"}
}
EOF
echo "pipeline complete: $OUTPUT"
