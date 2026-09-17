#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 3 ]]; then
  echo 'Usage: scripts/run_mv_sam3d.sh SCAN_DIR OBJECT_NAME OUTPUT_DIR [GPU_ID] [VIEW_COUNT]' >&2
  exit 2
fi
SCAN_DIR=$(realpath "$1")
OBJECT_NAME=$2
OUTPUT_DIR=$(realpath -m "$3")
GPU_ID=${4:-0}
VIEW_COUNT=${5:-8}
[[ "$VIEW_COUNT" =~ ^(2|4|8)$ ]] || { echo 'VIEW_COUNT must be 2, 4 or 8' >&2; exit 2; }
: "${MV_SAM3D_ROOT:?set MV_SAM3D_ROOT}"
: "${MV_SAM3D_PYTHON:?set MV_SAM3D_PYTHON}"
mkdir -p "$OUTPUT_DIR"
"$MV_SAM3D_PYTHON" experiments/prepare_mv_sam3d_inputs.py \
  --scan-dir "$SCAN_DIR" --output "$OUTPUT_DIR/inputs" \
  --object "$OBJECT_NAME" --counts "$VIEW_COUNT"
VIEW_DIR="$OUTPUT_DIR/inputs/views_$(printf '%02d' "$VIEW_COUNT")"
NAMES=$(seq -s, 0 "$((VIEW_COUNT-1))")
(
  cd "$MV_SAM3D_ROOT"
  CUDA_VISIBLE_DEVICES="$GPU_ID" "$MV_SAM3D_PYTHON" -u run_inference_weighted.py \
    --input_path "$VIEW_DIR" --mask_prompt "$OBJECT_NAME" \
    --image_names "$NAMES" --decode_formats gaussian,mesh \
    2>&1 | tee "$OUTPUT_DIR/inference.log"
)
echo "Selection and masks: $VIEW_DIR"
echo "Upstream output path is printed in $OUTPUT_DIR/inference.log (result.glb, result.ply)."
