#!/usr/bin/env bash
set -euo pipefail

# Post-process one completed physical held-object scan.  This script only
# consumes recorded RGB/mask/pose files; it does not touch the frozen grasp
# executor.  It is intentionally restartable and keeps each backend's logs
# separate.

PROJECT=/data1/home/rangeryx/fr3_moveit_grasp
HELD_RUN="${HELD_RUN:-real2sim_soup_held_regrasp_v6}"
SCAN="$PROJECT/results/$HELD_RUN/scan"
MVROOT="$PROJECT/results/mv_sam3d_soup_v1"
HELD_INPUT="$MVROOT/inputs_held_${HELD_RUN}"
HELD_OFFICIAL="$PROJECT/results/real2sim_held_official_soup_1800_${HELD_RUN}/official_pipeline"

cd "$PROJECT"
while [[ ! -f "$SCAN/scan_manifest.json" ]]; do
  sleep 60
done

if [[ ! -f "$MVROOT/held_input_${HELD_RUN}.done" ]]; then
  /data1/home/rangeryx/.conda/envs/robotics/bin/python -u \
    experiments/prepare_mv_sam3d_inputs.py \
    --scan-dir "$SCAN" --output "$HELD_INPUT" --object soup --counts 2,4,8 \
    > "$MVROOT/held_prepare_${HELD_RUN}.log" 2>&1
  printf 'MV_SAM3D_HELD_INPUT_READY %s\n' "$HELD_RUN" > "$MVROOT/held_input_${HELD_RUN}.done"
fi

# Official Scalable Real2Sim BundleSDF branch (tracking, global refinement,
# texture, payload adapter, CoACD/SDFormat, fresh Isaac reload).
if [[ ! -f "$HELD_OFFICIAL/scalable_real2sim_pipeline_manifest.json" ]]; then
  mkdir -p "$HELD_OFFICIAL"
  env PYTHONUNBUFFERED=1 PYOPENGL_PLATFORM=egl PYTHONPATH=. \
    /data1/home/rangeryx/.conda/envs/robotics/bin/python -u \
    calibration/run_scalable_real2sim_official.py \
    --grasp-result "$PROJECT/results/$HELD_RUN/soup_seed1030_GT.json" \
    --scan-dir "$SCAN" \
    --payload-run "$PROJECT/results/real2sim_soup_v1_final" \
    --output "$HELD_OFFICIAL" \
    --name soup_scalable_real2sim_held_1800 \
    --gt-mesh "$PROJECT/results/real2sim_soup_v1_final/eval_gt/mesh.obj" \
    --gt-physics "$PROJECT/results/real2sim_soup_v1_final/eval_gt/physics.json" \
    --bundle-python /data1/home/rangeryx/scalable-real2sim/scalable_real2sim/BundleSDF/.venv/bin/python \
    --isaac-python /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
    --gpu 3 --force-reconstruct \
    > "$PROJECT/results/real2sim_held_official_soup_1800_${HELD_RUN}.log" 2>&1
fi

# Official MV-SAM3D single-object path.  Use exactly the same model defaults
# and 2/4/8 view protocol as the static experiment; outputs are timestamped by
# the upstream script, so prior runs remain available for audit.
cd /data1/home/rangeryx/MV-SAM3D
source /data1/home/rangeryx/proxyon.sh || true
for n in 2 4 8; do
  INPUT="$HELD_INPUT/views_$(printf '%02d' "$n")"
  LOG="$MVROOT/mv_$(printf '%02d' "$n")_held_${HELD_RUN}.log"
  if [[ ! -f "$LOG.done" ]]; then
    names=$(seq -s, 0 $((n-1)))
    CONDA_PREFIX=/data1/home/rangeryx/.conda/envs/robotics \
    CUDA_VISIBLE_DEVICES=5 \
    /data1/home/rangeryx/.conda/envs/robotics/bin/python -u run_inference_weighted.py \
      --input_path "$INPUT" --mask_prompt soup --image_names "$names" \
      --decode_formats gaussian,mesh \
      > "$LOG" 2>&1
    printf 'DONE\n' > "$LOG.done"
  fi
done
printf 'HELD_POSTPROCESS_DONE %s\n' "$HELD_RUN" > "$MVROOT/held_postprocess_${HELD_RUN}.done"
