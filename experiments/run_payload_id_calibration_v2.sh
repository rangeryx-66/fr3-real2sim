#!/usr/bin/env bash
set -euo pipefail

# Frozen four-object v2 calibration.  The center-q files are the already
# recorded grasp handoff poses; they are motion setup only.  Estimation reads
# measured torque/kinematics NPZ files and never reads GT material.
ROOT=$(cd "$(dirname "$0")/.." && pwd)
PY=${PYTHON_BIN:-/data1/home/rangeryx/isaaclab-arena/.venv/bin/python}
OUT=${OUT_ROOT:-$ROOT/results/payload_id_calibration_v2}
mkdir -p "$OUT"

run_one() {
  local target=$1 seed=$2
  local center="$ROOT/results/payload_id_calibration_v1/$target/excitation_center.json"
  local gt="$ROOT/results/hand_calibration/gt_path_$target.json"
  local run="$OUT/$target"
  mkdir -p "$run"
  if [[ ! -f "$run/payload_id_v2_protocol.json" ]]; then
    "$PY" -u "$ROOT/calibration/launch_real2sim.py" --target "$target" --seed "$seed" \
      --output "$run" --center-q "$center" --baseline-only --payload-v2 --skip-scan
  fi
  if [[ ! -f "$run/payload_v2_payload.json" ]]; then
    "$PY" -u "$ROOT/calibration/launch_real2sim.py" --target "$target" --seed "$seed" \
      --output "$run" --center-q "$center" --gt-path "$gt" --capture-payload --payload-v2 --skip-scan
  fi
  # Evaluation is deliberately after both captures.  Pass --mesh only when a
  # reconstructed visual mesh is supplied by the unchanged scan pipeline.
  local mesh_args=()
  if [[ -n "${PHYSICAL_MESH:-}" ]]; then mesh_args+=(--mesh "$PHYSICAL_MESH"); fi
  "$PY" -u "$ROOT/calibration/evaluate_payload_id_v2.py" --run "$run" \
    --gt "$ROOT/results/hand_calibration/preflight_$target/${target}_force_F30_mu0.7_r0.json" \
    "${mesh_args[@]}" > "$run/payload_id_v2_report.stdout.json"
}

run_one soup 1030
run_one banana 1020
run_one bowl 1015
run_one mug 1035
echo "PayloadID v2 calibration complete: $OUT"
