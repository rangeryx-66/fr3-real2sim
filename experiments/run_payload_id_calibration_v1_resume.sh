#!/usr/bin/env bash
# Resume the frozen PayloadID calibration after an object-level safety abort.
# A failed payload capture is recorded and does not prevent the remaining
# objects from running; no grasp, controller, or estimator parameters change.
set -u
ROOT=/data1/home/rangeryx/fr3_moveit_grasp
PY=/data1/home/rangeryx/isaaclab-arena/.venv/bin/python
LAUNCH="$PY -u calibration/launch_real2sim.py"
OUT="$ROOT/results/payload_id_calibration_v1"
mkdir -p "$OUT"

run_one() {
  local obj="$1" seed="$2" gt="$3"
  local d="$OUT/$obj"
  mkdir -p "$d"
  echo "[PAYLOAD_ID_RESUME] $obj seed=$seed"

  if [[ ! -s "$d/system_id_payload.npz" || ! -s "$d/excitation_center.json" ]]; then
    set +e
    $LAUNCH --target "$obj" --seed "$seed" --mode GT --gt-path "$gt" --skip-scan --capture-payload --output "$d"
    local rc=$?
    set -e
    if [[ $rc -ne 0 ]]; then
      printf '{"object":"%s","stage":"payload","returncode":%d}\n' "$obj" "$rc" > "$d/payload_id_failure.json"
      echo "[PAYLOAD_ID_PAYLOAD_FAILED] $obj rc=$rc"
    fi
  fi

  if [[ ! -s "$d/system_id_baseline.npz" && -s "$d/excitation_center.json" ]]; then
    set +e
    $LAUNCH --target "$obj" --seed "$seed" --mode GT --baseline-only --center-q "$d/excitation_center.json" --output "$d"
    local rc=$?
    set -e
    if [[ $rc -ne 0 ]]; then
      printf '{"object":"%s","stage":"baseline","returncode":%d}\n' "$obj" "$rc" > "$d/baseline_failure.json"
      echo "[PAYLOAD_ID_BASELINE_FAILED] $obj rc=$rc"
    fi
  fi

  if [[ -s "$d/system_id_baseline.npz" && -s "$d/system_id_payload.npz" ]]; then
    set +e
    PYTHONPATH="$ROOT" "$PY" -m real2sim.payload_id "$d/system_id_baseline.npz" "$d/system_id_payload.npz" "$d/inertial_tcp.json"
    local rc=$?
    set -e
    if [[ $rc -ne 0 ]]; then
      printf '{"object":"%s","stage":"estimator","returncode":%d}\n' "$obj" "$rc" > "$d/payload_id_estimator_failure.json"
      echo "[PAYLOAD_ID_ESTIMATOR_FAILED] $obj rc=$rc"
    else
      echo "[PAYLOAD_ID_DONE] $obj"
    fi
  else
    echo "[PAYLOAD_ID_UNOBSERVABLE] $obj missing paired records"
  fi
}

cd "$ROOT"
set -e
run_one bowl 1015 results/hand_calibration/gt_path_bowl.json
run_one mug 1035 results/hand_calibration/gt_path_mug.json
echo PAYLOAD_ID_CALIBRATION_RESUME_COMPLETE
