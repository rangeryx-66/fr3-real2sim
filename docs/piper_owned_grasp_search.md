# Fixed-base PiPER: formal handle search after contact ownership repair

## Scope

This experiment searches handle-local grasp position, depth and small rotation.
It does not change the fixed PiPER base, official geometry, IK implementation,
0.05 rad margin, 10 N finger effort limits, friction or passive object joint.
R1/Dex1 and the previous canonical/contact diagnostic remain separate.

## Entrypoints

- `scripts/search_piper_owned_grasps.py`: bounded position/RPY search, exact IK,
  native-cooked and raw non-pad collision checks, open approach and pregrasp route.
- `scripts/run_piper_owned_trials.py`: fresh Isaac scene for each physical trial,
  fixed-base and canonical/config-hash guards, explicit Shanghai cutoff.
- `scripts/piper_owned_grasp_trial.py`: measured closure, full physics-step audit,
  small pull, then separately planned/executed 1/5/10/22 degree stages.
- `scripts/piper_owned_path.py`: cached native geometry worker for full measured
  closure replay and continuous path collision checks at actual finger positions.
- `scripts/summarize_piper_owned_trials.py`: measured funnel and failure report.

The existing `piper_mobile_demo/model.py` remains unchanged. Its exact IK and
joint-limit bounds are used without relaxing orientation or safety margin.

## Search frame

The initial 512-nearest-point PCA selected an across-handle surface direction,
instead of the long handle direction. The corrected neighborhood radius is
twice the official distal pad bounding-box diagonal. PCA is computed around
the selected grasp's pad center. The normal is the original grasp approach
projected perpendicular to the tangent; the lateral axis completes the frame.
This uses no object ID or object-specific offset. The maximum per-axis position
offset is 8 mm and RPY component is 12 degrees. Older-frame results are retained
as diagnostics, not silently relabeled as corrected-frame trials.

`--handle-regions 5 --samples 0` additionally applies the same bounded search
at five longitudinal anchors from the segmented point cloud's 5th–95th
percentiles. Each anchor uses its own local PCA neighborhood and retains the
reference grasp's offset from the observed local surface. These translations
can be centimeters along the existing handle; the 8 mm bound applies to the
local perturbation around each anchor, not to the total slide. Regions enter
physical validation in round-robin order, so a low-score center cannot starve
the upper/lower regions. Neither asset IDs nor fixed global offsets select them.

## Acceptance gates

1. Open-gripper approach collision-free, exact IK and margin >0.05 rad.
2. PhysX performs the complete closure under unchanged effort limit. Measured
   finger positions, not predicted width, define the subsequent closed geometry.
   Predicted width does not penalize or qualify the candidate in ranking either.
3. Both independently identified pad colliders maintain force >=0.2 N during
   the measured closure hold. Any positive non-pad native impulse is forbidden.
4. Replay every recorded closure physics step against exported native convex
   solids and unchanged official non-pad raw surfaces. Check scene and self
   collision and actual joint margins. Owner IDs must agree with native export.
5. Plan and physically execute 0.25/0.5/1/2 mm tangential exploration; require
   measured TCP displacement >=1 mm, bilateral contact, zero metal impulse,
   <=1 mm relative translation slip and the same complete geometry replay.
6. After a legal grasp, plan each requested door-angle stage from measured robot
   state. A failed later-angle preflight is recorded but does not suppress a
   smaller stage whose entire path passes. Hold each executed stage for two
   measured physics seconds. Door motion is passive contact-driven: no direct
   object-joint command or ideal attachment is used.

The 22-degree preflight alone is never reported as physical success. Every
stage reports measured angle, minimum pad forces, metal contact samples, slip,
minimum margin and failure. The angle tracking tolerance is explicitly 0.1
degree; the measured angle is always retained.

## Contact versus intersection

The native solver can generate a positive contact impulse with a small positive
surface separation because of the unchanged contact offset. A positive-distance
FCL query therefore does not authorize a metal contact. Acceptance combines
native surface ownership/force with cooked and raw geometry audits. In the
baseline re-run, four first-contact metal pieces had native separation about
14.56 micrometers and cached cooked FCL distance about 14.61 micrometers; the
maximum difference was 0.0443 micrometers. No extra tolerance was introduced.

## Reproduce on the server

Run from `/data1/home/rangeryx/fr3_real2sim_piper_mobile`. Use the existing
`anygrasp` environment for geometry/IK and Isaac Arena environment for simulation.
Do not pass `CUDA_VISIBLE_DEVICES`; choose the physical GPU with `--gpu`.

```bash
env -u PYTHONPATH /data1/home/rangeryx/.conda/envs/anygrasp/bin/python scripts/search_piper_owned_grasps.py \
  --source /data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --reference-plan results/piper_fixed_center_search/report.json \
  --reference-trial results/piper_owned_formal_hybrid_clock/report.json \
  --native-export results/piper_owned_formal_hybrid_clock/cooked_initial.json \
  --association results/piper_owned_formal_hybrid_clock/target_collider_association.json \
  --output results/piper_owned_local_search_frame_v2 \
  --variant-prefix frame2 --samples 160 --max-planned-trials 96

env -u PYTHONPATH /data1/home/rangeryx/.conda/envs/anygrasp/bin/python scripts/run_piper_owned_trials.py \
  --search results/piper_owned_local_search_frame_v2/report.json \
  --source /data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --fixed-base-report results/piper_owned_formal_hybrid_clock/report.json \
  --canonical-validation results/piper_owned_canonical_hybrid_recorded/full_validation.json \
  --output results/piper_owned_frame2_physics --gpu 7 --max-trials 96 \
  --deadline-shanghai 2026-10-03T05:00:00+08:00
```

Each run writes video, complete physics-step poses/contact records, actual cooked
exports, planning requests/results, closure/pull replay audits and `report.json`.
An early forbidden-contact stop has a measured *partial* closure aperture. It
must not be interpreted as a completed closure or a pad-only grasp.

## Execution integrity

Cooked-shape exports pause the physics timeline. Exporting may call Kit updates;
these must not advance an unrecorded closure or keep moving after a guard stops
the trial. Prediction uses the actual tensor moving-link pose and measured door
angle captured with the paused export, rather than an asynchronously updated USD
pose. Closure-hold also checks palm and scene contacts with a zero-force rule.

The native ownership callback caches decoded collider paths and the two finger
body prefixes. Recorded contact replay verifies identical ownership and force
statistics, including unknown collider and wrong-target rejection. This removes
repeated rebuilding of a 1,010-collider map for unrelated contact headers; it
changes neither geometry nor contact acceptance. The microbenchmark is a callback
timing result, not a grasp success or end-to-end simulation speed claim.

URDF/SRDF and the simulation importer proxy are published by atomic replacement.
This prevents a concurrent episode from reading an empty, partly written model.
A concurrent stress check performed 20 publications and 2,000 XML read pairs
without parse errors or changing the generated URDF bytes. Parallel episodes use
separate output directories and disjoint candidate queues; interrupted startup
runs are archived and rerun, not counted as grasp failures.

Final bounded-search measurements are recorded in
`piper_formal_owned_search_results.md` and the adjacent JSON files. The online
contact guard also checks each physics substep, avoiding cancellation inside a
control-window aggregate. Positive pad contact during pregrasp/approach rejects
the episode. Slip is reported as unmeasured until a legal relative grasp frame
has actually been established.
