# Semantic PiPER articulated interaction: minimal validation

The active entry is `scripts/run_semantic_interaction.py`. R1/Dex1, AnyGrasp,
rigid benchmarks, robot base, IK bounds/settings and object physical parameters
are unchanged. Prior CoACD/penetration-threshold/wrist-sign experiments are
retired; their local/server artifacts are preserved outside this entry.

## Active design

- `prepare_semantic_handle_proxy.py` ranks prepared dataset bars by visual rear
  clearance. PCA of the bar-center samples gives a straight centerline. Visual
  length, cross-section and clearance produce one oriented box and two end
  supports. Only selected handle collision is replaced; visual geometry,
  URDF joints, limits, scale, mass/inertia and all other collisions are retained.
  These are explicit semantic contact approximations, not official collision
  meshes or verified real handle surface geometry.
- `plan_semantic_nominal.py` checks **one central bar-side-pinch**, with an
  equivalent 180-degree finger swap. No position/depth/orientation search.
  It reuses existing PiPER IK/RRT, contactOffset-aware non-contact approach,
  self/scene collision and joint margin **>0.05 rad**. A prior home path may
  be reused only after every edge is checked against the new scene.
- `JawCenteredClosure` slowly closes with a cumulative opening command. First
  unilateral pad load pauses further squeeze at the measured opening. For pad
  centers c1/c2, displacement is along `(c1-c2)/||c1-c2||` times `f1-f2`:
  moving toward the loaded pad reduces its compression. No wrench vector/sign.
  Per-step displacement <=5 um; speed <=2 mm/s; total <=2 mm. Bilateral load
  transitions to explicit **0.5 N target preload**, capped by the unchanged
  official 10 N effort limit. Pad load >2 N stops the low-load experiment.
- `PadCompliantPull` is a bounded small velocity probe: 0.5 mm/s outward,
  pad-load imbalance supplies lateral compliance, orientation follows measured
  EE, and pose lead <=0.5 mm. It is **not** full six-axis F/T admittance.
  Neither object joint type/axis/origin nor a precomputed hinge arc is used.
- Native semantic collider IDs own pad/metal contact. Any actual metal impulse,
  wrong pad target, arm/scene contact, contact loss or low joint margin stops.
  Original visual/STL intersections are post-run diagnostics with **zero effect
  on acceptance**. There is no global raw penetration allowance/threshold.

The controller receives q/FK/EE pose, known robot pad centers, initial handle
pose and scalar left/right pad loads. Virtual pad loads require suitable tactile
sensing on hardware; a coupled single motor alone does not provide both loads.
GT link/joint states are collected separately for post-run slip/motion evaluation.
Online RGB-D slip tracking and a full F/T controller are not implemented here.
Initial handle geometry/pose uses prepared visual geometry and frozen placement;
this experiment is not a fresh perception-pose accuracy validation.

## Reproduction

Server repository: `/data1/home/rangeryx/fr3_real2sim_piper_mobile`.
Geometry interpreter: `/data1/home/rangeryx/.conda/envs/anygrasp/bin/python`.
Isaac interpreter: `/data1/home/rangeryx/isaaclab-arena/.venv/bin/python`.
Unset PYTHONPATH for both and CUDA_VISIBLE_DEVICES for Isaac; use `--gpu 6`.
`source` is the frozen scene directory:
`/data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb`.

```sh
python scripts/prepare_semantic_handle_proxy.py \
  --ranking results/dataset_handle_selection/ranking.json \
  --output results/semantic_interaction/asset

python scripts/run_semantic_interaction.py \
  --source /path/to/frozen/scene --asset-root results/semantic_interaction/asset \
  --output results/semantic_interaction/native --export-only --gpu 6

python scripts/plan_semantic_nominal.py \
  --source /path/to/frozen/scene --asset-root results/semantic_interaction/asset \
  --native-export results/semantic_interaction/native/cooked_initial.json \
  --association results/semantic_interaction/native/association.json \
  --output results/semantic_interaction/nominal

python scripts/run_semantic_interaction.py \
  --source /path/to/frozen/scene --asset-root results/semantic_interaction/asset \
  --plan results/semantic_interaction/nominal/plan.json \
  --policy config/semantic_interaction.json \
  --output results/semantic_interaction/trial --gpu 6 \
  --deadline-shanghai 2026-10-03T05:00:00+08:00

python scripts/audit_semantic_interaction.py \
  --trial results/semantic_interaction/trial \
  --source /path/to/frozen/scene --asset-root results/semantic_interaction/asset
```

The actual run reused an existing fixed-base home path with `--reference-plan`
after native scene edge checks. Output contains real RGB video, every physics
contact record, q/aperture/EE trajectories, separate evaluation GT and evaluation.
Physics advances exactly 1/240 s per control tick; render at 30 Hz never advances
extra physics. Numerical IK preserves float64 perturbations inside float32 state
buffers. Approach velocity feedforward avoids the previously observed servo lag.

## Measured result, 2026-10-02

Asset selected by geometry ranking: **7320, revolute**, original visual `original-19`.
Proxy: **3 primitives**, bar length 220.716 mm, width 23.962 mm, depth 8.767 mm;
estimated minimum rear clearance 14.570 mm. Base unchanged:
`(0.5, -0.55, -0.1 m; yaw=150 deg)`.

Nominal 0-degree finger labeling: NO_IK; symmetric 180-degree labeling: exact IK,
full approach/home path planned, minimum planned margin **0.063431 rad**.
Actual run minimum margin **0.063500 rad**, 6018 control samples.

**FAIL before pad contact**: at aperture **24.518 mm**, native `metal_497` on
`gripper_link1` exerted **0.180289 N** on semantic bar `handle_piece_000`.
Both pad forces were **0 N**. One native physics sample triggered immediate stop.
Thus centering never received a unilateral pad signal, bilateral hold was not
established, **no pull was executed**, and no articulation success is claimed.
The last-pose original visual/STL audit found **no triangle intersection** on
either finger; it did not participate in rejection. Nominal frame jaw direction
matches actual official pad geometry. The old immediate-stop rule triggered on a native non-pad impulse. Subsequent
first-substep diagnosis found positive official surface gaps and cooking
protrusion; this does NOT establish actual official non-pad surface contact.
See `first_nonpad_diagnostic.md` for the corrected interpretation. Do not infer that the centering controller is physically
validated from this failed closure.

No additional grasp search, geometry-resolution experiment, threshold adjustment,
base relocation, friction tuning or stronger clamp was performed after the user's
simplification instruction. An alternative semantic family or a different suitable
handle would need an explicit new minimal validation; bilateral grasp has not yet been tested past the diagnostic pause. No pose
replacement is justified by this first impulse alone.
