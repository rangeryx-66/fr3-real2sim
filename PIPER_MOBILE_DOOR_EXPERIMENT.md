# PiPER / official parallel gripper / stopped mobile-base experiment

Branch: `codex/piper-mobile-door`, based on `r1-7a` commit `cc8b930`.
Date: 2026-10-01. Experiments finished before the 2026-10-02 05:00 Shanghai cutoff.
R1 source, Dex1 geometry, scanning, perception models and existing benchmarks are unchanged.
Compact measured results: [`docs/piper_mobile_door_summary.json`](docs/piper_mobile_door_summary.json).

## Current conclusion

**Neither PiPER nor the archived R1/Dex1 contact test has established a certified stable door-opening grasp.** PiPER has substantially comfortable arm paths in this scene, but its actual closure still fails a strict surface audit. Mobile reposition cannot be credited with recovery: it has not been physically tested, and no evidence currently shows that relocating the base solves the finger/handle contact discrepancy.

The same door, tested **without a robot**, moves to 90 degrees under torque. Fixture, static cabinet and table contact peaks are zero throughout the sweep. A 0.0025 N m input opens 0.54 degrees in 1 second, corresponding to 0.00606 N at the 0.41264 m observed handle radius. This is an upper bound for that motion threshold, **not a calibrated static friction torque**. Zero torque causes no opening. Thus the present scene shows no hinge seizure or residual fixture interference. This does not certify the realism of the asset's collision cooking, handle surfaces or inertia approximation.

## Same inputs and comparison

Prepared PhysX-Mobility asset `7320_v1`, placement `(0.50,-0.05 m; yaw=-90 deg; fixture height=0.04 m)`.
Fixed arm mount `(0.50,-0.55,-0.10 m; yaw=150 deg)`, inherited from the existing R1 experiment.
Same frozen RGB-D, actual SAM3 mask and GraspGen-X output; input hashes are in the summary.
No new perception inference, grasp-model training or object-specific offset rule was introduced.

| Metric | R1 + Dex1 | PiPER fixed base | PiPER + stopped mobile base |
|---|---|---|---|
| Evidence | Archived same-scene closure diagnostic | New actual simulations and strict replay | Implementation; physical recovery not tested |
| Certified legal real grasp | 0/5 archived closures | Not obtained under final strict gate | Unknown |
| Arm-only planned opening | Not rerun for this comparison | 22 deg tested; not a global maximum | Not credited |
| Certified physical opening | Blocked before opening | Final strict episode blocks at closure | Not executed |
| Historical observed opening | Not used as a current valid baseline | 16.37 deg, then contact loss; **retrospective strict audit rejects closure** | Not executed |
| Minimum measured arm margin | 0.0967 rad across archived closures | 0.3615 rad in strict episode | Unknown |
| Main obstruction | Dex1 non-pad metal contact | Finger/handle geometric overlap and disagreement with PhysX contact witnesses | Contact obstruction is not a base reachability result |

These episodes are diagnostics with evolving validators, **not a statistically controlled success-rate benchmark**. The R1 comparison uses its archived `r1_contact_consistency` results and has not been rerun or retuned. The historical 0.28-degree R1 opening from an older contact configuration is not substituted for this baseline.

Open-gripper candidate funnels (none establishes actual closure):

| Search | Variants | Full 0–22 deg arm paths | Best full-path min margin |
|---|---:|---:|---:|
| Initial bounded manifold | 162 | 4 | 0.3565 rad |
| Observed-handle PCA slide/depth refinement | 84 | 51 | 0.5297 rad |
| Jaw-center search about initial full paths | 20 | 12 | 0.3615 rad |
| Jaw-center search about refined paths | 20 | 20 | 0.5302 rad |

## Model and contact findings

- Official [AgileX URDF repository](https://github.com/agilexrobotics/agx_arm_urdf), pinned `f6642ce0d7872c686f29c99e9e10cd23d1d49313`, includes six arm joints and the matching 100 mm parallel gripper. Official dimensions, meshes, masses, limits and the 10 N finger effort limits are retained. License and original description are vendored.
- The URDF's virtual opening joint is folded into the two actual finger joints with the same mechanical coupling `q2=-q1`. Independent asymmetric jaws were an invalid early diagnostic configuration and are excluded from final evidence.
- TCP is a **derived convention**, at `gripper_base z=0.138 m`, the distal finger plane. The permitted contact footprint is extracted from the original STL's flat distal inner faces. This contact region is not a separately certified manufacturer rubber-pad definition.
- Free opening/closing/reopening measures approximately `100 mm -> 0.076 mm -> 100 mm`. A drive initialization bug was corrected: re-authoring an unchanged `maxForce` rebuilt drives whose gains had only been set at runtime. Existing PiPER gains are now authored and force limits read-checked. No force or friction increase was used.
- Actual jaw coordinates replace point-cloud aperture estimates. A successful PhysX bilateral closure in the retained pose measures **23.869 mm**, with contact forces approximately **6.59 / 13.63 N**. Contact forces include coupling/arm reactions and are not motor-effort readbacks.
- URDF/Isaac TCP mismatch is 0.17 micrometers and 2.49e-7 rad at initialization. Loaded finger-frame mismatch is below 0.20 micrometers. The imported colliders report `convexHull`; raw bounds and world transforms are recorded. This is **not proof that cooked PhysX hull surfaces equal the raw offline triangle surfaces**.
- FCL BVH contact points can lie on the opposing object triangle. The first nearest-point projection correction was insufficient: it hid body intersections adjacent to pad edges. The final gate evaluates the **entire triangle-pair intersection segment**, permits only original distal-pad surface contact in the target region, and rejects unclassified contact. It does not enlarge pads, shrink meshes or disable collisions.
- The formerly retained closure has approximately **0.02–0.30 mm** non-pad surface intersections. PhysX reports pad-only contact at the same measured coordinates. Consequently `physx_pad_only_closure=true` is recorded separately from `legal_real_grasp=false`.
- The earlier permissive trial physically reached **16.37 deg**, then lost one finger contact; relative displacement was **1.08 mm**, margin **0.3615 rad**, fixture contact zero. It is labelled **diagnostic only** and is not a validated grasp or opening success. The final strict repeat stops before any opening command.

Remaining cause to resolve: compare actual cooked shapes, collision origins and contact witness semantics on **both the handle and fingers**, without changing dimensions or allowing metal contact. Do not attribute this unresolved mismatch exclusively to the robot or exclusively to the asset. The no-robot torque result rules against gross hinge/fixture obstruction, not against handle contact modeling problems.

## Independent entry points

- `piper_mobile_demo/model.py`: six-joint bounded exact-pose IK, full-size official hulls, FCL scene/self collision, joint-edge checks and a small RRT pregrasp planner.
- `scripts/piper_mobile_preflight.py`: bounded generic handle manifold, local PCA anchors, full 0–22-degree arm paths and conditional SE2 search.
- `scripts/piper_mobile_execute.py`: common scene bootstrap, native coupled fingers, actual closure, strict closed-path gate, passive-door physical execution, collision/contact loss/slip/margin stops and video.
- `scripts/run_piper_mobile_door.py`: fixed preflight -> jaw centering -> actual trials; kinematic failure permits base search -> collision-checked route -> reposition -> stop/lock -> manipulate. Contact failure alone does not trigger relocation.
- `scripts/piper_door_sanity.py`: ROS-independent reuse of the existing no-robot torque sweep, without loading a robot.

The chassis is a **parameterized simulation proxy**, not an official wheeled robot: box `0.34 x 0.30 x 0.20 m`, center `z=-0.66 m`, mast `0.10 x 0.10 x 0.46 m`, floor `z=-0.76 m`. Arm height is fixed; only x/y/yaw are searched. Root motion is kinematic and the arm root remains fixed during manipulation. No dynamic wheel navigation or simultaneous base/arm motion is claimed. Runtime reposition remains unvalidated.

This minimal experiment uses **FCL + bounded IK + RRT**, not a new PiPER MoveIt service. Existing R1 MoveIt files are preserved. This planner difference and the inferred PiPER pad definition limit claims of comparative reliability.

## Reproduce on the server

Repository copy: `/data1/home/rangeryx/fr3_real2sim_piper_mobile`.
Geometry Python: `/data1/home/rangeryx/.conda/envs/anygrasp/bin/python` (numpy/scipy/trimesh/python-fcl/matplotlib).
Isaac Python: `/data1/home/rangeryx/isaaclab-arena/.venv/bin/python`.
Prepare paths from the checkout, not committed absolute URDF filenames:

```sh
python scripts/prepare_piper_description.py
python scripts/run_piper_mobile_door.py \
  --source /data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --geometry-python /data1/home/rangeryx/.conda/envs/anygrasp/bin/python \
  --isaac-python /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  --output results/piper_independent_run --gpu 6 \
  --deadline-shanghai 2026-10-02T05:00:00+08:00
```

Final strict retained-pose replay uses the archived search ordering:

```sh
env -u PYTHONPATH -u CUDA_VISIBLE_DEVICES \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/piper_mobile_execute.py \
  --source /data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --plan results/piper_fixed_center_search/report.json --candidate-index 7 \
  --output results/piper_final_strict_fixed_repeat --gpu 6
```

Use saved `selected.T` and raw rank provenance when replaying: current centering sorts seeds by margin, so freshly regenerated row indices can differ. Never equate `FULL_PATH_PLANNED` with successful real closure. The final strict gate is authoritative and currently rejects the retained grasp.

Artifacts are in server `results/piper_delivery`, including torque sweep, measured reports, strict surface replay, imported collision metadata and labelled historical video. Local copies are in `results/piper_mobile_door/piper_delivery` (generated media/data are intentionally outside Git). `docs/piper_mobile_door_summary.json` is committed. No active experiment remains running.
