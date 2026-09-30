# Dex1 closure/collision consistency audit

Baseline: `9823374`, branch `r1-7a`. Fixed base `(0.50,-0.55,-0.10; yaw 150°)`, frozen perception and bounded local search retained. No IK parameters, >0.05 rad margin, friction, drive gains, force commands, official dimensions, scene installation or object-specific offsets changed. This is a contact diagnostic, not an opening-success experiment.

## Root causes and changes

1. The composed Isaac USD has **convexHull** collision on both metal finger bodies and distal pads. MoveIt and the point-cloud prefilter previously used the original STL triangles. Metal-body hull volume is **2.694778×** source volume; pad volume is unchanged within numerical precision. The hull fills finger concavities without changing outer dimensions. Imported collider-to-link transforms and the four fixed finger/pad transforms match official URDF exactly; scale is 1. Numeric mesh-vertex comparison is included in the audit artifacts.
2. The visible target span was incorrectly used as the terminal closure width, and both fingers were assumed symmetric. Actual stalled fingers are asymmetric and continue closing beyond that estimate. The former predicted-width sweep does not establish a valid real grasp.

The diagnostic now uses full-size **derived convex hull STL proxies** for MoveIt and point-cloud checks, matching Isaac's conservative collision envelope. Isaac's physical collision geometry is unchanged. Unitree source STL, visual mesh, joint geometry, scale, origins and inertias are preserved. All metal bodies remain obstacles; only distal pads are eligible for target contact. Proxy preparation records source/proxy SHA256, bounds, volume and origins. No collision is disabled or mesh reduced.

Actual closure records both finger joint coordinates, TCP transform, pad/body forces and robot joint state every physics step. MoveIt replays those measured states; the point-cloud checker uses the same asymmetric finger geometry. Reusing a raycaster under exact prismatic translations is numerically checked against freshly transformed meshes. Actual joint margins are checked separately. A metal contact or invalid measured closure state rejects the grasp before any loading. The predicted width remains a diagnostic field and is labelled `CLOSURE_UNVERIFIED`, rather than a closure pass.

## Rerun

The same 240 in-bounds poses yield 138 open-approach geometry passes, 42 LOW_CLEARANCE and 60 COLLISION under hulls. Of the 138, 135 fail the estimated sweep clearance and 3 remain **closure-unverified**. These 3 are not accepted grasps. All five formerly retained poses were again tested with exact IK, margin, real MoveIt pregrasp/approach and actual Isaac closure.

**Actual pad-only closure: 0/5.** Unified MoveIt collision checks now also reject the measured metal-contact states. The point-cloud replay extends through the actual endpoint rather than stopping near 37 mm. Loading and opening remain blocked.

This identifies a **closure/pose geometry restriction under the current conservative hull representation**, not a failure of exact IK for these five poses. It does not prove that the official physical gripper cannot grasp this handle or that the continuous pose space is empty. RGB-D misses hidden surfaces; complete MoveIt door mesh and actual contact monitoring remain authoritative. PhysX contact offsets and numerical contact resolution also differ from zero-distance mesh intersection. Conservative hull parity fixes the original false acceptance; it does not validate the hull as a faithful model of the gripper's concavities.

### Actual closure endpoints

| Pose | Point-cloud estimate (mm) | Actual opening, command convention (mm) | Error (mm) | Metal body force (N) |
|---|---:|---:|---:|---:|
| local_075 | 36.259 | 27.286 | -8.973 | 8.87 |
| local_082 | 36.470 | 27.362 | -9.108 | 8.80 |
| local_147 | 37.172 | 27.122 | -10.050 | 4.96 |
| local_161 | 36.932 | 27.219 | -9.713 | 8.90 |
| local_168 | 37.217 | 27.270 | -9.947 | 8.93 |

The table uses the existing simulated opening convention, computed from both measured finger coordinates. The independently computed official pad inner-surface gaps are 27.348 / 27.424 / 27.184 / 27.281 / 27.332 mm respectively, about 0.062 mm larger; both definitions are stored, and neither is inferred from the target point cloud.

All five episodes record 442 consecutive closure samples in the final dataset. Measured closure arm margins stay above 0.05 rad; the worst is 0.0967 rad. Each has 277–281 measured states with explicit MoveIt door/metal-body collision pairs. Point-cloud replay rejects all five: local_075/082 fail clearance; local_147/161/168 also penetrate a finger-body hull. Source and imported USD finger mesh vertices match exactly. Prismatic raycaster reuse agrees with freshly transformed mesh distances within **7.45e-9 m**.

A controlled 2×2 ablation fixes each measured TCP pose and switches only representation (source STL/hull) and endpoint (predicted symmetric/measured asymmetric). Predicted endpoints pass in both representations; measured endpoints fail the unchanged 3 mm clearance with source STL for all five, and the hull adds actual penetration for three. Thus **the underestimated closure travel is the primary missed gate, with convexification adding interference**. This ablation does not replace the complete physical closure trace.

Micron-scale settled joint-bound drift is normalized for MoveIt replay using the same pre-existing 1e-4 bound normalization as `ArticulatedBackend.measured`; raw telemetry/aperture are retained and every clamp is recorded. Actual arm margin is computed from raw measurements and still must exceed 0.05 rad.

The next modeling improvement, if pursued, should validate a full-size concavity-preserving convex decomposition consistently in both systems. No such decomposition or new pose optimization was introduced in this audit.

## Reproduce

Run proxy preparation with the geometry Python environment containing trimesh:

```sh
python scripts/prepare_dex1_collision_hulls.py
DEX1_COLLISION_PROFILE=isaac_convex_hull python scripts/verify_dex1_collision_profile.py
```

In the existing ROS environment, the saved articulated preflight defaults to the aligned profile; an explicit historical `--collision-profile official_stl` is available for audit comparisons only:

```sh
python scripts/preflight_articulated_saved.py \
  --source results/microwave_7320_table_edge_preflight_full_rgb \
  --variants-file results/r1_handle_local_search/geometry.json \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --output results/r1_contact_consistency/episode \
  --local-grasp-search --collision-profile isaac_convex_hull \
  --require-home-valid --record-overview
```

The local-search entry now performs **closure diagnostics only**, one physical episode per fresh scene. It does not authorize a 2 mm load path from a guessed closure state. Separate episode files select each existing pose without changing it.

Server artifacts: `results/r1_contact_consistency`, including per-episode reports/videos, `imported_usd.json`, `replay.json`, proxy provenance and numerical verification. Compact results are saved in `docs/r1_dex1_contact_consistency_summary.json`.
