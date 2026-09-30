# Door support and Dex1 non-pad collision audit

Baseline: `4a6b14d`, branch `r1-7a`. Only support/contact modeling and diagnostic code changed. Robot base `(0.50, -0.55, -0.10; 150 degrees)`, asset placement, IK settings, joint margin >0.05 rad, official robot/asset meshes, friction, drive gains and gripper force remain unchanged. No object-id exceptions or non-pad collision permissions were added.

## 1. Door versus fixture: fixed and physically verified

The old support used the entire static bounding-box footprint but placed its centre at the asset origin. The bounds are asymmetric, so this also displaced the support. The moving door and static shell share their lowest source-Y coordinate: with a full-footprint support, the door bottom had zero clearance from the support top and crossed its footprint during opening. The resulting 133–266 N forces were support interference, not passive hinge resistance.

`articulated_demo/fixture_geometry.py` now reads the actual prepared collision STLs and URDF transforms, sweeps the moving links over their joint limits in one-degree increments, and places a retained rectangular support behind that sweep with 5 mm clearance. The footprint uses the true static-bounds centre. In this scene the front edge is recessed 22.825 mm; support height remains 40 mm. Isaac and MoveIt receive the same corrected box. An impossible support footprint raises an error instead of disabling collision.

No-robot Isaac torque sweep, including table, fixture and static cabinet collisions:

| Torque (N m) | Angle after approximately 1 s | Fixture contact peak |
|---:|---:|---:|
| 0 | 0 degrees | 0 N |
| 0.001 | 0.216 degrees | 0 N |
| 0.0025 | 0.541 degrees | 0 N |
| 0.01 | 2.163 degrees | 0 N |
| 0.1 | 21.631 degrees | 0 N |
| 0.25 | 54.078 degrees | 0 N |

All tested positive torques through 2 N m had zero recorded fixture/table/static-body contact, including motion to the 90-degree joint limit. The operational threshold for >0.5 degrees in one second is bracketed by 0.001–0.0025 N m, or 0.00242–0.00606 N tangentially at the measured 0.41264 m handle radius. **This is a simulation acceleration threshold with zero joint friction, not a measured real-device breakaway torque.**

## 2. Dex1 versus door: filtering bugs fixed; usable load-test grasp remains blocked

The official finger-body and distal-pad STL definitions/origins were retained. `Link1_2/Link2_2` are metal bodies; only `Link1_3/Link2_3` are permitted target-contact pads. The existing MoveIt allowed-contact list was already restricted to pads and remains so.

Two prefilter defects were confirmed: target points inside the aperture were removed for *all* meshes, including metal bodies; and an open-gripper approach pass did not check the subsequent finger closure. `grasp_compare/collision.py` now checks every target point against non-pad meshes, permits pad contact only at the grasp endpoint, and offers a closure sweep using official geometry. The sweep uses the visible target span as an estimated contact aperture; partial RGB-D cannot establish the actual stalled opening or force closure. MoveIt and physical validation remain required.

The raw rank-32 pose passes open approach but fails the retained 3 mm clearance during closure: at 41.496 mm opening the body-to-cloud surface distance is 2.498 mm. At a diagnostic 30 mm opening, target points penetrate the official `Link2_2` mesh by 0.702 mm. These are geometric probes, not claims that the physically stalled fingers reached those openings.

The articulated variant filter now includes local-handle-patch alignment to the official pad-aperture centre and the same small generic pitch search. It preserves raw poses, uses each candidate's nearest/PCA patch, and adds no asset-specific offset. For rank 32, the bounded 16-variant run retained one geometry-safe pose; that pose failed exact IK with the unchanged base and solver settings. It was not executed.

Focused locked-door physical replay of the original pose:

- Closed-state MoveIt collision check passes; distal-pad forces are 7.390 / 11.302 N; metal-body forces are 0 / 0 N.
- Actual closed-state joint margin is approximately 0.232 rad.
- The first 0.25 mm tangential interpolation fails full MoveIt collision validation at `cabinet_door ↔ dex1_Link1_2`; the pull is not executed.
- The 2.142 N zero-displacement tangential preload is **not** grasp capacity. `max_stable_tangential_force` remains null.

Thus there is no evidence justifying shrinking official metal geometry or allowing metal contact. Corrected checks reject a grasp with insufficient non-pad motion clearance. The remaining requirement is a geometry-safe, IK-feasible grasp with a collision-free loading path; neither IK changes nor end-arc limit optimization were attempted in this round. **Not ready to report or measure a stable tangential-force capacity with the current grasp.**

## Reproduction and artifacts

`src/r1a7_articulated_sim_server.py --door-sanity-only ...` runs the torque sweep without loading the robot. `scripts/preflight_articulated_saved.py --contact-model-check ...` runs only pregrasp, approach, closed-state validation and a 0.25 mm locked-door probe, without searching an opening arc. Diagnostic sensors distinguish distal-pad and metal-body contact.

Compact results: `docs/r1_collision_model_fix_summary.json`. Full data on server: `results/r1_collision_model_fix/door_no_robot_fine.json`, `raw32_physical_v2/isolation.json`, `raw32_physical_v2/isolated_grasp.mp4`, `final_variants32.json`, and `pad_aligned_physical/report.json`.
