# Bounded handle-grasp feasibility experiment

Baseline: `1c2112d`, `r1-7a`. Robot base, object placement, model geometry, Isaac collision approximation, IK solver/seed counts/timeout, global >0.05 rad margin, friction, drive gains and clamp force were retained. Raw rank-32 GraspGenX output and the saved RGB-D/mask are immutable.

## Search and gates

`scripts/search_handle_grasp_geometry.py` derives each handle frame from the local nearest/PCA target patch. It samples translations along handle/lateral/approach axes within ±8 mm and rotations up to 20 degrees, around both the raw pose and the official pad-aperture alignment. Total displacement is bounded by 40 mm; the five retained poses actually moved only 4.0–9.5 mm. There are no asset-id rules or fixed object offsets.

`articulated_demo/local_grasp_search.py`, invoked with `scripts/preflight_articulated_saved.py --local-grasp-search`, jointly checks official-mesh approach/closure, existing exact IK, all seven joint margins, full MoveIt pregrasp/approach trajectories, and eight continuous Cartesian loading increments through 2 mm with the door fixed. IK solution collection exposes alternative states returned by the existing requests; solver parameters and seed counts do not change. Dense collision/margin checks are applied to every planned path.

Pregrasp points are also tested at 80, 60, 40 and 20 mm on the same checked approach line. The initial 80 mm test had kinematic solutions but no collision-free solutions for four poses; it was not a pure NO_IK. Selecting a nearer collision-free pregrasp does not move the base or change solver settings.

Actual locked-door closure must then have bilateral distal-pad contact, no metal-body contact, and a valid measured MoveIt state. Only after a physically executed 2 mm probe, at least 1 mm actual tangential displacement, retained contact and >0.05 rad measured margin may the existing criteria be used for a capacity sweep. All five physical trials failed before loading, so the capacity continuation was not exercised or validated in this experiment.

## Results

| Gate | Count |
|---|---:|
| Generated poses | 246 |
| Inside the fixed search bounds | 240 |
| Approach + estimated closure geometry pass | 5 |
| Exact IK + all-joint margin >0.05 | 5 |
| Full pregrasp/approach planned | 5 |
| 2 mm locked-door load path planned and collision checked | 5 |
| Actual closure with distal pads only | **0** |
| Actual 2 mm loading / capacity test | **Not run** |

The other in-bounds samples failed geometry: 175 LOW_CLEARANCE, 60 COLLISION. Six generated samples exceeded the search bounds and were not tested geometrically.

| Pose | Translation | Rotation | Selected pregrasp | Planned 2 mm path min margin | Closed pad forces L/R | Closed metal-body force L/R |
|---|---:|---:|---:|---:|---:|---:|
| local_075 | 4.0 mm | 18° | 40 mm | 0.1427 rad | 0 / 10.49 N | 8.84 / 0 N |
| local_082 | 4.0 mm | 20° | 40 mm | 0.0991 rad | 0 / 10.48 N | 8.81 / 0 N |
| local_147 | 9.5 mm | 14° | 80 mm | 0.1900 rad | 4.55 / 10.55 N | 4.57 / 0 N |
| local_161 | 7.9 mm | 18° | 60 mm | 0.1482 rad | 0 / 10.33 N | 8.92 / 0 N |
| local_168 | 7.2 mm | 20° | 20 mm | 0.0718 rad | 0 / 10.79 N | 8.92 / 0 N |

Every complete planned path retained the 0.05 rad gate; for example local_168's pregrasp plan minimum was 0.05516 rad. The real closure was rejected as soon as non-pad contact was measured. No collision was whitelisted, no loading was forced through the contact, and no opening arc was attempted.

## What excludes these poses

**Physical pad-only closure**, not exact IK or the 2 mm kinematic path, excludes all five final candidates. The tested bounded sample has no qualifying grasp. This is not a proof that the entire continuous pose space is empty.

The active imported robot USD (`assets/r1a7_dex1_4/r1a7_dex1.usda`, URDF hash prefix `fba1e619b978`) uses `convexHull` mesh collision approximations, whereas MoveIt uses the official STL triangles. This representation difference is confirmed by the active USD payload. Read-only convex-hull checks additionally reject local_075/082 at the unchanged 3 mm clearance; local_147/161/168 still pass their estimated closure sweeps.

A second limitation is the visible-point-cloud closure endpoint: it is estimated from target span, not the actually stalled finger positions. For local_161 it predicts about 36.93 mm opening, while measured finger coordinates map to about 27.22 mm under the current simulated opening convention. Thus a nominal closure-sweep pass does not cover the full physical closure. Both limitations explain why the estimated geometric gate needs physical confirmation; the present data do not isolate how much each contributes to the metal contact.

The closest physical result was local_147: both pads carry force, but the metal body also carries 4.57 N, so it still fails. `max_stable_tangential_force` remains null. Subsequent work should validate imported finger collision/contact surfaces and the full asymmetric closure endpoint before relying on geometric grasp ranking; this round did not modify them.

## Evidence

Compact results: `docs/r1_handle_local_search_summary.json`. Server artifacts are under `results/r1_handle_local_search`: `geometry.json`, `joint_search/local_search.json`, `joint_search_v2`, `physical_local_082/147/161/168`, `imported_hull_audit.json`, `collision_representation.json`, and `summary.json`. Each physical episode saves its force/state report and actual closure video. Compilation and `git diff --check` passed; physics and MoveIt were actually run for all five trials.
