# R1-7a + Dex1 rigid-object generalization benchmark

## Result

This is an Isaac Sim + ROS 2/MoveIt 2 simulation experiment on branch `r1-7a`. Thirty paired trials completed: six object shapes/sizes, each at five deterministic randomized tabletop XY/yaw placements. A fresh AnyGrasp inference ran for every scene, and the same saved top-K poses were used for its raw and Dex1-adapted executions. The AnyGrasp model, inference code, scores, and original candidate poses were unchanged. The adaptation offset/orientation search and scoring weights were unchanged across objects. Object dimensions, yaw, and official Dex1 terminal collision meshes were used in the geometric checks.

| Object | Dimensions (mm) | Raw success | Adapted success | Raw path-valid candidates | Adapted path-valid candidates |
|---|---:|---:|---:|---:|---:|
| Small box | 32 × 32 × 40 | 0/5 | 4/5 | 0 | 12 |
| Large box | 70 × 55 × 60 | 0/5 | 2/5 | 0 | 8 |
| Low box | 60 × 50 × 25 | 0/5 | 0/5 | 0 | 0 |
| Tall box | 38 × 38 × 90 | 3/5 | 1/5 | 4 | 22 |
| Long box | 85 × 30 × 45 | 0/5 | 3/5 | 0 | 6 |
| Cylinder | diameter 52 × height 65 | 1/5 | 3/5 | 1 | 12 |
| **Total** | | **4/30 (13.3%)** | **13/30 (43.3%)** | **5** | **60** |

For the 24 scenes with at least 30 target-mask points, raw succeeded 4/24 (16.7%) and adapted 13/24 (54.2%). The six other scenes each produced zero target-mask points at placement 4. AnyGrasp still ran and saved top-K JSON for these scenes, but its output cannot be treated as target-region grasps; they were classified `NO_GRASP`. The fixed camera/placement combination needs inspection. The exact occlusion cause was not established.

By position band, adapted success was near 5/12, center 5/6, far 3/12. All six zero-mask scenes are in the far band, so this band comparison mixes reachability and visibility.

Across the 421 top-K grasps from target-visible scenes, raw had 5 candidates passing the full kinematic path check and adapted had 60 (1.2% versus 14.3%). These are candidate counts, not independent trial success rates. The adapted final failure categories were `NO_GRASP` 6, `NO_EXECUTABLE_CANDIDATE` 9, and `CONTACT_LOSS` 2. The candidate diagnostic logs further record `BAD_GRASP_GEOMETRY`, `COLLISION`, `NO_IK`, and `LOW_JOINT_MARGIN` separately; those counts are affected by variant search and should not be read as scene failure counts.

## Adaptation offset and failure analysis

Fifteen adapted scenes selected a planned grasp. Their translation magnitude ranged from 0 to 24.94 mm, with median 10.0 mm. Only 2/15 were within 22.5–27.5 mm. Selected offset medians differed across objects: small box 21.38 mm, large box 8.37 mm, tall box 6.0 mm, long box 19.24 mm, and cylinder 5.03 mm. The low box had no planned grasp. These data do **not** support a universal fixed 25 mm correction; the small-box successes do use larger offsets than the others. The selected rotation magnitude had median 0 and maximum 15°.

The low box failed in all four target-visible placements before execution. Its 25 mm height leaves little clearance for the Dex1 fingers and the table; the recorded candidate checks include geometry, table collision, IK, and margin rejection. This is a geometry/clearance class of failure, not a reason to add a per-object heuristic. The tall box regressed from raw 3/5 to adapted 1/5: in two scenes the adapted grasp passed static screening/planning but the object did not follow the micro-lift (`CONTACT_LOSS`). The selected grasp's static geometry score therefore does not reliably predict loaded contact for that shape. The large and long box far/near failures include scenes with target points but no executable candidate. The six camera-zero scenes are a perception coverage limitation.

## Protocol and limits

- Scenario seed starts at `20260928`; object-specific seeds and XY/yaw values are saved under `results/r1a7_generalization/scenarios`.
- R1 base: `(0.329, -0.175, 0.237 m; yaw 56.295°)`; parameterized pedestal: `0.10 × 0.10 × 0.20 m`.
- Each raw/adapted trial resets the same object pose, uses the same original AnyGrasp top-K JSON, checks contact geometry, IK, joint margin, table/scene and self collision, pregrasp/approach/lift path, then uses MoveIt and Isaac execution. Success requires at least 8 cm lift and a 2.2 s hold without significant slip/drop; successful lifts were about 9.7–10.2 cm.
- The simulation uses simple rigid boxes/cylinder with specified masses. Material variation, clutter, real camera noise, and hardware were not tested. An FR3 baseline on these new objects was not run; its old single-box benchmark is not a matched comparison.

**Conclusion:** Dex1 adaptation improves reachability and simulated grasp success across several unseen rigid shapes, but 13/30 overall success, 0/5 on the low box, tall-box contact regression, and six camera-zero scenes do not establish stable cross-object grasping. R1-7a + Dex1 is not ready for articulated-object interaction on this evidence. The next bottlenecks are camera coverage, finger/table clearance for low objects, and predicting loaded contact after an adapted grasp. No articulated-object work was performed.

Machine-readable per-scene results, saved raw top-K poses/scores, and aggregate statistics are under `results/r1a7_generalization`; `analysis.json` is generated by `scripts/summarize_r1a7_generalization.py`.
