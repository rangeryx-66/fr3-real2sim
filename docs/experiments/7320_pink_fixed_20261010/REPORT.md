# 7320 Pinocchio + Pink, fixed base after grasp

Classification: **KNOWN_MODEL_DIAGNOSTIC**. Source branch `codex/piper-pink-fixed-base`, execution source through `61b12a9`.

Best actual physical opening found: **65.23786°**. All four trials started normally closed, physically grasped once, kept the base fixed, and performed zero intentional releases or regrasps. No episode reached the 90° target. This is the best measured result of the bounded station search with the existing grasp geometry; it is neither a proven global optimum nor a mechanical door limit.

| Trial | Base x, y, yaw (m, m, deg) | Reference scale | Maximum actual | Simulation s | Wall s | Original stop |
|---|---|---|---|---|---|---|---|
| physical_01 | 0.200, -0.650, 42.613° | 1 | 63.85784° | 614.48 | 2736.66 | joint margin |
| physical_02 | 0.190, -0.645, 42.613° | 1 | 64.48989° | 568.62 | 2511.27 | joint margin |
| physical_03 | 0.190, -0.650, 42.613° | 1 | 65.22234° | 587.67 | 2521.17 | arm joint speed |
| physical_04_slow | 0.190, -0.650, 42.613° | 0.8 | 65.23786° | 786.35 | 3227.86 | world TCP speed |

The previous fixed-base physical result was 56.252°. The best new result gains 8.986°. The initial station, solver and reference policy differ, so this comparison does not isolate a benefit of Pink alone. Pink's offline result at the original station was approximately 56°.

## Planning and execution

Pinocchio 3.9.0, Pink 3.3.0, DAQP; native physics and differential IK at 240 Hz. Search varied pre-grasp base SE(2), keeping the successful handle-relative grasp and fixed z=-0.1 m. The best consistent quarter-degree offline path reached 67.25°. Coarse candidate scores reaching 69° were not full-path or physical results. There is no exhaustive global-optimality claim. The selected base was initialized in the normal closed setup; a physical pre-grasp navigation route was not demonstrated.

The experiment runner is separate from frozen skills. It reuses unchanged physical approach/closure, loaded equilibrium bias, collision/contact policy, temporal force policy, 0.05 rad joint margin, 5 mm/s world TCP protection and inherited 5/3 rad/s arm velocity protection. No gains, mass, friction, clamp force, geometry or protection threshold were changed. There were no attachments, object-joint commands, artificial object forces, or scene stitching.

## Actual stopping behavior

Trials 01/02 reached the original joint-margin boundary: actual joint 5 margin approximately 0.05003/0.05004 rad, with the next command at the existing boundary.

Trial 03 reached 65.22234° and triggered the original arm velocity protection: joint 1 speed 1.66937 rad/s against 1.66667 rad/s. Its final actual minimum margin was approximately 0.19594 rad, so this did not establish a kinematic maximum.

Trial 04 repeated the same station with only reference time scaling 0.8. It reached 65.23786° and then triggered the original 5 mm/s world TCP protection. At t=786.34167 s, the finite-difference TCP speed was 40.2209 mm/s; the preceding steps were approximately 0.83–0.88 mm/s. Joint 1 velocity reversed from approximately -1.33257 to +1.35118 rad/s, and both pad loads became zero in that step. This abrupt physical/IK execution transition is the first recorded divergence. The minimum margin at that instant was about 0.19593 rad. No safety limit was relaxed and no additional retry was launched. `physical_04_slow/first_physical_divergence.json` preserves the surrounding samples.

Trial 04 maximum relative drift was 2.756 mm / 0.384°, peak pad loads 1.121 / 0.616 N. One 4.17 ms interval had no positive target-pad impulse at the stopping step. Raw zero values on one pad are retained; they were not made into new execution gates.

## Evidence and dependencies

Each output contains a continuous `contact_baseline.mp4`, lossless observation and physics chunks, sensor metadata, source snapshots, job and plans, event logs and metrics. Native counts for trial 04: 188723 observations and 188723 physics records at 240 Hz. Artifacts remain under `engineering_artifacts_20261010/7320_pink_fixed` locally and `results/7320_pink_fixed_20261010` on the server. Raw media/telemetry are not published to GitHub. `evidence_hashes.json` and per-trial source/config files identify them.

GT dependencies remain explicit: known successful grasp template, hinge geometry, actual moving-body/articulation feedback, and collision models. This is not visual or GT-free execution, hardware performance, or unknown-model inference.

All four owned Isaac actors were closed after export. The final hash audit checked 456 known-model baseline files and 32 visual-state baseline files, with no changes.
