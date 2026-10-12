# NERO purchase feasibility — one-page result

**Recommendation: INCONCLUSIVE. Kinematics: PROMISING. No real-hardware capability is claimed.**

The previous `link5–link7` and `link5–flange` blockers were convex-hull false positives. Across 22 audited configurations, original triangles never crossed: minimum surface gaps were **10.484 mm** and **1.715 mm**. Only those two proxy pairs were corrected; raw triangle checks remain in the shared scene adapter. Other collisions and original safety parameters were not relaxed.

Using the official NERO arm/flange/gripper with a single verified TCP convention, automatic own-station planning found complete home → approach → grasp → 30° routes:

| Object | Planned | Minimum trajectory margin |
|---|---:|---:|
| 45134 | 30° | 0.297 rad |
| 45443 | 30° | 0.373 rad |
| 45176 | 30° | 0.299 rad |
| 45415 | 30° | 0.221 rad |

Joint-3-fixed approximations still reach 30° for matched 45134/45415 starts, but only 8°/7° for matched 45443/45176 starts. Redundancy helps selected solutions; this is bounded IK evidence, not a physical result or global proof.

The official native jaw opens/closes and its measured mirror error stays below 1 μm. Runtime TCP FK agrees within 0.438 μm / 1.21 μrad. PiPER and native gripper meshes are identical; one global yaw-π tool-frame adaptation aligns the old common pad coordinates. Hardware mounting/electrical/firmware compatibility still needs vendor confirmation.

The one valid no-object Isaac probe uses finite 16/16/16/16/8/8/8 N·m arm limits and explicitly approximate CAD-derived PD plus gravity feedforward. It records **5,040 steps / 21 s** but fails holding/tracking: **0.622 rad** maximum joint error, **495.18 mm/s** TCP peak, and joint1 torque saturation for 48% of the run. Offline coupled/discrete analysis supports an unstable approximate-servo design (spectral radius 15.6), without proving full PhysX causality. Self-contact forces were not recorded. No gain-tuning loop followed.

**45134 door contact was not launched; zero of the maximum one door episodes were used.** Therefore actual opening, handle pad loads and grasp slip are unmeasured. The original video was misframed; it is retained, and a clearly labelled measured-q replay is supplied separately. The replay is not original Isaac video or a new physics run.

NERO is not ruled out. The next blocker is a stable, source-documented finite-effort actuator/physics interface, verified torque/velocity timing and complete self-contact sensing. Do not purchase on a simulated grasp-success claim that has not been demonstrated. Frozen PiPER/reference results remain unchanged; all owned Isaac processes are closed.

[Full report, plans, source hashes and evidence](REPORT.md).
