# R1-7a + Dex1 validation on labserver_inschool

## Setup and provenance

- Base placement: world origin, +90° yaw toward table +X, as approved.
- Arm and hand: unmodified Unitree URDF and STL sources at commit
  `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee`; adapter and TCP are
  generated fixed frames described in `R1A7_README.md`.
- `scripts/audit_r1a7_model.py` passed: 13 upstream joints retained, 15
  upstream link inertias retained, zero missing meshes. Generated model audit
  is in `results/r1a7_comparison/model_audit.json` on the test checkout.
- Isaac and MoveIt independently imported the model. Simulated joint state
  contains J1–J7 and two Dex1 fingers. Dexterity tests use MoveIt KDL 7DoF.
  The live Isaac/MoveIt TCP cross-check was <1 µm and <6 µrad at home.
  Dex1 open/close command tracking was checked at 90, 50, 10, and 90 mm.
- The generated Isaac USD physics layer was inspected: J1–J7 axes are
  Z, X, Y, X, Y, X, Z, matching the official URDF, and its degree-valued
  revolute limits match the URDF radian limits after conversion. Both Dex1
  prismatic joints have X axes and [−0.02, 0.0245] m limits. The USD has
  15 mass entries and 15 collision API entries. The runtime logged all
  nine explicit drive gain pairs; their values are in `src/r1a7_sim_server.py`.

## Frozen 81 tabletop poses

The same world TCP targets are on a 9×9 grid: X = 0.15–0.55 m,
Y = −0.25–0.25 m, Z = 0.25 m. The orientation is the frozen PiPER-home
orientation in `config/ik_tabletop_81.json`. Five seeds (home plus four
random seeds), 0.1 s MoveIt IK timeout per call. Relaxed orientation uses
11 deterministic samples within ±10° or ±20°; this is a discrete lower
bound, not an analytic continuous tolerance. Position-only uses bounded
least-squares FK with a 2 mm acceptance threshold.

| Robot | Exact 6D IK | Exact 6D collision-free first solution | Exact 6D collision-on IK | ±10° | ±20° | Position-only |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| FR3 | 78/81 | 60/81 | 67/81 | 81/81 | 81/81 | 81/81 |
| PiPER | 44/81 | 44/81 | 44/81 | 58/81 | 67/81 | 81/81 |
| R1-7a | 69/81 | 68/81 | 69/81 | 73/81 | 73/81 | 73/81 |

Controlled ablation: fixing only R1 J7 at 0 reduced exact 6D success to
35/81, all collision-free. Its 35 successes are a strict subset of the
full 7DoF arm's 69 successes. J7 therefore enabled 34 additional exact
poses under identical R1 geometry, TCP, world mount, solver settings, and
target set. The ablation used `--exact-only`; relaxed and position-only
rates are deliberately not reported for that variant.

“Collision-free first solution” checks the first kinematic solution without
another IK search. “Collision-on IK” passes that solution as a seed to
MoveIt with collision avoidance enabled; it may choose another solution.
No-scene collision checks here include self collision, but no tabletop box.

The first seed alone solved 77 FR3, 44 PiPER, and 69 R1-7a exact poses;
multiple seeds rescued one additional FR3 pose and no PiPER/R1-7a poses.
The R1-7a exact solutions have median joint-limit margin (radians):
J1 1.642, J2 0.839, J3 1.365, J4 1.025, J5 1.789, J6 0.171,
J7 1.046. J6 is within 0.1 rad of a limit on 27/69 solved targets,
the clearest arm-limit concentration. PiPER J5 median margin is 0.243 rad.

Exact 6D XY heatmap for R1-7a, X increasing downward and Y increasing
left to right; ● solved, · no kinematic IK:

```text
X\Y  -.25 -.19 -.13 -.06  .00  .06  .13  .19  .25
.15     ●    ●    ●    ●    ·    ●    ●    ●    ●
.20     ●    ●    ●    ●    ●    ●    ●    ●    ●
.25     ●    ●    ●    ●    ●    ●    ●    ●    ●
.30     ●    ●    ●    ●    ●    ●    ●    ●    ●
.35     ●    ●    ●    ●    ●    ●    ●    ●    ●
.40     ●    ●    ●    ●    ●    ●    ●    ●    ●
.45     ●    ●    ●    ●    ●    ●    ●    ●    ●
.50     ·    ●    ●    ●    ●    ●    ●    ●    ·
.55     ·    ·    ·    ·    ·    ·    ·    ·    ·
```

The 8 R1 position-only failures are at the far X edge of this grid.
Raw per-target data are saved as `results/r1a7_comparison/*_ik_81.json`
on the test checkout.

## Same-scene grasp benchmark

FR3 baseline run: `/data1/home/rangeryx/fr3_moveit_grasp/results/run_1788943864166403373`.
Its summary records 10/10 successes. R1-7a reuses those ten
`trial_XX_grasps.json` files without editing AnyGrasp outputs. Both runs
use the same fixed box/table scene and success rule: lift at least 8 cm,
hold 2 s, no slip/drop. The R1 backend records candidate statuses and
trial categories including `NO_IK`, `NO_PLAN`,
`NO_EXECUTABLE_CANDIDATE`, `BAD_CONTACT`, `CONTACT_LOSS`, `DROP`, and
`SUCCESS`.

| Robot | Successful trials | Trial outcome | Candidate outcome |
| --- | ---: | --- | --- |
| FR3 + Franka Hand | 10/10 | `SUCCESS` × 10 | At least one executable candidate per trial |
| R1-7a + Dex1 | 0/10 | `NO_EXECUTABLE_CANDIDATE` × 10 | `NO_IK` × 110 of 110 |

The ten baseline `trial_XX_grasps.json` files have identical SHA-256 hashes:
the benchmark is ten repeat runs of the same frozen scene, seed, object,
and grasp list, not ten independent objects. The R1 attempts used exactly
those files. All 13 IK seeds returned MoveIt code −31 (`NO_IK`) for every
candidate. No R1 attempt reached planning, execution, contact, or lift;
there are therefore zero observed `NO_PLAN`, `BAD_CONTACT`,
`CONTACT_LOSS`, or `DROP` failures. The ten trial JSONs and summary are in
`results/r1a7_comparison/benchmark10/`.

## Interpretation and limits

The shared 81 poses show substantially better exact 6D reachability for
R1-7a than PiPER (+25/81), but less than FR3 (−9/81). Because the robots
have different geometry, grippers, TCPs, and joint limits, that difference
alone cannot establish that J7 caused the gain. The same-model ablation
does establish a large J7 effect on this target set. The grid at Z=0.25 m is
also much higher than the actual grasp candidates around the tabletop;
the tabletop benchmark must be reported independently.
The replayed AnyGrasp grasp TCP targets are only Z = 0.0299–0.0400 m
at X = 0.4813–0.5200 m, explaining why the Z = 0.25 m IK grid is an
insufficient predictor of the full grasp pipeline.

**Conclusion:** the seventh joint substantially improves the sampled
6D pose reachability, but the specified R1 base placement, derived Dex1
adapter/TCP, and unchanged AnyGrasp grasps do **not** produce an
executable tabletop candidate. The grasp migration is implemented and
the Isaac/MoveIt stack runs, but the requested end-to-end grasp success
has not been achieved. The immediate limiting condition is low-height
exact 6D IK reachability. The mesh-derived Dex1 adapter is a remaining
physical-frame uncertainty until checked against Unitree's actual mounting
hardware; a frame error cannot be excluded by the home FK cross-check,
because both Isaac and MoveIt use the same generated mount.
