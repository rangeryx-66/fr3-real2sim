# V8.8 fixed-asset results, 2026-10-27

**45290 completed the measured task lifecycle. 45600 reached 90 degrees and
reversed, but failed controlled stopping and remains NOT PASS.** Both owned
Isaac instances exited; videos, telemetry, models and safety records were
sealed before independent dynamics evaluation. No further physical trials
were launched after evaluation accessed hidden dynamics.

The frozen experiment implementation was `06ff7d3fecc9508a9849552d49dc2b27878db844`.
Each case had an independent 4-hour maximum, including perception, planning,
motion, recovery, validation and exit. Cases ran sequentially, prismatic first.
Only one selected joint per asset was operated. Known joint geometry was
allowed; hidden dynamics were excluded from online decisions. GraspGenX was
disabled. The simulator uses a driven XY carriage with fixed height/yaw,
not a wheeled mobile platform.

| Actual result | 45290, prismatic | 45600, revolute |
|---|---:|---:|
| Maximum forward travel | 180.625 mm | 90.122 degrees |
| Actual reverse travel | 91.530 mm | 31.705 degrees, including failed stopping |
| Largest continuous forward travel with one grasp | 180.624 mm | about 63.26 degrees |
| Physical grasp attempts / regrasp recoveries | 2 / 1 | 2 / 1 |
| Unloaded station moves | 0 | 1, about 1.6 mm back to the measured 63-degree station |
| High-level Agent commands | 7 | 23 |
| Measured unloaded Agent wait | 455.45 s | 2226.95 s |
| Command processing/execution wall time | 2408.74 s | 4210.84 s |
| Total formal wall-clock | 3025.08 s / 50.42 min | 6931.29 s / 115.52 min |
| Independent prediction | Completed | Not executed |
| Final release / retreat / parking | All verified | Not confirmed after final fault |
| Final classification | Task lifecycle PASS | SAFETY-BLOCKED / NOT PASS |

Command counts are interface submissions, not measured LLM inference counts.
Door waiting totals exclude the separate latched diagnostic interval; total
wall-clock includes it. Process return code zero is not physical success.
The door's controller latch remains recorded as `HARD_STOP_UNPARKED` even
though the raw measured collision/force/speed hard-event count was zero.

## Actual coordination

All metrics below select actual loaded coordination rows, excluding unloaded
recovery rows that can retain the preceding segment ID. Translation and
orientation are reported separately. Net projected base contributions can
exceed 100% when arm compensation opposes part of the base displacement.

For the drawer's first forward segment, the projected net base contribution
was 100.31%, arm net translation compensation was 0.575 mm, maximum arm-joint
change was 0.184 degrees and maximum TCP position tracking error was 0.583 mm.
Actual base speed peaked at 4.686 mm/s across loaded segments. Median measured
positive finger-normal loads were approximately 0.916 / 1.067 N.

For the door's first grasp, projected net base contribution was 86.65%, but
arm net translation compensation was 338.8 mm and maximum arm-joint change
was 49.86 degrees while TCP orientation changed 63.24 degrees. For the
regrasped 63-to-90-degree segment, those values were 107.03%, 233.8 mm and
14.21 degrees, with TCP rotation of 26.77 degrees. Maximum loaded tracking
error was 1.777 mm; actual base speed peaked at 4.323 mm/s. Median positive
finger-normal loads were approximately 0.956 / 1.027 N. Thus base translation
dominance was demonstrated, while small arm compensation was strongly
demonstrated only for the drawer. Fixed base yaw limits the door comparison.

Single-pad normal-load loss did not automatically terminate useful drawer
motion. These normal loads are not full gripping wrenches or pressure areas.
Six-axis sensor records include tool gravity and motion; they must not be
compared directly with compensated hard-safety thresholds.

## Physics-ID and independent prediction

The original six-axis F/T estimator and FIT/VALIDATION separation were retained.
Hidden dynamics were read only after both owners closed and online evidence
hashes verified; models and raw data were not subsequently refitted.

| Error against authored posthoc reference | 45290 | 45600 |
|---|---:|---:|
| Effective mass/inertia, signed relative error | -1.712% | +9.369% |
| Viscous damping, signed relative error | -0.715% | -85.830%, unsupported |
| Resistance relative RMSE within observed FIT support | 0.00992% | 6.859% |
| Independent coordinate prediction RMSE | 0.01643 mm | N/A |
| Independent effort prediction RMSE | 0.0000538 N | N/A |

The drawer's independent trajectory physically moved **forward** about
24.196 mm from its actual recovered state, despite the retained request role
name containing `REVERSE`. It used the predeclared target/profile. Prediction
covered 8.933 s / 2145 supported samples, using held-out compensated wrist
effort and one initial measured q/qd, without later state correction. It is
not an effort-free forecast or validation over the entire travel range.
Unfiltered-input coordinate RMSE was 0.02085 mm. Authored mass/inertia is a
reference, not an independently calibrated PhysX effective inertia.

The door has only a provisional model from its completed forward FIT. The
reverse interrupted during stopping; variable FIT and independent validation
were not executed. Operation progress is not an identification PASS.

## Failures, recovery and remaining boundary

- Both formal cases encountered `TCP_PROJECTION_INFEASIBLE`. They returned
  to the Agent and preserved the World. The drawer safely parked, changed
  view and grasp, then completed its pending independent validation.
- Door recovery initially made three rejected bounded grasp-planning requests
  plus one mask rejection. A diagnostic batch showed 19/19 initial collision
  rejections because the planner reconstructed the launch base pose after
  loaded coordination had moved the real base. This was not proof of global
  IK infeasibility. Complete cumulative branch-level IK/collision counts were
  not persisted by the old planner and are unavailable.
- Returning through the existing checked station interface synchronized the
  measured 63-degree station and invalidated old plans. Fresh wrist geometry,
  a different roll/IK branch and 8100 native joint configurations passed;
  the next physical grasp opened from 63.26 to 90.12 degrees and reversed.
- Door reverse stopping then failed: over the final 5 seconds, TCP commands
  remained approximately 2.992–3.000 mm/s while the door moved from 60.000 to
  58.417 degrees. The native supervisor prohibited further motion and had no
  qualified reset owner. The World was retained for read-only diagnosis,
  then finalized with its latch intact. Release/parking was not fabricated.
- Short door qualification separately recovered from `LOCAL_RETREAT_NOT_FOUND`
  using a checked unloaded wrist route and then the original parking owner.

Against historical fixed-base runs, drawer maximum travel increased from
136.11 to 180.625 mm and grasp attempts decreased from 6 to 2. Door travel
increased from 32.185 to 90.122 degrees with 2 grasp attempts in both runs.
These are descriptive comparisons: plant draws, implementation and recovery
policies differ, so they do not establish a causal matched comparison.

## Prospective shared fixes after closure

See [shared recovery fixes](SHARED_RECOVERY_FIXES.md). The new code synchronizes
measured planning stations and brakes compensating joints with a common
slew-limited scale before the unchanged nonlinear checks. These fixes were
not hot-loaded into either formal instance. The cancellation replay fails
with the frozen implementation and stops within one second after the fix;
118 relevant tests pass. Frozen physical/native/release owners remain intact.
The new stopping fix still requires a future separately authorized physical
qualification. No additional objects or rerun were added to this experiment.
