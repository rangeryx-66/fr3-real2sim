# Post-run shared recovery fixes

The two formal actors ran frozen implementation `06ff7d3` and exited before
these changes. Failed results, latches and sealed online models remain intact.

| Condition before | Shared change | Safety authority after |
|---|---|---|
| Loaded XY motion updates the measured base but the legacy grasp planner reads its launch station | Before a new unloaded grasp search, copy the immutable source report with the measured base; retain provenance and invalidate plans/catalog/cache | No physical move, object reset, grasp transform or collision waiver; native path checks still required |
| Component-wise clipping toward zero removes compensating arm motion before the base slows; feasible-anchor projection can keep the old command | A zero-velocity request first scales all moving joints by a common fraction respecting every original joint slew bound | Original nonlinear world TCP speed/acceleration checks, collisions, five-second stop deadline and latch remain authoritative |
| Ordinary planning or numerical projection failure | Existing action-only recovery remains | World remains available; strategy must change |
| Unverified actual control stop or real hard event | No reset/clear interface added | Motion remains forbidden; failure preserved |

The stopping fault is reproduced with a base velocity and an opposing arm
contribution at a 3 mm/s world TCP cap. Before the fix, `stop_command` fails
after its original five seconds with `COUPLED_CONTROLLED_STOP_NOT_VERIFIED`.
After the fix, the same test stops in less than one simulated second while
independently asserting the original 3 mm/s speed and 8 mm/s² acceleration
bounds. This is a deterministic controller fault replay, not an Isaac trial.

Station regression tests verify that new planning uses the measured base,
the original source bytes and current wrist observations remain intact,
stale plans are invalidated, repeated unchanged state is idempotent, and
loaded or latched state is not modified.

Validation: 101 V8.7 tests and 17 V8.8 tests passed. The original pre-formal
qualification used its separately recorded 131-test gate. No physical
threshold, drive gain, object property, hard latch or estimator coefficient
was changed. The earlier numerical projection interrupts are not claimed
universally solved by these two fixes.
