# V8.1 recovery gate with extended DEBUG resources

Behavioral source: `0a7ff8aef7cbe6f28b0fb61901cff106005a623d`.
Historical recovery failures at `c735339` and earlier remain unchanged in
[RECOVERY_FIRST_REPORT.md](RECOVERY_FIRST_REPORT.md). This is a bounded
engineering check, not a benchmark, full-range result or independent repetition.

## Same-scene physical recovery: PASS

`v81_debug_relocation_gate_01` completed the missing branch through actual native
actuation. [Derived metrics](DEBUG_RECOVERY_GATE_METRICS.json) preserve every
operation, failed search, inspection hash and physical action status.

| Stage | Measured outcome |
|---|---|
| First automatic grasp | Bilateral acquisition, original 1 s hold, requested preload 1.0 N |
| First short interaction |0→18.059mm; reverse12.144mm to5.915mm; both TARGET_REACHED |
| Release/parking | Released, unloaded, full native checked parking |
| Actual XY base relocation |16.124mm; drawer stayed at its measured5.915mm state |
| New observation/selection | New wrist RGB/mask/depth/cloud inspected; incorrect upper-bar SAM association explicitly rejected using current depth evidence |
| New automatic grasp | New observed geometry and station; family changed from `[0,0,0,0]` to `[0,0.009375,0,0]`; no cached pose |
| Actual physical regrasp | Bilateral acquisition and original 1 s hold |
| Post-relocation interaction |5.850→18.108mm (+12.258mm); reverse12.188mm to5.920mm; both TARGET_REACHED |
| Final state | Verified unloaded native-rest parking; pad loads0/0N; actor rc0 and exited |

The Agent inspected the fresh images and current contact plots before each
physical approval. Both acquisitions used the same 1.0 N **requested** preload;
this is not a claim of constant measured normal force. The first batch's native
closing collisions/joint-margin rejections were retained; an explicitly requested
disjoint second batch found the first grasp. After relocation, a tentative
wrong-target proposal was rejected without physical execution. New depth-based
target association and a new actuator-executed oblique wrist view preceded the
approved recovery grasp. Neither the object joint nor robot joints were reset.

Load migration and moment transients remain visible in private full-resolution
plots. One pad's near-zero normal report is not proof of separation; no continuous
bilateral-symmetry gate was added. This short pass does not establish sustained
long-range retention. No 90°/150 mm run, new preload A/B or four independent
repetitions was launched.

## Resource and protocol changes

The explicit engineering-only `recovery_gate_debug_v1` profile rejects conflicting
values before episode creation. Actual job and prelaunch hashes bind 3600 s wall,
1800 s simulation, up to 600 s unloaded Agent wait and 600 s wall / 120 s simulation exit
reserve. Normal work therefore has at most 3000 s wall / 1680 s simulation. An absolute
deadline includes initialization; the outer launcher's bounded export/shutdown
grace does not authorize extra manipulation. Existing worker limits and all
physical owners are unchanged.

`AGENT_SESSION_STATUS.json` distinguishes WAITING, ACCEPTING, RUNNING, COMPLETED,
FINALIZING and CLOSED, with sequence, station/observation generation, remaining
work and exit reserve. Unloaded five-second heartbeats neither reset deadlines
nor step/freeze a loaded interaction. Publishers reject busy/stale/closed states;
atomic publication still requires actual ACK and completion. Recovery admission
uses measured same-episode observation/planning/acquired-operation costs; these
are estimates, not physical guarantees. The measured recovery estimate was 601 s
against about 1700 s remaining work before relocation, with exit reserve excluded.

The longest completed unloaded decision took 239.893 s, exceeding the old 180 s
window and succeeding within the new finite window. Total actor wall 2555.687 s
(42.59min); essential evidence66,886,092bytes before the small integrity ledger.
Agent waiting remains material, so this is not an efficiency PASS. All raw RGB-D,
240Hz motion/F/T/contact data and feedback images remain private on
`test-X640-G50`, under the episode's explicitly configured output root.
Private evidence-index SHA256:
`bbbb209f3ba8e71f6fe80144ac883d0d04b44686666061357374840af76bf8b1`.

Actual final directory size including the compact integrity ledger/derived
metrics is 67,003,405 bytes. Total completed-operation Agent waiting was 967.17 s;
planning 384.20 s; the two physical-operation blocks 861.94 s (including approach,
closure, original hold and exit). These are operation wall totals, not pure
contact-stepping times. The five-second status heartbeat is not a new command
or a way to extend the overall work deadline.

## Separate FIT / independent VALIDATION regression

**Separate short regression: PASS for physical completion and held-out prediction;
not a new blind parameter-accuracy result.**

The recovery gate preserved four qualified bidirectional FIT trajectories.
Both immediately following validation requests were `VALIDATION_DEFERRED` because
the latest asynchronous FIT packet was not yet published at the loaded boundary.
They are **not** completed validation. The actor released/parked rather than
waiting under load. A separately registered short regression uses the same frozen
behavior completed as follows. No data is stitched across these two scenes.

`v81_fit_validation_regression_01` acquired from fresh wrist geometry and
completed 18.046 mm pulling and 12.126 mm reverse travel. The first immediate
validation request was deferred, retained as such, and followed by safe parking.
The production worker then published 55 positive and 44 negative FIT bins from two
eligible trajectories. At the unloaded boundary the Agent inspected current
feedback, requested fresh wrist views, inspected them, regenerated an automatic
current-state grasp and approved a validation-only acquisition. No old pose was
replayed. The originally declared target/profile was unchanged.

The second acquisition passed bilateral contact and original 1 s hold. Independent
`VALIDATION_REGRESSION_FORWARD` physically completed 5.819→15.071mm (+9.252mm)
in 6.317 simulation seconds, then released and parked. The approximately 0.10 mm
coordinate change during acquisition is actual contact settling, not a reset.
The FIT model was sealed before execution; parameters were bitwise unchanged
after validation. Validation never entered fitting or model selection.

| Quantity | Result |
|---|---:|
| Complete physical validation |6.317s |
| Derivative-supported free-prediction interval |3.800s,913native samples,39regression bins |
| Effective mass, conditional1σ |1.57509 ±0.00301kg |
| Viscous damping, conditional1σ |5.60145 ±0.00581N·s/m |
| Constant resistance, conditional1σ |0.273532 ±0.0000188N |
| Scaled regressor condition |148.01 |
| Common-filter held-out effort RMSE |0.0000215N |
| Free position / velocity RMSE |0.0722mm /0.0286mm/s |
| Free final position error |0.1080mm |
| Unfiltered-input sensitivity: effort / position RMSE |0.00859N /0.0561mm |
| Retained measured FIT support |1.045–17.467mm |
| Final state |15.071mm; verified parking; pad loads0/0N; rc0 |
| Actor wall / essential episode storage |2077.704s (34.63min) /60,254,858bytes |

The selected constant model is the FIT-only choice with only two FIT trajectories;
position-dependent resistance is not established. Filtered and unfiltered-input sensitivity metrics are both reported and differ
substantially. The ideal virtual sensor is not a hardware-accuracy reproduction;
the small conditional covariance excludes compensation, selection and contact
systematics. No hidden authored parameter was used or revealed, so parameter
accuracy against authored/effective simulator truth is **N/A**, not claimed.
The complete physical trajectory is longer than the independently predicted
derivative-supported core. Forward integration uses one measured initial state
and held-out compensated wrist effort, without later state correction. These
short-range numbers do not validate full-stroke dynamics or long-range retention.

[Regression metrics](DEBUG_FIT_VALIDATION_METRICS.json) include the frozen model
seal, unchanged-parameter check, pre-first-acquisition validation declaration,
FIT eligibility, excitation and prediction details. Private evidence-index SHA256:
`1ea1802c8bede23731fcbaf8e8d2a1640959279a24781768c7e5731a975b0707`.
Both owned actors exited; the shared owned-actor lock was available afterward.
No other user's process was managed. Previous failures remain in their original
directories/reports. Full-range tests and all four repetitions remain stopped.

The regression's final directory including its integrity ledger is 60,373,763 bytes.
Completed-operation Agent waiting was 706.01 s, planning 300.22 s and physical
operation blocks 811.06 s. Its longest unloaded decision 198.59 s also exceeded the
old180s window without losing the scene. The production estimator completed its
final update in 1.383 s within the unchanged worker deadline. Actual command
ACK/completion counts were 12/12 for the recovery gate and 10/10 for the separate
regression. Both acquisitions' distinct `pregrasp`/`pregrasp_001` RGB-D
checkpoints are retained, not overwritten. Source/job/result/trace and private
integrity commitments are in [the final audit](DEBUG_GATE_FINAL_AUDIT.json).

Private evidence host: `test-X640-G50` (`labserver_inschool`). Episode roots:
`/data1/home/rangeryx/agentic_v81_private/episodes/v81_debug_relocation_gate_01`
and
`/data1/home/rangeryx/agentic_v81_private/episodes/v81_fit_validation_regression_01`.
Raw frames, native-rate telemetry and contact plots were not committed or copied
into a redundant raw archive. The small local copies used for actual Agent image
inspection remain private.

## Frozen baseline and limits

Local and server software suites:130 tests PASS. Frozen audits:606 physical sources,
71 revolute sources,22 external dependencies and956 release sources pass; loaded
V8.1 module hashes match prelaunch commitments. No object physics, colliders,
actuator authority, gripper mapping/drive caps, scoring, motion or hard safety
limits changed. This remains known-rail/state supervision, a simulated wrist
mount, ideal virtual F/T sensor and native XY carriage, not hardware deployment.
