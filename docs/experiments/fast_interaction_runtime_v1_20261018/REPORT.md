# Fast Isaac interaction runtime V1

Reference: `eb80bf3`. Label: **XARM7 / NATIVE_G1 / KNOWN_MODEL / APPROXIMATE_ACTUATOR_SIMULATION**.

**Outcome: short native runtime validation PASS.** The final paired development scenario is **2.244× faster** including initialization/exit. Real handle pulling, release, observed-panel contact, physical closing, separation and safe parking completed. This is not a large-range research result or hardware safety validation.

This is a separate engineering runtime, not a replacement of the archived research results. The 87 source hashes recorded by the previous implementation still match; no tracked baseline file was edited. Raw RGB-D, video, cooked geometry and native telemetry remain private.

## What changed

`interaction_runtime.runtime.Runtime.step` is the single execution owner. The new owner does not execute the old nested `step`/`run` wrappers or generate physical controllers by source substitution. Frozen perception, ranking, native G1 closure, Pink IK and current-door safe-exit planning are reused through explicit operations. The semantic input's obsolete pad ceiling and inherited job force overrides are removed before execution. The repository audit inventories 232 legacy guard sites in 4,706 tracked files and checks 24 selected delegate functions separately; old reference implementations intentionally remain reproducible.

One phase-aware policy owns motion and force monitoring. A warning is information; it does not throw or set a deferred stop. Sustained-force history survives phase and direction changes. Collision, joint, actuator and emergency stops remain authoritative, with no loaded latch clearing.

The policy file retains its pre-experiment candidate-status metadata so its tested hash remains unchanged. `RESULTS.json` records the subsequent evidence-bound short-test validation; neither is a hardware certification.

| Phase | Command TCP speed | Measured TCP stop | Command acceleration |
|---|---:|---:|---:|
| Free travel / parking | 80 mm/s | 150 mm/s | 250 mm/s² |
| Approach / separation | 20 mm/s | 50 mm/s | 80 mm/s² |
| Closure / release | 8 mm/s | 30 mm/s | 30 mm/s² |
| Loaded pulling / pushing | 12 mm/s | 30 mm/s | 40 mm/s² |

Translation and rotation have independent velocity/acceleration limits. Joint commands retain the native one-third speed bound, finite actuator authority and 0.05 rad margin. These are simulation operating envelopes, not hardware safety ratings. The new envelopes do not command the robot to move at their maximum throughout contact.

| Signal | Informational warning | Sustained stop, trailing 150 ms mean | Immediate emergency |
|---|---:|---:|---:|
| Per-pad positive normal load | 2 N | 4 N | 10 N |
| Gravity-compensated wrist force norm | 4 N | 8 N | 10 N |
| Gravity-compensated wrist torque norm | 0.3 N·m | 0.6 N·m | 1 N·m |

Pad readings are normal-only contact quantities, not complete interaction forces. The live guard conservatively includes downstream dynamic tool loading; 240 Hz raw wrench and body states are retained for separate segment-local inertia compensation. No extra grip force, changed G1 gain, friction, geometry or object assistance was used.

## Archived failures reproduced

| Archived event | Evidence and correction |
|---|---|
| Unloaded panel entry: 5.068036 mm/s | No positive object contact; actual tracking transient crossed the old universal speed stop. New approach envelope accepts it. |
| Centering → probe: 8.248790 mm/s measured | Independent FK showed **35.194251 mm/s commanded**: a real handover discontinuity. New measured-state/bias-preserving handover and bounded reference yield **0.125054 mm/s** on the same archived first command. This is offline controller replay, not a physical speed claim. |
| Closing: 2.000266 N | The recovery wrapper threw on the first soft warning before a sustained-overload criterion. Replay now returns informational warning, not termination. Transmission quality remains a separate issue. |

The measured-progress reference clock is retained for pull and push. Reversals share its signed implementation. Normal stopping decelerates joint commands rather than abruptly replacing a moving velocity target by zero.

## Runtime cost and speed

FAST_DEV keeps **240 Hz physics, native contact, F/T and motion samples**. It uses batched measured body poses, an algebraically identical single-tree FK traversal, exact-value G1 geometry caches, stationary-reference QP reuse, buffered gzip/pickle chunks, and atomic chunk publication. The FK comparison across 100 configurations had zero matrix difference and a 5.409× local calculation speedup.

FAST_DEV renders observations and short low-resolution event clips, with no default continuous high-resolution recording. Unused diagnostic camera render products are not initialized; the recorded diagnostic camera initializes on demand. Wrist RGB-D remains unchanged. EVALUATION retains continuous 1280×960 diagnostic video. Neither mode lowers the physics step or raw sensor rate. Logging exceptions are evidence issues rather than manipulation gates. Export occurs before Isaac's fast shutdown; the parent records process exit independently.

Matched short scenario: four native unloaded round trips between identical endpoints, same robot, gripper, cabinet, initial state and finite drives. Initialization and process exit are included.

| Runtime | Simulation time | Wall time | Simulation/wall |
|---|---:|---:|---:|
| Actual frozen owner, full original evidence | 8.800 s | 76.624 s | 0.115 |
| Final FAST_DEV, bounded faster time law | 5.867 s | 34.140 s | 0.172 |

**Overall wall improvement: 2.244×.** The simulation episode is 33.3% shorter; simulation throughput improves 1.496×. This is an equivalent endpoint scenario with a faster time law, not an identical-command timing comparison or a claimed 2.244× physics-engine speedup. Both execution clocks exclude the unchanged 240-step initialization settle; wall times include initialization and cleanup. Physics fidelity was not reduced. Initialization is now the largest fixed short-test cost.

The final free test's measured costs before final process shutdown: initialization 17.682 s; physics/contact callback 5.966 s; trajectory planning 3.573 s; measured states/F/T 1.883 s; actuation 1.415 s; collision/safety 0.911 s; export 0.238 s; main-thread evidence enqueue 0.005 s. Compression runs asynchronously and is not represented by enqueue time alone.

Timing variability is preserved: an earlier valid measurement was 84.648 → 39.399 s (2.148×); a subsequent pre-lazy-camera repeat took 45.359 s, with initialization alone taking 27.697 s. This exposed the remaining eager diagnostic-camera allocation and prompted its removal. Final measurements are one sequential pair on a shared GPU, not a confidence interval or a guaranteed cold-start latency.

The old full episode took 852.279 simulation seconds and 6,629.858 wall seconds. Its partial profile showed substantial rendering, duplicate telemetry and long approach/closure costs. Archived sensing+SAM3 took 22.642 and 17.579 s per observation; that inference path was not redesigned. We did **not** rerun a full research episode to estimate a full-task speedup. The short contact fixture reuses measured RGB-D surfaces and automatically recomputes the grasp, avoiding a new expensive SAM3/Agent survey; its 130.084 s offline planning cost is reported separately, not hidden inside a claimed end-to-end perception speedup.

## Safety and protocol regressions

Deterministic tests cover every mapped phase, historical numbers, independent translation/rotation acceleration bounds, sustained overload across mode changes, collision/joint/velocity/effort/emergency faults, 513-record lossless export, 200 concurrent command publications, eight malformed messages and producer/consumer ordering. A malformed command was also acknowledged recoverably between real operations in the native contact test.

Actual native robot-only tests rejected an out-of-limit joint command, excessive velocity command and a genuinely colliding route **before physics advanced**. Native finite effort readback remained [50, 50, 30, 30, 30, 20, 20] N·m; G1 master cap remained 2.645649 N·m. These are actuator limits, not measured motor output.

An independent **robot-tool-only** 4.2 N calibration load produced a 4.397 N measured wrist peak and continued for 48 steps. A separate 11 N tool load triggered `EMERGENCY_WRIST_FORCE` at 10.075 N after six steps (25 ms). No load was applied to the object, and these samples are excluded from manipulation/identification evidence. Sustained-window stopping was tested deterministically; it is not mislabeled as an additional physical overload experiment.

After lazy diagnostic rendering, the identical native guard test retained 198/198 samples at 240 Hz, with **zero difference** in joint position/velocity, TCP matrix, raw wrist wrench and compensated wrench. Native contact records also matched. The six-frame diagnostic event clip decoded correctly. This is an actual same-excitation physical/sensor check, separate from the faster-time-law performance comparison.

## Final native contact chain

`contact_05` starts normally closed, recomputes the grasp from measured wrist geometry, uses fresh measured-state pregrasp RGB-D correction, and completes the following in one scene:

| Operation | Actual result | Operation wall time |
|---|---|---:|
| Approach → bilateral grasp → 1 s hold → short pull → release/park | +1.015738°; acquired loads 0.518 / 0.501 N | 307.791 s |
| Actual wrist motion and fresh RGB-D | Codex inspected RGB/depth and chose a visible panel patch | 30.680 s |
| Current-state complete push-route validation | Entry/contact/separation/parking feasible | 0.820 s |
| Native compact-G1 panel push → separation → park | 189 positive contact-point reports; 0.642968° closing | 122.219 s |
| Finish | Already verified unloaded parking | 0.012 s |

Final angle **0.384869°**, final pad loads **0 / 0 N**. Maximum measured TCP: **80.354 mm/s** free, **7.543 mm/s** pulling and **10.107 mm/s** pushing. Peak pad normal load **1.332 N**; peak gravity-compensated wrist force **1.424 N**; minimum joint margin **0.21730 rad**. Maximum handle-relative translation during opening **0.366 mm**. There were no original collision/force/joint/contact-loss stops in this final chain. Panel contact is evidenced by native contact identities and wrist wrench; handle-pad loads are not substituted for panel load.

Execution: **179.346 simulation seconds**, plus the common one-second initialization settle; **535.613 wall seconds including initialization/exit and Agent inspection waits**. This contains two useful physical interaction segments, averaging 267.807 wall seconds each when all episode overhead is included. It is not a matched full-episode speed comparison against the longer old research action. 43,043 native records were exported losslessly, all chunk hashes verified, no export issues. The 275-frame event video decodes at 640×480; its 18.333 s encoded duration represents disjoint event clips, not continuous coverage.

Measured contact-chain costs: physics/contact callbacks 199.358 s; actuation 74.869 s; state/F/T 79.404 s; collision/safety 28.437 s; trajectory planning 9.443 s; Pink 1.295 s; command collision queries 2.191 s; rendering/video 11.146 s; export 0.347 s. Unattributed time includes Agent waits and calls into frozen observation/refinement helpers; it is not all mislabeled as physics. The unchanged slow closure alone used 69.217 simulation seconds. The remaining dominant costs are native stepping, drive/state acquisition and this deliberately preserved closure duration.

## Preserved engineering failures

The first contact regression acquired bilateral contact, held for one second and safely released/parked, but stopped its finite pull at 0.223°: migration had replaced the validated progress clock with a fixed lead. That shared software regression was corrected.

The next acquired/held, physically reached 1.001° and safely parked. Fresh wrist RGB-D supported an observed panel route. Native panel contact then stopped under `PANEL_PUSH_NOT_ON_EXISTING_DISTAL_CAP`; it is preserved as a failed contact regression. The contact points belonged to the **pre-integration** finger tip, but the checker used the post-step pose, shifted approximately 24 µm. Raw/cooked cap geometry agreed. Comparing with the measured pre-step frame accepts the real tip contacts while still rejecting a root point 10 mm away, with identical geometry and tolerance. This was a timestamp association repair, not a collision waiver.

An initialization-local capture error, an expired legacy comparison deadline and an unloaded Agent timeout are separately retained as infrastructure-invalid runs, with no claimed physical result.

`contact_04` completed physical pulling/pushing/parking but failed the subsequent command-continuity audit: normal panel exit zeroed velocity before separation, giving **6.554 rad/s²**. The shared normal-stop primitive now decelerates while the explicit panel contact remains authorized; safety exceptions still terminate immediately. The relevant `contact_05` repeat reduced the separation handover to **0.205 rad/s²**, with the push/deceleration phase bounded by **0.8 rad/s²**. Recoverable attempt results likewise retain the last velocity for guarded release instead of erasing it first; this path was exercised offline. No gains, forces or contact thresholds were changed for these repairs.

## Use and scope

Run deterministic checks with `PYTHONPATH=.:scripts python scripts/verify_fast_runtime_v1.py --output PRIVATE_OUTPUT`, then audit with `scripts/audit_fast_runtime_v1.py`. The native entry is `scripts/run_fast_interaction_v1.py --job PRIVATE_JOB --mode FAST_DEV --test free|guards|fixture|session`; use EVALUATION explicitly when continuous video is required. The bounded launcher serializes **our actors only**, admits shared GPUs, and exports on owner-requested termination.

The reusable session retains Agent operations for observation, evaluated wrist motion, geometry building, plan approval, bounded interaction, feedback inspection, unloaded recovery and parking. It was not validated here by launching another complete active-perception research episode. Cached engineering observation input is explicitly labeled; fresh pregrasp refinement and the panel observation use actual wrist RGB-D. Known hinge/state, registered station, native safety geometry, simulated wrist mount, ideal load cell and approximate actuators remain dependencies. The final diagnostic-render-only revision was physically checked with identical native guard excitation; all seven manipulation/control methods exercised in `contact_05` are AST-identical to the final source.

Final source/config hashes, aggregate results and private evidence references are in `SOURCE_PROVENANCE.json`, `RESULTS.json`, `OFFLINE_CHECKS.json` and `PRIVATE_EVIDENCE_INDEX.json`. All 14 final execution source/config hashes match the launched snapshots; all 87 frozen baseline hashes still match. No registered owned process remains, including stale/zombie PIDs. No long-range opening, dynamics-identification accuracy or hardware-deployment claim follows from these short engineering tests. **No further physical episode was launched after these checks.**
