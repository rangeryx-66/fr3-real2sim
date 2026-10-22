# V8.1 engineering progress — NOT PASS

The 90° revolute and ≥150 mm prismatic engineering lifecycles are not yet complete. No independent repetition has started; the launch gate remains closed. V8.0 (`2f021639`) and its 71 frozen revolute source/configuration files plus 22 external dependencies remain unchanged. All earlier outcomes are preserved.

## Recorded attempts

| Scene | Actual outcome | Acquisition / validation / exit |
|---|---|---|
| Revolute engineering 00 | Installed native speed API mismatch during initialization; no grasp | Infrastructure failure retained; indexed setter repaired without changing arm/G1 caps |
| Revolute engineering 01, source `d4d63b2` | Maximum actual opening 11.51°; largest single closing 6.97° | Two acquired grasps and original 1 s holds; three qualified FIT actions; one complete independent VALIDATION; verified release/parking |
| Prismatic engineering 00, source `cecd2b5` | Two fresh wrist observations, then unloaded Agent-command timeout | **Zero physical acquisitions**, no FIT/VALIDATION; guarded parking and process exit succeeded. Orchestration interruption, not evidence of failed drawer traction |
| Prismatic engineering 01, source `cecd2b5` | Maximum actual pull **160.375 mm**; complete reverse FIT **83.415 mm** | Three acquired grasps/1 s holds and fresh same-scene recoveries; qualified bidirectional FIT; **no complete VALIDATION**; all guarded exits and final parking succeeded |

Revolute engineering 01 used one continuous scene. Native actuators moved the base from approximately `(0.1961, -0.5328)` m to `(0.3000, -0.8200)` m after release/parking, with a measured endpoint error of `5.38e-8` m. The door was not reset. Old observations and executable routes were invalidated; fresh wrist geometry supplied the next plan. These are simulated XY-carriage results, not wheeled-base or hardware validation.

The original upper-bar grasp's proposed expansion was rejected before motion for a link3/cabinet collision. Subsequent full-arc searches selected paths ending around 45° at the original station and 65° at the relocated station before continuous IK failed. **Those are planning results, not physically achieved angles.** The finite scene ended without starting a further acquisition that could not credibly finish within its remaining work budget and exit reserve.

## Identification evidence

The first three FIT actions all reached their measured targets. Production processing retained 181 positive-velocity and 97 negative-velocity regression bins. Before independent validation, the online model was sealed. Its inertia and damping estimates were `0.129131 ± 0.001368 kg m²` and `0.235452 ± 0.003866 N m s/rad` (conditional standard deviations, excluding systematic/contact/model-selection error). Authored references have not been revealed for this report.

Independent reverse VALIDATION physically completed in **8.008 s**. Sealed-model forward prediction retained **6.996 s**, initialized once from measured state, with no later state correction: angle RMSE **0.0613°**, angular-velocity RMSE **0.000555 rad/s**, effort RMSE **0.178 mN m**. It remained within the small-angle FIT support. This does not validate 90° dynamics or establish authored-parameter accuracy. Full derived numbers and the private evidence-index commitment are in [R01_METRICS.json](R01_METRICS.json).

## Prismatic engineering 01: recovery worked, lifecycle incomplete

The first acquisition pulled 55.56 mm before a persistent sliding stall. Fresh wrist observations and a different automatic candidate at the same 1.0 N requested preload reached 160.37 mm, but its reverse action stalled after 1.81 mm. Both failures remained recorded and both used guarded release/parking. Requested preload is not measured pad force.

At that measured open state, the rest-position wrist image contained the gripper/floor, not the handle. Codex inspected it, proposed a new view, accepted the nearer alternative after the farther proposal failed native IK, and inspected the new RGB/depth/protrusion-hypothesis images. SAM supplied no candidate there; the existing observed-depth hypothesis covered the visible bar. No threshold, GT crop, pose correction or camera teleportation was used.

The third automatic candidate used a 180° roll and the already validated V8 prismatic rail-orthogonal unloading branch, still at a 1.0 N request. It completed **83.415 mm reverse FIT**. The subsequent positive FIT safely stopped after 6.39 mm on `PREDICTED_LOAD_MARGIN_REQUIRES_UNLOAD`; it was not relabeled complete. Because grasp orientation and trajectory unloading changed together, this does not isolate which caused the improved reverse transmission.

The production worker retained 314 positive and 166 negative regression bins across the qualified FIT data. A fresh validation-only candidate search subsequently exhausted feasible alternatives under the current recovery conditions. Remaining work resources did not support another complete observation/plan/acquisition/validation, so Codex finished with the exit reserve intact. **Independent prediction and authored-parameter error are N/A**, not inferred from the FIT. Original physics remains unrevealed during ongoing V8.1 actions. See [P01_METRICS.json](P01_METRICS.json).

Wall time was **4976.3 s**, including engineering/Agent decisions and rejected plans. The compact episode occupied **87.74 MB** before its private integrity index; zero hard-stop records, three verified guarded exits and final parking were recorded. This is useful contact/recovery evidence, but neither an efficient complete lifecycle nor a V8.1 PASS. Revolute engineering 02 is a separate fresh engineering scene; no independent repetition has begun.

## Shared engineering corrections after revolute engineering 01

* Keep the same strict station limits, but require a finite, continuously measured unloaded settling interval. Original parking position tolerance alone did not guarantee low joint velocity: the nearest retained sample to the first station refusal had `0.0387 rad/s` maximum arm velocity versus the new station condition's `0.01 rad/s`. Exact refusal flags were not captured in the old actor; new sidecars expose every flag and measurement. No hard stop or force history is cleared.
* Separate the Agent's long planning horizon from its finite executable probe. A short first FIT need not hide a later workspace limitation. The horizon does not command physical motion.
* Forecast the current **observed automatic candidate's** task arc at proposed stations before moving. Use the original solver and native collision safety representation. This is not a new grasp, does not establish that all alternatives are impossible, and cannot approve a future approach: fresh wrist reobservation, grasp generation and full timed validation remain mandatory after moving.
* Reuse the existing cached robot FK inside the isolated planner scope. Across 128 random states, both aperture and measured-finger modes gave exactly identical link/pad transforms; robot-only FK timing improved 5.53×. This is not a measured whole-episode speedup.
* Publish production-qualified signed FIT support and seal the validation model. No loaded worker/Agent wait is introduced. Safely interrupted FIT eligibility remains owned by the frozen production rules; incomplete VALIDATION is never admitted as complete.

The new unloaded native 20 mm outward/return check passed in **34.47 s wall time**, with zero acquisitions/object commands and TCP/FK translation error **0.401 µm**. Original arm effort caps remained `[50,50,30,30,30,20,20]`. This checks the interface and strict settling, not full contact manipulation. Software regressions currently pass 42 V8.1 tests; frozen V8 owners are not edited.

## Efficiency and remaining work

Revolute engineering 01 consumed **4511.9 s wall / 425.2 s simulation**, including **2205.1 s unloaded Agent waiting** and **909.6 s grasp planning**. Its final episode footprint was **93.90 MB** before the private integrity index. This is not an efficient full-range result. Development analysis must finish before live launches; live windows should be used for concise observation/action decisions. The prismatic interruption is retained as an explicit orchestration failure.

Next: complete both engineering lifecycles with fresh observations and the repaired interfaces, freeze exact behavioral sources, then execute the four preregistered fresh repetitions without replacing valid failures. Current limitations remain known articulation/state, development objects and unchanged engineering dynamics, ideal virtual wrist F/T, approximate actuators, simulated wrist mounting and XY-stage mobility. Neither passive geometry camera is an online input. No full benchmark or hardware deployment is enabled.
