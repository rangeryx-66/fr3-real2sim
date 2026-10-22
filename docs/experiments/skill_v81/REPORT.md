# V8.1 engineering progress — NOT PASS

The 90° revolute and ≥150 mm prismatic engineering lifecycles are not yet complete. No independent repetition has started; the launch gate remains closed. V8.0 (`2f021639`) and its 71 frozen revolute source/configuration files plus 22 external dependencies remain unchanged. All earlier outcomes are preserved.

## Recorded attempts

| Scene | Actual outcome | Acquisition / validation / exit |
|---|---|---|
| Revolute engineering 00 | Installed native speed API mismatch during initialization; no grasp | Infrastructure failure retained; indexed setter repaired without changing arm/G1 caps |
| Revolute engineering 01, source `d4d63b2` | Maximum actual opening 11.51°; largest single closing 6.97° | Two acquired grasps and original 1 s holds; three qualified FIT actions; one complete independent VALIDATION; verified release/parking |
| Prismatic engineering 00, source `cecd2b5` | Two fresh wrist observations, then unloaded Agent-command timeout | **Zero physical acquisitions**, no FIT/VALIDATION; guarded parking and process exit succeeded. Orchestration interruption, not evidence of failed drawer traction |

Revolute engineering 01 used one continuous scene. Native actuators moved the base from approximately `(0.1961, -0.5328)` m to `(0.3000, -0.8200)` m after release/parking, with a measured endpoint error of `5.38e-8` m. The door was not reset. Old observations and executable routes were invalidated; fresh wrist geometry supplied the next plan. These are simulated XY-carriage results, not wheeled-base or hardware validation.

The original upper-bar grasp's proposed expansion was rejected before motion for a link3/cabinet collision. Subsequent full-arc searches selected paths ending around 45° at the original station and 65° at the relocated station before continuous IK failed. **Those are planning results, not physically achieved angles.** The finite scene ended without starting a further acquisition that could not credibly finish within its remaining work budget and exit reserve.

## Identification evidence

The first three FIT actions all reached their measured targets. Production processing retained 181 positive-velocity and 97 negative-velocity regression bins. Before independent validation, the online model was sealed. Its inertia and damping estimates were `0.129131 ± 0.001368 kg m²` and `0.235452 ± 0.003866 N m s/rad` (conditional standard deviations, excluding systematic/contact/model-selection error). Authored references have not been revealed for this report.

Independent reverse VALIDATION physically completed in **8.008 s**. Sealed-model forward prediction retained **6.996 s**, initialized once from measured state, with no later state correction: angle RMSE **0.0613°**, angular-velocity RMSE **0.000555 rad/s**, effort RMSE **0.178 mN m**. It remained within the small-angle FIT support. This does not validate 90° dynamics or establish authored-parameter accuracy. Full derived numbers and the private evidence-index commitment are in [R01_METRICS.json](R01_METRICS.json).

## Shared engineering corrections after that scene

* Keep the same strict station limits, but require a finite, continuously measured unloaded settling interval. Original parking position tolerance alone did not guarantee low joint velocity: the nearest retained sample to the first station refusal had `0.0387 rad/s` maximum arm velocity versus the new station condition's `0.01 rad/s`. Exact refusal flags were not captured in the old actor; new sidecars expose every flag and measurement. No hard stop or force history is cleared.
* Separate the Agent's long planning horizon from its finite executable probe. A short first FIT need not hide a later workspace limitation. The horizon does not command physical motion.
* Forecast the current **observed automatic candidate's** task arc at proposed stations before moving. Use the original solver and native collision safety representation. This is not a new grasp, does not establish that all alternatives are impossible, and cannot approve a future approach: fresh wrist reobservation, grasp generation and full timed validation remain mandatory after moving.
* Reuse the existing cached robot FK inside the isolated planner scope. Across 128 random states, both aperture and measured-finger modes gave exactly identical link/pad transforms; robot-only FK timing improved 5.53×. This is not a measured whole-episode speedup.
* Publish production-qualified signed FIT support and seal the validation model. No loaded worker/Agent wait is introduced. Safely interrupted FIT eligibility remains owned by the frozen production rules; incomplete VALIDATION is never admitted as complete.

The new unloaded native 20 mm outward/return check passed in **34.47 s wall time**, with zero acquisitions/object commands and TCP/FK translation error **0.401 µm**. Original arm effort caps remained `[50,50,30,30,30,20,20]`. This checks the interface and strict settling, not full contact manipulation. Software regressions currently pass 42 V8.1 tests; frozen V8 owners are not edited.

## Efficiency and remaining work

Revolute engineering 01 consumed **4511.9 s wall / 425.2 s simulation**, including **2205.1 s unloaded Agent waiting** and **909.6 s grasp planning**. Its final episode footprint was **93.90 MB** before the private integrity index. This is not an efficient full-range result. Development analysis must finish before live launches; live windows should be used for concise observation/action decisions. The prismatic interruption is retained as an explicit orchestration failure.

Next: complete both engineering lifecycles with fresh observations and the repaired interfaces, freeze exact behavioral sources, then execute the four preregistered fresh repetitions without replacing valid failures. Current limitations remain known articulation/state, development objects and unchanged engineering dynamics, ideal virtual wrist F/T, approximate actuators, simulated wrist mounting and XY-stage mobility. Neither passive geometry camera is an online input. No full benchmark or hardware deployment is enabled.
