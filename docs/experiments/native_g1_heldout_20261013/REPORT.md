# Frozen native G1 held-out doors — first-six evaluation

Reference `a022bfe`. **ORACLE_HANDLE / KNOWN_MODEL / HELD_OUT_EVALUATION / XARM7_NATIVE_G1 / APPROXIMATE_ACTUATOR_SIMULATION**.

The first six pre-registered cases have completed one valid closed-start physical attempt each. Two pre-physics initialization failures were repaired and retried under the user's explicit protocol amendment. No completed physical attempt was repeated. The last four remain reserved.

## Registration and frozen execution

[Ten-case manifest](REGISTERED_OBJECTS.json) was published at `b7f9039` before IK or physical outcomes. Selection used metadata and closed geometry; no failed object was replaced. All ten received the same frozen geometry-driven planner and nominal 300 s budget: **10/10 executable candidates, 9/10 selected plans covering 30°**. The 45717 selected plan ends at 27° with `NO_CONTINUOUS_PINK_IK`, but it remained in the evaluation. 45916's final in-progress planning operation exceeded the budget argument: actual 312.35 s, reported rather than hidden.

All ten generated execution bodies have SHA256 `9c4132d61f88487e9282f2ee324b82f017c4f60b633bd633fe84cafa72140165`. Loaded entry SHA256 `c08fb22edcf6cffdddef32c3ddaf37a8123f6f7f59025d5e785e8305b97065e9`; native configuration SHA256 `68fb6af5f4efd432b2355cabe37876a67f6c5ea3eac0b224603f08fb18d62939`. Source hashes were checked before launch and in evidence preservation. Scoring, automatically chosen grasp/base plans, closure/preload, Pink control, timing, gains, effort, joint/force/contact/collision protections and object physics were unchanged. No saved success pose or per-object controller was used.

## Actual physical outcomes

| Order | Asset / handle | Geometry | Planned | Valid correct-handle grasp | Max actual after valid grasp | ≥30° | First limiting event |
|---:|---|---|---:|---|---:|---|---|
| 1 | 45526 / original-30 | vertical straight round bar | 30° | Yes | 30.000412° | PASS | No recorded physical divergence |
| 2 | 45638 / original-48 | vertical short curved bow | 30° | Yes | 30.000170° | PASS | No recorded physical divergence |
| 3 | 45667 / original-32 | horizontal straight round bar, low mounting | 30° | Yes | 30.000259° | PASS | No recorded physical divergence |
| 4 | 45623 / original-33 | vertical long curved bow | 30° | No | — | FAIL | RuntimeError('BILATERAL_HOLD_NOT_ESTABLISHED') |
| 5 | 45717 / original-35 | horizontal short bow, low mounting | 27° | No | — | FAIL | RuntimeError('BILATERAL_HOLD_NOT_ESTABLISHED') |
| 6 | 45850 / original-41 | vertical short straight bar, low cabinet | 30° | Yes | 30.000088° | PASS | No recorded physical divergence |

**30° physical task success: 4/6 (66.7%); valid physical grasp: 4/6.** All first-six planning cases had executable candidates. The six-case result is an interim held-out evaluation, not a robust population success-rate estimate; do not pool it with development cases or call it a ten-case result.

Failed closures reached only 0.002695° (45623) and 0.002129° (45717) before any valid grasp; these tiny excursions are not successful task progress. No release, regrasp, base motion after initialization, attachment, artificial object force/joint actuation, manual continuation, scene restart or stitching occurred in the valid episodes. All started normally closed.

## Contact, tracking and timing

| Asset | Peak pad load N | Max acquired-reference drift mm | Max loaded TCP mm/s | All-phase TCP mm/s | Min margin rad | Simulation s | Wall s | 240 Hz records |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 45526 | 1.073 | 2.184 | 2.473 | 20.746 | 0.543463 | 342.133 | 1952.503 | 82112 |
| 45638 | 0.819 | 0.214 | 0.866 | 26.208 | 1.088246 | 657.308 | 3737.989 | 157754 |
| 45667 | 0.914 | 2.456 | 1.651 | 24.994 | 0.389237 | 472.017 | 3015.054 | 113284 |
| 45623 | 0.658 | — | — | 28.436 | 0.627601 | 352.221 | 2088.673 | 84533 |
| 45717 | 0.211 | — | — | 19.357 | 0.661408 | 180.529 | 1123.670 | 43327 |
| 45850 | 1.155 | 1.956 | 2.601 | 17.257 | 0.706283 | 622.242 | 3841.789 | 149338 |

Loaded velocity uses actual world-frame TCP position differences at 240 Hz; the existing 5 mm/s loaded protection remains unchanged. Higher all-phase peaks occurred before loaded grasp. Missing acquired-grasp drift/loaded metrics for closure failures are unavailable, not zero. No recorded force-window warning/overload or both-filtered-pad unloading interval occurred after successful grasp. 45850 used the already-supported local centering correction once (0.125 s); the other successful cases did not. Acquisition L/R pad loads were 45526: 0.506/0.516 N, 45638: 0.481/0.528 N, 45667: 0.475/0.541 N, 45850: 0.494/0.519 N. Acquisition angles were respectively approximately 0°, 0.053100°, 0.060756°, 0.044945°; these tiny closure excursions are recorded and are not substituted for manipulation progress. Native joint/contact data, reference, sensor metadata, events and continuous video were retained; telemetry completeness remained passive. The frozen contact interface records normal point impulses, not friction anchors, and does not measure calibrated hardware total forces.

Total valid-episode simulation time **2626.450 s**; reported runner wall time **15759.679 s** (excluding prior initialization failures, offline planning and resource waiting).

## Failure patterns and infrastructure repairs

The physical failures did not establish the required bilateral hold: their recorded centering interval retained unilateral contact and nearly stationary doors. The planner predicted usable arm motion, which did not guarantee usable contact. Bow-handle visual rear gaps can differ from convex-hull collision gaps (recorded zero gaps); this is a representation/candidate concern, not a proven sole cause. 45638 succeeded despite the same zero-gap diagnostic, so this measurement alone cannot explain or certify physical infeasibility. The batch made no collision or candidate changes to chase outcomes. Single-pad unloading during successful opening is separately preserved and is not relabeled as task failure.

Two initial scene failures expected `l_1` under moving bodies that actually contained `l_0` (45526) and `l_2` (45638). The old example preparer's default link was wrong. Only `asset/manifest.json:door_link` was corrected from the authored moving visual descendant/registered handle link. Original malformed manifests, errors, reservations and exits remain under `initialization_01`; **zero physical manipulation attempts were consumed by them**. Four previously completed physical episodes were left untouched. The input integration was amended, so this is not described as an entirely unchanged-input launch batch. See [amendment](INPUT_INTEGRATION_AMENDMENT.md).

GPU scheduling now uses actual free VRAM, allows other users' jobs and imposes no idle/exclusive-GPU requirement. The observed actor used about 3.3 GB on its primary GPU plus small graphics contexts on others. Our own actors ran sequentially; no unrelated process was terminated. Finished actors exited and released resources. GPU/launch/input corrections did not change manipulation behavior or physical protections.

## Reserved objects and development comparison

| Asset / handle | Geometry | Selected plan | Physical status |
|---|---|---:|---|
| 45671 / original-35 | horizontal long curved bow, low mounting | 30° | RESERVED_NOT_EXECUTED |
| 45783 / original-49 | horizontal short bow, high mounting | 30° | RESERVED_NOT_EXECUTED |
| 45908 / original-37 | vertical long straight round bar | 30° | RESERVED_NOT_EXECUTED |
| 45916 / original-46 | vertical short curved bow, opposite-side handle | 30° | RESERVED_NOT_EXECUTED |

Development results remain separate: 45134 previously 30.000061° (physical plan predates the scoring repair; offline regression compatible), 45443 30.000179°, 45249 30.000196°. Frozen development and prior PiPER benchmarks were not overwritten.

GT registered handle mesh, articulation geometry, dynamic moving-body/joint feedback and collision models remain dependencies. This is no visual perception, unknown-axis, physics identification or hardware-equivalent result. The native G1 geometry and mount follow the official model; actuator control remains approximate functional simulation, without hardware servo calibration. The frozen runner's inherited report classification says development; the pre-registered manifest and this report identify these new cases as held-out evaluation without editing original evidence.

## Evidence and reproducibility

[Compact case metrics](CASE_METRICS.json), [interim results](INTERIM_RESULTS.json), [six-case evidence index](SIX_CASE_EVIDENCE_INDEX.json), [offline snapshot](OFFLINE_EVIDENCE_INDEX.json), [old completed-three snapshot](COMPLETED_EVIDENCE_INDEX.json). Historical snapshots remain intact; their old initialization-attempt accounting is superseded explicitly by the user-authorized amendment.

Private server root: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_heldout_20261013`. Per-case `physical_01/contact_baseline.mp4`, `observations_chunks`, `physics_steps_chunks`, `sensor_metadata.json`, contact collider mappings, source provenance and reports are indexed. Continuous videos were fully decoded; observation/native physics record counts were checked against every saved chunk. No optional telemetry field was made an execution gate.

Private six-case archive: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_heldout_six_cases_20261013.zip`; 1590028256 bytes; SHA256 `717801e3f7cc6518eca60ae6d2af8e5c1a721d303380d9f9154f29daacae7ea6`. Raw telemetry/assets/videos are private; GitHub receives only code, compact reports and hash/path indexes. No public Release or raw/video upload. The round stops after these six cases.
