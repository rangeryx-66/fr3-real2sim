# Benchmark A — 10 standard hinged doors / actual 30.0 degrees

**KNOWN_MODEL_DIAGNOSTIC. Task success: 1/10 (10%). Planning coverage: 6/10 (60%). Initial physical grasp confirmation: 5/10 (50%), or 5/6 attempted cases (83.33%).**

All ten IDs were selected by geometry and visual inspection and registered before any IK planner ran. No failed object was replaced. All ten went through offline planning; each of the six planning-feasible cases received exactly one independent normal-closed-start physical attempt. No official attempt was repeated or manually continued.

SUCCESS requires valid physical grasp, actual joint angle at least 30.0 degrees during subsequent opening, no intentional release/regrasp, and no protection-triggered failure. Actual joint readings determine the score; predicted/commanded angles and unsafe pre-grasp motion do not. No tolerance below 30 degrees is accepted. At the first safe threshold crossing the scene is paused/exported; there is no additional endpoint-validation requirement.

| Asset ID | Handle Type | Planning | Grasp | Max Actual Angle | ≥30° Success | Failure Reason |
|---|---|---|---|---|---|---|
| 45087 | vertical round bar | yes / partial 12 deg arc | yes | 2.01904° | NO | finger-back/root contact protection |
| 45176 | vertical curved bow | yes / 30 deg arc | yes | 0.35646° | NO | 5 mm/s TCP speed protection |
| 45203 | horizontal bow pull | yes / 30 deg arc | yes | 0.82186° | NO | contact loss after ineffective continuation |
| 45212 | horizontal round bar | no | not attempted | — | NO | offline: IK-solvable candidates still collided |
| 45244 | vertical curved bow | yes / 30 deg arc | yes | 30.00069° | YES | PASS |
| 45267 | horizontal round bar | no | not attempted | — | NO | offline: no initial Pink IK in bounded family |
| 45297 | vertical round bar | yes / partial 12 deg arc | no | — | NO | initial bilateral hold not established |
| 45384 | vertical short bow | no | not attempted | — | NO | offline: IK/collision/initial approach search exhausted |
| 45415 | bottom horizontal bow pull | yes / partial 27 deg arc | yes | 28.99028° | NO | existing 0.05 rad command joint-margin boundary |
| 45448 | horizontal round bar | no | not attempted | — | NO | offline: no initial Pink IK in bounded family |

Maximum angle in this table is measured during opening after the existing valid-grasp event, including the termination sample. The termination reason still governs whether that sample was safe; an unsafe threshold crossing is not PASS. “—” means no valid post-grasp opening measurement, not a fabricated physical zero. 45297's raw closure excursion was 0.04812° and is not credited.

45244 crossed the strict threshold at **30.0006885529°** and stopped successfully. 45415 reached **28.9902782440°**, then the next command reached the existing margin boundary: this is **FAIL**, with no rounding or relaxed margin. Its final actual margin was about 0.050094 rad.

## Actual grasp region: important qualification

The only task PASS, 45244, automatically selected **`original-34`**, a door trim/frame region. The visually registered protruding handle is **`original-38`**. The actor transmitted real opening motion through allowed distal-finger contact on the selected region; it did not attach or actuate the object. The stated valid-grasp/30-degree task criterion is satisfied, but this result **does not demonstrate correct handle selection**. A handle-specific ≥30° success rate would be **0/10** in this batch.

The existing generator ranks general geometric regions from the moving link. Visual eligibility metadata was used to select assets, not to hand-correct the automatically generated poses or replace its rankings. This distinction is preserved instead of hiding candidate selection mistakes.

| Asset | Registered handle mesh | Automatically executed grasp mesh |
|---|---|---|
| 45087 | original-27 | original-27 |
| 45176 | original-46 | original-39 |
| 45203 | original-53 | original-52 |
| 45244 | original-38 | original-34 |
| 45297 | original-27 | original-27 |
| 45415 | original-40 | original-40 |

## Physical timing and first divergence

| Asset | Simulation s | Physical wall s | Peak pad loads L/R N | First divergence / stopping evidence |
|---|---|---|---|---|
| 45087 | 121.667 | 632.619 | 1.031 / 1.101 | t=121.662s, finger-root contact; no earlier logged tracking discrepancy |
| 45176 | 53.733 | 306.598 | 0.625 / 2.144 | t=53.729s, actual TCP speed 5.178 mm/s >5 |
| 45203 | 717.492 | 3180.013 | 2.484 / 2.098 | t=173.867s, reference/object error >2° logged diagnostically; continued until contact loss |
| 45244 | 465.521 | 2082.750 | 0.592 / 1.126 | safe 30° threshold crossing; no protection event |
| 45297 | 105.133 | 581.468 | 0.592 / 0.126 | existing closure time window ended without valid bilateral hold |
| 45415 | 371.283 | 1594.196 | 0.588 / 1.100 | next command crossed original joint-margin boundary; no earlier logged tracking discrepancy |

Physical simulation time sums to **1834.829 s**; physical wall times sum to **8377.644 s** because scenes ran concurrently. The observed planning-plus-physical batch lasted **5462.849 s (91.05 min)**. This excludes pre-registration geometry preparation/visual inspection; per-stage process timings are preserved. Per-case wall time includes scene initialization and evidence export.

Failure categories among the nine failures: **planning 4; collision/contact placement 1; speed protection 1; contact loss 1; initial grasp failure 1; IK/joint margin 1**. No failure was reclassified as a newly discovered mechanical limit.

45203 illustrates the separation of diagnostics and execution: reference lag >2° was recorded at t=173.867 s, and the same scene kept running until t≈717.49 s. Relative motion and one-pad zero readings did not cause a new diagnostic stop; its eventual stop was the original physical contact-loss logic. Raw single pad-load peaks above 2 N were preserved and did not become new single-sample overload aborts. 45176 stopped because of TCP speed (5.178 mm/s), not its 2.144 N raw peak.

## Selection and unchanged pipeline

The local full archive contains 2024 assets. Before IK, source type/range screening found 96 conventional single-revolute vertical-hinge cabinet/door candidates. The first 24 new cabinet candidates excluding named development/prior benchmark IDs were prepared and visually inspected. These ten were chosen for clearly protruding horizontal/vertical straight/bow handles, left/right hinges, and nominal heights 0.6–0.9 m. Registered visual handle dimensions and central rear clearance are in `REGISTERED_OBJECTS.json`. No planner, predicted range, or official outcome was used for eligibility. The pre-test contact sheet and registration were published in **0fe0694** before planning.

All selected IDs are distinct and exclude 7320, 41529, 45007 and 7138, as well as every ID in the original 0/10 batch. `DISCOVERY_GEOMETRY_RANKING.json` and visual-inspection images preserve the selection evidence. This is a deliberately defined standard-door sample, not a random whole-dataset estimate.

The original Pinocchio 3.9.0/Pink 3.3.0/DAQP solver, geometric grasp generator, base x/y/yaw family, wrist/IK alternatives, physical approach/closure/contact rules, and continuous opening controller were reused unchanged. New files are task adapters: planning goal 30° rather than authored full range; report label; pause at the requested first safe ≥30° sample; retain result scenes until explicit owner cleanup. These changes were configured and published before any official physical attempt. No pose, gain, threshold, time scaling or controller was changed after observing outcomes.

Planning budget was 300 s per case, with the same candidate ordering/search. Planning coverage means an executable initial approach/grasp and an initial nonempty motion arc; full 30° prediction was not a new admission gate. Three feasible cases predicted full 30°, while 45087/45297 predicted 12° and 45415 predicted 27°. Every feasible case was physically attempted and allowed to continue toward the actual task target.

The cabinet and robot base were fixed in every official episode. The initial base was initialized normally at the optimized station; physical navigation to it was not demonstrated. There were zero intentional releases, zero regrasps, no attachments, no mid-manipulation object-state commands, no artificial object forces, no scene restarts or stitching, and no manual continuation.

## Asset physics and preservation limits

New source archive URDFs contain no collision tags. All discovery assets used the **existing unchanged importer**: source visual geometry at authored metadata scale, per-visual convex hull collision approximation, and its existing uniform-box COM/inertia approximation. Source mass, joint limits and friction settings were preserved. This is an explicit simulation preparation approximation, not calibrated real object physics. It was frozen before registration/planning and was never changed per object to make an outcome pass. No new bar/support proxy was generated.

Existing actuator/clamp effort, controller gains, contact rules, temporal force policy, 0.05 rad joint margin, joint motion limits and 5 mm/s world TCP protection stayed unchanged. Known hinge geometry, moving-body pose, articulation progress and collision geometry remain GT inputs; this is not estimated-model, visual-grasp, autonomous reconstruction or hardware execution.

Final preservation checks cover **1491 frozen baseline/old-benchmark files**, with no changes, and unchanged registered source asset URDF/manifest hashes. Completed/failed scenes were paused and exported before explicit owner cleanup. All owned discovery/setup/planning/physical actors were closed; no active recovery scene remains.

## Evidence and reproducibility

Each official case preserves a continuous MP4, 240 Hz lossless observation and native physics chunks, raw contact impulses/points/normals/collider IDs, raw/filtered loads, finger/TCP/joint state and effort/reaction getters, actual object state and reference, phases/events, metadata and source/config/plan hashes. Optional telemetry remained passive and was not used to decide PASS. Reports/configurations/metadata/source hashes are published; raw media and telemetry are retained locally/on the server.

Server: `results/benchmarkA_30deg_20261011`. Local: `engineering_artifacts_20261011/benchmarkA_30deg/results`. Static registration/geometry images are separate from official execution evidence. `archive_verification.json` and `evidence_hashes.json` identify and verify the retained raw files. Reproduction requires the listed source assets, preserved preparations, shared robot/calibration files, and the recorded job/configuration; a new output/registration must be used rather than reusing an official attempt reservation.

Recorded dataset attribution: [Caoza/PhysX-Mobility](https://huggingface.co/datasets/Caoza/PhysX-Mobility); prepared manifests record CC-BY-NC-4.0.
