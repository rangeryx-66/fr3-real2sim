# 7320 continuous-contact coordination result

**KNOWN_MODEL_DIAGNOSTIC — one-grasp90° physical demonstration PASS, with same-scene operator recovery. This is not an uninterrupted automatic reproduction.**

A normal closed-start scene physically grasped the handle, opened to20.304°, moved the base29.999mm while retaining the grasp, and continued from20.005° to30.513° (+10.508°). The same grasp subsequently reached90.000009°. There were no intentional releases, regrasps, attachments, direct object forces, runtime object-joint writes, base teleports or scene stitching.

| Configuration | Maximum actual | Contact evidence / stop |
|---|---:|---|
| Previous fixed-base control |56.252°|Existing IK branch exhausted|
| Previous world-fixed TCP compensation |61.659°|About10.25mm relative drift;79.167ms positive-impulse gap|
| Previous direct body following |19.947°|Existing5mm/s protection triggered at5.003mm/s|
| Smooth coordinated experiment |90.000009°|No positive pad-handle impulse gap at240Hz; target reached and held|

| Evidence | Actual result |
|---|---:|
| Completed approximately30mm base adjustments |11|
| Interrupted partial movements |3 (14 physical movement attempts total)|
| Net base displacement |318.468mm|
| Sum of movement-segment endpoint displacements |338.634mm|
| Net world XY change |-311.818, 64.737mm|
| Maximum actual world TCP speed after verified grasp |4.830mm/s|
| Peak left / right reported pad loads |1.212 / 1.154N|
| Maximum whole-episode body-relative drift |4.481mm / 0.597°|
| Final relative drift |0.024mm / 0.0061°|
| Latest corrected segment maximum drift |0.428mm / 0.060°|
| Latest corrected segment maximum actual TCP speed |1.091mm/s|
| Minimum / final joint margin |0.049991656 / 0.058168rad|
| Simulation / wall duration |2244.796s / 13572.597s|
| Native physics and observation records |538751 each,240Hz|

The entire recorded grasp interval had a positive allowed pad-handle impulse in every sample, with zero both-pad raw-zero samples. This establishes contact continuity at the native240Hz resolution. Bilateral filtered load above the unchanged0.05N threshold existed for61.76% of the whole grasp interval; one-pad zero-load periods were preserved rather than relabeled as bilateral contact. The final corrected base-motion portion retained bilateral load throughout. These measurements are diagnostics, not execution gates.

Relative drift is computed against the one physically acquired body-to-TCP transform. It is a measured relative-motion proxy, not a claim of pure surface slip. The whole-episode4.481mm peak includes earlier correction attempts; it must not be hidden by reporting only the final0.428mm segment. TCP speed excludes the unloaded initial approach.

The practical fixes were smooth SE(3) references and monotonic lag-slowed opening progression, continuous IK with actual base feedback, station transitions based on the planned command-reference interval, and stopping both arm and base during holds while preserving jaw effort. Base reference peak speed changed from0.3 to1.5mm/s after the slow compensation was physically overtaken near a workspace boundary. Original arm/jaw gains, clamp effort, collision geometry, physical parameters and5mm/s /0.05rad protections remained unchanged. No new fatal validation or telemetry-quality gates were added.

The first reference variants oscillated or stalled near4.7°; run_01 is retained separately as a failed trial. In run_02 the first physical protective stop occurred at58.933° (joint5 margin). Later stops occurred near66.49°,69.70°,72.69° and87.08°. Existing protection always stopped advancement and preserved the actor. Local0.0002rad position commands moved the limiting joint away from its bound through real control; joint states were never set. One initial correction selected joint6 incorrectly, immediately stopped again under the original guard, and was corrected to joint5. Minimum margin0.049991656rad is the stopped-state evidence, not a relaxed threshold or a claim that the whole run stayed above0.05rad.

There were9 continuation dispatches:7 execution continuations,1 paused kinematic inspection and1 no-op caused by a file-delivery race. Subsequent commands were delivered atomically. All used actor4193624; no intermediate state was restored. The final integrated runner contains the corrections, but has not yet had a separate uninterrupted closed-start reproduction. Freeze the demonstration evidence; do not treat it as a newly validated automatic baseline.

| Base adjustment | Door before → after | World ΔX / ΔY (mm) | Measured joint margin start → end (rad) |
|---|---:|---:|---:|
|1|20.304 → 20.005°|-29.999 / -0.000|0.379936 → 0.372360|
|2|58.932 → 60.371°|-30.000 / -0.001|0.050046 → 0.261737|
|3|61.974 → 64.090°|-30.000 / -0.001|0.139869 → 0.249008|
|4|66.466 → 69.226°|-29.999 / -0.000|0.050245 → 0.089155|
|5|69.672 → 69.979°|-29.999 / -0.000|0.050250 → 0.292363|
|6|72.697 → 73.062°|-29.999 / -0.000|0.050249 → 0.266271|
|7|74.642 → 75.060°|-29.999 / -0.001|0.136385 → 0.298795|
|8|77.626 → 78.337°|-21.213 / 21.213|0.114952 → 0.303751|
|9|80.632 → 80.820°|-29.998 / -0.001|0.111077 → 0.286454|
|10|83.665 → 85.756°|-21.211 / 21.213|0.077263 → 0.153011|
|11|87.088 → 87.041°|-21.212 / 21.213|0.050250 → 0.293366|

All existing telemetry fields were non-null for538751 observations, including raw contacts/impulses, raw and filtered loads, joint position/velocity/effort/reaction, TCP/finger/base poses, articulation position/velocity, reference, aperture, effort command and control mode. Native records and observations are lossless gzip/pickle chunks. Sensor frames, contact-pair mapping, impulse-to-load conversion,19-sample equal-weight filtering, gain1 and bias0 remain archived. These are simulated contact reactions, not hardware-calibrated forces.

Remaining GT dependencies are known initial grasp geometry, hinge geometry, actual moving-body pose/articulation feedback and collision models. The base is the same physically driven XY carriage used by prior probes, not a wheeled-base or real-hardware demonstration. This is not estimated-model or GT-free execution.

Frozen audit:456 legacy code/config files and32 visual-state90° files unchanged; the carriage helper also unchanged. Both run_01 and run_02 actors were closed after report, raw data and video preservation. Unrelated jobs were untouched.

Evidence remains in `engineering_artifacts_20261009/7320_continuous_smooth/run_02/` locally and `results/7320_continuous_smooth_20261009/run_02/` on labserver_inschool: `continuous_same_scene.mp4`, `FINAL_METRICS.json`, `analysis.json`, `observations_chunks/index.json`, `physics_steps_chunks/index.json`, `sensor_metadata_existing_mapping.json`, `joint_sensor_index_metadata.json`, source/config snapshots and SHA256 manifests. The37.413-minute video is a lossless chronological concatenation from the same actor; wall-clock pauses are omitted, and no separate scenes or successful state fragments were stitched. Source, operational config and this report are published; video and raw telemetry remain local/server evidence.
