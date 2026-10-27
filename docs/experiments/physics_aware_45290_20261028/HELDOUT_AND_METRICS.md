# Independent A/B/C execution and measured outcomes

The inherited drawer VALIDATION admission required a numerically qualified
model fitted inside the same episode. That would prevent model-free A and
stale-model B from operating unless they first identified the changed plant.
The study adapter therefore separates manipulation admission from model
accuracy for explicitly registered `reactive`, `stale`, and `updated` roles.
These actions retain VALIDATION labels and contain no FIT intervals. Their
native current-state path, coupled motion, collision, actuator and stop
checks still run. Qualification, calibration and M0/M1 identification episodes
retain the original FIT/VALIDATION publication rules. No READY result or
numerical validation result is fabricated. The adapter writes its scope to
`INDEPENDENT_HELDOUT_XX.json` before native execution.

Formal prediction assessment must use the frozen external M0/M1 and held-out
telemetry without refitting or reselecting a model. An episode's native
estimator can correctly report insufficient local FIT data; this neither
invalidates an actually completed manipulation nor establishes model accuracy.
All models, calibration data and selected control outputs must be sealed
before their corresponding formal manipulation.

`python -m agentic_interaction_v88.study_predictions EPISODE --model M0.json
--model M1.json --output PREDICTIONS.json` applies the original prediction
routine to complete held-out segments after actor closure. It never fits or
selects a model, reports excursions beyond FIT support, and records each input
model hash. Verify those hashes against the pre-execution seals separately.

`python -m agentic_interaction_v88.study_metrics EPISODE --output OUTPUT`
requires an exited actor and produces native segment metrics and motion/load
curves. It uses the original bias and robot-tool gravity compensation at the
measured wrist sensor origin. Tool inertia is retained, matching the live
guard. Raw gravitational load and joint-origin moments are not mislabeled
as compensated wrist interaction loads.

Contact diagnosis uses measured rigid-body translation and rotation, native
contact points/normals and the existing pre-integration alignment. Reported
material sliding distance integrates one median instantaneous tangent speed
per native contact step, with missing steps excluded. It is not a tracked
contact centroid or TCP/handle drift. Normal-report absence alone does not
establish grasp separation. All failed segments remain in statistics. Native
operation time, Agent waiting, diagnostic hold and total launcher-to-exit time
are separate. Jerk comparisons require matched actual speed or completion
time; a lower number after slowing down does not demonstrate better control.
Material slip is pose-derived and includes discretization jitter; no noise
deconvolution or new binary slip threshold is implied. Six-axis wrist and
simultaneous measured base/arm displacement curves are exported separately.

This change supplies admission and analysis tools, not A/B/C outcome evidence.
No benefit of physics-aware control is claimed from qualification runs.
