# Multi-view RGB-D contact geometry → native-G1 short opening

Reference: `3103651`. Paired development cases, in order: **45600, 45606**.
**45671 is offline only.** No new objects, full-range opening or controller tuning.

Each case permits at most one valid normally closed-start physical episode:
fresh RGB-D/SAM3 → three calibrated scene-camera views → automatic grasp and
approach/IK → original closure → unchanged one-second post-acquisition hold →
original short-opening policy, approximately +3°. The existing ≥2° actual
post-acquisition outcome denotes short-range transmission, not a 30° success.
An initialization/export infrastructure error before manipulation is diagnosed
separately; a physical failure is preserved and never retried.

The registered initial base station remains fixed. No saved grasp transform or
GT handle surface is passed to visual generation or contact scoring. Known
hinge/dynamic object state, robot/camera calibration, native scene collision
checks and registered contact labels remain explicit controlled dependencies.

The new adapter reuses `3103651` preservation and metric fusion: original view
plus −55/0/+55° observed-panel-relative views, 0.7 m standoff, +0.1 m height,
0.5 mm point voxels. Open adjacent-depth-pixel triangle patches stay separate
from rear hypotheses at 0.75/1/1.25 times observed transverse width. Positive
front-facing convex caps (normal dot observed outward normal ≥0.1) are omitted from uncertain completion so that they do
not replace the measured front. Hypothetical lateral/rear faces are labeled
assumed. Missing depth does not certify empty space or a solid obstacle.

Scoring retains the frozen 18-candidate diverse shortlist, 5 aperture samples,
15 pose/aperture perturbations, 9×7 pad grid and 0.7 mean / 0.3 lower-quartile
aggregation. Acquisition uses measured-patch intersections. Retention/task
support is evaluated across the three declared rear hypotheses; no hypothesis
is selected using GT, and perturbed failures are soft scores. Friction-cone
μ=0.5 is an inherited modeling assumption, not measured friction or a force
estimate. All original candidate families are reused.

An exact R-tree triangle-ray backend avoids brute-force memory growth without
mesh simplification. Separate offline evaluation refines first-intersection
brackets with eight bisections of the inherited 41-aperture sweep (~0.008 mm
numerical brackets). This does not change physical aperture commands.
The common planning wall budget is 1,200 s, including denser surface scoring;
execution timing is unchanged. GT/cooked surfaces are opened only by the
separate evaluator after candidate selection and cannot correct executable poses.

Frozen closure, native finite-position actuator, preload, Pink opening policy,
one-second hold, colliders, dynamics and all existing physical protections are
unchanged. No new runtime quality gate or continuous-bilateral veto is added.

Views are acquired by **simulation-only scene-camera repositioning**, with the
physics timeline paused and camera pose/clipping restored before manipulation.
This is a perception feasibility study, not deployable active wrist sensing.

Full RGB-D, video and lossless chunked 240 Hz telemetry remain private. Public
deliverables contain code, compact metrics, source/configuration hashes and
evidence indexes only. Finished owned actors must be closed after export.
