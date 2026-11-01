# Controller replacement audit

Parent development:22e6524; last executed legacy owner:23c73e6. The old app
checkout and both of these checkouts are preserved. Only a new engineering
controller selection uses the replacement. No old physical record is changed.

| Condition | Decision and new coverage |
|---|---|
| Native xArm/G1/XY effort, speed, drive and joint authority | KEEP, unchanged. XY +/-1m, .08m/s; original effort/gains. Request preload is not measured finger load. |
| Measured collision, invalid state/F/T/clock, joint margin, hard velocity, emergency wrench | KEEP, original native independent supervisor and latch. No reset interface added. |
| Original measured joint margin .05rad | KEEP including command and sampled stop. Not reinterpreted as a manufacturer-certified safety rating. |
| .012m/s loaded command and .03m/s measured TCP envelopes | KEEP, unchanged. Action speed may be lower. No 90%-of-hard extra interruption. |
| Loaded .04m/s² command acceleration and action-selected lower acceleration | MERGE in one owner. Task-selected acceleration changes coordinated command and brake timing. |
| .75 pose error per solve (.75/dt velocity) | REPLACE with time-based gain6/s at60/240Hz, bounded task demand, damping .03. This is an explicit control design change, not a raised safety threshold. |
| SciPy bounded nine-variable LS + nonlinear final governor/SLSQP | REMOVE from new execution. One weighted 6x6 DLS and one transactional boundary/geometry owner. |
| Pink online velocity_step substitution via scoped patch | REMOVE from new execution. Offline grasp/approach Pink remains as an existing geometric planner. |
| CartesianReference, nominal seed bias, integrated measured QP, final governor all modifying trajectory | MERGE: measured feedback + command integration in one owner. Progress clock remains the task generator. Bounded contact centering is explicit before IK. |
| Old command anti-windup | RETAIN one bounded compliance check using original stopping-distance allowance; no hidden per-asset multiplier. Soft saturation requests deceleration, not hard termination. |
| Ordinary load warning, provisional sustained band, predicted load | KEEP continuous demand adaptation through the existing classified force monitor. Emergency precedence unchanged. They cannot alone destroy World. |
| Original .3/.6/1Nm wrist and other load bands | UNCHANGED. Provisional sustained band remains classified regulation; report actual safety semantics, not certified limits. |
| Every8tick iterative governor stopping preview, <=5s, optimizer per step | REPLACE with exact finite scalar-ray brake built without IK/optimizer; same discrete command path used for actual stop. Cache consecutive FK poses. Geometry samples all9 DOFs at0.5mm XY/.005rad arm resolution plus first/end; actual brake geometry and contact supervision remain every step. |
| Independent per-joint clipping or instant freeze as braking | NOT USED. All9 velocities share the same decreasing scalar. FK curvature and strict bounds still may reject an unsafe candidate. |
| Command dq=0 treated as completed stop | REPLACE with distinct SETTLING and measured configuration dwell. Confirmation:XY<=1e-4m/s, arm<=.01rad/s,TCP<=1e-4m/s,TCP rotation<=.01rad/s, object<=1e-4(native/s),150ms. Native projected dq is separately retained and hard-supervised. These are exit-confirmation tolerances, not hard stop thresholds. |
| Failed measurement dwell | ADJUST to typed pending zero-command hold; staged exit cannot release or restart from it. Existing finite C supervision/exit budget still applies. |
| IK numerical failure/candidate collision | Reject proposal, execute previous accepted brake, return action-level recoverable failure if measurements trusted. No numeric exception is relabelled a measured hard hazard. |
| Real changed obstacle blocking an already certified stop | Remains control/safety diagnostic uncertainty, not a successful stop. Never move through it or clear a latch. |
| Future90deg/180mm complete parking route as acquisition prerequisite | Still removed by existing staged exit. Current action route and actual-state release/retreat/parking remain checked. |
| Planning and recovery subbudgets | Existing finite resource accounting retained; no new recovery-count or per-asset caps added. This is a short controller check, not a4h task success claim. |
| Nominal pacing advisory invalidates whole FIT | REMOVE for the new profile. Actual sample safety/finite/sliding/direction/filter support and complete VALIDATION requirements remain. No estimator compensation/fitting mathematics changed. |
| Historical special stops/latches | Preserve original evidence and owner locks. No incident is resumed or rewritten. |

Remaining planning/constructor/ABC wrappers still use some scoped bindings;
they are not online motion solvers or drive writers. This is a controller-sized
replacement, not a rewrite of perception, grasp ranking or native physics.

Pre-physics checks:107 focused existing+new tests passed. Actual robot URDF
world Jacobian max absolute finite-difference error8.03e-11. CPU perfect-state
240-step motion had no candidate failure. These do not prove contact or stopping
under native drive lag. Physical results belong in RESULTS.md after execution.

K closed-command replay:3/4 sample states have valid new brake; final old state
fails strict bounds in BOTH old and new methods. Source telemetry arm float32
makes reconstruction approximate. This is a real unresolved admission case,
not proof of full K recovery; new execution must certify stop before acceptance.
D slip dynamics require actual contact rerun, cannot be reproduced by a fake
force/parameter-threshold script or kinematic command replay.
