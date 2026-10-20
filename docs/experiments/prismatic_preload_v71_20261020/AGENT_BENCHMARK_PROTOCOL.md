# Frozen Agent instructions for all remaining V7 slots

No object-specific prompt, pose, score, physics or safety tuning. Keep the
original eight objects, two configurations each, original hidden-file commitment
and finite resources. Previous valid results cannot be rerun. The versioned
activation authorizes only the ten remaining slots in their original order.

Inspect actual fresh wrist RGB, SAM3 masks, metric depth, local cloud and measured
surface/uncertainty plots before selecting views or approving a grasp. Use at
most four Agent-selected view adjustments. Initial no-route fallback supplies
only an actual unloaded wrist image, never a saved view or object pose. Use the
existing deterministic view/IK/collision validation and normal actuators.

Automatically generate candidates from current measured geometry. Inspect the
selected native-G1 geometry and predicted first contacts, and validate the full
current-state approach before contact. Revolute closure/preload remains frozen.
For prismatic acquisition the shared initial preload request is1N. At a verified
unloaded boundary the Agent may choose tested0.5/0.75/1/1.5N requests based on
actual normal/tangential contact, wrist F/T, motion and prior failures. A request
is not measured gripping force; do not change drive authority or safety limits.

Select finite informative FIT profiles from current reachability, contact and
motion feedback. Aim for20–30deg doors or80–150mm drawers where feasible; begin
with physically supported intervals. Include different velocities/accelerations
and overlapping reverse FIT. Declare a distinct complete VALIDATION trajectory
before executing it, before optional range expansion. FIT-only fitting and
segment-local processing remain unchanged. Record unsupported parameters and
retained prediction horizons honestly. Do not use hidden dynamics/seed or GT
handle surfaces to choose any action or model.

After failed acquisition/traction/planning, inspect the actual contact/motion
feedback and fresh wrist image. Separate command-to-measured TCP error from
measured-TCP-to-object motion. Missing positive pad-normal reports alone do not
prove separation. Use the original safe release/parking boundary before slow
Agent reasoning. Reobserve in the same scene, regenerate a current catalog,
select an eligible alternative family with an explicit observed-feedback reason,
and evaluate complete current-state execution. Do not abandon after one invalid
candidate or repeat an unchanged failed family. Never clear a hard safety latch.

Maximum four handle acquisitions and40 Agent commands per original registration;
5400s wall/2400s simulation with300s/90s exit reserves. Recovery consumes these
resources. Keep essential240Hz numerical evidence, sparse wrist checkpoints and
no continuous video. A full independent action is not established by an
incomplete segment or a small fitted residual. Preserve every failure, terminate
only for safety/no feasible recovery/finite resources, and verify parking and
owned-process exit. Never wait for Agent reasoning while unstable and loaded.

This remains simulated wrist mounting, registered base station, known joint
kinematics/state, ideal virtual wrist F/T and approximate actuators. Hidden truth
is revealed only after all online benchmark actions and decisions finish.
