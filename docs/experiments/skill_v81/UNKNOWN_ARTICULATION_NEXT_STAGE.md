# Next stage: unknown hinge/rail geometry — preparation only

**Not implemented/validated as an autonomous capability. No benchmark is launched.**
The current V8/V8.1 interaction and dynamics results still use a known articulation
axis, joint type/state and registered station. Reaching a motion target under
those priors must not be reported as unknown-articulation success.

## Data boundary

The future online process receives timestamped wrist RGB/optical-Z depth, K,
measured camera extrinsics from robot calibration, robot/base/gripper state,
raw/causally compensated wrist F/T, contact diagnostics and actual prior actions.
It may use robot meshes and ordinary collision/emergency protections. Neither
passive fixed camera is an input.

The evaluator alone retains asset URDF/joint type/axis/limits, native object
coordinates, native body poses, authored dynamics and privileged object meshes.
Separate files/process interfaces must enforce this split; not merely a prompt
asking an Agent to ignore fields in the same packet. Current
`ARTICULATION_INTERFACE.json`, `identification_input.json`, `joint_q/joint_dq`,
`moving_link_pose`, native object-transform APIs and `native_joint_prior()` are
evaluation-only in that stage. The current runner cannot simply be relabeled.
An independent native safety shield may return allow/stop without exposing a
true articulation frame or using it to choose grasps/probing targets.

## Observation and estimation

1. Use wrist observations to distinguish the stationary cabinet and the moving
   panel/drawer/handle as uncertain rigid associations. Retain alternatives;
   segmentation alone does not prove which handle is attached to which part.
2. After observed-geometry grasping, perform small bounded Cartesian probes
   with measured F/T and robot-motion limits. Do not request degrees or rail
   displacement using hidden simulator coordinates. Start with low excitation
   and stop/reobserve on poor transmission or unreliable visual tracking.
3. Register **measured** moving-part surface patches between states, using the
   stationary scene/robot calibration as a reference. Separate registration
   uncertainty, occlusion and contact slip. Gripper/TCP motion alone cannot be
   treated as object motion; a slipping grasp must not invent a hinge axis.
4. Compare revolute and prismatic hypotheses. A prismatic hypothesis estimates
   the dominant translation direction with nearly constant orientation. A
   revolute hypothesis fits a common screw axis/rotation center to multiple
   visual rigid transforms. Use robust low-dimensional fits and weighted
   residuals; reject incompatible rigid associations instead of forcing a model.
5. Estimate relative coordinates from these visual transforms. Drawer distance
   is metric RGB-D translation along the estimated rail; revolute angle is
   unwrapped measured relative rotation about the estimated axis. Initial
   zero and axis sign are declared gauges, not hidden simulator zeros/signs.
6. Report axis/center/state uncertainty and observability. Short nearly straight
   arcs can confound a distant hinge with a rail. Tiny motion, repeated views
   of one narrow patch or planar degeneracy may not identify a hinge center.
   An incomplete hypothesis must remain unknown.

The numerical estimator owns hypotheses and uncertainty. The Agent inspects
fresh images, F/T, transmission and hypothesis ambiguity, then chooses a useful
reachable view/probe or a safe exit. Safe station relocation invalidates old
camera routes and requires new measured observations of the current state.

## Controller/identification interface

A future explicit `ArticulationHypothesis` should contain joint-type hypotheses,
estimated world axis, optional pivot, relative state, covariance, observed
support, timestamps and observation hashes. It must expose provenance and an
`estimated` source flag. It must not silently fall back to native axes/state.
Until uncertainty is adequate, execute guarded Cartesian probes rather than
pretending a precise native-coordinate trajectory is available.

Generalized effort uses the compensated measured six-axis wrist wrench projected
through the **estimated** articulation Jacobian. Propagate axis/pivot/state
uncertainty into torque/force and dynamics estimates. Correct V3.1 segment-local
filtering and FIT/VALIDATION isolation remain necessary; reliable geometry is
not proof of sufficient inertia or damping excitation.

## Evaluation to preregister later

Freeze the geometry/interaction implementation before selecting blind trials.
Use held-out ordinary objects, fresh dynamics, both joint types and genuinely
different axes; distinguish development geometries from unseen ones. Declare
probe/view/acquisition/relocation budgets and failure denominators in advance.

Evaluate joint-type classification, axis-direction error, revolute axis-line
distance, gauge-aligned visual state error, held-out surface-motion prediction,
contact transmission, same-episode FIT and independent VALIDATION, final parking,
runtime and storage. Report how geometry uncertainty affects dynamics accuracy.
Compare active probing with passive wrist observation and with an explicitly
labeled known-articulation upper bound. All truth comparison is post hoc; no
truth-selected views, filter coefficients, model variants or recovery poses.

No full-range claim follows from accurate short-stroke geometry or prediction.
The physical lifecycle and safety limits still bound what can be observed.
