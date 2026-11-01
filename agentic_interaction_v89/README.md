# Coordinated DLS engineering controller

This opt-in replaces the V87 governor and V88 online solver/stop together.
It is not a certified hardware controller. Short physical validation and
remaining limitations are recorded in `docs/experiments/controller_simplification_20261101`.

ABC and its current wrist observation, automatic grasp generation, native
contact/closure, object progress clock, FIT/VALIDATION separation and same-World
recovery remain. Object dynamics, robot/hand authority and hard measurements
are unchanged. The carriage is the existing driven XY slide, not wheels.

Control: task progress -> world TCP goal -> weighted DLS velocity (6x6 solve)
-> single command owner -> synchronous NativeDrive XY/arm/G1 target -> physics
-> measured independent supervision. The DLS uses m, rad, seconds and
Log(R_goal R_measured.T). Pose arrays use xyzw. Native Isaac wxyz is converted
by the existing measured-pose boundary. The TCP is `tcp_link`, not the wrist
sensor or finger centre. FK uses base yaw converted from degrees to radians.

The gain is 6/s, rather than copying HiddenKinematics' per-step joint delta
or the old .75/dt gain. Weighted DLS favours base motion as geometry and arm
margin permit, with no asset-specific allocation. Action speed and acceleration
and Physics-Aware preload remain actual inputs to execution, not report-only
values. Soft load feedback changes reference demand, without abruptly shrinking
the accepted command's authority. Tracking deficit is recorded; the unsupported
stopping-distance veto that froze low-load acquisition compensation is removed.

The command owner computes a pure proposal. It applies one joint/task boundary
calculation, exact FK hard-speed guard, native geometry, and a finite coordinated
brake certificate. Only acceptance commits its position, velocity, twist and
generation. Rejected candidates keep the previous command and certified stop.
The runtime consumes that stop before returning a recoverable action error.
It never reruns a failed IK/projection to brake. Fixed-station acquisition and
prechecked unloaded paths use the original30Hz candidate geometry cadence;
loaded/coordinated commands and actual braking check every240Hz step. Native
contact checks240Hz and measured geometry checks30Hz remain independent.

Stopping follows one common scalar multiplying ALL nine accepted velocities.
The discrete integration used for preview is exactly the integration executed.
This preserves cancellation; FK dangerous-speed guard, actual collision and measured loads
are still checked. TCP design speed/acceleration deviations are recorded as four
ratios and do not veto an otherwise safe command or brake. Desired speed and
acceleration continuously regulate the command; they are not certified hard
physical limits. The candidate guard uses the UNCHANGED measured-speed safety
envelope, and actual measured supervision is independent. Stop geometry is sampled along all9 joint coordinates at
0.5mm XY / .005rad arm resolution (plus first/end samples); actual brake samples
are checked individually. These are sampled
geometric checks, not a continuous contact/dynamic proof. The twofold nominal
stop duration reserves acceleration for curvature; a trajectory that still
violates actuator bounds, native geometry or the dangerous-speed envelope is
rejected, not force-fit by an optimizer. No strict TCP design-acceleration
guarantee is claimed for nonlinear FK; those deviations remain evidence.

States are RUNNING -> BRAKING -> SETTLING -> STOPPED. Command zero and measured
stationarity are separate. Release and restart require the latter. Failure to
settle holds the zero command under supervision and returns a typed pending
state to every staged-exit dwell, not an invented measured collision. The
later settle() path also catches pending; it cannot leak into session teardown. A missing certified
brake or a changed blocking scene is diagnostic/control uncertainty; it never
authorizes automatic motion or latch clearing. Real hard supervision retains
its original latch and evidence.

Stationarity uses the original MotionDwell stop cell (TCP1mm/s, arm.01rad/s,
TCP rotation.01rad/s, object1e-4native/s,300ms) plus XY1e-4m/s,
from consecutive240Hz measured XY/arm/TCP/object configurations,
including TCP orientation, extending the existing MotionDwell convention.
PhysX projected joint dq is retained separately and still goes through the
original hard supervisor. Native dq can disagree with actual configuration
motion during constraint stabilization; neither quantity is silently relabelled.

The online path has no runtime `patch.object`, old governor, bounded least-squares
optimizer, SLSQP, iterative stop projection or second CartesianReference filter.
The established offline coupled geometric continuation remains for grasp/path
feasibility; it writes no commands. The unnecessary new offline DLS planner
was removed to avoid changing grasp availability while replacing the servo.
Existing constructor-only scene bindings, grasp planners and ABC operation
wrappers remain outside this replacement. The inherited free-space timed paths
also pass through the same final owner and submission interface.

## Launch

Use the existing explicit private bundle/environment and V81 launch protocol.
In an engineering `v87_abc_hybrid_recovery_v1` spec, retain `staged_exit:true`,
`constraint_policy:classified_continuous_v2`, and
`coordination_policy:native_xy_arm_joint_coordination_v1`; add:

```json
{"controller":"coordinated_dls_v1"}
```

For the automatic free-space prerequisite ONLY, additionally use
`"controller_validation":"free_stop_restart"`. Omit that field for normal
fresh-observation ABC. The free check also executes a small current-Jacobian XY/opposing-arm
cancellation input through the same owner, then confirms measured rest.
No grasp is imported or automatically authorized.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=.:scripts "$ISAAC_PYTHON" -m agentic_interaction_v81.launch \
  --spec /absolute/private/spec.json --environment /absolute/private/environment.json
```

The existing isolated simulation lease, safety holds, frozen asset/closure
commitments and process cleanup rules still apply. The replacement has a
separate explicit source audit; it reports replaced source hashes and checks
all other frozen files. It does not pretend to be unchanged V8 reproduction.
Commit and freeze this checkout BEFORE launching; never edit a live actor.

Run CPU checks with the Isaac environment's Python (numpy/scipy/pin installed):

```bash
"$ISAAC_PYTHON" -m unittest discover -s tests -p test_simple_controller_v89.py -v
PYTHONPATH=.:scripts "$ISAAC_PYTHON" -m agentic_interaction_v89.validate_kinematics \
  --config /absolute/robot_io/NATIVE_CONFIG.json --output /absolute/kinematics.json
PYTHONPATH=.:scripts "$ISAAC_PYTHON" -m agentic_interaction_v89.replay \
  /absolute/closed/episode --output /absolute/replay.json
```

## Rollback

Close the current trial through export/owned cleanup first. Use a separate
checkout of `23c73e6e7b05893162f11865ea036a9f86935839` for the last frozen
executed controller, or `22e652417f5afc9683fdfcca15fc0690d58e34dc` for its
later policy/report tooling. Omit `controller` in the old spec. Never hot-load
the old controller into the new World, clear old incidents, or reinterpret
an unparked exit as safe. The modified checkout is not a frozen V8 baseline.

## Source attribution

DLS algebra is adapted from HiddenKinematics `src/envs/utils/compute.py`,
with its action/drive wiring audited in `perform_action.py` and
`franka_pose_cabinet_base.py`, pinned to
`d0318abec626fc8416f194619f21fbe7145e2ebf`.
MIT notice is retained in `third_party_licenses/HiddenKinematics.txt`.
No RL, saved pregrasp, reset, camera or finger-zero-gain code is used.
