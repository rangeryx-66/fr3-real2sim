# Input-response physics identification

This is an internal project analysis module, not an official Act2See physics
experiment. `interactive_twin/sysid.py` consumes independently executed Isaac
rollouts. It contains no door ODE surrogate and no simulator state-replay code.
The successful grasp and unknown-articulation baseline files are untouched.

## Evidence boundary

`SIM_TO_SIM_BLIND_SYSID` compares a hidden-parameter simulation reference with a
twin. It establishes a simulation-surrogate result only. `REAL_LOG_TO_SIM` uses
the same observable schema, but requires actual real robot logs before a
real-to-sim claim is possible.

Reference logs contain only robot timestamps, applied commands, q/qdot, EE pose,
gripper opening/effort, and optional motor current/SDK effort. GT hinge, object
joint state, moving-link trajectory, contact manifold and external torque fields
are rejected. Evaluation truth belongs in a separate file outside this API.

Current and SDK effort are not automatically external torque. B3 requires both
reference and twin to supply the same **independently established** sensor
calibration ID. A robot-only current bias summary does not satisfy that gate.

The external runner must attest that each candidate was independently evolved
in Isaac, with initialization only at episode start, no observed-q replay,
no object-state replay, no direct object actuation and no attachment. The
command hash must match the reference's complete timed command sequence.
These checks enforce a logged contract; they are not cryptographic proof of
the runner's behavior. The runner and control files still need auditing.

The constrained-controller replay interface records exogenous Cartesian input
(`ee_reference_x_m/y_m/z_m`, direction, active/hold flag and gripper commands).
Each plant recomputes feedback effort from its own measured q/J/M under the
unchanged controller. These plant-specific feedback torques are diagnostics,
not the replayed input. Freezing another plant's gravity/feedback effort is a
different open-loop experiment and is not interchangeable with this interface.

## Frozen robot calibration

`interactive_twin/calibration.py` provides:

```python
robot = calibrate_robot_response(
    [robot_only_repeat_1, robot_only_repeat_2],
    noise_floors={
        "position_m": 20e-6,
        "rotation_rad": 2e-5,
        "joint_rad": np.deg2rad(.001),
        "joint_velocity_rad_s": .001,
        "ee_velocity_m_s": .0001,
        "start_time_s": .02,
        "tracking_m": 20e-6,
        "current_a": None,
        "effort": None,
    },
    output_dir=output,
)
noise = NoiseScales(**robot["noise_scales"])
```

Floors above are explicit configuration examples, not an assertion of PiPER
hardware accuracy. Use measured calibration and documented quantization floors
for the final frozen configuration. Every calibration log has `split=calibration`
and `provenance.no_contact_supervisor_certified=true`. Repeats must have identical
commands, source, robot model and controller. Output takes the maximum of the
configured floor and pairwise repeat variation, writes `robot_calibration.json`,
and derives a content-based calibration ID for subsequent trials.

Current bias and velocity summaries are reported when available. Command
latency remains null without an experiment separating communication delay from
mechanical response. A descriptive position gain is reported only when explicit
joint-position reference fields and sufficient excitation exist. Torque-only
inputs do not justify inventing a scalar servo gain. This module never changes
robot gains or absorbs unmodeled robot response into a calibrated robot model.

## Log schema

```python
log = {
    "schema_version": 1,
    "mode": "SIM_TO_SIM_BLIND_SYSID",  # alternatively REAL_LOG_TO_SIM
    "episode_id": "asset/config/repeat",
    "probe_id": "P1",
    "split": "train",  # heldout, sensitivity, calibration also accepted
    "time_s": times,   # N monotonic timestamps, relative to protocol origin
    "commands": {
        "time_s": command_times,
        "values": command_values,   # M x D, actually applied input
        "fields": command_fields,
        "kind": "arm_effort",
    },
    "signals": {
        "q_rad": q,                 # N x joint_count
        "qdot_rad_s": qdot,
        "ee_T_world_tcp": measured_ee,  # N x 4 x 4, genuine SE(3)
        "gripper_opening_m": aperture,  # N
        # optional: gripper_effort [N], motor_current_a/sdk_effort [N,J]
    },
    "provenance": {
        "source": "isaac_physics",  # real_robot_log for real reference
        "robot_model_id": frozen_robot_hash,
        "robot_calibration_id": robot["calibration_id"],
        "controller_id": frozen_controller_hash,
        "simulator": "isaac_physx",
        "complete": True,
        "initialization_only": True,
        "robot_state_replayed": False,
        "object_state_replayed": False,
        "direct_object_actuation": False,
        "attachment": False,
        "command_applied_sha256": command_hash(commands),
    },
}
```

A real reference does not need simulator replay attestations. Twin predictions
always do. Native measured joint efforts must not be relabeled as SDK current.
If no trustworthy effort/current signal exists, omit it and run motion-only B2.

## Sensitivity before fitting

```python
sensitivity = sensitivity_report(
    {"LOW": low_repeats, "MEDIUM": medium_repeats, "HIGH": high_repeats}, noise
)
```

The analyzer receives condition labels, **not hidden reference parameter values**.
Each repeat must use the same complete timed input. Curve signatures include EE
position, q, qdot and EE velocity. Between-condition differences are compared
with the larger of the noise scale and observed repeat variability. No measurable
difference returns `UNIDENTIFIABLE_UNDER_CURRENT_PROBE`.

At least two repeats per condition are needed before accepting physical
identification. A sensitivity result with one repeat explicitly reports that
repeat variability was not measured. Two-parameter rank may be evaluated from
an estimator-owned twin grid, never by reading hidden reference parameters.

## Frozen protocol, candidate budget and identification

P1/P2/P3 are the only training actions. P4 is held out before executing any fit;
its exact command protocol (reverse when safe, or the predeclared short-pulse
fallback) is frozen by the benchmark runner. Kinematics must already be accepted.

```python
grid = candidate_grid(c_values, b_values, J_eff_prior=J, budget=12)
# Execute every grid member in the real Isaac replay runner, then analyze:
result = fit_resistance(
    {"P1": reference_p1, "P2": reference_p2, "P3": reference_p3},
    [{**phi, "probes": {"P1": twin_p1, "P2": twin_p2, "P3": twin_p3}}
     for phi in grid],
    noise, J_eff_prior=J, budget=12, sensitivity_evidence=sensitivity,
    expected_grid=grid, censored_candidates=failed_candidates,
)
```

`budget` counts parameter candidates. Each candidate needs all three training
probe segments. They may occur in one continuous, independently evolving Isaac
rollout; the runner records actual launch counts. The analysis reports
`probe_segments_evaluated` and leaves `physics_rollouts_used=null`. All
P1/P2/P3 rollouts are mandatory. Incomplete or safety-stopped rollouts cannot be
silently truncated/dropped to improve a fit; the benchmark records that failure.
The caller must count calibration, sensitivity and held-out executions separately
in the full wall-clock/simulation budget.

`expected_grid` is mandatory for a coverage-verified identification claim.
The analysis automatically marks missing grid members as censored, supplements
their failure details from `censored_candidates`, and returns the full frozen
parameter range if any member is missing or unsafe. It never derives a narrower
search domain from the subset of safe/completed candidates. The best observed
grid point and its profile are retained as diagnostics, not an accepted T2.

Distance compares time-aligned EE position/orientation, q, qdot, EE velocity,
motion onset and final displacement. If the command contains
`ee_reference_x_m/y_m/z_m`, tracking-error profiles are also compared. Pure effort
commands have no position reference; that metric is explicitly unavailable and
the complete input-response trajectories are used. Units are divided by frozen
noise scales, and normalized component losses receive equal weights. There is
no time warping, reference-dependent gain adjustment, endpoint-only objective or
per-asset noise retuning.

`J_eff` remains the same prior for all candidates. If no scalar equivalent
inertia is independently known, pass a frozen articulated-model handle instead
of inventing a dimensional scalar:

```python
J = {"kind": "fixed_articulated_model", "sha256": inertial_blocks_sha256,
     "scalar_J_eff": None, "frozen": True}
```

Every rollout uses the same unchanged prior body inertias. Outputs are **effective
simulator resistance parameters**, not calibrated real hinge torque. Depending
on the backend, the mapping into passive native Coulomb/damping attributes must
be recorded by the runner, including units and USD radian/degree conversion.

Acceptance requires a passed repeated sensitivity experiment from the same
frozen robot/calibration/controller, a fully observed uncensored grid, a rank-two
parameter-response map above noise, and profile support narrower than 80% of
each searched parameter range. Otherwise `accepted_parameters=null` and T2
physics must not be declared identified. Returned `parameter_intervals` are
discrete profile-loss support sets, **not 95% confidence intervals**. Small grid
support does not exclude model mismatch outside the searched parameter family.
`best_fit_rms_noise_units` reports absolute training mismatch; no new TEST
adequacy threshold is chosen after observing results. A narrow support set alone
does not establish a noise-consistent model or prove that resistance was not
compensating for kinematic/robot/contact model error. Held-out improvement is
still necessary.

## Held-out comparison

```python
comparison = heldout_comparison(
    reference_p4,
    {"B0": initial_twin_p4, "B1": kinematics_only_p4,
     "B2": calibrated_twin_p4, "Oracle": oracle_p4}, noise,
)
```

- B0 / T0: initial articulated/physics prior.
- B1 / T1: estimated kinematics, original physics.
- B2 / T2: estimated kinematics and motion-only calibrated resistance.
- B3: optional calibrated comparable current/effort; unavailable is not failure.
- Oracle: evaluation-only diagnostic upper bound.

Missing experiments are `NOT_RUN`; uncalibrated B3 is `UNAVAILABLE`. The module
never manufactures an Oracle, physics update or held-out improvement. Both
absolute and relative held-out loss reductions are returned, including negative
values when calibration worsens prediction. File helpers save strict JSON and
candidate CSV; the runner generates plots and the per-episode stage taxonomy.

## Analysis tests

```bash
python scripts/test_interactive_twin_sysid.py
```

Tests cover privileged-data rejection, replay rejection, complete timed-command
equality, frozen robot IDs, trajectory-vs-endpoint distinction, calibration
certification, current gating, held-out leakage, flat-response failure and a
synthetic identifiable grid. These are analysis tests; they provide no evidence
of physical grasp or Isaac identification performance.

## Frozen-denominator benchmark reporting

```python
from interactive_twin.reporting import (
    build_benchmark_report, plot_sensitivity_curves, plot_heldout_predictions,
)

summary = build_benchmark_report(
    frozen_manifest, episodes_root, report_directory, make_plots=True,
)
plot_sensitivity_curves(conditions, report_directory / "sensitivity.png", report=sensitivity)
plot_heldout_predictions(reference_p4, predictions, report_directory / "heldout.png")
```

The reporter verifies the manifest hash and reads
`episodes_root/<episode_id>/episode_summary.json` first. Its `stages` dictionary
uses A through I (or the corresponding semantic names), each containing a
`status` and optional `reason`. The wrapper may specify `selected_report` as a
path relative to the episode directory. Without that selection, direct
`report.json` or `reference/report.json` is used. Multiple nested candidate
reports are **ambiguous**, not an invitation to choose the best result.

The TEST denominator is at least four asset slots × three configurations, even
when source selection or execution is incomplete. DEV has separate totals.
Each stage reports successful, evaluated and planned counts. When evaluated
count is zero, the measured success rate is null; missing episodes remain
visible in the planned denominator. Unidentifiable physics, missing held-out
comparisons and unavailable B3 never become T2 improvement claims.

Outputs include `benchmark_summary.json`, one normalized A–I JSON per episode,
`episodes.csv`, `per_asset.csv`, `articulation_errors.csv`,
`failure_taxonomy.csv`, a stage-count plot and a concise README. Raw source
reports are retained. Actual final joint angle and displacement relative to the
episode's initial articulation state are distinct fields. Full true slip is
null unless directly available; contact-plane drift is separately labeled.

Reporting tests: `python scripts/test_interactive_twin_reporting.py`.
