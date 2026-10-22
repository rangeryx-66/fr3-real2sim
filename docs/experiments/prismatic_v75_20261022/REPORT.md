# V7.5 — prismatic lifecycle, loads and model applicability

**Engineering development; readiness is not yet established.** This is not a new
benchmark and does not replace any V7.2–V7.4 outcome. Fresh native-contact results
are reported individually, including unsuccessful acquisitions and recovery.

## What the archived failures actually show

| Failure | Measured evidence | Interpretation / unresolved issue |
|---|---|---|
| 19179_0 sustained overload | Last 150 ms: pad maximum mean 3.882 N; wrist force norm mean 3.913 N; sensor-origin moment norm mean **0.604644 N m**. Useful rail force 0.194 N versus transverse force 3.899 N. | The original moment channel, not a generic 2 N stop, terminated execution. Predominantly non-task loading; high drawer resistance is not established. Contact geometry, constrained motion and actual finger settling remain possible contributors. |
| 19179_1 reverse collision | Native `gripper_link2 / original10` collision after recovery and useful bidirectional motion. Its recorded nominal grasp passes the new whole-chain offline check (540 configurations). | A nominal geometric preflight does **not** prove loaded reversal safety. Original online native collision protection remains necessary. |
| 27044 traction / recovery cost | Some pulls have both increased commanded-to-measured TCP error and increased measured-TCP-to-drawer relative motion; these are separate quantities. The last recovery completed three FIT actions but had insufficient resources for full VALIDATION. | Neither arm tracking nor bilateral closure alone establishes useful traction. Admission must include complete validation and exit before another acquisition. |
| 45746_1 safe exit | Actual released terminal state had first arm joint near 4.818 rad. The old rest representations required a long sweep. With the observed retreat normal, old planning rejected after 112.38 s; legal equivalent joint winding found an original-predicate-valid full route in 1.496 s. | One rejected return was not an unavoidable reachability limit. This is an offline route result until actuator execution verifies the terminal state. |
| Failed closure feedback | The old feedback path could display the preceding successful identification segment. | Attempt/generation/step provenance, bounded writer barrier and current-attempt-only numeric/contact selection replace that fallback. Missing current data is explicitly unavailable. |

Native contact manifolds are not pressure distributions. Missing positive normal
reports are not proof of physical separation. Native impulse/anchor forces are
diagnostic cross-checks only; identification still uses robot-side six-axis F/T.
PiPER and G1 results are not matched physical controls. Revolute torque and
prismatic force are not compared numerically without task geometry/virtual work.

## Load-channel audit and calibration

The active original policy uses per-pad maximum positive normal load, the norm of
the world-oriented wrist force, and the norm of moment at the **instantaneous
sensor origin**. Raw sensor bias is subtracted and downstream-tool gravity is
compensated. Downstream inertia remains in this conservative causal protection
channel. The estimator separately compensates tool inertia exactly once and
applies common SG121 filtering after a segment is complete. Centered estimation
operators never feed protection.

| Channel | Informational warning | Trailing 150 ms sustained stop | Immediate emergency |
|---|---:|---:|---:|
| Maximum reported pad normal | 2 N | 4 N | 10 N |
| Wrist resultant force | 4 N | 8 N | 10 N |
| Sensor-origin resultant moment | 0.3 N m | 0.6 N m | 1 N m |

`force.window_s=0.05` is an inactive configuration field; `sustained_s=0.15`
is the actual guard interval. Histories are retained across mode changes and
reversal. Native collision, joint margin, speed/acceleration and actuator guards
remain authoritative. **No load band or native drive authority was increased.**

The official [xArm specification](https://docs.xarm.ufactory.cc/8.technical_specifications.html)
does not permit converting payload into a safe handle pull or clamp limit. The
installed model is the pinned official
[G1 URDF](https://raw.githubusercontent.com/xArm-Developer/xarm_ros/aad7e1611c9c46eb719045414394bfdd42dcb0f8/xarm_description/urdf/gripper/xarm_gripper.urdf.xacro),
with legacy position/speed actuation, not a G2 force-command interface. Requested
preload is converted through the unchanged native mapping; measured pad load is
reported separately. Independent motor drive force is unavailable. Explicit zero
effort and projected joint reactions are not reported as motor output.

The reference [AI1500 specifications](https://docs.accessories.ufactory.cc/6_Axis_Force_Torque_Sensor/5.Technical_Specifications.html)
state 400 N /20 N m range, 0.4 N /0.01 N m resolution and 200 Hz sampling.
Resolution is not a Gaussian standard deviation; capacity is not a task safety
rating. The implemented sensor remains an ideal 240 Hz virtual interface, with
no physical AI1500 mass/adapter or calibrated hardware-accuracy claim. Raw versus
compensated meanings follow the [UFACTORY SDK convention](https://docs.api.ufactory.cc/xarm_python_sdk_docs/7.-Force-Torque-Sensor.html).

Short native sensor fixtures, all preserved:

- First fixture was infrastructure-invalid before any valid applied-load samples
  (missing recorder coordinate unit; cleanup masked the original exception).
  The exact wiring issue was repaired; it is not a physical failure.
- Normal fixture: 11.733 simulation seconds /39.65 wall seconds; unloaded static
  orientations, unloaded actuator motion, independent 1 N XYZ and 0.05 N m XYZ
  tool loads. Sensor reaction had the expected opposite sign. Object motion and
  pad contact were zero. This is not an exact matched loaded-grasp calibration.
- Abrupt 9 N tool-load contrast triggered the original 0.03 m/s phase velocity
  guard first, at 0.03100085 m/s. This is **not** a sustained-force-stop result.
- A separately predeclared 1 N/s tool-load ramp triggered original sustained
  overload at 8.055 N over the 150 ms window; speed was 0.000371 m/s. No hard
  latch was reset. These tool fixtures are not drawer-jam or manipulation PASS.

The isolated prismatic extension forecasts the **same** trailing sustained window
at the bounded stopping horizon using past loads and a declared linear trend.
It requests ordinary deceleration/unloading before predicted margin exhaustion;
it cannot clear, override or guarantee avoidance of a hard stop. Archive replay
and fresh contact outcomes must be reported separately.

## Shared prismatic changes

All additions are versioned V7.5 modules; the 71 source/configuration and 22
external revolute hashes, call paths and historical results remain frozen.
The initial requested preload stays 1.0 N; only the previously tested
0.5/0.75/1.0/1.5 N menu is available at a verified unloaded boundary.

1. Candidate feasibility now includes the Agent-declared pull/reverse/validation
   chain, unchanged CENTER envelope/preload mapping, command bias, stopping
   distance, release and full exit. Scores and observed contact surfaces are
   unchanged. Current-state native checks continue during actual execution.
2. Parking enumerates only legal 2*pi representations of original rest regions
   for explicit revolute/continuous robot joints, within original margins.
   Complete paths and measured endpoints use the original predicates/executor.
3. Acquisition admission counts actual waypoint execution, measured observation
   cost, original closure/hold bound, all FIT actions, **complete independent
   VALIDATION**, worker closure and safe exit. Forecast uncertainty is explicit;
   an unknown cost is not treated as zero. Original deadlines/reserves remain.
4. Contact evidence carries episode, physical/logical attempt, observation
   generation, half-open native step interval, simulation times and source
   hashes. Current failed closure cannot borrow an older successful segment.
5. Explicit operations/session wiring preserves the V7.3 protocol/safety control
   flow (AST parity check with only the operations factory changed). Commands
   retain atomic publication, sequence numbers and ACKs. Decisions occur only
   at verified unloaded boundaries; orchestration timeout is separate from
   physical failure.

## Estimator applicability, not truth-matching

Read-only comparison covers all eight V7.2 prismatic recordings, eight V7.4
recordings and five V7.3 engineering recordings. Original selected model family,
compensation, regularization and held-out segments are fixed for each comparison.
Hidden truth was not used to choose filters, samples or coefficients.

| Recording | Old eligibility mass (kg) | Complete SG-support mass | Equal-count/time control | Independent sliding momentum mass |
|---|---:|---:|---:|---:|
| V7.2 45290_0 | 4.63928 | 2.32342 | 4.50649 | 2.34624 |
| V7.4 45746_1 | 2.74288 | 1.84323 | 2.69356 | 1.81904 |
| V7.4 45290_0 | 2.44684 | 2.44652 | 2.44686 | 2.44027 |

The first two changes are not explained by sample count alone. V7.4 45746_1
retains 16,063/16,796 samples; V7.2 45290_0 retains 7,027/8,212. The momentum
diagnostic integrates unfiltered, fully compensated effort over complete sliding
windows with endpoint velocity/displacement and the same friction basis. It is
an independent diagnostic, not a hidden replacement for the production result.

Same-family leave-one-FIT-out and 1-second contiguous-block bootstrap supplement
conditional covariance. Raw-input scenarios cover bias, quantization, independent
noise, 1/2-sample delay, tool mass/COM errors and noisy/quantized joint position
with derived velocity. FIT/VALIDATION noise streams are independent. Empirical
structured-residual warnings are **uncalibrated diagnostics**, not new gates or
proof that every statistically detectable micro-newton residual invalidates mass.

Analytical raw-wrench fixtures are explicitly **not native physics**. With the
production ridge 0.001, the ideal constant plant [2 kg,3 N s/m,0.3 N] gives
[1.999705,2.999489,0.300001]. An asymmetric opening/closing resistance plant
does not have one true Coulomb coefficient: the constant fit absorbs mismatch
into mass (1.622 kg) while the integral formulation disagrees. That model is
flagged, not presented as successful recovery of a nonexistent parameter set.
AI1500-resolution quantization destroys the small excitation's mass information;
10 micrometre position-noise scenarios can leave insufficient qualified data.
Ideal native-velocity accuracy therefore does not establish hardware generality.

Physical VALIDATION duration, retained derivative-supported prediction horizon,
support interval and parameter availability are separate fields. A small free
trajectory RMSE does not validate every parameter or unobserved friction peaks.

## Fresh engineering validation — in progress

R1 source freeze: `23a30b2`, with `SOURCE_FREEZE_R1.json`. One continuous 19179
scene has begun. First automatic grasp and original 1-second hold succeeded;
pull stopped at 1.373 mm under original persistent sliding-stall logic, then
guarded release and verified parking succeeded. Commanded TCP moved 3.510 mm,
measured TCP 3.185 mm, drawer 1.373 mm: added tracking error was 0.325 mm versus
1.813 mm measured-TCP/drawer relative motion. This supports transmission/compliance
degradation rather than an arm-tracking-only explanation. It does not uniquely
identify friction slip versus coupled-finger deformation.

The Agent inspected actual fresh RGB/mask/depth/local cloud, incremental surfaces,
current contact curves and native contact locations. In the same scene it chose
an opposite oblique view, a newly generated shallower/changed-section candidate,
and a tested 1.5 N requested preload at the unloaded boundary. This changes both
pose and preload and is **not** an isolated preload A/B experiment. Full recovery
chain and current-state transition passed preflight; actual outcome is pending.

No prospective slots or new benchmark have been launched. Readiness and final
engineering outcomes must be completed before any reliability claim.
