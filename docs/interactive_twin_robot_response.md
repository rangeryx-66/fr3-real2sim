# Shared robot response and passive object physics

Base: `79b100c`. Mode: **SIM_TO_SIM_BLIND_SYSID**.

## Reproduce (lab server)

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_interactive_twin_response_experiment.py \
  --config configs/interactive_twin_robot_response.yaml
```

The absolute deadline in the config applies to native jobs. A later run must use
an explicitly new output/deadline registration; frozen outputs cannot be silently
reconfigured. Dataset assets and previous train banks must be present on server.

## Robot prerequisite

Two certified no-contact logs share one fit of effective command delay and a
single directional Cartesian response pole. Frozen drive constants K=150 N/m,
D=100 Ns/m are retained. The reduced model is

`x_dot = v; T v_dot + v = active * (K/D) * (reference(t-Delta)-x)`.

Stop disables the position drive, matching the existing controller. It does not
continue to servo toward a stopped reference. First activation/stop fits the
parameters; the later restart/stop validates them. Separate no-contact Isaac
runs start at the saved robot postures for 7320 and 45621 and validate exactly the
same parameters. No asset-specific fit occurs. The robot base, gains, geometry,
forces and task scenes are untouched. Only the separate robot calibration scene
uses the existing out-of-reach object placement, as prior robot-only calibration did.

The logged pose belongs to the end of its command's physics step. Analysis uses
that convention consistently (4.1667 ms at 240 Hz); execution is unchanged.

The fitted T is an **effective Cartesian pole**, not measured firmware latency
or an independently identified actuator parameter. Native PhysX already contains
robot inertia/controller response: the reduced model is **not cascaded onto it**.
No arbitrary extra lag is inserted into robot commands. Calibration covariance
is propagated; a failed independent robot gate stops object fitting.

## Uncertainty and model selection

The unchanged selection score receives `Sigma_repeat + Sigma_structure +
Sigma_robot`. The original adequacy threshold stays 10.

- Repeat covariance: the existing repeated/zero-drive calibration.
- Structure covariance: existing EE-only bootstrap/Hessian estimates. Native
  response secants at one common wrong physics prior propagate the existing
  structure distribution; only training probe trajectories enter the covariance.
  Missing response directions receive paired native finite differences at the
  existing covariance scale, before test execution. These derivative models do
  not enter selection. At most eight derivative structures are allowed; the
  current 45621 bank needs four (two missing directions), or 12 train replays.
- Robot covariance: training-only local Delta/T information, accounting for
  temporally correlated samples and duplicate deterministic repetitions. Local
  low-frequency response derivatives propagate it. Systematic prediction bias
  is not relabeled as random noise.

All scales are frozen before new held-out response reads. No q/FK double counting
is introduced. The existing complete-action scoring and top-K/support policy are
reused unchanged. Static response covariance and native secants are local
approximations; passing their adequacy test does not prove exact object physics.

M2 uses tau_c,b. M3 additionally uses tau_s and is tested only if every M2
candidate fails validation adequacy. Per condition/family: at most two distinct
existing structure anchors, 12 bounded continuous function evaluations each.
Nelder-Mead runs in normalized parameter coordinates, within prior frozen ranges.
Only train actions drive optimization; all evaluated candidates subsequently
receive the same validation action. A budget ending is not evidence of a unique
optimum. Top-K and behavior-equivalent support remain available.

## Evaluation firewall

Two new command tapes are saved before object fitting:

- 0.400 mm/s pulse, 10 s, active 1.25–5.25 s;
- 0.450 mm/s start/stop/restart, 12 s, active 0.75–2.75 / 5.25–8.75 s.

The initial draft proposed 0.375 mm/s; a protocol audit found that speed in an
older P4. It was replaced before object fitting or any new test execution. Both
registrations are retained; no response informed that correction.

Each twin initializes once at the common observable grasp state, then receives
actual task-space commands and evolves through native contact. No observed
q/door trajectory replay, attachment, object actuation, robot gain fitting or
contact-rule modification is added. Old P4 is a regression diagnostic only.
New test responses never reselect parameters, axes, thresholds or scales.

Outputs retain native safety failures. A predictive PASS must be distinguished
from parameter identifiability and from a full-task or real-robot result.

These micrometre-scale no-contact errors describe deterministic simulation only;
they are not PiPER hardware accuracy or sensor calibration claims.
