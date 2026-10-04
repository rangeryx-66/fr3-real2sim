# Conditional dynamics under structure uncertainty

Independent experiment based on `4558a28`. No changes to the successful physical
contact, robot/proxy geometry, force limits, joint margins, controller, or mobile
recovery implementations. Mode is **SIM_TO_SIM_BLIND_SYSID**.

## One command (experiment server)

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
 /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
 scripts/run_interactive_twin_conditional.py \
 --config configs/interactive_twin_conditional.yaml --stage full
```

The configuration and data are frozen before running. The dated deadline is
2026-10-05 05:00 Asia/Shanghai. Repetition requires a separate output and deadline.
No grasp pose searches are performed.

## Two distinct questions

**A — Conditional dynamics:** initialize the robot once from the previous
experiment's stable force-hold q, finite-difference qdot, EE pose, gripper state
and actual hold command. The part assembly uses the initial visual/prepared pose
prior, estimated unchanged at the pre-probe stable grasp; no hinge angle, axis,
origin or moving-link trajectory is read to place it. This initial pose estimate
has uncertainty and is not claimed to be exact measured part tracking.

No contact manifold/impulse is copied. The standard guarded physical step loop
rebuilds contact for 0.5 seconds using the existing force hold. Bilateral contact
must pass the unchanged window test. All candidates use the same snapshot and
actual Cartesian command tape; subsequent robot and object states evolve freely.
The old unobserved one-second home warmup is omitted **only for A**. Render-only
warmup initializes camera buffers without advancing physics. Robot state is not
reset after contact reconstruction. Every failed reconstruction remains a result.

**B — Complete task:** retain the original initial state, passive warmup,
approach, physical closure, compliant motion and safety stops. A complete actual
command tape is generated from the reference with the unchanged controller;
each selected twin independently replays the same inputs. A's success never
implies B's success. Full-task failure cannot veto another A parameter candidate.

## Frozen bounded selection

Five correlated structure candidates come from the opening halves and reverse
EE segment scatter in two axis-direction and two axis-line coordinates. The
central candidate is the previous robust SE(3) estimate. No vertical-axis prior
or GT selection criterion exists. Each candidate first passes complete-action
kinematic train/validation residual checks. Initial solids/COM/inertial frames
are preserved by the existing twin compiler.

Each accepted structure receives the same nine `(tau_c, b)` values. Reference
resistance is hidden from the fitting function. No optimizer iteration beyond
this grid and no force/gain/mass changes are allowed. Native rollouts are shared
between reference resistance conditions because the actually issued input is
identical; this avoids spending the same simulation twice.

| Role | Complete action |
|---|---|
| Train P1 | Opening, 0.25 mm/s, active 1–6 s; 8 s total |
| Train P2 | Closing, 0.25 mm/s, active 1–5 s; 8 s total |
| Validation P3 | Opening stop/restart, 0.5 mm/s, active 1–3 and 5–7 s |
| New test P4 | Opening double pulse, 0.375 mm/s, active 0.8–2.4 and 3.2–5.6 s |

The preserved legacy log schema labels P3 as `train`; the frozen action contract
and this experiment's fitter override that label: P3 is validation only.
P4 differs from the already inspected reverse pulse. Predictions for P4 are
read only after train/validation choices and candidate IDs are saved. Adjacent
frames are never random-split. Selection includes trajectory, velocity, start
delay, final displacement, tracking and held stop-drift response. A stopped
task drive still has a grasped door and robot damping/gravity compensation; it
is **not** a completely free passive door.

The three comparisons use identical initialization:
1. fixed refined structure + wrong prior;
2. uncertainty candidate + same wrong prior;
3. jointly selected uncertainty candidate + calibrated effective resistance.

Profile intervals retain structure/resistance ambiguity. They are empirical
discrete support, not statistical confidence intervals or precise hinge torque
measurements. A useful selected simulator predictor does not prove uniquely
recovered physics. Updated twins are experiment artifacts, never promoted to
replace the successful baseline.

## Transfer and retained failures

Only if DEV same-start P4 held-out position RMSE improves by at least the frozen
5% gate in both resistance conditions does the runner attempt the preregistered
successful `test_45621_00` deployment. Controller, thresholds and grid stay fixed.
Its original resistance remains unchanged. Existing 12-scene outcomes remain in
their original benchmark and are never removed from the denominator.

All initialization failures, conditional failures, complete-task failures,
command tapes, video and native contact logs are retained. Full-task status and
actual opening angle are reported separately from conditional prediction errors.

## Sequence continuation versus same-start held-out response

The original P4 predictions follow independently evolving P1–P3. They are valid
sequence predictions, but their absolute error includes state offsets accumulated
before P4. After model selection, the same preregistered P4 waveform is therefore
also executed alone from the common stable-grasp snapshot, with fresh physical
contact reconstruction. No model, threshold, axis or resistance is reselected.
Both absolute and action-relative errors, velocity, delay, final displacement
and held-stop drift are retained. The full runner includes this verification.
The verification was added after inspecting the sequence prediction, so it is
explicitly recorded as a post-selection initialization check rather than a newly
invented unseen test waveform.

`REPORT.md` and `comparison.csv` distinguish both P4 scopes. All reported position
RMSEs are per-coordinate RMS, not Euclidean distance RMS. `updated_twins` contains
selected simulator predictors even when parameter identifiability is rejected;
read the accompanying `conditional_twins.json` and `selection_semantics.json`.
The diagnostic selected point must not be presented as a recovered physical value.
