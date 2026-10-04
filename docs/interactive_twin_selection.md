# Batch physics model selection

Independent experiment on `170bc15`; the contact, control, proxy, mobile,
structure fitting and safety baselines remain unchanged.

Completed native results: [7320 / 45621 report](interactive_twin_selection_results_20261004.md).

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_interactive_twin_selection.py \
  --config configs/interactive_twin_selection.yaml --stage full
```

Stages: calibration, prepare, bank, select, full, summarize. `--episode` selects
one registered asset/configuration. Existing measured responses are reused when
they are exactly the frozen proposal commands. No unseen final test response is
available to noise estimation, ranking, structure construction, or parameter choice.

## Fixed experiment contract

- Use all previously frozen, training-derived structural candidates from
  `3e5e856` and `170bc15`, with axis-line duplicate removal; at most ten. This
  retains the earlier hypothesis space, without selecting only its test winners
  or running a new structure fitter. No asset-specific pose or axis is added.
- The original nine `(tau_c,b)` points remain unchanged. All available low/high
  forward, stop/dwell, and safe reverse responses are collected before ranking.
  A reverse below 1 mm is unavailable; other data and hypotheses remain usable.
- New scoring uses EE translation/rotation/velocity, command-relative onset,
  and incremental stop response. q/qdot are diagnostics only. No duplicated
  final-displacement/tracking-error terms are added to the selection loss.
- Noise comes from existing robot-only repeats, contact repeats, and zero-drive
  high-frequency fluctuations. A 25-sample quadratic trend separates smooth
  motion from short-timescale fluctuations. Joint covariance is regularized
  using OAS with temporal blocks as the effective count. Onset uncertainty is
  propagated by block bootstrap at both existing command speeds.
- These are simulator repeatability scales, not real sensing accuracy. Common
  deterministic bias is not estimated by repeats and is not silently called
  noise. Candidate residuals cannot inflate the calibration scales.
- Evaluate all complete candidates on train and validation actions together.
  Retain every log-weight and at least top five hypotheses; no sequential hard
  pruning. The support is not a calibrated continuous confidence interval.
- Adequacy remains a whitened RMS threshold of 10. If **every** two-parameter
  candidate exceeds this threshold on validation, allow at most twelve additional
  native simulations points. Their static excess grid is `{0.004,0.012}` above
  dynamic friction, seeded from at most three structures and two train-ranked
  dynamic parameter points per structure. This cap covers the union of reference
  conditions for one asset. No further grid refinement is allowed.
- All original candidates remain recorded after extension. The optional model
  uses `tau_s>=tau_c` only in the PhysX native static-friction property at setup;
  moving friction and viscosity stay `tau_c,b`. No per-step object force, joint
  state write, attachment, drive, geometry change, or controller gain change.
- The unseen final command is a 12-second, 0.425 mm/s three-pulse action,
  registered before fitting. Old P4 is a separate, already-seen regression
  diagnostic. It cannot change selection or any weight/threshold.

PhysX describes this native static/dynamic distinction in its
[articulation friction documentation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html).
The extension must improve validation and the new test before acceptance;
otherwise keep the two-parameter family and report inadequacy/unidentifiability.

## Scope

`SIM_TO_SIM_BLIND_SYSID`, conditional dynamics A: one common observable grasp
initialization per independent action, real contacts rebuilt, then free evolution.
This is not a new full approach/grasp/manipulation experiment and not a real
hinge torque measurement. The existing complete-task results remain separate.

Outputs include calibration provenance, all candidate losses/support, native
setup audits, old-training-only reselection, regression and new held-out tables,
continuous probe videos, and the model-family acceptance/rejection decision.
