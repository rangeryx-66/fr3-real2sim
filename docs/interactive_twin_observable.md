# Observability-aware physics probes

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_interactive_twin_observable.py \
  --config configs/interactive_twin_observable.yaml --stage full
```

Run from the server checkout. Stages: `prepare`, `bank`, `adaptive`, `full`,
`summarize`; `--episode` selects one registered episode. Completed native trials
are reused only when the job matches exactly. Configuration is frozen on first
run. The configured Shanghai deadline remains a hard stop.

## Scope and unchanged components

This is **SIM_TO_SIM_BLIND_SYSID**, conditional dynamics experiment A. It reuses
the unchanged `run_interactive_twin_conditional_episode.py`, cold contact rebuild,
constrained feedback controller, force limits, collision acceptance, 0.05 rad
joint margin, robot calibration, SE(3) fitter, physical parameter grid and loss.
No baseline file is replaced. Full approach/grasp/manipulation results from
`3e5e856` remain separate; this experiment does not newly demonstrate full-task
replay or real hardware friction measurement.

Each reference probe and matching twin trial starts **once** from the same saved
observable stable-grasp initialization. Contacts are physically established
again. There is no contact-cache copy, attachment, moving-part trajectory input,
or execution-time object/robot state replay. Trials do not form one uninterrupted
physical action: this restriction makes the candidate predictions comparable
without hidden-state copying. Within every trial, PhysX evolves freely.

## Structure uncertainty

The longest available complete EE action provides the center. Existing accepted
joint fits can be supplied as centers. Four temporal blocks, leave-block-out
fits, four block bootstrap fits, and any separately observable reverse action
describe local variation. A finite-difference robust SE(3) Hessian profiles out
reference-pose and per-frame phase nuisance variables. The axis-line point has a
fixed gauge. Empirical scatter and local covariance generate at most five
correlated axis/axis-line candidates, screened against existing EE actions.

The unchanged 1 mm fitter gate is checked before fitting each segment. Short
reverse data are recorded as unavailable, and never fabricated or made mandatory.
Local covariance is an approximation, not a calibrated statistical confidence
interval; correlated contact/model bias can remain.

## Active action selection

The action set is frozen before execution:

- low-speed forward;
- higher-speed forward;
- short forward excitation followed by stop/dwell (validation action);
- forward then reverse, with the reverse part required to pass its own 1 mm
  observability check. The forward prefix avoids blindly driving into an
  unobserved closed stop; it is not counted as reverse excitation.

All external Cartesian inputs are generated from the saved **EE-estimated**
articulation and observable EE pose. The native controller still uses each
simulation's own q/qdot and computes its own feedback effort. This is not
reference-effort or q-trajectory replay.

At most 5 structures × 9 `(tau_c,b)` points are used. Each point is simulated
once for each of the four short action proposals (at most 180 native prediction
trials; 34 seconds of action time per parameter point). No extra optimizer
iteration occurs. Reference experiments execute at most four selected actions.
The prediction bank contains no future reference response.

After the safe low-speed seed, complete observable responses update joint
structure/physics support. Optional actions are ranked by noise-normalized
predictive variance per second over the remaining candidate responses. A
decision is saved before that reference action is physically executed. Censored
or predicted-unsafe actions are not scored as successful. A reverse that cannot
reliably produce 1 mm is skipped, and no replacement action is invented.

Train-profile selection and validation adequacy use the same loss and thresholds
as `3e5e856`. Low/high forward train physics; dwell selects/validates; a usable
reverse adds training evidence. A preferred grid predictor is reported separately
from identifiability. Multiple supported parameter combinations or inadequate
validation residual yield `UNIDENTIFIABLE` and a feasible parameter set.

## Held-out and regression

A 0.325 mm/s two-pulse action is frozen in configuration before training. It is
not in the candidate prediction bank and is run only after selection is saved.
Its response never changes model choice. Report trajectory RMSE, velocity,
startup delay, final displacement and held-stop drift.

The previously seen `3e5e856` P4 action is rerun separately as a **regression
diagnostic**, using its original native protocol and same initialization. It is
not called unseen data. The predeclared regression allowance is 0.01 mm absolute
RMSE, which is a prediction comparison criterion, not a collision tolerance or
a claim about physical PiPER sensing accuracy.

The three comparison labels retain their exact meanings:

1. `fixed_refined_wrong_prior`: center structure + wrong physics prior;
2. `uncertain_structure_same_prior`: EE-supported structure choice + same prior;
3. `uncertain_structure_calibrated`: structure + physical grid predictor.

The first model is not an untouched visual reconstruction or GT oracle.

## Artifacts

`structure_candidates.json` includes unavailable segments, covariance and all
fits. `prediction_bank_index.json` preserves native failures.
`adaptive/*/decision_*.json` and `posterior_*.json` establish decision chronology.
`selection_frozen.json` excludes held-out input. `heldout/results.json`, plots,
`comparison.csv`, `updated_twins.json` and native continuous videos contain the
actual results. GT-only evaluation files remain outside estimator interfaces.
