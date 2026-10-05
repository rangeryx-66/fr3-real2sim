# Paper storyline: internal structure-update comparison

Question: does a small amount of physical interaction and online articulation
updating improve **subsequent object-motion prediction and manipulation**, compared
with no global hinge update and a one-shot estimate?

This round uses the already-exposed six-asset/twelve-configuration diagnostic
cohort from `2df0826`. It is not a new unseen evaluation. All prior deployment,
approach and grasp failures remain in the system denominator. The physical
contact baseline, robot/gripper/proxy, force/friction, mobile recovery, structure
fitter and existing budgets are frozen.

## Run

On the experiment server, in `/data1/home/rangeryx/fr3_real2sim_piper_mobile`:

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
/data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
scripts/run_paper_evidence.py --config configs/paper_structure.json
```

The immutable comparison manifest fixes twenty native trials: five original
logging-only runs (three useful interactions plus two controls), then three
methods on five previously graspable configurations. Seven shared prefix
failures are retained without redundant deployment/grasp runs. No test assets
are selected or discarded during this experiment.

## Internal ablations

- **B0:** existing directional compliant drive with no global hinge model. Uses
  the original four observed-frame probe directions, retains the useful direction,
  and continues bounded compliant interaction. Its offline prediction baseline
  is a frozen local linear model of probe motion, not a fitted global hinge.
- **B1:** same discovery stage, then freeze the articulation estimate. Model
  confidence can reduce step/replan, but cannot invoke new structure fitting.
- **B2:** existing online updating/robust refinement. The existing independent
  validation action may reject a proposed refinement; it then falls back to
  discovery rather than erasing safe operation already performed.

All use the same approach/closure/hold prefix. Each trial physically establishes
contact from the same initial configuration; impulses are not copied. The actual
observable post-grasp q/EE/contact state is compared in `paired_grasp_states.csv`.
No attachment, object force, execution-time object joint commands, new controller,
physics fitting or asset-specific offset is introduced.

Budgets are common **maximum** budgets, not guaranteed equal realized motion.
Report actual probe cost and final operation progress; early safety stops count.
No continuation is permitted after a physical safety stop.

## Three separate outcomes

1. Operation: actual angular displacement, >=5 degree small interaction, hold,
   dangerous events and true relative drift.
2. Model value: prediction of a later excluded segment, separately for EE and
   actual moving object. No final test results enter model selection.
3. Reconstruction: type, axis angular error, axis-line error and uncertainty.
   The historical 0.30 mm EE residual remains a diagnostic, not object accuracy.

Predictions use measured EE orientation to supply phase and one initial test
pose for anchoring. They are **conditional geometric predictions**, not autonomous
command-to-dynamics forecasts or physics parameter identification. GT object
orientation may separately supply phase in the observability upper-bound
analysis; that is explicitly an oracle diagnostic.

## Ground-truth isolation

`EvaluationLogger` receives the simulator moving-link handle before the controller
GT gate is sealed. It writes object/EE SE(3) at the original render cadence and
returns no pose/error/state. Native control and fitting do not read its file or
buffer. Only the post-episode analysis consumes the GT object trajectory.
The first three logging-only trials retain the original method including its
original rejection stop; they do not use the new fallback behavior.

Full relative drift is computed from `inv(T_object) @ T_ee`, anchored at probe
start. Normal object opening is not counted as slip. Report maximum and final
translation/rotation drift. This independent evaluation signal is unavailable
to the present EE/contact-plane-only controller. Bilateral force is not evidence
of zero slip.

## Evidence and limitations

Object-oracle fitting uses exactly the same time support, robust SE(3) fitter and
excluded subsequent segment as EE fitting. It assesses the observation ceiling,
not online algorithm success. A better oracle fit does not authorize new visual,
tactile or collision development during this round.

These are project-internal ablations, not reproductions of official Act2See or
Tac-Man experiments. A 5 degree result is small interaction, not full opening.
A diagnostic benefit is not fresh cross-object generalization evidence.

Stop after this fixed comparison. Freeze outcomes, list what operation and
reconstruction evidence actually supports, and decide whether an independent
object observation is necessary before scheduling an eventual independent test.

The reporting layer also evaluates all frozen predictors on a common excluded
response per episode (`COMMON_ACTION_PREDICTION.csv`). Dataset GT structure is
an explicitly labeled diagnostic oracle, not a visual Real2Sim prior. No
post-evaluation metric affects acceptance, control, budgets, or parameters.

## Execution provenance

The first diagnostic attempt copied `selected_job` from a summary. That summary
omitted `active_structure` injected by the actual native runner. All three affected
logging-only configurations were rerun from their **actual** `job_private.json`.
The incomplete runs are retained under `paper_structure_20261005_v1` and excluded
as infrastructure-invalid, not silently counted as method outcomes.

Both valid regression controls are reused. A duplicate control initialization
was terminated; its collided video/process log is not delivered as complete
evidence. The original control completed with a complete independent object log.
Five logging-only EE tapes were verified byte-identical to the original tapes.
The effective design remains twenty valid native trials; the four infrastructure
attempts make twenty-four attempted trials, below the original cap of thirty-eight.

## Equal-phase prediction

The original local-linear translation-phase result is retained as a diagnostic.
The primary common-response comparison also provides a no-global-hinge local
geometric model: the probe-only EE translation/rotation slope is frozen, and
held-out **rotation only** supplies phase. It has no global hinge center and is
never used by the controller. B1/B2 likewise receive measured EE rotation for
phase. All use one initial held-out object pose for evaluation anchoring; none
fits a reference to the remaining test positions.

`frozen_prediction_protocol.json` timestamps this reporting protocol before any
useful held-out comparison completed. It does not alter online validation,
model selection, probing, control, or safety. B2's **proposed** refined predictor
and **actually adopted** predictor are separate columns: a rejected proposal is
not claimed to have guided the final manipulation.

## Frozen outcome

See [final report](../artifacts/paper_structure_20261005/FINAL_REPORT.md).
All methods retain 0/12 strict task success and 2/12 actual >=5 degree progress.
One-shot geometry improves common conditional object prediction over the local
no-global-hinge predictor on three configurations across two assets. Refined
proposals worsen position prediction on all three and are never adopted.
The same-period object-pose oracle exposes an EE-as-object observation limit.
This round stops and freezes evidence; no new test/perception/physics work follows.
