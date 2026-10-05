# Task-relevant articulation acceptance

Independent entry based on `bc0da74`:

```bash
python scripts/run_operational_structure_benchmark.py --config configs/operational_structure.yaml
```

Use the existing Isaac virtual environment on the server. For another experiment,
choose a new output directory and an unexpired Shanghai deadline. The recorded
configuration, method hash and manifest are immutable.

## Three distinct decisions

Physical safety retains the baseline contact-plane drift, bilateral contact-loss,
force, speed, effort, joint margin >0.05 rad and dangerous collision guards. The
contact-plane observation cannot measure every tangential slip component. Final
true relative slip is available only to the evaluation-only module after execution.

Model confidence retains the original 1 mm / 0.1 s monitor, but no longer calls
its event `SUSTAINED_RELATIVE_SLIP`. During refinement it pauses the existing
directional drive, fits all available training EE observations with the unchanged
robust fitter, reduces the next increment and refreshes the tangent. Three
unexplained independent windows, each adding at least 1 mm observed motion,
stop with `MODEL_CONFIDENCE_UNRESOLVED`; repeated dwell/refits cannot increment
that counter. After refit, confidence forecasts start at the newest observed
pose; the fitted latent reference and global reconstruction error remain recorded.
No threshold
change turns model error into a physical penetration allowance.

During independent validation and held-out execution the model stays frozen.
The monitor may evaluate its local prediction but cannot fit those observations.

Reconstruction accuracy preserves the original 0.30 mm high-fidelity criterion.
Operational acceptance instead requires stable family/support, non-worsening
training behavior, reliable excluded-segment prediction and safe subsequent
estimated-model manipulation. Predictive acceptance alone is not counted as
operational manipulation success.

## Calibration and leakage boundaries

Before DEV execution, existing safe DEV/regression EE logs are split contiguously.
All four declared diagnostic DEV assets and both regression logs are included.
The final approximately 2 mm segment is excluded from the calibration fit. Their
predictive error envelope fixes model-explanation and task-prediction scales.
The legacy fitter noise scales remain unchanged. No TEST outcome or GT enters
this calibration. The original .30 mm reconstruction criterion is not edited.

Episode training uses `ESTIMATED_FOLLOW` only. `STRUCTURE_VALIDATION` is a separate
complete action and is never appended to fit support. Its predictions fix axis and
axis line, anchor only its first measured pose and derive phase from measured EE
rotation. Chord tangents use at least 1 mm measured motion; noisy raw per-frame
tangents remain diagnostic. No constant reference is fitted across held-out positions. This is
conditional geometric prediction, not a claim of autonomous dynamics prediction.

Following acceptance, `HELDOUT_MANIPULATION` / dwell continue with the saved T2.
Those observations do not feed either fitter or model selection. Online stopping
uses estimated motion; actual ≥5° and true retained-grasp pose are checked after
execution by evaluation only.

After the unchanged held-out command, a frozen T2 can guide up to 60 further
incremental segments / 60 seconds toward the existing baseline's 5.5° estimated
goal. This uses only the remaining original 120 mm / 270 s motion budget; IK,
collision, force and contact guards remain active. Refinement is still capped at
120 segments, so the total scheduling bound is 180 segments. No additional fit
uses the validation, held-out or continuation observations.

## Freeze and reporting

DEV: 45600, 38516, 45403, 45134. Regression: original 7320 and 45621 jobs.
Both controls also execute the independent structure protocol, separately from
their legacy regression. Fresh TEST requires both baseline controls to pass, at least two DEV/regression scenes to
reach independent validation and actual soft refit events. Selection excludes
all prior exposure IDs and source pools, uses geometry/metadata only, and freezes
two configurations per asset before execution.

`SYSTEM.csv` retains every frozen episode. `STRUCTURE.csv` conditions only on
stable grasp and ≥5 mm useful interaction. Deployment failures are not structure
algorithm failures. Per-asset results remain separate from per-configuration
results. Incomplete episodes have no invented prediction metric.

All legacy grasp/contact/proxy/mobile/fitter files remain byte-identical.
No physics calibration, attachment, direct object force or runtime object-state
command is introduced.

## Completed 2026-10-05 experiment

[Report and immutable evidence](../artifacts/operational_structure_20261005/REPORT.md).
Fresh TEST: 6 assets × 2 configurations; 8/12 deployment-feasible, 5/12 bilateral grasp, 3/12 useful interaction, 0/12 end-to-end. All 3 useful interactions reached excluded-segment validation; 2/3 position RMSE improved, 0/3 operational acceptance, 1/3 original high-fidelity criterion. Both regression controls pass.

The original high-fidelity criterion measures EE reconstruction/consistency, not GT hinge accuracy. Post-evaluation full relative drift exceeded the original 1 mm retention bound in all 3 useful TEST interactions, while the online contact-plane projection stayed small. The model-validation stop and the subsequently observed full relative drift remain separate fields. The 0.30 mm criterion and physical safety limits were not relaxed.
