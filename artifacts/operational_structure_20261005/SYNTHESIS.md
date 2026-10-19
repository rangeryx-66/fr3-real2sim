# Completed task-relevant structure experiment

Method commit: 7143b7a, based on bc0da74. No physics fitting. All original 63 baseline files remain unchanged.

## SYSTEM — every frozen fresh TEST episode

| Stage | Count / all |
|---|---:|
| reachable | 8/12 |
| mobile_recovered | 0/12 |
| bilateral_grasp | 5/12 |
| valid_interaction | 3/12 |
| end_to_end_success | 0/12 |

## STRUCTURE — initial stable bilateral hold plus >=5 mm useful measured interaction

| Metric | Count / valid interaction |
|---|---:|
| accepted_or_provisional_discovery | 3/3 |
| entered_refinement | 3/3 |
| heldout_validation_reached | 3/3 |
| operational_structure | 0/3 |
| high_fidelity_structure | 1/3 |
| heldout_improved | 2/3 |
| end_to_end_success | 0/3 |

## Per-asset success

At least one successful configuration: 0/6 assets.
Both configurations successful: 0/6 assets.

## First stop layer — distinct from post-evaluation retention failure

| Layer | Reason | Episodes |
|---|---|---:|
| DISCOVERY_OBSERVABILITY | UNOBSERVABLE | 2 |
| PHYSICAL_SAFETY | DANGEROUS_LOADED_CONTACT | 1 |
| HELDOUT_MODEL_CONFIDENCE | OPERATIONAL_PREDICTION_REJECTED | 3 |
| DEPLOYMENT | NO_MOBILE_RECOVERY | 4 |
| GRASP_ESTABLISHMENT | BILATERAL_HOLD_NOT_ESTABLISHED | 2 |

## Excluded-segment prediction, physical outcome and reconstruction

| Episode | T1/T2 pos RMSE mm | T1/T2 endpoint mm | T1/T2 tangent deg | EE reconstruction mm | Original HF flag | Actual opening deg | Final true relative slip mm | Result |
|---|---:|---:|---:|---:|---|---:|---:|---|
| test_45385_01 | 0.1222 / 0.0951 | 0.1211 / 0.2387 | 0.8558 / 1.2941 | 0.0498 | True | 2.9694 | 3.1247 | OPERATIONAL_PREDICTION_REJECTED |
| test_45671_00 | 0.1713 / 0.1003 | 0.2817 / 0.2236 | 1.0979 / 0.5121 | 0.7975 | False | 3.9159 | 1.2641 | OPERATIONAL_PREDICTION_REJECTED |
| test_45671_01 | 0.0326 / 0.0562 | 0.0554 / 0.1314 | 0.9970 / 1.0254 | 0.3728 | False | 4.8563 | 1.3451 | OPERATIONAL_PREDICTION_REJECTED |

## Interpretation boundaries

- Original HIGH_FIDELITY is the retained EE reconstruction/consistency criterion, not proof that the GT hinge or object trajectory was recovered accurately. See post-episode axis/axis-line errors.
- Direct model-validation stops are separate from physical safety stops. Post-evaluation true relative slip can reveal drift not observable in the frozen online contact-plane projection; it does not retroactively become an online signal.
- True relative slip is measured from gripper-to-moving-link transforms at episode end; maximum true slip over the trajectory is unavailable. Contact-plane drift is reported independently.
- The unchanged video overlay calls this contact-plane proxy "tactile drift"; it is neither complete slip measurement nor a claim that PiPER has independent tactile arrays.
- Held-out predictions are conditional geometric predictions using measured EE rotation as phase; no held-out poses enter fitting. This is not autonomous physics/input-response prediction.
- This fresh collection contains six Cabinet assets from PhysX-Mobility, two frozen configurations each. Approximate handle interaction proxies remain frozen; category-wide household generalization is not established.
- Both 7320 and 45621 regression controls pass; DEV 45600 demonstrates safe operational manipulation with 0.368 mm reconstruction residual, above the unchanged 0.30 mm high-fidelity criterion.
- DEV 45134 is retained as FINAL_TRUE_RELATIVE_SLIP rather than success despite opening 5.42 degrees.
- No TEST outcome was used to change thresholds, candidate budgets, controller or fitter. Full commands, observations, contact logs, failures and continuous videos remain in the original server run.

## Reproduce

```bash
python scripts/run_operational_structure_benchmark.py --config configs/operational_structure.yaml
```
For a future new run, use a new output directory and unexpired deadline; preserve this frozen evidence.

See REPORT.md, SYSTEM.csv, STRUCTURE.csv, per_episode.csv, per_asset.csv, POST_EVALUATION_SAFETY.csv, plots/, diagnostic_plots/, and evidence_index.json.
