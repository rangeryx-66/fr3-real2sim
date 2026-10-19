# Cross-object articulation structure benchmark

SIM_TO_SIM physical-contact benchmark on original PhysX-Mobility visuals with frozen approximate semantic interaction proxies. No physics fitting.

Frozen main set: 8 previously unprepared assets × 2 configurations; 16 episodes. DEV 7320 / 45621 are separate controls.
Interaction coverage: **6/16**. Correct type + accepted refinement among observable episodes: **0/6**. End-to-end: **0/16**.
Assets with >=1 success: **0/8**; both configurations: **0/8**.
Fixed-base feasible: 5; physically repositioned recovery: 3.

Discovery gates passed: 0. If zero, the stricter discovery-conditioned refinement rate is undefined, not a successful 0/0 result.

| Asset | Category | Interaction | Refined ID | End-to-end | First failure stages |
|---|---|---:|---:|---:|---|
| 9128 | Door Set | 0/2 | 0/2 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |
| 12480 | Dishwasher | 0/2 | 0/2 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |
| 7130 | Microwave | 0/2 | 0/2 | 0/2 | BILATERAL_GRASP_FAIL;BILATERAL_GRASP_FAIL |
| 10797 | Refrigerator | 0/2 | 0/2 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |
| 45600 | Cabinet | 2/2 | 0/2 | 0/2 | ARTICULATION_UNOBSERVABLE;ARTICULATION_UNOBSERVABLE |
| 38516 | Dressing Cabinet with Mirror | 2/2 | 0/2 | 0/2 | ARTICULATION_UNOBSERVABLE;ARTICULATION_UNOBSERVABLE |
| 45403 | Storage Cabinet | 2/2 | 0/2 | 0/2 | ARTICULATION_UNOBSERVABLE;ARTICULATION_UNOBSERVABLE |
| 48721 | Tall Storage Cabinet | 0/2 | 0/2 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |

## Interpretation and limits

- Failures and unexecuted cutoff episodes stay in the denominator. Reachability is not counted as a fitter error.
- Model acceptance uses measured EE only and a complete independent validation action. Reverse below 1 mm remains unavailable.
- Held-out command is frozen before execution. Native predictions replay inputs, not q or object states; incomplete replays have no full-trajectory RMSE.
- T0 is the dataset structure prior and is near oracle in this simulation. It is not a claimed visual Real2Sim reconstruction.
- Contact supervision still uses the existing simulated contact sensor; full online slip observability on real PiPER is unproven.
- The unchanged native SUSTAINED_RELATIVE_SLIP label combines contact-plane drift and estimated-model consistency error. It is not by itself evidence of true gripper slip; both signals and final post-evaluation relative slip are retained separately.
- Initial assembly uses the dataset URDF; controller, mobile selection and fit do not receive GT hinge. GT metrics are computed after the native episode stops.
- Mobile platform uses the frozen kinematic SE(2) route, then stops; wheel/navigation dynamics are not evaluated.
- No prismatic controller was added; this table covers revolute opening objects only.
- The conditional structure-ID denominator is retained grasps with >=5 mm EE travel. Some such trajectories still lack sufficient angular excitation or fail the frozen reconstruction residual gate.
- A provisional correct revolute label is not an accepted discovery or a successful refined model.
- Online stopping uses the estimated model and existing safety conditions. The actual door angle and final relative slip are read only after execution stops.
- Per-episode CSV separates passive startup drift from subsequent door displacement. A door that moved before bilateral grasp is not credited as a successful physical interaction.

## Physical motion and identification evidence

| Episode | Bilateral | EE travel mm | Door motion deg (post-eval) | Final relative slip mm | Min margin rad | Provisional position residual mm | First stop |
|---|---:|---:|---:|---:|---:|---:|---|
| test_7130_00 | False | — | 3.965 | — | 0.382 | — | FINGER_BACK_OR_ROOT_HANDLE_LOAD:gripper_link2 |
| test_7130_01 | False | — | -4.856 | — | 0.342 | — | BILATERAL_HOLD_NOT_ESTABLISHED |
| test_45600_00 | True | 20.008 | 1.683 | 0.239 | 0.153 | 0.381 | UNOBSERVABLE |
| test_45600_01 | True | 20.015 | 1.642 | 0.343 | 0.148 | 0.419 | UNOBSERVABLE |
| test_38516_00 | True | 20.003 | 0.835 | 0.102 | 0.079 | 0.792 | UNOBSERVABLE |
| test_38516_01 | True | 20.004 | 0.852 | 0.378 | 0.549 | 0.882 | UNOBSERVABLE |
| test_45403_00 | True | 20.006 | 1.061 | 0.623 | 0.074 | 0.431 | UNOBSERVABLE |
| test_45403_01 | True | 20.008 | 1.074 | 0.722 | 0.168 | 0.408 | UNOBSERVABLE |

## Held-out structural prediction

**NOT REACHED**: no accepted refined model completed the frozen held-out segment. T0/T1/T2 prediction improvement is unproven; no RMSE is fabricated or replaced by a training residual.

## Regression controls

- 7320: SUCCESS; grasp=True, opening=5.620856761932373 degrees.
- 45621: SUCCESS; grasp=True, opening=5.584717273712158 degrees.

Supplemental controls use the same complete refinement protocol. They remain outside the unseen denominator:

- 7320: SUCCESS; robust refinement accepted=True, actual opening=8.293981552124023 degrees.
  - T0: REPLAY_COMPLETE; held-out coverage=1.000, complete EE RMSE mm=0.000.
  - T1: SUSTAINED_RELATIVE_SLIP; held-out coverage=0.000, complete EE RMSE mm=N/A (incomplete replay).
  - T2: REPLAY_COMPLETE; held-out coverage=1.000, complete EE RMSE mm=0.018.
- 45621: SUSTAINED_RELATIVE_SLIP; robust refinement accepted=False, actual opening=5.767993927001953 degrees.
  - Stop-signal peaks: model consistency 1.007 mm; contact-plane drift 0.104 mm. The existing guard uses their maximum; no safety threshold was changed.

## Infrastructure provenance

The selected assets, initial configurations, grasp/controller parameters and scientific thresholds were not changed after seeing outcomes. See infrastructure_history for archived failed runs, implementation repairs and every affected rerun. Reporting-only corrections are recorded separately and recompute all rows.

## Reproduce

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/run_cross_object_structure_benchmark.py --config configs/cross_object_structure.yaml
```

Frozen assets and all exclusions: frozen_asset_manifest.json, asset_preparation/source_pool.json and preparation_inventory.json. Full logs and continuous videos remain under episodes/<id>/candidate_*/.
