# Bounded SE(3) structure / physics refinement

Based on `f598240`. Mode: **SIM_TO_SIM_BLIND_SYSID**. Successful physical
contact (`f5dcc6f`) and unknown-joint control (`cd61660`) remain unchanged.

## Run on the experiment server

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_interactive_twin_refinement.py \
  --config configs/interactive_twin_refinement.yaml --stage full
```

The dated config stops native runs at 2026-10-04 05:00 Shanghai time. To repeat
on another date, copy the config to a **new output directory** and set a new
explicit deadline. Existing completed actions are never overwritten or silently
re-run. `--stage offline` reviews saved data only; `--stage supplement` spends
the one supplemental-action budget for the selected `--episode`.

## Scientific separation

- The calibration fitter optimizes 2 axis DOFs, 2 line DOFs, a free reference
  SE(3), and per-frame angles. The line's axial gauge is fixed at the position
  centroid. Setting the first latent angle to zero defines a coordinate and
  does not pin the first noisy measurement.
- Robust, noise-normalized position and SO(3) residuals are minimized together.
  Position-only and rotation-only axes are reported separately; short-arc
  conditioning is retained. No vertical-axis prior or GT acceptance rule.
- Calibration uses complete opening actions. A complete reverse action is
  independent validation. Transition/contact-anomaly samples are separately
  reported. Segment estimates are consistency diagnostics, not test data.
- Validation phase comes from measured orientation alone; measured validation
  position cannot choose per-frame phase. A new action can fit its constant
  grasp reference pose, but cannot move the axis/axis line.
- Discovery control retains its accepted coarse model and all original safety
  gates. Calibration changes separate twin artifacts, never the reference.
- The one supplemental action requests approximately 10 degrees estimated
  opening followed by 2.5 degrees reversal. It stops on the unchanged safety
  guards. Requested estimated angle is never labeled measured GT angle.

## Whole-action split

| Role | Action |
|---|---|
| Physics train | P1 opening 0.25 mm/s; P2 opening 0.5 mm/s |
| Physics validation | P3 stop/restart |
| Unseen test | P4 reverse pulse, 0.30 mm/s, active 1.0–4.8 s |

The legacy observable file schema calls P3 `train`; the independent experiment's
frozen `action_split.json` overrides that label. The new estimator accepts P1,
P2, P3 only, trains exclusively on P1/P2 and validates on P3. It rejects P4.
The P4 response is scored after structure and physics selections are saved.
The previous round's already inspected P4 is not called an unseen test.

Each parameter candidate receives the same **actually issued Cartesian command**
tape; feedback is recomputed from its own simulated robot. Observed robot q and
object joint trajectories are never imposed. Maximum 9 parameter candidates,
fixed robot response, inertia prior, contact friction and force limits.

## Diagnostics and limitations

- URDF geometry/inertial-frame invariance and native initial world COM/inertia
  are checked independently before fitting physics.
- GT combinations are isolated oracle diagnostics. Only the scene assembler
  and posthoc evaluator receive GT; fitters take EE/robot observable logs.
- The original resistance is retained. A second frozen nonzero hidden
  configuration (0.004, 0.3 effective simulator units) receives the same input.
  Native attribute authoring and actual signal separation are both checked.
- A best grid point is not precise friction recovery. Intervals are discrete
  profile support, not statistical confidence intervals. Censored/insufficient
  data or model inadequacy returns UNIDENTIFIABLE.
- Maximum true tangential slip is not fully observed by the existing contact
  safety model; final posthoc relative slip and observed surface drift are
  reported separately.
- The transfer protocol covers **45621 only**, three existing deployments.
  This run stopped at the DEV scientific gate, so none of those new transfer
  episodes ran. Original 12-scene failures remain; this is not four-asset
  generalization or real-to-sim success.

### Transfer registration (before new 45621 runs)

The passive parameters belong to the asset. Config 00 is preassigned to fit the
9-point grid; configs 01/02 reuse those parameters and intervals without fitting
new values. Each configuration still obtains its own EE-only kinematic estimate
and complete-action held-out prediction. GPU jobs share a bounded 4-device queue;
this changes scheduling only, not command sequences or physical limits.
