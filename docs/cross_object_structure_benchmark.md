# Cross-object articulation structure benchmark

Completed frozen run: [results and limitations](benchmark_evidence/cross_object_structure_20261004/RESULTS.zh-CN.md),
[machine-generated report](benchmark_evidence/cross_object_structure_20261004/REPORT.md).
All 16 unseen scenes remain in the results, including deployment and grasp failures.

## Run

On the experiment server, from `/data1/home/rangeryx/fr3_real2sim_piper_mobile`:

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_cross_object_structure_benchmark.py \
  --config configs/cross_object_structure.yaml
```

The registered run stops at 2026-10-05 05:00 Asia/Shanghai. To register a later
replication, copy the config, change the output directory and deadline, and keep
all scientific settings identical. Existing results are immutable.

Stages: `freeze`, `controls`, `controls-structure`, `heldout-controls`, `benchmark`, `summary`,
`full` (default). The original 5.5° contact/interaction controls gate main-set
execution. Supplemental full-structure controls then exercise the refinement
and held-out scheduling on the same DEV deployments, including when no unseen
episode reaches those stages. Their outcomes remain outside the main table.
Only accepted complete structure controls proceed to autonomous T0/T1/T2
prediction. `--prediction-gpus` changes compute allocation only; it does not
change scientific parameters or allow a failed model into prediction.

## What is frozen

The original contact baseline, unknown-joint controller, robust SE(3) optimizer,
mobile planner, collision checks, drive limits and proxy generator are imported
unchanged. The independent native entry verifies insertion points and baseline
hashes. Its additions schedule complete actions, update the estimated model,
and collect experiment records. There is no physics optimizer in this entry.

Asset selection uses only the already downloaded PhysX-Mobility archive,
metadata and visual geometry. All 25 previously prepared IDs are conservatively
excluded. A seeded, category-balanced pool is prepared using the existing
metadata-size import rule. Existing handle screening thresholds determine
eligibility. Eight assets are selected by category round robin and fixed
geometry score; two deployment configurations per asset are resolved and saved
before any main-set execution. Failed or unreachable assets are never replaced.

Original textured visuals are retained. Contact uses the existing approximate
semantic bar-and-support proxy. No claim of exact physical asset reconstruction
or hardware validation follows from this experiment.

## Information boundary

Scene setup owns the dataset URDF and one initial state initialization. Mobile
selection sees the initial visual handle descriptor, static cooked scene and
robot-only model. The controller uses q/EE/contact observations and its
estimated memory. The object joint getter is guarded until world pause and
saved estimates. Post-episode evaluation owns GT axis/line/final displacement.
The online stopping outcome is saved separately from the post-episode >=5°
evaluation result.

The simulated contact supervisor is unchanged. This is not a demonstration of
full slip observability using only actual PiPER hardware sensors.

## Discovery, refinement and held-out action

1. The frozen constrained-motion loop discovers articulation using measured EE.
2. The existing bounded refinement opens to an estimated 10° and attempts the
   existing small reversal. A reverse below the unchanged 1 mm threshold is
   marked unavailable; it does not fabricate an independent validation signal.
3. A complete, bounded forward validation segment is recorded separately.
   Robust joint fitting uses the preceding forward action. The existing noise
   scales, optimizer budget, modality-consistency and validation improvement
   rules decide acceptance. Independent validation cannot be replaced with a
   random split of neighboring samples.
4. Accepted T2 is written to structured memory. It drives the preregistered
   eight-second forward / two-second dwell held-out segment using the same
   constrained controller. This segment never enters fitting or selection.
5. After execution ends, T0 (dataset prior), T1 (discovery), and T2 (refined)
   receive the identical issued command tape in independent native worlds.
   Geometry/COM/inertia initial frames are preserved by the existing compiler;
   passive physics is unchanged. No per-frame robot/object state is injected.
   Incomplete prediction rollouts retain their failure and coverage, with no
   fabricated full held-out RMSE.

T0 is an explicitly labeled dataset prior, close to an oracle in SIM_TO_SIM.
It must not be described as a degraded visual Real2Sim reconstruction. Geometric
cross-action residual is reported separately from autonomous native prediction.

## Denominators and provenance

`per_episode.csv`, `per_asset.csv`, `failure_breakdown.csv`, `REPORT.md` include
every registered main episode. Controls are separate. Interaction coverage,
identification conditional on observable motion, end-to-end success, assets
with at least one success and assets with both configurations successful are
reported separately. Native startup/budget failures use a separate infrastructure
category rather than being relabeled as failed articulation identification.

Any infrastructure repair retains old logs and frozen manifest under
`infrastructure_history/`; all affected episodes must be rerun with identical
scientific settings. No successful-looking subset may be substituted.
