# Interactive twin experiment — 2026-10-03

## Scope and provenance

This experiment uses `SIM_TO_SIM_BLIND_SYSID`. No hardware data were collected. The reference and candidate twins are independent Isaac/PhysX simulations with real simulated gripper contact; there are no attachments, direct object forces or execution-time object state assignments.

DEV native execution version: `fa08d00`; TEST implementation amendment: `6e1f767` on `codex/piper-mobile-door`. The amendment only joins four scalar CLI options as `--option=value` to preserve negative scientific-notation coordinates. Frozen baselines `f5dcc6f` and `cd616606415bae9176f7545dd3e071f5e658fcbe` are unchanged. Native results are stored on the lab server under `results/interactive_twin_benchmark_20261003_v2`. The fixed deadline is 2026-10-04 05:00 Asia/Shanghai.

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
env -u PYTHONPATH -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u all_proxy \
  /data1/home/rangeryx/.conda/envs/anygrasp/bin/python \
  scripts/run_interactive_twin_benchmark.py \
  --config configs/interactive_twin.yaml --stage full
```

## Development evidence

DEV asset 7320 is excluded from unseen results. Its independent D-stage run completed actual contact interaction and EE-only identification:

| Metric | Measured result |
|---|---:|
| Actual door displacement, post-run evaluation | 5.620857° |
| Accepted EE model updates | 34 |
| Axis angular error | 0.758556° |
| Axis-line distance error | 11.0282 mm |
| EE-only fitted circle position residual | 0.24617 mm |
| GT-constrained reconstruction RMSE, post-run evaluation | 0.64269 mm |
| Final relative translation slip | 0.14539 mm |
| Minimum joint margin | 0.0634903 rad |

Source: `results/interactive_twin_dev_20261003/final_D_stage/report.json`. The slip value is a final, post-run relative-transform measurement; it is not a claim that complete tangential slip was observable online.

The saved final D estimate is frozen before physics probing. Each E-stage run re-establishes grasp and uses its own measured initial EE pose. Old D observations provide estimate provenance, not replayed state or current trajectory samples.

Two robot-only runs completed with the same external command. They establish simulation repeatability with declared signal floors. They do not establish real servo calibration or current-to-torque accuracy.

All six formal sensitivity runs (LOW/MEDIUM/HIGH, two repeats each) completed the common command protocol with bilateral grasp retained. The normalized pairwise response distances were 15.599, 63.564 and 48.078 noise units. This establishes response sensitivity under the assumed simulation floors; separate parameter identifiability and held-out prediction remain distinct tests.

All nine T1 training grid replays completed safely. The grid optimum was `tau_c=0, b=0`, exactly the initial physics prior. The program labels this `IDENTIFIABLE_ON_FROZEN_GRID`, with rank 2 and discrete support `{0} × {0}`; its best normalized RMS residual is **21.4233 noise units**. This is evidence of a distinguished grid minimum, **not noise-consistent physical parameter recovery**. No physics parameter changed in this DEV T2. The independent held-out comparison must determine prediction quality; no improvement is implied by the label. Kinematic/contact/robot model mismatch remains a possible source of the residual.

All four independent P4 replays completed:

| Twin | EE position RMSE | Final displacement error | Motion-onset metric error | Normalized loss |
|---|---:|---:|---:|---:|
| B0: dataset kinematics + initial physics | 8.858 μm | 2.572 μm | 29.167 ms | 32.4103 |
| B1: estimated kinematics + initial physics | 119.981 μm | 399.204 μm | 962.5 ms | 7599.9147 |
| B2: estimated kinematics + fitted physics | 119.981 μm | 399.204 μm | 962.5 ms | 7599.9147 |
| Oracle: GT kinematics + GT physics | 0 | 0 | 0 | 6.14e-8 numerical floor |

B2 exactly matches B1 because fitting selected the original physics prior. **Held-out improvement from physics calibration is 0%.** The kinematic update worsened this DEV prediction relative to the accurate dataset articulation. Opening successfully and identifying a revolute axis do not by themselves validate the twin's predictive accuracy. Oracle's autonomous, same-input response matches the reference; there was no state replay.

The onset metric is not servo latency. P4's drive starts at 1.0 s, and onset is the first sustained displacement above the preregistered 60 μm threshold relative to P4's starting EE position. Reference onset is 1.67083 s; B0 is 1.70000 s; B1/B2 are 0.70833 s, before the new pulse. During P4's initial one-second hold, reference and B0 drift by at most 24.47/25.41 μm, while B1/B2 drift by 98.09 μm. Thus the large onset error includes continued passive settling after the preceding autonomously executed probes. This is a real response mismatch and a limitation of the frozen onset metric, not a 0.9625 s command-latency measurement. The frozen metric is retained, with the physical trajectory errors reported alongside it.

The final DEV manipulation independently re-established contact and loaded the saved estimated articulation. It completed **5.61554°** actual opening, with final relative translation slip **0.14537 mm**, minimum joint margin **0.0634903 rad**, and peak single-finger handle load **0.57774 N**. This validates reuse of the estimated kinematic memory for manipulation, while the physics-prediction result remains negative. The original runtime field `physics_updated=true` means a fit was accepted; here its numerical parameters did not change from the initial prior. The controller did not use a physics-optimized policy.

## Development corrections retained in provenance

- External input replay uses the recorded Cartesian drive and gripper command. Every twin recomputes the unchanged feedback controller from its own robot state. Literal motor-effort replay had inadvertently frozen reference gravity compensation and produced unsafe responses; those development failures are retained.
- T0/T1/T2 retain the prepared asset's physical joint stops. The short operational policy window is metadata, not a replacement for physical limits. An earlier compiler erroneously allowed backward door motion during closure; its failed runs remain diagnostic evidence.
- E/F/G/H use the completed D-stage estimate. A short, independent 8 mm re-probe no longer replaces the refined D model before physics fitting.
- Nonzero initial-state baking preserves world poses, mass/COM/inertia and shifted physical limits. Initialization occurs once before execution.

At approximately 20:28–20:35 Shanghai time, the inherited proxy failed TLS connections to the NVIDIA asset server. DEV B2, Oracle, final interaction and several TEST setup processes failed before physics execution. Their logs were retained under `infrastructure_failures`; they are not physical grasp failures. Direct access loaded the same Isaac ground asset successfully. Recovery changes only the subprocess proxy environment.

TEST setup also exposed an argparse edge case for a negative, near-zero coordinate represented in scientific notation. A separate implementation amendment will pass scalar options as `--option=value`, preserving the exact parsed float. It must not round or change the deployment coordinate, touch the frozen baseline loader, change selection or drop any of the twelve TEST configurations.

## Interpretation rules for final results

- The TEST denominator is four preregistered asset IDs, each with three configurations. Failures and unexecuted configurations remain in the denominator. Three configurations of an asset are not three independent objects.
- Selection is conditioned on prepared revolute assets with visually suitable bar handles. It is not a random household-object sample.
- `OBSERVABLE_RESPONSE` does not mean Coulomb and viscous resistance are separately identifiable.
- `IDENTIFIABLE_ON_FROZEN_GRID` describes the fixed 3 × 3 simulator grid. Parameter support is not a statistical confidence interval or a real hinge-friction measurement. Report the training residual and independent held-out error alongside any accepted value.
- Report attempted, blocked, failed and completed stages separately. A prerequisite-blocked physics stage is not a measured physics-identification failure.
- Independent B0/B1/B2 comparisons must be counted explicitly. A partly completed held-out stage does not establish all three baselines. Relative improvement is undefined when the comparison error is zero.
- The physics update is evaluated through held-out prediction. Final manipulation uses the unchanged constrained controller and estimated kinematics; it does not demonstrate a newly physics-optimized manipulation policy.
- The inherited closure/centering/preload controller, grasp retention checks and safety supervisor use PhysX contacts. These need a hardware counterpart; they are not stock PiPER tactile sensing. The kinematic/physics estimator excludes those signals, but the complete grasp policy is not proven deployable with stock PiPER feedback. B3 is unavailable without independent current/effort calibration.
- Interaction geometry is an approximate proxy. No geometry-reconstruction improvement, complete joint limits, full online slip observability or hardware transfer is claimed.

## Batch results

Pending the bounded run. The final report must include every frozen TEST configuration and distinguish completed evidence from unexecuted stages.

The outcome-independent selection preview uses these visual mesh parts (dimensions are PCA length and cross-section extents, not a measured gripper aperture):

| Asset | Selected visual part | PCA extents (mm) | Minimum screened rear gap (mm) |
|---|---|---|---:|
| 7138 | original-8 | 403.78 × 15.91 × 27.37 | 15.11 |
| 45166 | original-26 | 208.33 × 42.01 × 49.99 | 30.54 |
| 45130 | original-29 | 55.66 × 22.65 × 46.46 | 9.62 |
| 45621 | original-33 | 130.87 × 25.57 × 32.63 | 23.02 |

Each has three preregistered initial configurations. These geometric screening values do not certify reachability or actual grasp success.

## 2026-10-03 visual import correction

The user correctly flagged the rendered TEST assets as incomplete. The cabinet
and door visual meshes are from the existing PhysX-Mobility dataset, not newly
constructed cabinet primitives. The contact handle remains the previously
approved semantic interaction approximation.

Isaac's URDF/OBJ converter emitted `invalid uvs` and omitted source visual parts.
Naming every visual uniquely did not fix this. For asset 45621, only 1 of 29
source visuals survived the original conversion. Omitting UV references from a
separate, source-immutable visual import copy restored all 29 parts. This fallback
preserves vertex records, face vertex indices, normals, scales and visual origins;
original texture appearance is not preserved. Material diffuse colors remain.
The original URDF, joint/inertial parameters and collision files remain unchanged.
The independent benchmark plant now uses this compatibility copy for import;
existing physical-contact and unknown-articulation baseline files remain intact.

Native Isaac preflight was repeated for configuration 0 of all four frozen TEST
assets. Repaired visual counts are 7138: 21/21; 45166: 22/22; 45130: 25/25;
45621: 29/29. Every complete cooked-collision JSON is exactly equal to its prior
configuration-0 export, including shape vertices, transforms and contact settings.
Candidate outcomes and saved feasible solutions are unchanged. Planning timings
naturally differ. These are implementation-equivalence audits, not additional
successful benchmark episodes. The earlier incomplete renders are invalid visual
illustrations; their original logs and physical observations remain preserved.

Local deliverable: `results/interactive_twin_deliverables/restored_dataset_scenes.png`.
Server audit: `results/interactive_twin_visual_import_audit_20261003/`.
