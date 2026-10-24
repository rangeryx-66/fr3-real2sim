# R02 recovery and active 90-degree interaction — 2026-10-24

**Manipulation lifecycle completed: actual 90.0109° opening, then 24.1368°
reverse travel, completed independent validation and verified unloaded parking.
Physics-ID accuracy is not established.** This report distinguishes two
separately registered episodes. No best results from different episodes are
combined into a successful trajectory. Both owned actors exited normally.

See [FINAL_AUDIT.json](FINAL_AUDIT.json) for derived outcomes and evidence hashes,
[RUN_CONFIGURATIONS.json](RUN_CONFIGURATIONS.json) for the actual motion profiles,
and [EXACT_SEALED_VALIDATION.json](EXACT_SEALED_VALIDATION.json) for predictions
using the exact models sealed before validation execution.

## Sources and preserved baselines

- V8.1 main: `81580cf0e39f762d2ad457612ae5677dcf27f458`, unchanged.
- Recovery release: `751bfeea73da2ec6ecd4036ee577e668946eddb8`, unchanged;
  exact original behavior source `dd83cb5a21da60490fbd44df3161d9cb05e93093`.
- Active experiment: `6728a2d7eeb5fecd99795e9b23b6ac1d51de3ffd` on
  `codex/r02-active-recovery`, based on V8.1 main.
- Follow-up code: `54a4074a13ceddc723197b295016dfd5cd6de436`, **not loaded into
  the active actor**. It fixes a newly demonstrated first-reverse admission
  error and requires post-goal chronology for reverse/validation completion.
- Shared private physics commitment:
  `ef92278ff59b4df57dbffd8e784e5eab56c51637f97cbf44830d920a023d523e`.

Original physical controllers, force limits, preload, native collision checks,
hard safety latch and exit reserves were retained. Prelaunch frozen-source
audits preserved 606 physical, 71 revolute, 956 release and 22 external files.
There was no drawer, unknown-joint or benchmark run.

Historical evidence remains separate: the historical R02 run opened 63.086°
and reversed 31.580° with completed validation/parking; the preceding restore
opened 63.053° and reversed 31.540° but lost transmission at 44.215° during
validation, so its overall result remains NOT PASS. The earlier `c1c1d22`
alternative-action and reachability work is preserved, not presented as a
90-degree result.

## Original-source reproduction

Episode `v81_r02_core_recovery_20261024_01` used `dd83cb5`, the existing entry
point and original motion profiles. Its first grasp stalled at 5.1128° and
safely unloaded. Fresh observation and a different grasp recovered within
the same episode:

| Measurement | Result |
|---|---:|
| Recovered opening | 63.0418196° |
| Completed reverse travel | 31.5447028° |
| Coordinate after reverse | 31.4971168° |
| Independent validation final coordinate | 57.2988941° |
| Final parked coordinate | 57.3010592° |
| Final pad loads | 0 / 0 N |
| Actor exit | normal, code 0 |

Validation initially waited for reverse FIT publication, then completed after
a fresh current-state regrasp. The frozen validation model hash was
`815020ff60447666e4f115548fb7c99e60c5f72e9320ff297d9541968655709f`.
FIT had 835 positive/445 negative regression bins; held-out validation had
1258 bins and was not used for fitting or model selection. Coordinate
prediction RMSE was 0.0467951 rad (2.681°), final error 0.0799053 rad (4.578°).
The successful physical trajectory does not establish parameter accuracy.

## Recovery implementation and regression

Episode-level bookkeeping preserves the original goal and measured current
coordinate through individual planning failures, safe partial progress,
transmission degradation, new views, new grasps and station moves. Failed
requests cannot be replayed unchanged merely by changing the reason string.
Recoverable action timeouts return through the original unloaded boundary;
hard stops, untrusted controls and unavailable safe boundaries remain terminal.
Finite wall/simulation budgets and protected exit reserves still apply.

Old-angle observations are retired before reconstruction but retained as raw
evidence. Native whole-chain certificates resolve reverse-target coverage;
they do not bypass native IK/collision rejection. Numerical publication
failures are kept separate from controller faults, immutable failed FIT jobs
may be retried, and pending validation does not erase manipulation progress.

Before launch, all 166 V8.1 tests passed on the server. The follow-up code ran
168 local tests: 167 passed and one environment-dependent test skipped.
Tests cover actual session recovery paths, premature finish, repeated failed
requests, native coverage, observation retirement and model publication.
The opt-in engineering profile is restricted to the registered revolute setup;
it is not a claim of generalization to drawers, unknown joints or hardware.

## Active episode recovery evidence

Episode `v81_r02_active90_20261024_01` preserves the 90° goal throughout.

1. The first grasp stopped on contact transmission stall at about 19.192°,
   unloaded and parked. The episode continued with fresh wrist observations.
2. Several first-reverse requests were rejected before native checking because
   their first target lay outside the nominal forward arc. The outer error
   misleadingly reported IK search exhaustion. These refusals **do not prove
   workspace infeasibility or a door limit**. Private counterexample artifacts
   retain the exact candidate pool, responses, source hash and coverage audit.
3. Reordering the same current-state chain to put a short forward probe first
   passed the unchanged native checker. A new mid-bar grasp reached 25.801°,
   reversed to 13.160° (12.641° travel), and then opened to 86.1561155° before
   sustained low transmission invoked the original safe unloading procedure.
4. A new high-angle camera request was rejected by IK. A proposed carriage
   station at [1.27, -0.60] m exceeded the native carriage limit with stopping
   reserve; [1.15, -0.60] m passed. This is a carriage constraint, not a door
   limit. After moving, fresh wrist evidence produced a different upper-bar
   grasp and a checked chain to 90° followed by reverse motion to 1.15 rad.

All station changes occurred unloaded, with no object joint command. Current
wrist boards, plan boards and contact feedback were actually viewed. The
corrected feedback renderer separates Cartesian servo error from rigid-hinge
residual; the legacy radians-versus-metres rail panel was not used for decisions.

## Final active result

| Measurement | Result |
|---|---:|
| Opening target | 90° actual joint coordinate |
| Completed opening coordinate | 90.0109376° |
| Maximum recorded coordinate | 90.0109718° |
| Subsequent completed reverse travel | 24.1368094° |
| Reverse endpoint and final parked coordinate | 65.8741283° |
| Third-attempt acquisition | original one-second hold passed |
| Both final validation segments | TARGET_REACHED, transmission valid |
| Final park | native_rest reached, unloaded, 0 / 0 N |
| Normal finish / actor exit | code 0; actor and launcher PIDs absent |
| Wall time | 6208.094 s |

Three physical grasps and two recovery station relocations were used, in
addition to the initial station preparation. The first two contact failures
remain in the record. No hard latch was cleared, no force threshold changed,
and no object-state reset or direct object-joint actuation was performed.
The failed 86.156° segment was not counted as reaching the goal; the fresh
upper-bar grasp completed the remaining motion.

The executed `6728a2d` finish guard reports total reverse travel including
the early probe. The post-run audit explicitly checks temporal order:
segment 5 reached 90°, and segment 6 then completed the 24.1368° reverse and
independent validation. Thus the stronger post-goal criterion implemented in
`54a4074` is independently satisfied by this run as well.

The third-attempt reverse feedback has maximum absolute TCP tracking error
0.525 mm and rigid-hinge residual 0.729 mm. Raw positive-normal reports vary
across the fingers; absent positive reports alone are not interpreted as
physical separation. Actual object motion and the original transmission
monitor determine progress.

## Physics-ID outcome

Qualified FIT has 1621 positive and 233 negative regression bins from four
trajectories. The frozen selected model estimates effective inertia
0.130266 kg m², damping 0.513135 N m s/rad and Coulomb-like resistance
0.023101 N m, with conditional standard deviations 0.029145, 0.063991 and
0.000623 respectively. These are conditional estimates, not calibrated
physical truth. The position-dependent resistance candidate was estimated
but did not improve FIT-only trajectory cross-validation: its effort CV RMSE
was about 0.005544 N m versus 0.004874 N m for the constant-resistance model.
It was therefore not selected; spatial resistance accuracy remains unproven.

Both final validation actions were sealed with model SHA256
`6d7c8b9af817dc3bae7faf62c884837dcdcb8a19a163533fd7e94fdf52834c65`.
The production worker republishes FIT-only estimates after new data arrive,
with very small numerical changes. To avoid confusing those republished
coefficients with the pre-execution model, the final metrics below were
recomputed after actor exit using **the exact sealed coefficients**, the
unchanged production processing/prediction functions, and no fitting or
model selection. No validation sample was used to fit/select the model.

| Held-out segment | Full physical duration | Supported prediction duration | Coordinate RMSE | Final coordinate error |
|---|---:|---:|---:|---:|
| Final opening to 90° | 8.4750 s | 7.4625 s | 0.8193° | -1.4607° |
| Post-goal reverse | 30.5083 s | 29.4958 s | 6.4386° | +11.1810° |

Filter-boundary exclusion explains the shorter prediction horizons. Both
predictions leave observed FIT support; neither uses subsequent measured
state correction. The reverse error is substantial despite effort RMSE of
about 0.003475 N m. **Physical validation execution passed; a claim of accurate
full-range dynamics or parameter identification did not.**

## Evidence limitations

Raw 240 Hz telemetry, wrist images, model snapshots, commands and failures stay
private. No hidden authored-physics parameters or passive camera images were
used for online planning. The existing FAST_DEV recorder saves short event
clips, **not continuous video**; the original-source clip had 29 frames and
duration 1.933333 s; the active episode also has 29 frames at 15 fps, 1.933333 s.
Full numeric trajectories are retained. No claim of
continuous visual coverage is made.

Conditional Physics-ID estimates exclude unknown systematic/sensor error.
Low effort-prediction error alone cannot establish parameter accuracy.
