# V7 engineering preflight — benchmark not yet started

Reference: `25e2189`. This file records engineering progress, not a V7 result.
No blind benchmark configuration or research episode has been executed yet.

Engineering resumed after the user's explicit prismatic-repair request. The
historical stop section below remains unchanged as a record of that stop.
Current work and negative repair tests are documented in
[PRISMATIC_REPAIR.md](PRISMATIC_REPAIR.md). Native transmission/reversal must
pass on both development drawers before the prismatic research episodes.
The user's subsequent continuation instruction authorizes the preselected
revolute subset if the current drawer regressions remain unresolved: four doors,
two blind configurations each. The eight drawer episodes will remain explicitly
deferred, without replacement or invented results.

## Intended object set

| Type | Asset / registered contact mesh |
|---|---|
| Revolute | 45936 / original-39; 46427 / original-38; 45600 / original-25; 45606 / original-24 |
| Prismatic | 19179 / original-11; 45290 / original-37; 27044 / original-16; 45746 / original-46 |

Selection used metadata, closed geometry and coarse station IK, before native
contact testing. The four doors are development objects, not unseen objects.
The fixed registered station and native safety geometry remain known-model
dependencies. Online grasp geometry comes only from fresh wrist RGB-D/SAM3.
Library URDFs have no collision elements: preparation uses the existing
per-source-piece convex-proxy convention. This is not exact collision CAD.
The public final manifest and blind configuration commitment must precede the
16 research episodes; neither planning nor physics failures may replace cases.

## Completed checks

- SI virtual-work projection, metre-valued progress/recording, unit validation,
  synthetic revolute/prismatic recovery and held-out isolation: 10 tests passed.
- Existing adaptive-segment and phase-aware safety regression scripts passed.
- V6 archived numerical replay: both constant and extended models reproduced
  with maximum parameter difference **0**. Archived results were not overwritten.
- Short native robot-only smoke: 278 physics steps, valid wrist F/T throughout;
  no object contact. Native linear-axis readback: Coulomb 0.3 N, damping 3 N·s/m.

## Contact engineering evidence (not benchmark outcomes)

| Scene | Asset | Bilateral acquisition + 1 s hold | First commanded target | Actual gain | Outcome |
|---|---|---|---|---|---|
| native_contact_00 | 27044 | yes | 20 mm | 0.114 mm | transmission failure; safe release/park; recovery reobserved but Agent budget expired before a second plan |
| native_contact_01 | 45290 | yes | 20 mm | 0.0021 mm | first grasp transmission failure; same-scene fresh observation/replanning selected another candidate; human interrupted its closure before probing |

Both first probes stopped around 1.804 simulation seconds under the unchanged
slip/progress protection, not overload. No reverse motion or independent
VALIDATION has yet completed. Neither scene establishes prismatic capability.

For 27044, archived actual TCP rail travel was 0.705 mm, door travel 0.114 mm,
relative drift about 0.594 mm, while scalar reference travel was 1.934 mm.
The reference lead must not be reported as measured slip. Commanded-TCP FK
travel was 1.921 mm; absolute command/actual TCP errors were approximately
0.646/0.609 mm at probe start/end. These recordings do not yet isolate grasp
geometry from compliance/tracking and progress-detection timing.

Agent recovery must inspect fresh wrist images and measured force/motion,
return to a verified unloaded boundary, regenerate candidates and check full
current-state execution. It must not simply advance to another benchmark case.
No clamp, gain, contact physics or safety-limit changes were made for these tests.

Private evidence is on `labserver_inschool` (`test-X640-G50`), under
`/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/cross_object_v7_20261019/`.
Raw camera, force and motion data remain private. Final benchmark statistics,
identification metrics and completion claims are pending native-contact preflight.

## Human-requested stop

The user stopped development to inspect the repeated stalls. All three owned
engineering actors exited (return code 0); no research benchmark episode was
launched. The second scene's recovery was interrupted during closure, with no
interaction segment, and is **not** another transmission-failure outcome.
Its last release/parking was not completed after the interruption; simulator
shutdown/export must not be called verified terminal parking. Both completed
first probes had previously released and parked successfully.

The repeated ~1.804 s probe termination consists of three 0.5 s increasing-slip
windows plus the original 0.3 s stationary hold. The new rail allowance is
1.85 mm (nominal loaded-speed stopping distance plus one physics step). It has
not yet been physically validated as an appropriate loading allowance.
27044 began >10 µm measured motion at ~1.333 s and progressed ~0.099 mm in
the final half-second window; this still met the inherited drift>progress
condition. Thus the common stop is not evidence of an immovable drawer.

Full tool-inertia compensation applied once gives SG121 reliable-interior
rail-effort medians 0.162/0.177 N, against authored Coulomb 0.3 N. Unfiltered
peaks (excluding two derivative-edge samples) are 0.355/0.266 N; these peaks
retain derivative/compensation uncertainty. No overload or informational-force
warning occurred during either probe. Damping contributes only 0.009 N at
the requested 3 mm/s. Authored moving-body masses are 1 kg abstract + 1 kg
contact-bearing link per drawer, not independently verified effective mass.

Archived cooked-geometry checks at 0/2/20 mm additional rail travel found no
moving-body intersection with table, pedestal or fixture. The inherited
asset importer disables internal asset self-collision; this is a representation
limitation, not evidence of realistic rail mechanics. No new physics was run.

The current evidence favors weak/delayed load transmission plus a possibly
premature rail stall diagnosis over excessive viscous damping. It does not
identify pad friction, compliance, controller tracking and effective joint
breakaway resistance separately. No friction, force, gain or protection change
is justified merely to manufacture a passing result. Development is paused.
