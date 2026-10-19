# V7 — frozen revolute subset registered

Reference `25e2189`; implementation `2a91dbf`. Before any research episode,
[BENCHMARK.json](BENCHMARK.json) freezes all eight objects / sixteen configurations,
71 source/protocol hashes and blind configuration commitment
`ef92278ff59b4df57dbffd8e784e5eab56c51637f97cbf44830d920a023d523e`.
The user-authorized active subset is four revolute objects / eight episodes;
all eight prismatic episodes are deferred as PRISMATIC_ENGINEERING_UNRESOLVED.
Research execution is **paused by user request**. All owned actors have exited. The engineering history below is preserved.

## Live research progress (blind; not final evaluation)

`45936_0` completed and its actor exited with return code 0. Its first grasp
acquired bilateral contact and held for one second, but opening stopped after
7.191 degrees of gain under CONTACT_TRANSMISSION_STALL. This failure is retained.
Fresh wrist observations and a different existing, fully checked grasp family
supported same-scene recovery. The recovery completed reverse travel to 2.979
degrees, opening to 9.014 degrees, a predeclared independent reverse validation
to 4.968 degrees, and held-out expansion to 20.053 degrees. Guarded release and
measured unloaded parking passed. Two physical handle acquisitions were used.

The immutable online selected model used FIT only. Its within-support validation
angle RMSE was 0.02235 degrees, while expansion beyond the approximately
0.53–8.88 degree FIT support had 6.03 degree RMSE. The latter is an extrapolation
failure, not validated wide-range dynamics. Conditional covariance excludes
compensation/contact/model-selection uncertainty. Authored truth remains hidden
until all eight authorized episodes and online decisions end. Wall time was
3610.7 seconds; private evidence was approximately 60.5 MB.

`45936_1` also completed with actor exit code 0 (2617.3 s wall). Its first
bilateral grasp/hold reached 9.399 degrees but stopped under
ADAPTIVE_HORIZON_WITHOUT_HEALTHY_PROGRESS / TRANSMISSION_FAILURE. Command-to-TCP
error remained approximately 0.6–0.8 mm while measured TCP/handle drift grew to
approximately 2.8 mm. Fresh same-scene observations and a different existing
candidate supported recovery: reverse to 3.972 degrees, opening to 12.020,
complete independent reverse validation to 7.961, then held-out expansion to
22.066 degrees. Release and measured unloaded parking passed. Two physical
handle acquisitions were used. The FIT-selected constant model's validation
angle RMSE was 0.01317 degrees and held-out expansion RMSE was 0.06893 degrees;
the latter remains outside measured FIT support. Hidden truth is still unread.

`46427_0` acquired bilateral contact and completed the original one-second hold.
Its three FIT actions reached 20.031, 6.988 and 15.011 degrees, but the following
predeclared VALIDATION triggered CONTACT_LOST_PAUSE_IN_SAME_SCENE / SAFETY_STOP.
Validation is incomplete and excluded; held-out metrics are N/A. Recorded
TCP/handle displacement had accumulated to approximately 10 mm before the last
reversal while command-to-measured TCP error remained approximately 0.5 mm.
Missing positive normal reports alone do not prove physical separation.

The frozen operation/serve lifecycle treats this SafetyStop as terminal and
closed the actor before the subsequent Agent feedback request. Guarded release
and parking were **not** verified. This preserves a contact-retention failure
and a recovery-interface limitation; it does not establish that no safe unloaded
exit could exist. No retry or controller modification was made. Actor exit code
was 0, wall time 1202.8 s. Exit code 0 is not task or safe-parking success.

`46427_1` completed 20.031 degree opening and reverse travel to 7.988 degrees,
then safely released and parked for Agent feedback. Measured relative drift
was approximately 4.6 mm with approximately 1 mm command/actual TCP error;
normal loads became asymmetric despite healthy progress. This justified a
proactive planned reacquisition rather than an uninterrupted long action block.
Fresh current-state wrist views were acquired; one proposed view was rejected
NO_IK, and a closer alternative succeeded. Two central grasp wrist variants
failed complete approach IK. Both failures and explicit automatic fallback were
retained. A different existing, native-feasible finger arrangement was selected.

The second planned acquisition/one-second hold succeeded, followed by a third
overlapping FIT to 16.026 degrees and complete independent reverse VALIDATION
to 10.989 degrees. Final guarded release and measured unloaded parking passed.
The FIT-only selected smooth-resistance model had conditional I=0.477668 kg m²
and b=0.181398 N m s/rad, with held-out angle RMSE 0.03810 degrees and effort
RMSE 0.00005652 N m within measured FIT support. Truth remains unread; these are
not yet parameter-accuracy claims. Two planned acquisitions were used, not a
failed-action recovery. No causal claim is made that wrist roll alone improved
retention. Actor exited with return code 0, wall time 3870.9 s, private footprint
64.90 MB. All low-level sources and policy hashes remained frozen.

`45600_0` acquired/held the handle and completed 20.037 degree opening and
reverse travel to 7.979 degrees, followed by verified unloaded parking. However,
the Agent did not publish its next passive-feedback command within the frozen
360 s unloaded wait: command 9 was published 816.6 s after response 8, after
the actor had already exported BOUNDED_AGENT_STOP. Command 9 was not consumed;
the subsequent reobservation request was never published. This is an Agent /
orchestration continuation failure, not physical grasp failure. Independent
VALIDATION is absent and N/A. No valid-episode retry was made. Actor exited
with return code 0 after 1883.4 s. FIT data and safe parking remain preserved.
Future passive feedback may be queued with a finite approved action; no new
physical decision is automated, and the frozen runtime/time limits are unchanged.

`45600_1` completed all predeclared first-acquisition targets: 20.025, 7.981,
14.024 and 9.986 degrees, including three FIT trajectories and complete
independent reverse VALIDATION. Original one-second hold and verified unloaded
parking passed. No recovery was needed. FIT-selected constant model:
I=0.713700 kg m², b=0.284666 N m s/rad, C=0.0245521 N m; conditional standard
deviations 0.000723 / 0.002258 / 0.0000218. Held-out angle RMSE 0.06701 degrees,
effort RMSE 0.00008786 N m within FIT support. Truth remains withheld. Passive
feedback was prequeued without any automatic subsequent physical decision.
Actor exited with return code 0 after 1820.1 s; private evidence 55.82 MB.

`45606_0` ended before observation or physical interaction with
INITIAL_VIEW_UNAVAILABLE: the initial coarse wrist observation route returned
NO_ARM_PATH. Runtime recorded zero physics steps, zero numerical records and
no Agent commands; the process exited after 30.57 s. This is a pre-physics
viewpoint/motion-planning limitation, not a demonstrated manipulation failure or
confirmed simulator crash. Its underlying route/input cause has not yet been
isolated. No physical attempt was consumed. The user requested a pause, so no
repair/relaunch was performed. `45606_1` is not started; eight drawer episodes
remain deferred. All seven launched research actors are confirmed exited.

## Pause summary

| Episode | Maximum actual angle | Complete independent VALIDATION | Verified final park | Acquisitions / outcome |
|---|---:|---|---|---|
| 45936_0 | 20.053° | yes; angle RMSE 0.02235° | yes | 2; first transmission failure, successful recovery |
| 45936_1 | 22.066° | yes; angle RMSE 0.01317° | yes | 2; first transmission failure, successful recovery |
| 46427_0 | 20.031° | no; contact SafetyStop, metrics N/A | **not verified** | 1; terminal interface prevented further Agent recovery |
| 46427_1 | 20.031° | yes; angle RMSE 0.03810° | yes | 2; proactive planned reacquisition, not failed-action recovery |
| 45600_0 | 20.037° | no; Agent continuation timeout, metrics N/A | yes | 1; useful bidirectional FIT preserved |
| 45600_1 | 20.025° | yes; angle RMSE 0.06701° | yes | 1; no recovery needed |
| 45606_0 | N/A | not executed | N/A | zero; pre-physics initial route unavailable |
| 45606_1 | N/A | not started | N/A | zero |

Six valid physical episodes used nine handle acquisitions. All six first
acquisitions established bilateral contact and the original one-second hold;
all six achieved useful opening and reverse travel. Four have complete
independent VALIDATION; five have verified final parking. These are stage
counts on development objects, not population robustness claims. The two
actual failed-action recoveries both succeeded; the proactive reacquisition
is counted separately. No final benchmark PASS or hidden-reference parameter
accuracy claim is available. Authored truth remains withheld while the two
remaining authorized configurations are unfinished. Raw evidence stays private.
No further actor launch or manipulation development is authorized during this
pause.

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
