# 45415 grasp-closure repair: one physical verification

**ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — verified physical PASS, 30.000082°.**

Exactly one new independent normal-closed-start episode physically grasped registered handle `original-40` and opened the door to **30.00008201599121°** through robot contact. There was no release, regrasp, base/cabinet movement, attachment, artificial object actuation, restart, stitching or manual continuation. Existing collision/contact/force, 5 mm/s loaded-TCP and 0.05 rad joint-margin protections remained unchanged. The endpoint stopped successfully rather than through a protection exception.

| Configuration | Valid handle grasp | Maximum actual opening after valid grasp | Command reopenings | Result |
|---|---|---|---|---|
| Original Benchmark A station, original closure | Yes | 28.990278° | Not reevaluated here | Existing joint-5 margin limit |
| Improved station/plan, uncorrected closure, prior development_01/02 | No | None; tiny closure excursion uncredited | 188 | Bilateral hold not established |
| **Same improved station/plan, physically loaded repair, this trial** | **Yes** | **30.000082°** | **0** | **PASS** |

The old improved-plan failures are the controlled comparison. The original 28.99° result used a different station and is context, not an isolated comparison of this one-line repair.

## Execution proof and control scope

Starting reports/source were read at commit `93d6706`. The owner launched the explicit absolute executable:
`/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/scripts/run_oracle_door_closure_continuity.py`.
No owner's filename was replaced or inferred.

| Source/input | SHA256 |
|---|---|
| Actual runner | `5fb2530b0b637f266be51cc2025b5fae7f7f8b139358ee5a59ae57108ef02638` |
| Loaded active closure | `b522bcef78a1191a4a2a5c756c34d11d7df2e22ddaa5f782be61343947a1704c` |
| Exact copied whole plan | `97ab7f32022c2d119845b504afa7f78070701911903a0635135087727d02ee11` |
| Exact copied grasp/approach plan | `0b9d7173369db4f7c61c68dc2724a7d741d6f4f596039286e3fa01fe47da2b22` |
| Unchanged opening-controller excerpt | `08131e645e34119507edfd8fc0365ea22be1f67e907c37795a6560c4f74eb6f1` |

Runtime recorded the loaded closure class alias, actual update-code file, argv and source hashes. The active file's canonical path resolves through the repository's results symlink into `/data1/home/rangeryx/fr3_real2sim_piper_mobile/results/45415_closure_verification_20261011/provenance/active_closure.py`; this is the same verified file, not a fallback.

The sole control change is retaining the existing aperture command during unilateral centering:
`self.opening = min(self.opening, measured_opening)`.
Original centering, preload/effort, gains, contact thresholds, confirmation window, physical parameters, initial geometry/IK branch and Pink opening control were kept. Runtime executed **22,218 closure updates**, including **1,350 unilateral repair-branch calls**, with **zero position-command reopenings**. This is actual physical execution proof; the separate 24,000-input offline replay (188 → 0 reopenings) was only a pre-launch check.

## Physical evidence

- First positive intended-handle contact: **89.3125 s**.
- Existing bilateral/FORCE_HOLD confirmation: **97.975 s**, at effectively closed angle; filtered left/right loads **0.52165 / 0.52930 N**. Native contact target is `.../l_2/tn__l_2_original40_aL/tn__l_2_original40_aL`.
- Actual threshold crossing: **797.125 s**, measured angle **30.000082°**. Final double-precision conversion of the same native joint reading is 30.0000827971°.
- Loaded TCP maximum: **1.314707 mm/s**, below unchanged 5 mm/s. No loaded-speed violation or other terminating protection event occurred.
- Minimum joint margin: **0.130148 rad after valid grasp**, **0.122364 rad across all phases**; final margin **0.149717 rad**. The original 0.05 rad threshold was not relaxed.
- Peak pad loads, left/right: **0.96935 / 1.25244 N**.
- Maximum acquired-handle-to-TCP drift: **1.28029 mm / 0.40771°**.
- Trial duration: **797.129 s simulation / 3472.339 s actor wall** (~57.87 min, including initialization/export). Owner process lifetime was 3476.399 s. Continuous original video duration: **797.100 s**; the separate full-sequence 20× preview is not real-time execution.

**Contact stability is not solved.** First missing positive left-pad native report after acquisition occurred at **219.420833 s / 8.542219°**, initially intermittent. Across 167,797 loaded observations, positive native-contact duty was **77.6826% left / 100% right**; duty above the existing 0.05 N reported-load threshold was **68.1365% / 100%**. At the endpoint loads were **0 / 1.04408 N**. There were no intervals with both pads simultaneously lacking positive intended-handle contact. The door continued moving through physically effective interaction, and the original asymmetry logic did not convert a one-pad report change into task failure. This PASS establishes initial bilateral grasp and successful manipulation, not uninterrupted bilateral contact or fully reliable effort estimation.

## Pre-contact motion discontinuity: retained and disclosed

The same real closure-entry transient recurred at **5.404167 s**: **100.785419 mm/s**, with zero pad loads. The prior actual-q sweep over 5.383–5.458 s had approximately **0.557 mm** TCP excursion, no collision in the unchanged model, and **28.496 mm** minimum cooked gripper-component/object clearance. The cause is the existing switch from held arm position command to measured-q/static-IK command, with a single-step velocity feedforward increment; it was not sensor noise. It was kept to isolate the requested aperture fix. No loaded speed protection was weakened.

For completeness, the maximum over **all** phases was **441.113 mm/s at 1.516667 s in unloaded PREGRASP**, not 1.315 mm/s. Closure after first handle contact peaked at **0.943016 mm/s**. The existing 5 mm/s Cartesian guard is armed after valid grasp; the all-phase and pre-contact values must not be presented as compliant loaded motion. This round is a simulation result under the retained protections, not hardware motion-safety certification.

## Export reliability, preservation and evidence locations

Before physical launch, exact active-source loading and recorded-input replay were checked. Cooperative marker and real empty-Isaac SIGINT tests exercised the same pause/export finalizer on archived fixtures, with no robot/object and no physics steps. Each restored 2,048 observation, native and command records with exact values/types/float bits, sealed video and exited. The initial test's pickle object-identity comparator was corrected to compare raw data rather than cross-chunk string aliasing; that test record was preserved. These infrastructure checks consumed **zero physical trials**.

This actual episode exported **191,311 observation records**, **191,311 native physics records**, and **22,218 closure-command records**, with lossless gzip/pickle chunks and complete available 240 Hz contact/pose/joint/aperture/effort/reaction fields. Raw impulses, contact points/normals/collider IDs, sensor definitions, conversion/filter metadata and events remain intact. All indexed streams match buffered record counts; telemetry issues are empty. Metrics remained passive.

The unique reservation recorded one physical attempt, no retries or other assets. **11,247 frozen/old files were checked with no changes**; old and copied plans/source/manifest hashes match. The physical actor and all self-test processes exited normally; no paused experiment was left behind.

Server evidence root:
`/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/45415_closure_verification_20261011`.
The continuous original is `physical_01/contact_baseline.mp4`; short preview `physical_01/summary_20x.mp4`; raw streams `physical_01/{observations,physics_steps,closure_commands}_chunks`; sensor metadata, events, exact source/config hashes, startup/active provenance and export acknowledgement are beside them.

Local verified archive root:
`engineering_artifacts_20261011/45415_closure_verification/results`.
GitHub retains source, configurations, hashes, metrics and static evidence figures; raw video/telemetry stays on the server and local workspace. See [physical metrics](PHYSICAL_METRICS.json), [repair execution proof](closure_fix_execution.json), [contact changes](CONTACT_REPORT_CHANGES.json), [pre-launch checks](PREFLIGHT.json), and [evidence figure](PHYSICAL_EVIDENCE.png).

## Recommended next engineering action — not executed

Freeze this exact successful repair/configuration as a **one-run capability baseline**. The next narrowly scoped control correction should smooth the **unloaded approach-to-closure equilibrium handover**, retaining position/velocity-command continuity instead of hiding the 100.8 mm/s kick or relaxing protections. Separately examine acquired-relative-pose tracking/gravity compensation behind the later single-pad loading, using these saved raw records. Any future independent reproducibility or paired physical test requires a new task/trial budget; none was started here.

GT/oracle dependencies remain: handle mesh geometry, known hinge model, dynamic moving-body/articulation feedback and collision models. This is not visual/GT-free grasping, autonomous reconstruction, physics identification or real PiPER hardware performance.
