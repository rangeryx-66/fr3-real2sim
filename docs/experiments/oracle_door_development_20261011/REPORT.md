# Focused PiPER door skill development — 2026-10-11

**ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT. No new ≥30° physical success.** Planning coverage was 2/3; one of three development objects established a correct-handle grasp. There were three new physical episodes: 45176 once, 45415 once plus the sole extra episode. 45203 remained a planning failure. No fourth episode or new benchmark batch was started. The original Benchmark A result and all frozen pipelines remain unchanged.

This round produced useful failure diagnosis, correct-handle candidate selection and improved predicted station margin, but **did not establish a reliable improved grasp-and-open skill**. Two execution-handling errors limit the evidence: the 45176 native SIGINT exit bypassed buffered raw export, and the sole targeted retest mistakenly loaded the original runner rather than the submitted closure repair. Neither is hidden or counted as a successful repair validation.

## Before / after

| Object / intended handle | Archived Benchmark A | New planning | Correct-handle physical grasp | New maximum valid opening | ≥30° | Actual blocker |
|---|---|---|---|---|---|---|
| 45176 / original-46 | Wrong trim original-39; 0.35646°, TCP speed stop | 30° arc, minimum approach/arc margin 0.07997 rad | Yes; valid-grasp sequence completed and acquired transform saved | **1.30697° observed at 1 Hz**; exact native maximum unavailable | No | Correct handle still gave ineffective transmission: unilateral contact, opening stalled/receded, 10.073 mm final sampled drift. Owner ended bounded development attempt; not a mechanical limit. |
| 45203 / original-53 | Wrong trim original-52; 0.82186°, ineffective pulling then contact loss | No initial Pink IK in the registered family | Not attempted | — | No | Required candidate TCP height 0.80888–0.80896 m exceeds even the loose 0.79324 m kinematic height bound at the unchanged base height. |
| 45415 / original-40 | Correct handle; **28.99028°**, joint-5 margin boundary | 30° arc, minimum approach/arc margin **0.12236 rad** | No in either new episode; correct handle touched, bilateral preload not established | —; raw closure excursion 0.00004348° is **not credited** | No | Original unilateral-centering branch repeatedly reopened the position command and unloaded contact; both episodes exhausted unchanged closure budget. |

No new success-rate claim can be made from these selected development cases. In particular, the previous 1/10 benchmark is not overwritten, and its trim-grasp success is not relabeled as handle success.

## Archived diagnoses that determined the changes

**45415:** Stable original contact and low slip transmitted opening to 28.99028°. Actual J5 was about −1.171636 rad; commanded J5 about −1.171727 rad against the −1.2217304 rad lower limit. The next command reached the unchanged 0.05 rad margin. Loaded TCP maximum was 0.622 mm/s and relative position drift 0.421 mm. This supports selecting a station/branch by worst margin over the full approach/opening arc, rather than changing the protection.

**45176:** At t=53.729167 s in the archived failure, measured TCP speed rose to 5.17761 mm/s as pad loads changed to 0/2.14442 N. Reference translation stayed at 0.71613 mm/s and maximum joint-command increment remained approximately 0.000061636 rad, consistent with preceding steps. The impulse/contact redistribution correlates with the spike; it does not prove a unique causal ordering within one physics step. The wrong trim contact was the first correction, not controller-gain tuning.

**45203:** The TCP tracked its reference within 35.7 μm while the commanded articulation led actual opening by over 2°. Relative drift eventually reached 17.448 mm / 4.191°, followed by loss of both loads. This was failure to transmit motion through the selected trim, not a global IK or tracking-quality veto.

These diagnoses used archived reports, continuous videos, native collider identities and 240 Hz records. No physical replay was used to rediscover the old failures.

## Exact shared changes and their validation status

1. **Oracle handle ROI:** Each job supplies only its registered handle mesh. Candidate poses are still generated geometrically from that asset's sections, axis, outward normal and pad geometry. No successful-object grasp template or manually tuned per-object pose is used.
2. **Native closing-sweep ranking:** Sweep the original 100 mm aperture down to zero and inspect native hull contact positions using the existing distal/inner contact definition. Record finger-root contacts and non-handle closing collisions for candidate ranking. The full mesh width includes supports and cannot substitute for local bar contact width. This is planner evidence, not a new runtime assertion.
3. **Approach / opening station ranking:** Search the existing bounded base x/y/yaw, orientation and wrist-branch family. Rank contact risk, achievable interval, contact estimate and minimum margin over approach plus 0–30°. Retain original FCL/camera collision checks, Pink limits and tolerances. Refine candidate arcs at 1° spacing and validate the existing home/pregrasp/approach route. No global optimality is claimed, and a partial arc can still be selected.
4. **Correct offline scene reference:** Derive the source base reference from the immutable cooked export. The old source report had subsequently been changed to the executed station, so it is not a reliable reference for translating the exported support into new candidates. Runtime continues to export its actual newly initialized scene.
5. **Opening controller unchanged:** Keep the physically acquired moving-handle-to-TCP transform, acceleration-limited reference, loaded equilibrium bias and continuous Pink velocity step. No opening-gain, force, velocity or margin change was supported by the archived evidence.
6. **One-line closure repair, not physically validated:** In unilateral CENTER, retain the last position command instead of copying measured, contact-deflected aperture back to the command: `self.opening = min(self.opening, measured_opening)`. This lives in a separate runner namespace; original closure source stays unchanged. Recorded-input replay reduced command reopenings from **188 to 0** over 24,000 inputs, with policy/gains/thresholds unchanged. Such replay does not predict the resulting physical contact dynamics.

The targeted owner used a quoted bare-filename substitution that did not match the quoted `scripts/run_oracle_door.py` launch path. Its actual cmdline and saved source hashes prove development_02 loaded unchanged closure logic. `targeted_retest_owner_executed.py` preserves that owner; the corrected owner is supplied for future reproduction only. **The sole extra physical budget was consumed; no further trial followed.**

## Actual physical measurements

| Episode | Valid grasp / opening | TCP velocity | Slip | Peak pad loads, L/R | Joint margin | Simulation / wall time |
|---|---|---|---|---|---|---|
| 45176 development_01 | Correct handle grasp; sampled max 1.30697°, last sampled 1.17607° | Exact 240 Hz maximum unavailable | Last sampled 10.073 mm; orientation unavailable | Sampled peak lower bounds ≥0.46947 / ≥1.09215 N | Sampled loaded minimum 0.17495 rad; true minimum may be lower | Video 201.500 s, latest live state 200.996 s / owner wall 943.570 s |
| 45415 development_01 | No valid grasp, no credited opening | Closure maximum 100.785 mm/s **before contact**; after first positive handle contact maximum 0.227 mm/s | Not defined after grasp; no valid grasp | 0.093761 / 0.073372 N | Minimum across all phases 0.122364 rad | 105.404 / 553.562 s |
| 45415 development_02 | Same unchanged controller and same failure | Same measured trajectory | Not defined after grasp | 0.093761 / 0.073372 N | 0.122364 rad | 105.404 / 492.896 s |

45415's 100.785 mm/s sample occurs at t=5.404167 s, with zero pad loads, on entry to static closure IK. Command increment was 0.000712222 rad. First positive handle contact was at t=89.3125 s. After that contact, the maximum measured TCP speed was 0.226827 mm/s at t=98.145833 s. The legacy loaded-TCP protection is armed after valid grasp; its code and 5 mm/s threshold were not changed. The unloaded closure-entry correction remains a motion-continuity issue for a later, separately authorized correction. No safety success is inferred from a missing loaded metric.

45415 had 25,297 observation records and 25,297 native physics records in **each** episode at 240 Hz, with complete available native-contact, pad-load, joint, pose, aperture and effort fields. There was no valid acquired grasp, so zero-valued pregrasp slip fields do not prove stability and pregrasp no-contact samples are not labeled grasp-loss intervals.

45176's first acquired transform was persisted after original valid-grasp confirmation. Continuous video shows the selected handle interaction, and the allowed-collider metadata names original-46. One-pad zero samples did not cause a new fatal check. The owner stopped ineffective transmission after opening plateaued and declined while drift increased. Native SIGINT handling exited Isaac before Python's final export: **no raw 240 Hz chunks or buffered events survived for this episode**. Thus exact peak speed/load, orientation drift and contact-loss intervals are unknown. The surviving video, 1 Hz states, metadata, transform, job and hashes are preserved; the missing fields are not substituted with zeros.

## Collision representation and remaining blockers

The native/visual comparison confirms unchanged world bounds but filled rear cavities in the convex approximations of 45176/45203 bow handles. No collider was removed, split, regenerated or silently replaced. Native-hull/visual mismatch remains a simulation representation limitation; the current first-contact ranking is not a certification of good transmission.

45203 has an immediate workspace blocker for this generated grasp family. The sum of all URDF-chain origin-translation lengths is 0.893242 m; adding the unchanged root height −0.09999998 m gives a loose absolute TCP-height upper bound 0.793242 m, even before joint/orientation restrictions. Required candidate heights exceed it. This explains the family failure; it does not prove every possible grasp or a raised robot installation is impossible. Base x/y/yaw alone cannot raise the bound. Object geometry and robot mounting were not altered.

45415's better predicted branch did not compensate for a closure implementation defect. Retaining the last aperture command is implemented and supported by offline evidence, but an actual physical result of that repair is still absent. 45176 further shows that semantic correctness alone does not guarantee sustained contact transmission.

## Preservation, scope and evidence

All episodes use real Isaac contact. The base and cabinet stay fixed after normal selected-station initialization. There were no releases, regrasps, attachments, artificial object forces, mid-run object-joint commands, restarts or state stitching. Oracle handle meshes, GT hinge geometry, moving-body pose, articulation progress and native collision models remain inputs. This is not autonomous visual grasping, unknown-structure reconstruction or hardware performance.

Preservation audit: **11,247 frozen/source/old-result files checked, no changes or missing files**. Existing geometry, mass/friction/inertia preparation, gains, clamp effort, collision/contact logic, joint/speed/temporal-force limits and 0.05 rad margin stayed unchanged. All three owned actors exited; no paused experiment remains. Unrelated server jobs were not terminated.

Archive verification: **434 actually saved files**, SHA256 matched locally. Server: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/oracle_door_development_20261011`. Local: `engineering_artifacts_20261011/oracle_door_development/results`. Continuous originals are `ASSET/development_NN/contact_baseline.mp4`; raw chunks and metadata remain beside them. The 45176 missing buffer is an explicit archival failure, not a checksum mismatch. Short 8× full-sequence previews are separately labeled; they do not represent real-time playback.

GitHub publishes shared source, complete selected plans, registration, diagnosis, hashes, passive metrics, static figures and this report. Raw video/telemetry remains on the server and local workspace. See [one-page summary](ONE_PAGE_SUMMARY.md), [reproduction](REPRODUCTION.md), [targeted trial discrepancy](TARGETED_RETEST.md), [preservation audit](FINAL_AUDIT.json), and [evidence figure](PRESENTATION_EVIDENCE.png).

**Next steps, not performed in this round:** independently physically validate the closure-continuity repair on the same plan; make evidence flushing/cooperative shutdown reliable before any owner stop; improve contact-transmission ranking with pad-patch/handle-hull overlap rather than first contact alone; then replace oracle ROI with perception and run a newly registered evaluation. Do not present the untested repair or predicted 30° arcs as physical capability.
