# Native G1 contact-feedback grasp correction — one bounded development trial

Scope: **NATIVE_G1_CONTACT_FEEDBACK_CORRECTION / ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT**. References `bb3a443` and the unilateral fixed-TCP diagnostic `1dc554e`. The frozen benchmark, scores, source/configuration, grasp scoring and original automatically selected plans remain unchanged. This report concerns acquisition/holding only, not 30° task success. **Outcome: the one 45671 trial did not acquire bilateral grasp. Feedback unloading was observed, but residual tracking error and opposing clearance remained. The policy is an experimental development version, not a validated robust grasp Skill.**

## Archived CENTER diagnosis

The original 240 Hz records were replayed through the frozen closure module. Its aperture commands reproduced exactly (maximum error zero), allowing the unsaved TCP targets to be reconstructed. Measurements below compare reconstructed commands with the recorded physical TCP; the maximum tracking error includes the one-physics-step input/command alignment used by the original update.

| Archived case | CENTER time | Maximum commanded offset | Last measured offset along closing axis | Last TCP target error | Opposite contact |
|---|---:|---:|---:|---:|---|
| 45623 | 35.52 s | 2.000 mm | 2.027 mm | 0.528 mm | Absent |
| 45717 | 52.05 s | 2.000 mm | 2.180 mm | 0.511 mm | Absent |
| 45671 | 41.08 s | 2.000 mm | 0.879 mm | 1.239 mm | Absent |
| 45916 | 46.14 s | 2.000 mm | 1.913 mm | 1.126 mm | Absent |
| 45638 positive control | 3.69 s | 1.092 mm | 0.938 mm | 0.597 mm | Acquired |

All four failures saturated the existing **2 mm** command-displacement bound. For 45671, much of the commanded correction was not physically achieved. The other failures did move roughly 2 mm, so inaccurate arm tracking is not a universal explanation.

The actual closing axis is left-pad center minus right-pad center. A translation toward the contacting left finger retreats that finger and advances the opposite finger at fixed aperture. Signed native left-finger reaction projected positively onto this axis in all four failures; measured motion also had the expected sign. There is no evidence of a hardcoded world-axis sign error or swapped finger identity. Absolute native contact-normal alignment with the closing axis averaged 0.85–0.93 for the failures, versus nearly 1.0 for 45638. Only 28–85% of failed-case contact samples lay within the nominal pad XY footprint; 45638's corresponding samples all did. Whole-finger reporting does not independently certify a good fingertip contact patch.

The aperture plateau compounds these geometry/tracking limitations: unilateral CENTER schedules no continued closure, only a passive `min(previous command, measured aperture)` ratchet. The original 100 s closure budget eventually expires. At the last recorded pose, coarse forward rays from the opposite inner-pad footprint to unchanged native handle hulls found sampled minimum gaps of approximately **3.02 / 8.96 / 5.77 / 7.53 mm** for the four failures, versus **0.096 mm** for 45638. These are offline geometric projections, not measured tactile gaps or exact closest distances. They explain why a saturated local translation plus frozen aperture leaves the opposing finger unsupported; they are not safe squeeze targets.

The prior 45717 fixed-TCP experiment remains negative evidence: gradual additional closure alone raised one-sided load to **2.027 N**, with no right contact, and triggered the original sustained-force protection. It did not prove every centered pose infeasible and is not overwritten here.

## Isolated shared change

`jaw_contact_feedback_closure.py` replaces the closure policy **only in this development runner**. The previously unvalidated unconditional CENTER-closing module is not used or merged into any benchmark input.

During unilateral contact the new policy starts with aperture held and incremental TCP unloading. It derives correction direction from actual pad centers and load identity, retaining the original imbalance-to-speed mapping and its existing speed, step and 2 mm displacement limits. Over the existing filtered-load averaging interval, it records actual translation along that direction and the change in the contacting finger's load. A measured unloading response with motion in the expected direction permits slow aperture advancement; load growth or reaching the original preload returns to aperture hold/unloading. This feedback bounds reference progression rather than changing force protection. There are no new fatal checks, force symmetry requirements or sample-count convergence gates.

The slow-closure rate, native finite position actuator, load thresholds and original LOW_PRELOAD/FORCE_HOLD equations are retained. The actual response interval is the existing 19-sample filter at 240 Hz (**79.17 ms**, nominal 80 ms). Grasp pose, planner, opening controller, collision geometry, friction, dynamics, efforts and gains are unchanged.

`run_native_g1_contact_correction.py` reuses the frozen normal initialization, approach, closure loop, IK/collision/force protections, video and cooperative chunk export. At closure entry it latches `commanded q − measured q`, carrying the same position-servo equilibrium offset technique already used by frozen opening/local correction. It adds this constant offset to the existing centering IK target. This preserves holding equilibrium at the measured-pose handover; it does not change gains or re-anchor a slip reference. Opening is removed from this isolated grasp-and-hold runner.

## Offline verification

Rotated and reversed finger-frame tests cover either finger contacting, independent of world axes. They verify correction sign, the original incremental/displacement bounds, non-reopening aperture, holding on contact entry, resuming closure on unloading and pausing on renewed loading. A bilateral-contact sequence reproduces the original LOW_PRELOAD/FORCE_HOLD outputs exactly. Generated source and the exact approach/closure sequence compiled before Isaac launch, with policy/runner/configuration/original-plan hashes recorded.

Archived old/new replay covers the same four failures and positive control 45638. Old aperture commands reproduce exactly; new commands are non-reopening, while TCP-centering targets remain identical under the fixed recorded inputs. Feedback varies aperture advancement rather than replacing the grasp pose. These comparisons are counterfactual decisions using **old** measured inputs, not predicted successful physics. See `ARCHIVED_OLD_NEW_REPLAY.json` and `OFFLINE_VALIDATION.json`.

## Physical outcome

One fresh normally closed 45671 episode executed without manual correction, restart, stitching, release or regrasp. The original automatically selected plan/station was unchanged. The actual imported module and update filename were verified against prelaunch hashes. The earlier unconditional CENTER-closing version was not loaded.

| 45671 evidence | Original benchmark | Contact-feedback development |
|---|---:|---:|
| Bilateral grasp / FORCE_HOLD | No | **No** |
| Final measured aperture | 24.678 mm | 23.367 mm |
| Last commanded aperture | Original passive plateau | 23.083 mm |
| Final measured correction along closing axis | 0.879 mm | 1.367 mm |
| Last TCP target error | 1.239 mm (input-aligned) | 1.582 mm (post-step) |
| Maximum measured correction from acquisition origin | 1.098 mm | 1.993 mm |
| Peak raw left / right load | 0.446 / 0 N | 0.622 / 0 N |
| Opposing-pad sampled minimum gap | 5.774 mm | 4.887 mm |
| Existing 2 mm command bound | Reached | Reached |
| Simulation / wall time | 166.183 / 1067.677 s | 166.183 / 1085.820 s |

The approximately one-step alignment difference in the tracking-error columns is documented; neither error is used as a new execution gate. Axial motion improved, but **overall TCP tracking was not solved** and left loading did not decrease relative to the original 45671 case. The fixed equilibrium offset alone does not compensate all contact-dependent/off-axis pose error.

First unilateral filtered contact occurred at **123.617 s**, aperture 26.478 mm. The policy recorded **537 response windows**, with **225** showing load reduction and actual movement in the expected direction. During unilateral CENTER, aperture advancement was permitted for **805 updates** and held for **9411 updates**; no command reopening occurred. There were **10,221 nonzero incremental TCP command updates**, including small direction/boundary changes, with cumulative commanded path **2.008 mm** and maximum radial offset **2.000 mm**. These counts are control updates, not repeated grasp attempts.

Maximum measured TCP speed after unilateral contact was **0.976 mm/s**; minimum arm margin was **1.068 rad**. Maximum handle-relative acquisition displacement/rotation was **1.952 mm / 0.324°**. This includes intentional acquisition correction and is not post-grasp slip evidence. There was no valid acquired-grasp reference to reset. Native positive contacts remained left-finger–registered `original-35` only; the right finger never contacted. Raw contact points/normals, collider ordering, finger poses, drive/mimic state, aperture and commands are retained privately.

**Stop reason:** original 100 s closure budget expired with `BILATERAL_HOLD_NOT_ESTABLISHED`. No force/collision/joint/speed protection event occurred, and no limit was relaxed. Incidental door motion peaked at 0.0274° before any valid grasp and is not an opening success. The first failure mechanism persists at acquisition: bounded correction and measured unloading do not eliminate the opposing clearance. Remaining actuator/IK-to-physical tracking error also matters; the evidence does not isolate collision approximation as the sole cause.

## Generalization assessment and evidence

The closure policy consumes measured TCP, current finger centers, left/right simulated loads and the existing robot/gripper policy. It contains no object IDs, GT handle pose, articulation progress or GT grasp correction. The **episode still uses oracle initial planning and known collision models**; GT moving-body transforms are retained for offline relative-motion measurements. This is not fully perception-based or GT-free execution.

The rotated/reversed geometry tests and archived decision replays support reuse of the feedback rule, but one negative physical trial does **not** establish robust cross-object acquisition. The next concrete investigation should resolve physical TCP tracking under contact and the remaining simultaneous opposing-contact gap within permissible local corrections; squeezing harder is unsupported. No grasp-planner/collision change, additional physical test or further tuning was performed here.

Native PhysX whole-finger loads are simulation-only feedback. Physical G1 hardware deployment requires a suitable contact/force estimation interface; this report makes no hardware transfer claim.

Both 240 Hz streams fully decoded: **39,884 observations and 39,884 native physics records**. All **24,000 closure command records** decoded, with matching indexes and no chunk errors. The **166.167 s, 1280×960, 4,985-frame** video decoded without errors. The actor and launcher exited after cooperative export. Videos and raw streams remain private, backed up on the server and locally; only compact metrics, source/config provenance and evidence indexes are published.

See `PHYSICAL_SUMMARY.json`, `ARCHIVED_CENTER_MOTION.json`, `ARCHIVED_DIRECTION_CHECK.json`, `ARCHIVED_OPPOSING_GAP.json`, `CORRECTED_OPPOSING_GAP.json`, `ARCHIVED_OLD_NEW_REPLAY.json`, `OFFLINE_VALIDATION.json`, `PRELAUNCH.json`, `SOURCE_INTEGRITY.json`, `EVIDENCE_INDEX.json` and `PRIVATE_ARCHIVE.json`.

Active policy SHA256: `76a34ad55910febf0de8fc8853f6dd0aca9071ebc12581d0c1b8ab9e3d1e1d22`. Runner: `0cc6baf4f8480341c8a6a8b929c410a0f78a1d45a2e2ce60424c32211d1567f0`. Original plan: `f8ce794d9ebaf87fcc54f2588d3ff0d43837b488856b5d56a23b3b9f20b5546f`. Native config: `68fb6af5f4efd432b2355cabe37876a67f6c5ea3eac0b224603f08fb18d62939`. The inherited export label `shared_progressive` means centering was retained; active-source provenance identifies the contact-feedback module above, not the prior unconditional-closing module.
