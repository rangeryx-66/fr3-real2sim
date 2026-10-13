# Native G1 initial-contact grasp selection — bounded development

Scope: **ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT / NATIVE_G1 / INITIAL_CONTACT_SELECTION**. Reference `5d15b90`. The frozen ten-door benchmark and both isolated closure-development versions remain unchanged. One short 45671 grasp-and-hold trial was executed; no full opening, manual grasp offset or repeated physical trial. **Bilateral acquisition succeeded with the original closure controller. Terminal holding then unloaded the left finger; continuously bilateral stable holding is not demonstrated.**

## Finding and shared change

The previous selection can score an attractive final bilateral aperture even when the first finger engages much earlier and blocks reaching it. `closing_quality` accumulates the two first-contact records across different widths; `common_quality` then takes the best final common-aperture support. That is not evidence the original safe closure can reach that final pose without loading the first finger. The failed 15° pitched grasps and the negative fixed-TCP squeezing diagnostic support this mechanism.

The new shared function `initial_contact_quality` uses the **same** closing audit and visual/native bilateral-support curves, but evaluates their interpolated support at the earliest native finger-contact aperture. It replaces only the contact utility used in ranking. Candidate generation, insertion/orientation variations, the verified direct-normal opening transmission term, root/other-object collision treatment, IK costs/branches, base search, approach checks and original 300 s search budget are retained. No new runtime acceptance/safety gate is introduced.

Archived native contact data also distinguish the regions: the four failures were curved bow handles, all selected at section0/insertion0/roll0/pitch15°, versus the short flat/straight 45638 handle at the same nominal parameters. During CENTER, only about 27.6%, 51.9%, 51.7% and 84.7% of contact samples for 45623/45717/45671/45916 fell inside the nominal pad XY footprint, versus 100% for 45638. These footprint projections are diagnostics, not added contact gates; zero rear-clearance alone is not a sufficient failure explanation.

All four failed cases' selected grasps had almost zero support at entry. Existing alternative candidates improve that entry support. The successful 45638 candidate remains near the top, with its original entry score about 0.000570. These are offline comparisons, not new physical outcomes on the other objects.

Fine diagnostics use unchanged native cooked object hulls and the official native G1 geometry. For 45671 the old pitched pose predicts finger engagement at **26.037 versus 10.806 mm**: a **15.230 mm** aperture mismatch and approximately **7.893 mm** opposing sampled gap at first engagement. Both-contact estimates after further hypothetical closure are not safe commands.

An existing 0°-pitch, +9.375 mm insertion-family candidate at section +0.3 predicts first engagements at **16.543 / 16.940 mm**, mismatch **0.396 mm**, with approximately **0.094 mm** opposing sampled gap and no detected open-hand approach or non-pad/other-body collision. Positive insertion here is translation along the existing outward normal; no bespoke offset was added. Fine collision/ray results are geometric diagnostics, not contact certification or exact tactile gap measurements.

## Tracking error sensitivity

The prior experimental command-to-measured TCP transform has **1.582 mm** translation and **0.389°** rotation error. Its translation components in commanded TCP coordinates are approximately (+1.442, −0.625, −0.189) mm. Applying that same transform to the ultimately selected mirrored candidate **offline only** changes the first-aperture mismatch to **1.963 mm** and opposing sampled clearance to **0.980 mm**. Thus the error materially affects geometry, but does not reproduce the old multi-millimeter opposing gap. For comparison, the physically successful 45638 archived pose predicts 3.152 mm first-aperture mismatch and about 1.899 mm opposing sampled clearance before its original bounded centering.

This is sensitivity to a transform recorded at a different grasp/station and under the isolated contact-feedback controller. It is not a fixed calibration model, a claim of new tracking accuracy or a GT correction applied to execution. Neither arm gains nor actuator authority were changed.

## Automatic plan and frozen execution

One automatic planning invocation completed in **173.402 s**, checking 1179 existing arc branches. It selected section **+0.3**, insertion-family **+9.375 mm**, roll **180°**, pitch **0°**, and base **(0.897618, −0.821380, −0.100000 m; yaw119.820°)**. No manually specified pose or saved successful grasp was inserted. The original failed station served only as the existing search seed.

Predicted minimum margins: home/preplan and approach **0.369 rad**, grasp **0.463 rad**. The saved future opening path reaches only **28°**, with 0.05 rad margin at its future boundary. That unused future limitation does not block this grasp-only trial; no opening is executed.

The short runner loads the **original frozen JawCenteredClosure**, SHA256 `b522bcef78a1191a4a2a5c756c34d11d7df2e22ddaa5f782be61343947a1704c`. Neither the unconditional CENTER-closing policy nor the contact-feedback policy is used. The original 100 s closure budget, finite native position actuator, gains, clamp authority, collision/force/joint/loaded-TCP protections, smooth approach and original LOW_PRELOAD/FORCE_HOLD behavior are retained. The base is placed before initialization and fixed throughout physical interaction.

## Physical outcome

Exactly one normally closed episode executed automatically. The original controller verified grasp at **132.867 s**, filtered left/right loads **0.4498 / 0.5662 N**, and aperture **16.803 mm**. Native contacts from both official finger links were against registered handle `original-35`. No attachment, object actuation, manual continuation, release, regrasp or scene restart occurred.

| 45671 | Frozen benchmark | Contact-feedback trial | New initial selection + original closure |
|---|---:|---:|---:|
| Initial bilateral acquisition | No | No | **Yes, original FORCE_HOLD/verification** |
| First unilateral measured aperture | 26.315 mm | 26.478 mm | **17.358 mm** |
| Final measured aperture | 24.678 mm | 23.367 mm | 16.787 mm |
| CENTER time | 41.079 s | 42.567 s | **2.433 s** |
| Maximum commanded centering | 2.000 mm | 2.000 mm | **0.468 mm** |
| Peak raw left / right load | 0.446 / 0 N | 0.622 / 0 N | 0.710 / 1.022 N |
| End-of-hold bilateral contact | Not acquired | Not acquired | **Left unloaded; right retained contact** |

The new candidate avoided the old approximately 26 mm early collision and reached bilateral filtered contact at **131.725 s**, aperture **16.856 mm**. Maximum measured acquisition correction was **0.871 mm**. Replaying frozen closure inputs reproduced aperture commands exactly; reopening count remained zero. Maximum centering target error was **0.889 mm**, final approximately **0.888 mm**, versus the earlier 1.582 mm endpoint. The station/IK configuration changed automatically too, so this one experiment does not isolate geometry from configuration's contribution to tracking.

Minimum measured joint margin was **0.371 rad**; maximum TCP speed after unilateral contact was **1.000 mm/s**. No force, collision, joint or speed protection event occurred. These measurements did not add execution gates. Maximum incidental door angle was only 0.000012°; no opening outcome is claimed.

### Holding limitation — do not hide the terminal unloading

The frozen runner returned `CLOSURE_HOLD_VERIFIED` and exported a 1.004 s holding interval. That original flag is preserved. **Raw evidence qualifies it:** the left finger fell below the existing 0.05 N filtered-contact level for the final **0.429 s**, and had no positive native contact for the final continuous **0.392 s**. The right finger retained contact throughout. Terminal raw/filtered loads were about **0 / 0.994–0.996 N**. This is not merely force asymmetry or a single-sample flicker.

The aperture command stayed fixed at **16.215 mm**, without reopening. Maximum handle-relative displacement/rotation during holding was only **0.056 mm / 0.035°**. The observation is consistent with small settling/load redistribution under a fixed position/preload command, but its cause was not isolated by an additional experiment. One-pad unloading alone does not prove gross grasp loss; no opening probe was performed to test continued transmission. Therefore report **bilateral acquisition verified, terminal holding contact degraded**, rather than a robust stable grasp-and-open PASS.

## Interpretation, preservation and stop

The actual native geometry supports an initial bilateral grasp in an existing candidate region; there is no evidence here that the present colliders make every 45671 grasp infeasible. The shared entry-accessibility ranking removed the earlier unilateral acquisition bottleneck without enlarging CENTER bounds or changing the original controller. Post-acquisition contact stability remains unresolved. Keep the improvement as an isolated development version and investigate terminal load redistribution separately before claiming robust holding/manipulation; do not squeeze harder or retrospectively tune this case.

Offline improvements on the other failures are not physical successes. The unchanged 45638 candidate remaining highly ranked is not a new regression trial. No benchmark denominator/result was rewritten and no further physical episode was run.

All streams decoded completely: **32,129 observations**, **32,129 native physics records** and **16,988 closure updates**. Continuous video decoded without errors: **133.867 s, 1280×960, 4,016 frames**. Episode simulation/wall time: **133.871 / 984.185 s**. Raw contacts, sensor metadata, actuator/joint states, source/configuration and plan snapshots remain private, with server/local backups and indexed hashes. Finished actor/launcher processes were closed after cooperative export.

See `OFFLINE_SUMMARY.json`, `PRELAUNCH.json`, `PLANNING_PROTOCOL.json`, `PHYSICAL_SUMMARY.json`, `HOLD_COMPLETENESS.json`, `SOURCE_INTEGRITY.json`, `EVIDENCE_INDEX.json`, `PRIVATE_ARCHIVE.json` and `PROCESS_CLEANUP.json`.

New ranking SHA256: `d5ded9611c7b7e61f256fbac5db4435048b8520dc147bcac31b1b2718af68cbc`; thin planning adapter: `00e744dc719a84a467de43524078b3d8e202a0317f9246f4217a6ce40cf5a49f`. Actual physical runner SHA256: `d6593c9d0d6dba0bbb4f392c3f12f5cbf8abacce0acfac8ca24be9e9446ddeb8`; frozen closure: `b522bcef78a1191a4a2a5c756c34d11d7df2e22ddaa5f782be61343947a1704c`; native configuration: `68fb6af5f4efd432b2355cabe37876a67f6c5ea3eac0b224603f08fb18d62939`. Initial execution sequence SHA256 `79a6b9d68f0b093e17014bfd2350e07b6c3983f7ae726b2e0cea6fd86f80442f` matches the original benchmark's sequence. The inherited diagnostic label `shared_progressive` means centering is retained; loaded-source provenance proves the original controller was used.
