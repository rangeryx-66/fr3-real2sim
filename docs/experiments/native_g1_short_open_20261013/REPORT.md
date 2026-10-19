# 45671 short physical opening — force transmission not demonstrated

**ORACLE_HANDLE / KNOWN_MODEL / xArm7 + native G1 / approximate simulation actuator.** Reference `4890542`. Exactly one fresh closed-start episode reused its byte-identical improved plan and original closure, native actuator, Pink and contact-aware continuation. No controller/physics/force-limit change, new candidate, release, regrasp, object actuation, attachment, restart or stitching. The original 4/10 benchmark and grasp-only result remain unchanged.

**Result: FAIL to demonstrate ≥2° short-range transmission.** The initial bilateral grasp reproduced, but maximum actual angle gained after verification was **0.006068°**, ending essentially closed. The approximately +3° target was not reached. This is not a 30° result or a mechanical-limit claim.

| Evidence | Actual result |
|---|---:|
| Grasp verification, simulation time | 132.867 s |
| Verified filtered left/right load | 0.450 / 0.566 N |
| Opening start after the full validated 1 s hold | 133.871 s |
| Opening-start left/right load | 0 / 0.996 N |
| Maximum actual gain after grasp | **0.006068°** |
| Maximum advancing hinge reference | About 0.187°; **reference is not actual progress** |
| Maximum TCP–handle translation/rotation from acquisition | 2.293 mm / 0.179° |
| Actual TCP endpoint displacement, world XYZ | (−0.308, −2.278, −0.105) mm |
| Maximum actual TCP displacement from opening start | 2.301 mm |
| Maximum commanded-TCP tracking error | 1.326 mm / 0.187° |
| Maximum measured TCP speed during opening/correction | 2.843 mm/s, below unchanged 5 mm/s protection |
| Minimum joint margin | 0.371 rad, above unchanged 0.05 rad protection |
| Peak opening raw left/right load | 0.434 / 1.081 N |
| Episode simulation / wall time | 138.504 / 809.809 s |

The target was a runtime +3° endpoint; the selected saved plan was not rewritten or re-planned. Both 0.5 s holding segments from the validated grasp-only prefix were retained, so the known left unloading was not avoided. The original opening implementation aligns its drive reference after holding; acquisition-anchored and opening-anchored drift were independently reconstructed from preserved raw body/TCP poses.

## First physical divergence and original stop sequence

1. **133.871 s:** opening begins with left contact unloaded and right load approximately 0.996 N. The existing protections permit single-pad unloading; it did not itself abort the episode.
2. **134.871 s:** reference advances to approximately 0.118°, while actual door angle remains essentially zero and acquisition-relative motion reaches approximately 0.906 mm. This is the early physical mismatch: robot motion is not producing corresponding door motion. The original relative-motion diagnostic starts at 135.046 s and does not terminate execution.
3. **135.375 s:** original contact-aware policy records its first stall: approximately **1.500 mm** opening-reference relative drift, nearly zero actual angle, loads **0 / 0.693 N**. It pauses reference advancement and attempts its one existing local correction.
4. **136.000 s:** approximately **0.150 mm** original TCP-centering correction briefly reacquires bilateral filtered contact. The door peaks at only about **0.006°** during this transient and returns close to zero. There is no useful continued opening.
5. **138.504 s:** a second original stall records approximately **2.301 mm** relative drift and loads **0 / 0.582 N**. The existing policy returns **`CONTACT_TRANSMISSION_FAILURE_AFTER_ONE_LOCAL_CORRECTION`**. The scene pauses, exports and closes normally; no retry is made.

Right-finger native contact remained present throughout opening. Left contact was present in only approximately **28.6%** of opening/correction frames; the local recovery was temporary, followed by another continuous native-contact absence of approximately **1.279 s**. Full interval details are in the result summary. The right load briefly peaked near 1.08 N, then decreased toward 0.58 N as relative motion grew: this was not sustained force growth or overload.

**No force-overload, collision, speed, joint-margin or simulator failure triggered the stop.** No both-pad grasp-loss stop occurred. The original broad tracking discrepancy diagnostic remained unset; the recorded stop was its existing progress-versus-relative-motion stall condition. Instrumentation added no execution gate.

The evidence supports ineffective contact transmission/sliding under the frozen setup, with non-negligible TCP tracking error. It does not isolate friction, contact-normal direction, hinge resistance or actuator settling as a unique cause. It also does not prove the object cannot open under another future skill. Initial bilateral acquisition alone is insufficient evidence of useful force transmission. No further correction, controller retuning or experiment was undertaken.

## Evidence and provenance

All records decoded with matching indexes and no chunk errors: **33,241 observations**, **33,241 native physics records**, **16,988 closure updates**. Continuous video decoded successfully: **138.5 s, 1280×960, 4,155 frames**. Raw points, normals, impulses, collider IDs, poses, joint/effort/reaction channels and sensor metadata remain private, backed up on the server and locally. Finished owned actor and launcher processes exited normally; unrelated processes were untouched. GPU selection used free VRAM, not exclusivity. Initialization required no repair in this episode.

Only this concise report, result summary and provenance are published. The executed wrapper, generated executable, full inputs and raw evidence are preserved privately. See `RESULT_SUMMARY.json` and `SOURCE_PROVENANCE.json`.

Selected-plan SHA256: `e599e02f7ce788e1deaabf5c893448ded4ccfe5320e303c07a8244d207ad432d`. Actual short-runner SHA256: `468013ac5beb6ffe97d71266d8d72a9bf44c15dec1cc779a73d1f87396fb9b7c`. Frozen closure: `b522bcef78a1191a4a2a5c756c34d11d7df2e22ddaa5f782be61343947a1704c`; opening source: `284090eb4779b5d188f5c81a2432fbba590fbc05a1d5286d2a5cbc200a991681`; initial sequence: `79a6b9d68f0b093e17014bfd2350e07b6c3983f7ae726b2e0cea6fd86f80442f`. Frozen native configuration and shared-policy hashes are retained in provenance.
