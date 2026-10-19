# Transferable PiPER grasp-and-open skill: bounded development result

**ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT. Four pre-registered independent physical episodes; one unchanged shared implementation; no retries or tuning between cases.**

| Case | Correct handle grasp | Maximum actual opening | 30° target | Maximum relative drift |
|---|---|---:|---|---:|
| 45415, frozen positive control | Yes | 30.000027° | PASS | 1.391 mm |
| 45176, transmission-failure control | Yes | 8.068333° | FAIL | 3.760 mm |
| 45134, new horizontal bar | Yes | 20.906580° | FAIL | 0.494 mm |
| 45443, new vertical bar | Yes | 17.534973° | FAIL | 0.870 mm |

**Shared contribution.** Both pads are ranked at one common pose/aperture against visual and unchanged native collision geometry, using contact support, opposing normals and opening-force direction. Range has bounded weight. A thin adapter reuses the proven closure/handover/Pink primitives. Actual progress relative to increasing drift can hold the reference and attempt one existing bounded centering correction, without resetting the original slip reference. Only 45176 invokes it.

**What works.** The positive baseline is preserved. All four cases grasp the registered handles and transmit physical opening. Two new handle configurations maintain low drift without correction. No loaded-TCP speed violation occurs; complete 240 Hz data and continuous video are exported and actors close normally.

**What remains unresolved.** Three automatically selected fixed-base plans terminate at the original joint-margin protection, consistent with their predicted partial intervals. This is not full 30° transfer and not a global hardware/mechanical limit. 45176 still unloads one pad and drifts. Native convex hulls erase visible rear cavities on several bow handles; colliders remain unchanged.

**Next step.** Improve automatic station/continuous IK branch coverage of 30° while preserving contact utility. Evaluate any collision representation change as a separate family-wide environment revision with the same controller. GT geometry and dynamic state remain required. Four development cases do not establish a robust success rate or replace prior benchmark scores.

Full source hashes, plans, measurements, capability matrix and raw evidence paths: [report](REPORT.md).
