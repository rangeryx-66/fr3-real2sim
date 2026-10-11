# Archived evidence diagnosis — ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT

No new physical trials were used for this diagnosis. Sources are the immutable Benchmark A videos, collider associations, selected plans and native 240 Hz records.

| Object | Actual mechanism | Evidence | Supported shared correction |
|---|---|---|---|
| 45176 | Planner selected trim original-39, not handle original-46. Contact redistribution caused a measured TCP spike; no preceding command discontinuity was observed. | At t=53.729167 s: TCP 5.17761 mm/s, left/right loads 0/2.14442 N. Reference 0.71613 mm/s, maximum commanded joint increment 0.000061636 rad, matching preceding increments. | Restrict candidates to registered handle; rank distal pad/contact and closing sweep geometry. Do not raise the speed limit or infer a Pink instability solely from this impulse. |
| 45203 | Trim original-52 was grasped instead of handle original-53; the robot motion did not transmit to the door. | At t=173.866667 s reference led by >2 degrees while actual opening was approximately zero and TCP tracking error only 35.7 micrometres. Relative drift eventually 17.448 mm / 4.191 degrees, followed by loss of both pad loads. | Handle-only geometry and contact/closing sweep ranking. The existing acquired handle-to-TCP reference is already present; avoid adding another tracking veto. |
| 45415 | Correct handle original-40 transmitted motion stably, but selected base/wrist branch exhausted joint 5 travel. | Maximum 28.99028 degrees, loaded min margin 0.050094 rad. J5 command -1.17172749 rad against lower limit -1.2217304 rad; next command hit unchanged 0.05 rad boundary. Max loaded TCP 0.622 mm/s, slip 0.421 mm. Old planner selected by range and stopped at its first adequate plan, not whole-arc minimum margin. | Rank grasp/base/wrist branches by whole approach/opening path margin and contact quality, with original constraints unchanged. |

The contact impulse correlation in 45176 does not establish a unique causal direction within a physics step. It supports fixing the wrong contact geometry before attempting controller changes. No evidence presently justifies changing gains, physical parameters, jaw effort or safety thresholds.

A convex-hull approximation can fill a bow handle's rear gap. The offline audit must report visual-versus-cooked geometry and actual closing contacts; geometry will not be removed or silently replaced. Failures caused by this representation will remain explicit blockers.

Scope: three initial development trials, at most one additional evidence-backed shared-fix retest, maximum four. No old benchmark reinterpretation as a new success, no individual pose tuning, no regrasp, fixed base after normal setup.

## Native collider audit and planning implementation

The blue visual/orange native hull comparison confirms that the 45176 and 45203 bow handles have their rear cavities filled by the existing per-mesh convex approximation. The supplied meshes were not edited. Native and visual world bounds agree within sub-micrometre import error. Closing width must be swept to zero: the full mesh width includes mounting/support geometry and cannot substitute for the local bar width. The new planner uses that sweep and the unchanged distal-contact definition only for candidate ranking.

Candidate ordering considers original contact-region risk, achieved opening interval, pad contact estimate and minimum joint margin over both approach and opening. It no longer returns the first nominal 30-degree endpoint. Offline source-to-scene base transform is read from the immutable cooked export, rather than a source report subsequently updated to an executed station. Runtime still exports the actual new scene and uses the unchanged physical guards.
