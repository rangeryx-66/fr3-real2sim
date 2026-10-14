# Visual input reliability: paired development round

**Reference:** `9ae41fb`. **Frozen input update:** `4e91cc2`.
**Scope:** RGB-D + SAM3 handle input / known hinge, state and collision model /
xArm7 + native G1 / approximate simulation actuator. Three paired development
cases, not a held-out robustness estimate. Raw RGB-D, video and telemetry remain
private.

**Outcome: detection coverage improved, but end-to-end physical reliability did not.**
The same three development cases remain **1/3** for grasp acquisition and ≥2°
short opening. 45606 improves; previously successful 45671 regresses. Do not
promote this complete input update as a more reliable replacement for `9ae41fb`.

## What changed

Only the visual input and its approximate scoring surfaces changed. The
JawCenteredClosure implementation, native G1 actuation, preload, approach
handover, Pink controller, one-second hold, physics, collision geometry and
existing force/velocity/joint protections retain their previous bytes.

1. On a failed full-frame handle detection, one bounded crop is proposed from
   observed depth: an observed panel and connected protruding geometry, excluding
   the robot using measured robot state. The same prompt and **0.25** threshold
   are reused. No GT bounding box, instance mask, manual click or saved grasp.
2. SAM seeds are extended using connected visible depth. An observation-derived
   depth-layer split removes mixed panel/back-layer points. The hidden backside
   remains a declared prior: **0.75/1/1.25 × observed local width**. Scores are
   aggregated across these hypotheses using the existing mean/lower-quartile
   weights; the best imagined backside is not selected. No all-samples veto.
3. Passive auditing records raw query scores, confidence filtering, later mask
   rejection, crop/view recovery and pregrasp correction separately.

Candidate families, scoring weights, registered stations and the 450 s planning
budget are unchanged. Fresh physical observations regenerate all grasps and
approach/IK paths. The pregrasp common-visible RGB-D correction and robot
self-exclusion remain active. No GT grasp fallback was introduced.

Implementation: [input adapter](../../../scripts/native_g1_visual_input.py),
[observed geometry](../../../scripts/native_g1_observed_regions.py),
[SAM3 inference](../../../scripts/infer_sam3_observed.py),
[soft hypothesis scoring](../../../scripts/native_g1_visual_hypothesis_rank.py).

## Saved-input diagnosis

Three fresh SAM3 loads × three repetitions per identical RGB file produced
one raw-logit hash per input (27 inferences, three files). Raw highest scores
were **0.271484 / 0.217773 / 0.007538** for the saved shadow/physical/alternate
images. The latter two failed the unchanged 0.25 confidence filter, before
mask-size/border rejection. RGB bytes differ despite equal K/extrinsics/robot q.
The four missing `convs.3` keys belong to the output discarded by `scalp=1`;
zeroing that branch leaves active logits unchanged. No active loading defect
was confirmed. Same-image observed-depth cropping recovers score **0.660156**.

45606's old observed width was **5.94 mm** versus about **15.53 mm** offline GT
projection. The model forecast ~6.06 mm common aperture, but actual left contact
started at 21.42 mm; no right contact followed. Final sampled right clearance
was 4.83–4.95 mm, while pregrasp registration correction was only 0.000643 mm.
The supported change is observed-geometry cleanup, not an established coordinate
frame repair or matching one Oracle grasp. See [DIAGNOSIS.md](DIAGNOSIS.md) and
the offline JSON summaries for full saved-input evidence.

## Archived four-case check before physics

| Case | Updated observed-point surface P95, offline GT only | Planned interval | Planning wall time |
|---|---:|---:|---:|
| 45600 | 0.733 mm | 30° | 66.0 s |
| 45606 | 0.710 mm; previously 21.586 mm | 30° | 148.9 s |
| 45671 | 0.559 mm | 20° | 29.9 s |
| 45638 | 0.864 mm | 30° | 100.0 s |

These checks used archived inputs. GT evaluation ran after planning in a
separate offline evaluator and supplied no pose corrections or admission
threshold. Sampled native-surface contact mismatch remains possible despite low
point-to-surface distance. Ray support is approximate geometry, not measured
pressure area or a physical grasp certificate.

## Physical batch

User-registered order: **45600 → 45606 → 45671**. One normal-closed-start episode
each, sequential actors, no between-case changes or physical retries. The
sequence is fresh visual input → pregrasp observation → physical closure →
unchanged one-second hold → approximately +3° opening. The reported short
outcome is **actual gain ≥2° after GRASP_VERIFIED**, not commanded progress,
pre-grasp door disturbance or a 30° task success.

All three physical actors completed, without initialization repairs, physical
retries, manual continuation or between-case tuning.

| Case | Previous visual result | Updated detection/recovery | Updated grasp | Actual gain after acquisition | ≥2° |
|---|---|---|---|---:|---|
| 45600 | missed detection; no motion | one depth crop succeeds | no bilateral hold | — | fail |
| 45606 | unilateral closure; no opening | full frame; no recovery | verified bilateral acquisition | **3.005721°** | **PASS** |
| 45671 | verified grasp; +2.998820° | failed full frame/crop; existing alternate view succeeds | no bilateral hold | — | fail |

Matched three-case detection coverage: **2/3 → 3/3** after bounded recovery.
Grasp acquisition: **1/3 → 1/3**; short transmission: **1/3 → 1/3**. This is not a
net physical-reliability improvement. The previous four-case visual result
remains **2/4**; the fourth case, 45638, was checked offline only this round.

45600 and 45671 never reached GRASP_VERIFIED. Their unverified all-phase angle
maxima, **0.000238° / 0.000098°**, are preserved but do not count as grasped
opening. Both ended with the original BILATERAL_HOLD_NOT_ESTABLISHED condition
after the original 100 s closure budget; neither invoked the opening controller.

| Case | Handle SAM3 forwards | Depth crops | Camera reobservations | Auxiliary panel calls | Initial/pregrasp RGB-D captures |
|---|---:|---:|---:|---:|---:|
| 45600 | 2 | 1, successful | 0 | 0 | 1 / 1 |
| 45606 | 1 | 0 | 0 | 0 | 1 / 1 |
| 45671 | 3 | 1, unsuccessful | 1, successful | 1 | 2 / 1 |

For 45671 the fresh full-frame/crop maxima were **0.015503 / 0.022949**. The
alternate full-frame view scored **0.84375**. No second crop or GT fallback was
used. Recovery is part of the episode, not an excluded preparation trial.

### Physical mechanisms and remaining uncertainty

45600's fresh full-frame score was **0.1884765625**. The one depth-proposed crop
scored **0.66015625** and recovered detection, but the resulting grasp still
failed acquisition. Its model expected approximately **12.54 mm** common
aperture; actual first left contact occurred at **25.30 mm**, and no positive
right normal contact was recorded. Closure ended at **20.40 mm** measured /
**20.37 mm** commanded after the original budget, with peak pad load only
**0.0918 N**. This is a remaining contact-geometry prediction mismatch; opening
never began and no mechanical range limit was tested. Final aperture tracking
error was only about **0.031 mm**. Low observed-point surface error does not establish completeness
of the hidden cross-section or agreement with the native collision surface.

45606's fresh full-frame detection scored **0.734375**, without crop/view
recovery. The shared ranking selected a different observed handle section and
an existing insertion-depth candidate, with no manual pose offset. The original
closure acquired **0.488/0.502 N** left/right loads. After the unchanged
**1.004 s** recorded hold, actual post-acquisition gain was **3.0057°**. Maximum
post-acquisition relative motion was **0.808 mm / 0.418°**, peak raw pad load
**1.138 N**, minimum joint margin **1.048 rad**, maximum measured post-grasp TCP
speed **1.164 mm/s**. No original force warning/stop was recorded.

The left pad had approximately **13.69 s** without a positive allowed normal
contact during early opening while the right retained contact and moved the
door. This is short-range force transmission despite unloading, not proof of
bilateral contact throughout, long-duration retention or a complete contact
wrench estimate. The all-phase TCP peak was **21.81 mm/s** in unloaded PREGRASP;
it is retained separately from the loaded/post-acquisition metric.

The same-input crop test directly supports the detection-recovery explanation.
The 45606 geometry cleanup and hypothesis ranking were tested as one coherent
input update, so this batch does not isolate their individual causal effects.
Remaining backside/collision-surface uncertainty is explicit, particularly for
45600; no GT evaluation residual was used to admit, reject or correct a pose.

45671 regressed despite a feasible short plan. Its updated observed length was
**195.0 mm**, versus **291.4 mm** previously. The shared rule selected section
+0.3, zero insertion and −15° pitch, versus the previous visual candidate's
9.375 mm insertion and 0° pitch. These are automatically selected candidate
parameters, not new manual offsets. The updated model expected **11.34 mm**
common aperture, while first left normal contact appeared at **24.66 mm**
(t=145.05 s); no positive right normal contact followed. Closure ended at
**18.96 mm measured / 18.92 mm commanded**, with peak pad load **0.1675 N**.

The evidence supports an observed-model/native-contact mismatch. Foreground
completion also changes visible extent and candidate ranking; this experiment
does not isolate their effects or prove one exact hidden-backside model caused
the regression. One-way point-to-GT surface distance measures agreement of
retained visible points, not completeness of the unobserved cross-section or
closing-path support. Width-scaled depth hypotheses did not resolve this.
Registration and finite actuator feedback remain recorded; no GT correction,
stronger squeeze or CENTER tuning was applied to rescue the trial.

| Case | Approx. model common aperture | First actual left / right contact aperture | Closure-relative TCP change |
|---|---:|---:|---:|
| 45600 | 12.536 mm | 25.295 mm / none | 2.538 mm |
| 45606 | 10.477 mm | 16.176 / 14.706 mm | 0.847 mm |
| 45671 | 11.336 mm | 24.655 mm / none | 2.856 mm |

Closure-relative change includes original centering and measured tracking/settling;
it is **not** the post-acquisition slip metric. Common-aperture scores are
approximate forecasts, not commands or physical certificates. First positive
normal contact can be transient; the existing verification is reported separately.

## Measurements, runtime and evidence integrity

| Case | Peak raw pad load | Minimum joint margin | All-phase / post-grasp TCP peak | Verified-grasp relative shift / rotation |
|---|---:|---:|---:|---:|
| 45600 | 0.0918 N | 1.0040 rad | 20.977 / — mm/s | — |
| 45606 | 1.1376 N | 1.0479 rad | 21.811 / 1.164 mm/s | 0.808 mm / 0.418° |
| 45671 | 0.1675 N | 1.0927 rad | 20.606 / — mm/s | — |

All three all-phase velocity peaks occurred in unloaded PREGRASP with both
reported loads zero. They are unfiltered measured TCP position differences over
recorded dt, retained rather than masked. The existing **5 mm/s loaded-TCP**
protection was unchanged. No force warning/hard-stop event was recorded in this
batch. Earlier force peaks remain in their original evidence; these new records
do not overwrite or pool them.

| Case | Previous sim / runner wall | Updated sim / runner wall | Previous / updated live planning |
|---|---:|---:|---:|
| 45600 | 0 / 75.6 s | 179.904 / 1331.0 s | no plan / 94.2 s |
| 45606 | 196.092 / 1540.7 s | 154.842 / 1183.4 s | 269.2 / 201.9 s |
| 45671 | 245.925 / 1928.1 s | 185.613 / 1329.5 s | 73.4 / 41.0 s |

Total new simulation **520.358 s**; launch-to-last-process-exit wall time
**3853.417 s (64.22 min)**. Per-phase simulation counts and forward-inference
costs are in RESULT_SUMMARY.json. Forward timers exclude checkpoint loading;
complete perception-phase wall timers were not instrumented. Live planning time
is distinct from the archived-input checks above. Different outcomes and system
load prevent a controlled speed comparison: 45600 now performs physics instead
of failing before motion, and 45671's shorter run is an early failure, not an
execution-speed improvement.

| Case | Observation records | Physics-step records | Closure-command records | Full video decode |
|---|---:|---:|---:|---|
| 45600 | 43,177 | 43,177 | 24,000 | pass |
| 45606 | 37,162 | 37,162 | 18,151 | pass |
| 45671 | 44,547 | 44,547 | 24,000 | pass |

Video QA also confirms a recording limitation: 45671's close view is arm-occluded;
the continuous overview inset and raw contact evidence remain available. The
legacy failed-case overlay `slip=0` is a pre-acquisition placeholder, not proof
of stable contact. Chunk counts match every index; no supported-telemetry export errors were
reported. Raw contacts, impulses, points/normals, collider IDs, joint reactions,
gripper/TCP states, RGB/depth/calibration, source snapshots and sensor metadata
remain private. All three actors exited with OS code 0 after cooperative export
(including the two reported manipulation failures); recorded actor/recorder
processes are no longer running. No other user's process was terminated. GPU
selection used free capacity, without an exclusivity requirement.

SOURCE_PROVENANCE.json records module SHA-256 values, generated/loaded control
hashes, checkpoint hash, per-job hashes and post-run process audit. All three
loaded the same frozen control code; all registered module bytes still match
the pre-physics freeze. Job differences from the previous version are limited
to paths/episode labels, GPU/deadline resources and visual-input version.
EVIDENCE_INDEX.md locates the private archive and its checksum. No raw archive,
video or RGB-D was uploaded to a public GitHub Release.

The experiment stops here. Keep the prior successful versions; retain the crop
recovery evidence and the 45606 improvement, but treat the complete updated
geometry/ranking adapter as a development version with a demonstrated regression.

## Interpretation limits

Remaining dependencies are the registered initial station, known hinge geometry,
dynamic GT object/articulation state, robot/camera calibration and original scene
collision checks and registered collider identities for permitted-contact
checks/reporting. Observations come from the existing calibrated **scene
camera**, not autonomous wrist-camera observation. Original native geometry
remains in physical contact/collision checks; GT handle surfaces do not generate
visual candidates or contact scores.

The three backside models are declared priors, not calibrated vision confidence
bounds. This round is not a factorial ablation separating completion from
hypothesis scoring. Normal contact impulses omit friction-anchor forces and are
not complete interaction wrenches. G1 contact feedback here is simulation-only;
this is not hardware validation.

Previous four-case visual results, Oracle versions and the original **4/10**
30° benchmark remain unchanged. No extra objects, controller redesign,
full-range trials or further tuning are authorized by this report.
