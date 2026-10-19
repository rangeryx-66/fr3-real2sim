# Saved-input diagnosis before physical outcomes

Reference `9ae41fb`; previous four-case results are preserved. No Isaac
manipulation episode was used to develop this update.

## 45600: actual filtering, not a presumed random model

Three fresh model loads × three repeats per identical RGB file gave identical
raw logit hashes per input. Native CUDA bfloat16 query/presence scores:

| Saved RGB | Highest score before filtering | Retained at 0.25 |
|---|---:|---|
| Shadow initial | 0.271484375 | one |
| Physical initial | 0.2177734375 | none |
| Physical re-observation | 0.007537841796875 | none |

The latter two were removed by confidence filtering, before later mask
size/border rejection. The initial camera K/extrinsics and robot q are equal
between shadow/physical records; RGB is not byte-identical. This establishes
sensitivity to the actual input, not a sole explanation from darkness or RNG.

The four missing `backbone.vision_backbone.convs.3` parameters are the lowest
resolution neck output removed by `scalp=1`. Models are in eval mode. Their
randomly initialized values differ across loads, but active logits do not.
Zeroing that branch gives exactly identical raw logits and scores. No confirmed
active loading/configuration defect was found; warnings remain visible and no
checkpoint substitution is made.

A single depth-observed crop on the saved failed initial image finds a complete
handle mask with score **0.66015625**, using the same prompt/threshold. The crop
comes from observed panel/protrusion geometry and measured robot exclusion,
not GT bounds. It is one bounded recovery, not a lowered confidence threshold.

## 45606: contact forecast differs from physical geometry

The old observed loft's median width was about 5.94 mm; offline GT projection
is about 15.53 mm. These are geometry diagnostics, not Oracle pose agreement.
At the selected old grasp, the observed model predicted bilateral first contact
at about 6.06 mm aperture. Native GT ray checks at the same pose/aperture instead
show strongly asymmetric gaps (one side about 7 mm inside the first surface).
The actual run stopped near 18.7 mm with one pad loaded; the other pad's sampled
native surface gap remained about 4.83–4.95 mm. Native contact rays are partial
geometry evidence, not measured pressure areas or a physical grasp certificate.

Pregrasp RGB-D registration translation was only **0.000643 mm**. The remaining
command/actual TCP difference was about 1.56 mm, much smaller than the nominal
contact forecast error. The evidence supports fixing mixed-depth/incomplete
observed geometry; it does not establish a frame/registration or actuator defect.
Difference from a saved Oracle grasp (another handle section) is not itself an
estimation error and is never an executable correction.

Shared input completion uses SAM seeds plus connected observed protruding depth,
with an observation-derived Otsu depth layer to separate mixed panel/back-layer
pixels. Unknown backside depth remains explicit: 0.75/1/1.25 × local observed
width, aggregated with existing mean/lower-quartile weights at common pose and
aperture. No best-imagined-backside selection or all-hypotheses veto.

## Four archived-case checks

| Asset | Updated point-to-GT surface P95 (offline only) | Planned interval | Shared rule |
|---|---:|---:|---|
| 45600 | 0.733 mm | 30° | depth crop + completed visible surface + three depth priors |
| 45606 | 0.710 mm (old 21.586 mm) | 30° | completed visible surface + three depth priors |
| 45671 | 0.559 mm | 20° | existing alternate observation + same input rules |
| 45638 | 0.864 mm | 30° | same input rules; offline regression only |

These are bounded offline results, not physical successes. Native collision
checks and known hinge/state remain allowed. GT errors are diagnostics with no
admission threshold. The input configuration is frozen before the registered
physical batch (45600, 45606, 45671); no per-case tuning follows outcomes.
