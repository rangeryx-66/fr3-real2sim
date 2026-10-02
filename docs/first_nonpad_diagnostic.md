# First nominal non-pad impulse: 2026-10-02

The frozen nominal grasp was replayed once and stopped at its first non-pad
impulse. No pose search, base change, IK change, friction change, force change,
geometry change or additional closure followed this diagnostic.
Here `metal` means non-pad finger collider ownership, not physical material.

## Result

The original official finger body **does not touch or intersect the semantic
handle proxy** in the recorded pre/post integration poses. The response is
primarily a collision-representation problem: native cooking makes `metal_497`
protrude approximately **0.248515 mm beyond the original pad plane**. The pad
collider does not acquire the same protrusion. Consequently the cooked non-pad
reaches the handle before the actual official pad or non-pad surface.

This is not an A-type confirmed official surface penetration. It is a positive
raw-gap, representation-induced near-contact response. It cannot be attributed
solely to entering the contactOffset shell: the same pair reported **103 prior
zero-impulse substeps** inside that shell. No counterfactual offset/cooking
experiment was performed, so their separate contributions are not quantified.

| Quantity at first impulse, physics index 6017 | Value |
|---|---:|
| Official left non-pad surface → proxy | 0.247310 mm |
| Official left pad surface → proxy | 0.247022 mm |
| Official right non-pad / pad surface → proxy | 0.276982 mm |
| Active authored non-pad collider → proxy | 0.248257 mm |
| Active cooked non-pad gap, before / after integration | 0.000527 / 0.000940 mm |
| SDK reported separation | −0.000541 mm |
| Finger contactOffset / restOffset | 0.530000 / 0 mm |
| Handle body's native contactOffset values | 0.340625, 0.355538 mm |
| Possible pair contactOffset sum | 0.870625–0.885538 mm |
| Contact impulse norm | 0.000751203 N·s |
| Equivalent force, impulse / physics dt | 0.180289 N |
| Physics dt | 1/240 s |
| Bilateral pad forces | 0 / 0 N |

The handle's runtime shape ordering is not identified; its bar's exact offset
must not be inferred from body-array order. Therefore the pair sum is an interval.
The SDK separation is about −0.54 µm while exported cooked geometry has a small
positive gap at both bracketing poses. These are not identical sampling instants;
this discrepancy is not evidence of official body penetration.

World contact point, metres:
`(0.670211375, -0.201687053, 0.150937140)`.
Native normal:
`(-0.999164999, 0.040856455, 0.000296016)`.
The report's original collider ordering is retained; no force-sign inference is
used for control.

## Classification and scope

The diagnostic records one **potential/representation contact event** and zero
**confirmed loaded official non-pad surface contacts**. A real cooked-shape
impulse was measured; it is not discarded or called harmless. The old rule
`non-pad impulse > 0 => actual finger-body collision` overstates this observation.
Entering a proximity shell alone must not count as confirmed loaded forbidden
contact. The existing baseline acceptance remains unchanged, while this replay
pauses for classification instead of declaring a new grasp failure/success.

No bilateral grasp, preload hold, 2 mm pull or articulated motion was completed.
The next step is to resolve the cooked pad/body boundary and implement contact
classification consistently before resuming closure; replacing the grasp pose
is not justified by this event. No penetration allowance is introduced.

## Artifacts and reproduction

- [Full measured fields](experiments/semantic_interaction_20261002/first_nonpad_diagnostic.json)
- [3D collider/contact view](experiments/semantic_interaction_20261002/first_nonpad_geometry.png)
- [Exact contact-plane slice](experiments/semantic_interaction_20261002/first_nonpad_cross_section.png)

Green: pad; red: non-pad (dashed authored versus solid cooked in the slice);
blue: handle proxy; black: native contact point and normal. Distances use the
official STL minus the explicitly owned pad faces, tested against the unchanged
semantic handle proxy solid. They are not distances to the original visual mesh.

Use the frozen plan and scene described in `semantic_interaction.md`, append
`--diagnose-first-nonpad` to `run_semantic_interaction.py`, then run:

```sh
python scripts/diagnose_first_nonpad.py \
  --trial results/semantic_interaction/first_nonpad
```

Full physics/cooked-shape traces remain on the server at that trial path.
The public artifact contains the measured values and plots, not the large traces.
For API semantics, NVIDIA documents contact generation via offsets in its
[collision guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/rigid_bodies_articulations/collision.html).
